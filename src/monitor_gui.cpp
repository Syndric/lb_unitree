/**********************************************************************
 * Unitree Go1 Power & Battery Monitor GUI
 *
 * This application uses the Unitree Legged SDK to create a
 * real-time GUI dashboard for monitoring power and battery stats.
 *
 * Updates:
 * - Displays Total Energy (Wh) on GUI.
 * - Logs Total Energy (Wh) to CSV.
 * - Logs Joint Torques (Nm) to CSV.
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <mutex>   // For thread-safe data access
#include <chrono>  // For time calculations
#include <vector>  // For plot history
#include <numeric> // For std::accumulate
#include <fstream> // For CSV logging
#include <iomanip> // For std::setprecision
#include <atomic>  // For std::atomic<bool>
#include <array>   // For std::array (to hold joint torques)
#include <algorithm> // For std::max

// --- ImGui & Backend Headers ---
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// --- ImPlot Header ---
#include "implot.h"

// --- GLFW / OpenGL Headers ---
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

using namespace UNITREE_LEGGED_SDK;

// Helper to limit the size of a data vector for plotting
void limit_data_vector(std::vector<float> &vec, size_t max_size)
{
    if (vec.size() > max_size)
    {
        vec.erase(vec.begin(), vec.begin() + (vec.size() - max_size));
    }
}

// Data structure to hold all monitored values
struct MonitorData
{
    // Raw BMS State
    BmsState bms;

    // Calculated Values
    float totalVoltage_V = 0.0f;
    float current_A = 0.0f;
    float power_W = 0.0f;
    double totalEnergy_Wh = 0.0;
    double averagePower_W = 0.0;
    double runTime_s = 0.0;

    // Joint Torques
    std::array<float, 12> jointTorques;

    // Plotting History (using 500 samples for better ImPlot resolution)
    std::vector<float> socHistory;
    std::vector<float> powerHistory;
    std::vector<float> voltageHistory;
    std::vector<float> currentHistory;
    std::array<std::vector<float>, 12> jointTorqueHistory;
};

class RobotMonitor
{
public:
    RobotMonitor() : safe(LeggedType::Go1),
                     udp(HIGHLEVEL, 8090, "192.168.12.1", 8082),
                     running(false),
                     dt(0.002), // 500Hz
                     loop_count(0),
                     loop_udpSend(nullptr),
                     loop_udpRecv(nullptr),
                     loop_control(nullptr),
                     plotHistorySize(500) // Store 500 samples (1 second at 500Hz)
    {
        udp.InitCmdData(cmd);
        cmd.mode = 0; // Set to idle mode for safety
    }

    ~RobotMonitor()
    {
        StopMonitoring();
    }

    void StartMonitoring()
    {
        if (running)
            return; // Already running

        // Reset data struct
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            data.bms = {}; // Zero-initialize the BMS struct
            data.totalVoltage_V = 0.0f;
            data.current_A = 0.0f;
            data.power_W = 0.0f;
            data.totalEnergy_Wh = 0.0;
            data.averagePower_W = 0.0;
            data.runTime_s = 0.0;
            data.jointTorques.fill(0.0f); 
            data.socHistory.clear();
            data.powerHistory.clear();
            data.voltageHistory.clear();
            data.currentHistory.clear();
            for (auto &vec : data.jointTorqueHistory)
            {
                vec.clear();
            }
        }

        // Open log file and write header
        logFile.open("go1_bms_log.csv", std::ios::out | std::ios::trunc);
        if (logFile.is_open())
        {
            // Write Standard Headers
            logFile << "Timestamp(ms),Runtime(s),SOC(%),Voltage(V),Current(A),Power(W),TotalEnergy(Wh),";
            
            // Write Torque Headers
            const char* logJointNames[12] = {"FR_Hip","FR_Thigh","FR_Calf","FL_Hip","FL_Thigh","FL_Calf","RR_Hip","RR_Thigh","RR_Calf","RL_Hip","RL_Thigh","RL_Calf"};
            for(int i=0; i<12; ++i) {
                logFile << logJointNames[i] << "_Tau(Nm),";
            }

            // Write Cell Voltage Headers
            for (int i = 0; i < 10; ++i)
            {
                logFile << "Cell" << i + 1 << "(mV)" << (i == 9 ? "\n" : ",");
            }
        }
        else
        {
            std::cerr << "Error: Could not open log file!" << std::endl;
        }

        running = true;
        startTime = std::chrono::high_resolution_clock::now();
        lastUpdateTime = std::chrono::high_resolution_clock::now();
        loop_count = 0;

        // --- Use LoopFunc ---
        loop_udpSend = new LoopFunc("udp_send", dt, 3, boost::bind(&RobotMonitor::UDPSend, this));
        loop_udpRecv = new LoopFunc("udp_recv", dt, 3, boost::bind(&RobotMonitor::UDPRecv, this));
        loop_control = new LoopFunc("control_loop", dt, 3, boost::bind(&RobotMonitor::RunMonitorLoop, this));

        loop_udpSend->start();
        loop_udpRecv->start();
        loop_control->start();
    }

    void StopMonitoring()
    {
        if (!running)
            return; // Already stopped
        running = false;

        // Stop and delete LoopFunc objects
        if (loop_udpSend) { delete loop_udpSend; loop_udpSend = nullptr; }
        if (loop_udpRecv) { delete loop_udpRecv; loop_udpRecv = nullptr; }
        if (loop_control) { delete loop_control; loop_control = nullptr; }

        if (logFile.is_open())
        {
            logFile.close();
            std::cout << "Log file closed." << std::endl;
        }
    }

    bool IsRunning() const
    {
        return running.load();
    }

    // --- LoopFunc Callbacks ---

    void UDPRecv()
    {
        udp.Recv();
    }

    void UDPSend()
    {
        cmd.mode = 0;
        udp.Send();
    }

    void RunMonitorLoop()
    {
        if (!running) return;

        udp.GetRecv(state);
        auto now = std::chrono::high_resolution_clock::now();
        double dt_actual = std::chrono::duration<double>(now - lastUpdateTime).count();

        if (dt_actual < 0.001) return; // Avoid spikes

        lastUpdateTime = now;

        // --- Calculations ---
        float voltage = std::accumulate(state.bms.cell_vol.begin(), state.bms.cell_vol.end(), 0) / 1000.0f; // mV to V
        float current = state.bms.current / 1000.0f; // mA to A
        float power = voltage * current;

        double runTime = std::chrono::duration<double>(now - startTime).count();

        // Local copy for accurate energy calculation
        double currentTotalEnergy_Wh = 0.0;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            currentTotalEnergy_Wh = data.totalEnergy_Wh;
        }

        double energy_Ws = power * dt_actual; // Energy in Watt-seconds (Joules)
        double totalEnergy_Ws = (currentTotalEnergy_Wh * 3600.0) + energy_Ws; // Convert Wh back to Ws to add

        // --- Lock and Update Data ---
        {
            std::lock_guard<std::mutex> lock(dataMutex);

            data.bms = state.bms;
            data.totalVoltage_V = voltage;
            data.current_A = current;
            data.power_W = power;
            data.runTime_s = runTime;

            // Copy joint torques
            for (int i = 0; i < 12; ++i)
            {
                data.jointTorques[i] = state.motorState[i].tauEst;
            }

            // Update totals
            data.totalEnergy_Wh = totalEnergy_Ws / 3600.0; // Convert back to Wh
            data.averagePower_W = (runTime > 0) ? (totalEnergy_Ws / runTime) : 0.0;

            // Update plot history
            limit_data_vector(data.socHistory, plotHistorySize);
            limit_data_vector(data.powerHistory, plotHistorySize);
            limit_data_vector(data.voltageHistory, plotHistorySize);
            limit_data_vector(data.currentHistory, plotHistorySize);

            data.socHistory.push_back(state.bms.SOC);
            data.powerHistory.push_back(power);
            data.voltageHistory.push_back(voltage);
            data.currentHistory.push_back(current);

            // Update joint torque history
            for (int i = 0; i < 12; ++i)
            {
                limit_data_vector(data.jointTorqueHistory[i], plotHistorySize);
                data.jointTorqueHistory[i].push_back(data.jointTorques[i]);
            }
        }

        loop_count++;
        
        // Log to file periodically (every 50 loops ~ 10Hz)
        if (loop_count % 50 == 0 && logFile.is_open())
        {
            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            logFile << timestamp << ","
                    << std::fixed << std::setprecision(3) << runTime << ","
                    << (int)state.bms.SOC << ","
                    << voltage << ","
                    << current << ","
                    << power << ","
                    << std::setprecision(6) << data.totalEnergy_Wh << ","; // Log Total Wh

            // Log Torques
            for (int i = 0; i < 12; ++i)
            {
                logFile << std::setprecision(3) << state.motorState[i].tauEst << ",";
            }

            // Log Cells
            for (int i = 0; i < 10; ++i)
            {
                logFile << state.bms.cell_vol[i] << (i == 9 ? "\n" : ",");
            }
        }

        udp.SetSend(cmd);
    }

    MonitorData GetDisplayData()
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        return data; 
    }

private:
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    float dt;
    long long loop_count;
    const size_t plotHistorySize;

    std::atomic<bool> running;

    MonitorData data;
    std::ofstream logFile;
    std::mutex dataMutex;

    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::high_resolution_clock::time_point lastUpdateTime;

    LoopFunc *loop_udpSend;
    LoopFunc *loop_udpRecv;
    LoopFunc *loop_control;
};

// --- GLFW Error Callback ---
static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

// --- Main Application ---
int main(int, char **)
{
    std::cout << "Initializing Robot Monitor..." << std::endl;
    RobotMonitor monitor;

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow *window = glfwCreateWindow(1024, 768, "Unitree Go1 Power Monitor", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); 

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext(); 
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    const char *jointNames[12] = {
        "FR_Hip  ", "FR_Thigh", "FR_Calf ",
        "FL_Hip  ", "FL_Thigh", "FL_Calf ",
        "RR_Hip  ", "RR_Thigh", "RR_Calf ",
        "RL_Hip  ", "RL_Thigh", "RL_Calf "};

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        MonitorData displayData = monitor.GetDisplayData();

        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        ImGui::Text("UNITREE GO1 POWER MONITOR (ImPlot Version)");
        ImGui::Separator();

        // --- START/STOP BUTTONS ---
        if (monitor.IsRunning())
        {
            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.6f, 0.6f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0.0f, 0.7f, 0.7f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0.0f, 0.8f, 0.8f));
            if (ImGui::Button("Stop Monitoring", ImVec2(ImGui::GetContentRegionAvail().x, 0)))
            {
                monitor.StopMonitoring();
            }
            ImGui::PopStyleColor(3);
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.3f, 0.6f, 0.6f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0.3f, 0.7f, 0.7f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(0.3f, 0.8f, 0.8f));
            if (ImGui::Button("Start Monitoring", ImVec2(ImGui::GetContentRegionAvail().x, 0)))
            {
                monitor.StartMonitoring();
            }
            ImGui::PopStyleColor(3);
        }
        ImGui::Separator();

        // --- DATA DISPLAY ---
        if (!monitor.IsRunning())
        {
            ImGui::Text("Monitoring is stopped. Press 'Start' to begin.");
            ImGui::Text("Log file 'go1_bms_log.csv' will be created/overwritten.");
        }
        else if (displayData.runTime_s < 0.5)
        { 
            ImGui::Text("Connecting to robot... (Runtime: %.2f s)", displayData.runTime_s);
        }
        else
        {
            // --- Stats Window ---
            ImGui::BeginChild("Stats", ImVec2(ImGui::GetContentRegionAvail().x * 0.4f, 0), true);
            
            // UPDATED: Display Total Energy Wh here
            ImGui::Text("STATUS");
            ImGui::Text("Runtime:      %.2f s", displayData.runTime_s);
            ImGui::Text("Total Energy: %.4f Wh", displayData.totalEnergy_Wh);
            
            ImGui::Separator();
            ImGui::Text("SOC:        %d %%", (int)displayData.bms.SOC);
            ImGui::BeginChild("Cells", ImVec2(0, 150), false, ImGuiWindowFlags_HorizontalScrollbar);
            for (int i = 0; i < 10; i++)
            {
                ImGui::Text("Cell %2d: %d mV", i + 1, displayData.bms.cell_vol[i]);
            }
            ImGui::EndChild(); 

            ImGui::Separator();
            ImGui::Text("Joint Torques (Est. Nm):");
            ImGui::BeginChild("Torques", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), false, ImGuiWindowFlags_HorizontalScrollbar);
            for (int i = 0; i < 12; i++)
            {
                if (i % 3 != 0)
                    ImGui::SameLine(150.0f * (i % 3));

                ImGui::Text("%s: %5.2f", jointNames[i], displayData.jointTorques[i]);
            }
            ImGui::EndChild(); 

            ImGui::EndChild(); // End Stats

            ImGui::SameLine();

            // --- Plots Window ---
            ImGui::BeginChild("Plots", ImVec2(0, 0), true);
            ImGui::Text("REAL-TIME PLOTS (Last %d samples)", (int)displayData.socHistory.size());

            if (!displayData.socHistory.empty())
            {
                std::vector<float> absPowerHistory(displayData.powerHistory.size());
                std::vector<float> absCurrentHistory(displayData.currentHistory.size());
                for (size_t i = 0; i < displayData.powerHistory.size(); ++i)
                {
                    absPowerHistory[i] = -displayData.powerHistory[i]; 
                }
                for (size_t i = 0; i < displayData.currentHistory.size(); ++i)
                {
                    absCurrentHistory[i] = -displayData.currentHistory[i]; 
                }

                // Calculate height for 3 plots
                float plotHeight = (ImGui::GetContentRegionAvail().y - ImGui::GetStyle().ItemSpacing.y * 2.0f) / 3.0f;
                plotHeight = std::max(plotHeight, 50.0f);

                // --- Plot 1: SOC & Voltage ---
                if (ImPlot::BeginPlot("SOC & Voltage", ImVec2(-1, plotHeight)))
                {
                    ImPlot::SetupAxis(ImAxis_Y1, "SOC (%)", ImPlotAxisFlags_None);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 100, ImPlotCond_Always);
                    ImPlot::SetupAxis(ImAxis_Y2, "Voltage (V)", ImPlotAxisFlags_Opposite);
                    ImPlot::SetupAxisLimits(ImAxis_Y2, 18.0, 26.0, ImPlotCond_Always);

                    ImPlot::PlotLine("SOC", displayData.socHistory.data(), displayData.socHistory.size());
                    ImPlot::SetAxis(ImAxis_Y2); 
                    ImPlot::PlotLine("Voltage", displayData.voltageHistory.data(), displayData.voltageHistory.size());
                    ImPlot::EndPlot();
                }

                // --- Plot 2: Power & Current ---
                if (ImPlot::BeginPlot("Power & Current (Usage)", ImVec2(-1, plotHeight)))
                {
                    ImPlot::SetupAxis(ImAxis_Y1, "Power (W)", ImPlotAxisFlags_None);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 500, ImPlotCond_Always);
                    ImPlot::SetupAxis(ImAxis_Y2, "Current (A)", ImPlotAxisFlags_Opposite);
                    ImPlot::SetupAxisLimits(ImAxis_Y2, 0, 20, ImPlotCond_Always);

                    ImPlot::PlotLine("Power", absPowerHistory.data(), absPowerHistory.size());
                    ImPlot::SetAxis(ImAxis_Y2);
                    ImPlot::PlotLine("Current", absCurrentHistory.data(), absCurrentHistory.size());
                    ImPlot::EndPlot();
                }

                // --- Plot 3: Joint Torques ---
                if (ImPlot::BeginPlot("Joint Torques (Est. Nm)", ImVec2(-1, plotHeight)))
                {
                    ImPlot::SetupLegend(ImPlotLocation_East, ImPlotLegendFlags_Outside);
                    ImPlot::SetupAxis(ImAxis_Y1, "Torque (Nm)", ImPlotAxisFlags_None);
                    ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30, ImPlotCond_Always);

                    for (int i = 0; i < 12; ++i)
                    {
                        if (!displayData.jointTorqueHistory[i].empty())
                        {
                            ImPlot::PlotLine(jointNames[i], displayData.jointTorqueHistory[i].data(), displayData.jointTorqueHistory[i].size());
                        }
                    }
                    ImPlot::EndPlot();
                }
            }

            ImGui::EndChild(); // End Plots
        }

        ImGui::End(); // End Main

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    std::cout << "Stopping robot monitor thread..." << std::endl;
    monitor.StopMonitoring();
    std::cout << "Cleaning up GUI..." << std::endl;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext(); 
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout << "Done." << std::endl;

    return 0;
}