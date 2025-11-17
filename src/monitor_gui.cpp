/**********************************************************************
 * Unitree Go1 Power & Battery Monitor GUI
 *
 * This application uses the Unitree Legged SDK to create a
 * real-time GUI dashboard for monitoring power and battery stats.
 *
 * It uses Dear ImGui for the GUI, GLFW for the window,
 * and OpenGL for rendering.
 *
 * It now uses the SDK's LoopFunc class for communication,
 * matching the structure of the working terminal monitor.
 **********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <unistd.h>
#include <string.h>
// #include <thread>        // No longer using std::thread for comms
#include <mutex>   // For thread-safe data access
#include <chrono>  // For time calculations
#include <vector>  // For plot history
#include <numeric> // For std::accumulate
#include <fstream> // For CSV logging
#include <iomanip> // For std::setprecision
#include <atomic>  // For std::atomic<bool>

// --- ImGui & Backend Headers ---
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

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

    // Plotting History
    std::vector<float> socHistory;
    std::vector<float> powerHistory;
    std::vector<float> voltageHistory;
    std::vector<float> currentHistory;
};

class RobotMonitor
{
public:
    RobotMonitor() : safe(LeggedType::Go1),
                     udp(HIGHLEVEL, 8090, "192.168.12.1", 8082),
                     running(false),
                     dt(0.002), // 500Hz, matching monitor.cpp
                     loop_count(0),
                     loop_udpSend(nullptr),
                     loop_udpRecv(nullptr),
                     loop_control(nullptr)
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
            // reset all members manually
            data.bms = {}; // Zero-initialize the BMS struct
            data.totalVoltage_V = 0.0f;
            data.current_A = 0.0f;
            data.power_W = 0.0f;
            data.totalEnergy_Wh = 0.0;
            data.averagePower_W = 0.0;
            data.runTime_s = 0.0;
            data.socHistory.clear();
            data.powerHistory.clear();
            data.voltageHistory.clear();
            data.currentHistory.clear();
        }

        // Open log file and write header
        logFile.open("go1_bms_log.csv", std::ios::out | std::ios::trunc);
        if (logFile.is_open())
        {
            logFile << "Timestamp(ms),Runtime(s),SOC(%),Voltage(V),Current(A),Power(W),TotalEnergy(Wh),";
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

        // --- Use LoopFunc, just like monitor.cpp ---
        loop_udpSend = new LoopFunc("udp_send", dt, 3, boost::bind(&RobotMonitor::UDPSend, this));
        loop_udpRecv = new LoopFunc("udp_recv", dt, 3, boost::bind(&RobotMonitor::UDPRecv, this));
        loop_control = new LoopFunc("control_loop", dt, boost::bind(&RobotMonitor::RunMonitorLoop, this));

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
        if (loop_udpSend)
        {
            delete loop_udpSend;
            loop_udpSend = nullptr;
        }
        if (loop_udpRecv)
        {
            delete loop_udpRecv;
            loop_udpRecv = nullptr;
        }
        if (loop_control)
        {
            delete loop_control;
            loop_control = nullptr;
        }

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

    // --- LoopFunc Callbacks (from monitor.cpp) ---

    void UDPRecv()
    {
        udp.Recv();
    }

    void UDPSend()
    {
        // We must send commands to keep the connection alive
        cmd.mode = 0;
        udp.Send();
    }

    // This is the main "control" loop, run by LoopFunc
    void RunMonitorLoop()
    {
        if (!running)
            return; // Check if we should stop

        udp.GetRecv(state);
        auto now = std::chrono::high_resolution_clock::now();
        double dt_actual = std::chrono::duration<double>(now - lastUpdateTime).count();

        if (dt_actual < 0.001)
        { // Avoid spikes if dt is too small
            return;
        }

        lastUpdateTime = now;

        // --- Calculations ---
        float voltage = std::accumulate(state.bms.cell_vol.begin(), state.bms.cell_vol.end(), 0) / 1000.0f; // mV to V
        float current = state.bms.current / 1000.0f;                                                        // mA to A
        float power = voltage * current;

        double runTime = std::chrono::duration<double>(now - startTime).count();

        // Local copy for accurate energy calculation
        double currentTotalEnergy_Wh = 0.0;
        {
            std::lock_guard<std::mutex> lock(dataMutex);
            currentTotalEnergy_Wh = data.totalEnergy_Wh;
        }

        double energy_Ws = power * dt_actual;                                 // Energy in Watt-seconds (Joules)
        double totalEnergy_Ws = (currentTotalEnergy_Wh * 3600.0) + energy_Ws; // Convert Wh back to Ws to add

        // --- Lock and Update Data ---
        {
            std::lock_guard<std::mutex> lock(dataMutex);

            data.bms = state.bms;
            data.totalVoltage_V = voltage;
            data.current_A = current;
            data.power_W = power;
            data.runTime_s = runTime;

            // Update totals
            data.totalEnergy_Wh = totalEnergy_Ws / 3600.0; // Convert back to Wh
            data.averagePower_W = (runTime > 0) ? (totalEnergy_Ws / runTime) : 0.0;

            // Update plot history (limit to 300 samples)
            limit_data_vector(data.socHistory, 300);
            limit_data_vector(data.powerHistory, 300);
            limit_data_vector(data.voltageHistory, 300);
            limit_data_vector(data.currentHistory, 300);

            data.socHistory.push_back(state.bms.SOC);
            data.powerHistory.push_back(power);
            data.voltageHistory.push_back(voltage);
            data.currentHistory.push_back(current);
        }

        loop_count++;
        // Log to file periodically (e.g., 10Hz)
        // 500Hz / 50 = 10Hz
        if (loop_count % 50 == 0 && logFile.is_open())
        {
            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            logFile << timestamp << ","
                    << std::fixed << std::setprecision(3) << runTime << ","
                    << (int)state.bms.SOC << ","
                    << voltage << ","
                    << current << ","
                    << power << ","
                    << std::setprecision(6) << data.totalEnergy_Wh << ",";

            for (int i = 0; i < 10; ++i)
            {
                logFile << state.bms.cell_vol[i] << (i == 9 ? "\n" : ",");
            }
        }

        // SetSend in the control loop, just like monitor.cpp
        udp.SetSend(cmd);
    }
    // ---------------------------------------------

    // Public method to get a copy of the data safely
    MonitorData GetDisplayData()
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        return data; // This works, as MonitorData is copyable
    }

private:
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    float dt;
    long long loop_count;

    std::atomic<bool> running;

    MonitorData data;
    std::ofstream logFile;
    std::mutex dataMutex;

    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::high_resolution_clock::time_point lastUpdateTime;

    // SDK LoopFunc objects
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
    // --- 1. Initialize Robot Monitor ---
    std::cout << "Initializing Robot Monitor..." << std::endl;
    RobotMonitor monitor;

    // --- 2. Initialize GLFW Window ---
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
    glfwSwapInterval(1); // Enable vsync

    // --- 3. Initialize ImGui ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // --- 4. Main GUI Loop ---
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Get a thread-safe copy of the data
        MonitorData displayData = monitor.GetDisplayData();

        // Set the main window to fill the whole screen
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        ImGui::Text("UNITREE GO1 POWER MONITOR");
        ImGui::Separator();

        // --- START/STOP BUTTONS ---
        if (monitor.IsRunning())
        {
            // Make button red when running
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
            // Make button green when stopped
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
        { // Give a bit more time to connect
            ImGui::Text("Connecting to robot... (Runtime: %.2f s)", displayData.runTime_s);
        }
        else
        {
            // --- Stats Window ---
            ImGui::BeginChild("Stats", ImVec2(ImGui::GetContentRegionAvail().x * 0.4f, 0), true);
            ImGui::Text("STATUS (Runtime: %.2f s)", displayData.runTime_s);
            ImGui::Separator();
            ImGui::Text("SOC:         %d %%", (int)displayData.bms.SOC);
            ImGui::Text("Voltage:     %.2f V", displayData.totalVoltage_V);
            ImGui::Text("Current:     %.2f A", displayData.current_A);
            ImGui::Text("Power:       %.2f W", displayData.power_W);
            ImGui::Separator();
            ImGui::Text("Total Usage: %.4f Wh", displayData.totalEnergy_Wh);
            ImGui::Text("Avg. Power:  %.2f W", displayData.averagePower_W);
            ImGui::Text("Cycles:      %d", displayData.bms.cycle);
            ImGui::Separator();
            ImGui::Text("LOGGING: Writing to 'go1_bms_log.csv' at ~10Hz.");
            ImGui::Text("This log contains SOC vs. mV data for all cells.");

            ImGui::Separator();
            ImGui::Text("Cell Voltages (mV):");
            ImGui::BeginChild("Cells", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), false, ImGuiWindowFlags_HorizontalScrollbar);
            for (int i = 0; i < 10; i++)
            {
                ImGui::Text("Cell %2d: %d mV", i + 1, displayData.bms.cell_vol[i]);
            }
            ImGui::EndChild();
            ImGui::EndChild(); // End Stats

            ImGui::SameLine();

            // --- Plots Window ---
            ImGui::BeginChild("Plots", ImVec2(0, 0), true);
            ImGui::Text("REAL-TIME PLOTS (Last 300 samples)");

            if (!displayData.socHistory.empty())
            {
                // 1. SOC, Voltage
                ImGui::PlotLines("SOC (%)", displayData.socHistory.data(), displayData.socHistory.size(), 0, NULL, 0.0f, 100.0f, ImVec2(0, 100));
                ImGui::PlotLines("Voltage (V)", displayData.voltageHistory.data(), displayData.voltageHistory.size(), 0, NULL, 18.0f, 26.0f, ImVec2(0, 100));

                // --- Create temporary, inverted data for plotting ---
                std::vector<float> absPowerHistory(displayData.powerHistory.size());
                std::vector<float> absCurrentHistory(displayData.currentHistory.size());

                // Invert the signs (Usage/Discharge = Positive value)
                for (size_t i = 0; i < displayData.powerHistory.size(); ++i)
                {
                    absPowerHistory[i] = -displayData.powerHistory[i]; // E.g., -50W becomes 50W
                }
                for (size_t i = 0; i < displayData.currentHistory.size(); ++i)
                {
                    absCurrentHistory[i] = -displayData.currentHistory[i]; // E.g., -5A becomes 5A
                }

                // 2. Plot Inverted Data
                ImGui::PlotLines("Power (W) - Usage", absPowerHistory.data(), absPowerHistory.size(), 0, NULL, 0.0f, 500.0f, ImVec2(0, 100));      // Min Y is now 0.0f
                ImGui::PlotLines("Current (A) - Usage", absCurrentHistory.data(), absCurrentHistory.size(), 0, NULL, 0.0f, 20.0f, ImVec2(0, 100)); // Min Y is now 0.0f
            }

            ImGui::EndChild(); // End Plots
        }

        ImGui::End(); // End Main

        // --- 5. Rendering ---
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // --- 6. Cleanup ---
    std::cout << "Stopping robot monitor thread..." << std::endl;
    monitor.StopMonitoring();
    std::cout << "Cleaning up GUI..." << std::endl;

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout << "Done." << std::endl;

    return 0;
}