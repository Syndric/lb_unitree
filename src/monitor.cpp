/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <iomanip> // Used for formatting the output
#include <unistd.h>
#include <string.h>

using namespace UNITREE_LEGGED_SDK;

class RobotMonitor
{
public:
    RobotMonitor() : safe(LeggedType::Go1),
                     udp(HIGHLEVEL, 8090, "192.168.12.1", 8082)
    {
        udp.InitCmdData(cmd);
    }

    void UDPRecv();
    void UDPSend();
    void MonitorData();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    float dt = 0.002; 
    long long loop_count = 0;
};

void RobotMonitor::UDPRecv()
{
    udp.Recv();
}

void RobotMonitor::UDPSend()
{
    // We must send commands to keep the connection alive, 
    // even if we are just monitoring.
    // Mode 0 is idle/default stand, which is safe for monitoring.
    cmd.mode = 0; 
    udp.Send();
}

void RobotMonitor::MonitorData()
{
    udp.GetRecv(state);
    loop_count++;

    // Only print every ~200ms to make the terminal readable
    // dt is 0.002, so 100 cycles * 0.002 = 0.2 seconds
    if (loop_count % 100 == 0)
    {
        // Clear screen (ANSI escape code) for a dashboard effect
        std::cout << "\033[2J\033[1;1H"; 
        
        std::cout << "=============================================" << std::endl;
        std::cout << "           UNITREE GO1 MONITOR               " << std::endl;
        std::cout << "=============================================" << std::endl;

        // --- Battery Information ---
        std::cout << "[BATTERY STATUS]" << std::endl;
        std::cout << "SOC:     " << (int)state.bms.SOC << " %" << std::endl;
        std::cout << "Current: " << state.bms.current << " mA" << std::endl;
        std::cout << "Cycles:  " << state.bms.cycle << std::endl;
        
        // Calculate total voltage roughly by summing first few cells or just checking logic
        // Usually specific voltage is sum of cells, but here we list the first 4 for brevity
        std::cout << "Cells (mV): " 
                  << state.bms.cell_vol[0] << ", " 
                  << state.bms.cell_vol[1] << ", " 
                  << state.bms.cell_vol[2] << "..." << std::endl; 
        
        std::cout << "---------------------------------------------" << std::endl;

        // --- Motor Torque Information ---
        // Go1 has 12 motors: FR(0-2), FL(3-5), RR(6-8), RL(9-11)
        // 0: Hip, 1: Thigh, 2: Calf
        
        std::cout << "[MOTOR TORQUE (N.m)]" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "      |   FR   |   FL   |   RR   |   RL   |" << std::endl;
        std::cout << "------+--------+--------+--------+--------+" << std::endl;
        
        std::cout << "Hip   | " 
                  << std::setw(6) << state.motorState[FR_0].tauEst << " | " 
                  << std::setw(6) << state.motorState[FL_0].tauEst << " | " 
                  << std::setw(6) << state.motorState[RR_0].tauEst << " | " 
                  << std::setw(6) << state.motorState[RL_0].tauEst << " |" << std::endl;

        std::cout << "Thigh | " 
                  << std::setw(6) << state.motorState[FR_1].tauEst << " | " 
                  << std::setw(6) << state.motorState[FL_1].tauEst << " | " 
                  << std::setw(6) << state.motorState[RR_1].tauEst << " | " 
                  << std::setw(6) << state.motorState[RL_1].tauEst << " |" << std::endl;

        std::cout << "Calf  | " 
                  << std::setw(6) << state.motorState[FR_2].tauEst << " | " 
                  << std::setw(6) << state.motorState[FL_2].tauEst << " | " 
                  << std::setw(6) << state.motorState[RR_2].tauEst << " | " 
                  << std::setw(6) << state.motorState[RL_2].tauEst << " |" << std::endl;
        
        std::cout << "---------------------------------------------" << std::endl;
        
        // --- IMU Quick Check ---
        std::cout << "IMU (r,p,y): " 
                  << state.imu.rpy[0] << ", " 
                  << state.imu.rpy[1] << ", " 
                  << state.imu.rpy[2] << std::endl;
    }

    udp.SetSend(cmd);
}

int main(void)
{
    std::cout << "Starting High-Level Monitor..." << std::endl
              << "Press Ctrl+C to exit." << std::endl;

    RobotMonitor monitor;

    LoopFunc loop_control("control_loop", monitor.dt,    boost::bind(&RobotMonitor::MonitorData, &monitor));
    LoopFunc loop_udpSend("udp_send",     monitor.dt, 3, boost::bind(&RobotMonitor::UDPSend,     &monitor));
    LoopFunc loop_udpRecv("udp_recv",     monitor.dt, 3, boost::bind(&RobotMonitor::UDPRecv,     &monitor));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    };

    return 0;
}