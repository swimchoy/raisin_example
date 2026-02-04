/**
 * @file example_odometry.cpp
 * @brief Monitor robot position via subscribeOdometry()
 *
 * Essential: state.x, y, z, yaw, vx, vy, omega
 */

#include <iostream>
#include <iomanip>
#include <thread>
#include <csignal>
#include <atomic>
#include "raisin_sdk/raisin_client.hpp"

std::atomic<bool> running{true};

void signalHandler(int) {
    running = false;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <robot_id>" << std::endl;
        std::cout << "Example: " << argv[0] << " 10.42.0.1" << std::endl;
        return 1;
    }

    std::signal(SIGINT, signalHandler);

    std::string robot_id = argv[1];
    raisin_sdk::RaisinClient client("odometry_example");

    std::cout << "Connecting to robot: " << robot_id << std::endl;
    if (!client.connect(robot_id)) {
        std::cerr << "Connection failed" << std::endl;
        return 1;
    }
    std::cout << "Connected!" << std::endl;

    // ===== ESSENTIAL =====
    client.subscribeOdometry([](const raisin_sdk::RobotState& state) {
        std::cout << "\r" << std::fixed << std::setprecision(3)
                  << "Position: (" << state.x << ", " << state.y << ") "
                  << "Yaw: " << (state.yaw * 180.0 / M_PI) << "deg "
                  << "Vel: (" << state.vx << ", " << state.vy << ") m/s        " << std::flush;
    });
    // ==================

    std::cout << "Monitoring odometry... (Ctrl+C to stop)" << std::endl;
    std::cout << std::endl;

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl << "Shutting down..." << std::endl;
    return 0;
}
