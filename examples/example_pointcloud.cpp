/**
 * @file example_pointcloud.cpp
 * @brief Subscribe to LiDAR pointcloud via subscribePointCloud()
 *
 * Essential: points[].x, y, z, intensity
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
    raisin_sdk::RaisinClient client("pointcloud_example");

    std::cout << "Connecting to robot: " << robot_id << std::endl;
    if (!client.connect(robot_id)) {
        std::cerr << "Connection failed" << std::endl;
        return 1;
    }
    std::cout << "Connected!" << std::endl;

    // ===== ESSENTIAL =====
    client.subscribePointCloud([](const std::vector<raisin_sdk::Point3D>& points) {
        std::cout << "\rPointCloud: " << points.size() << " points    " << std::flush;
    });
    // ==================

    std::cout << "Monitoring pointcloud... (Ctrl+C to stop)" << std::endl;
    std::cout << std::endl;

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << std::endl << "Shutting down..." << std::endl;
    return 0;
}
