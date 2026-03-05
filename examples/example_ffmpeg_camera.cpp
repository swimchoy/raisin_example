/**
 * @file example_ffmpeg_camera.cpp
 * @brief Subscribe to robot camera stream topic (`raisin_interfaces::msg::FfmpegPacket`)
 *
 * Essential:
 * - connect to robot
 * - list FfmpegPacket topics
 * - subscribe to selected stream
 * - print packet metadata on each message
 *
 * Note: FfmpegPacket carries encoded video data and must be decoded for visualization.
 */

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include "raisin_interfaces/msg/ffmpeg_packet.hpp"
#include "raisin_network/node.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <robot_id>" << std::endl;
        return 1;
    }

    auto robotId = std::string(argv[1]);
    std::vector<std::vector<std::string>> threads = {{"main"}};
    auto network = std::make_shared<raisin::Network>("ffmpeg_camera_example", "external_sdk", threads);

    std::shared_ptr<raisin::Remote::Connection> connection;
    const auto connectStart = std::chrono::steady_clock::now();
    while (!connection) {
        if (std::chrono::steady_clock::now() - connectStart > std::chrono::seconds(3)) {
            std::cerr << "\nFailed to connect to robot: " << robotId << " (timeout)" << std::endl;
            return 1;
        }
        connection = network->connect(robotId);
        if (!connection) {
            std::cout << "\rConnecting to " << robotId << "... (retrying)" << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }


    auto getTopics = [connection]() {
        std::vector<std::string> list;
        std::scoped_lock lock(connection->mutex);
        for (const auto& [name, info] : connection->publishers) {
            if (info.dataType.find("raisin_interfaces::msg::FfmpegPacket") != std::string::npos ||
                info.dataType.find("FfmpegPacket") != std::string::npos) {
                list.emplace_back(name);
            }
        }
        return list;
    };

    auto topics = getTopics();
    if (topics.empty()) {
        std::cerr << "No FfmpegPacket topics found on " << robotId << std::endl;
        return 1;
    }

    std::cout << "Available FfmpegPacket streams:" << std::endl;
    for (size_t i = 0; i < topics.size(); ++i) {
        std::cout << "  " << (i + 1) << ") " << topics[i] << std::endl;
    }

    std::size_t choice = 0;
    while (choice == 0) {
        std::cout << "Select stream [1-" << topics.size() << "]: ";
        if (!(std::cin >> choice)) {
            std::cerr << "Invalid input." << std::endl;
            return 1;
        }
        if (choice < 1 || choice > topics.size()) {
            std::cerr << "Out of range. Exiting." << std::endl;
            return 1;
        }
    }

    const std::string topic = topics[choice - 1];
    size_t packetCount = 0;
    const auto topicName = topic;
    auto node = std::make_shared<raisin::Node>(network);
    auto subscriber = node->createSubscriber<raisin::raisin_interfaces::msg::FfmpegPacket>(
        topic, connection,
        [topicName, &packetCount](const raisin::raisin_interfaces::msg::FfmpegPacket::ConstSharedPtr& msg) {
            ++packetCount;
            std::cout << "\r[" << topicName << "] packet "
                      << packetCount << " | "
                      << msg->data.size() << " B | "
                      << msg->encoding << " | "
                      << msg->width << "x" << msg->height
                      << std::flush;
        });

    if (!subscriber) {
        std::cerr << "Failed to subscribe: " << topic << std::endl;
        node->cleanupResources();
        return 1;
    }

    std::cout << "Subscribed " << topic << ". Press Enter to stop." << std::endl;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::string stop;
    std::getline(std::cin, stop);

    std::cout << std::endl << "Shutting down..." << std::endl;
    node->cleanupResources();
    return 0;
}
