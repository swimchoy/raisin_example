/**
 * @file raisin_client.hpp
 * @brief Convenience wrapper for Raisin Autonomy service clients
 *
 * This header provides a simple interface for controlling Raisin robot
 * autonomous navigation without dealing with low-level network details.
 */

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <chrono>
#include <iostream>
#include <cmath>
#include <mutex>
#include <atomic>
#include <algorithm>

#ifndef _WIN32
#include <ifaddrs.h>
#include <net/if.h>
#endif

#include "raisin_network/raisin.hpp"
#include "raisin_network/network.hpp"
#include "raisin_network/node.hpp"
#include "raisin_interfaces/srv/set_waypoints.hpp"
#include "raisin_interfaces/srv/get_waypoints.hpp"
#include "raisin_interfaces/srv/set_laser_map.hpp"
#include "raisin_interfaces/srv/string.hpp"
#include "raisin_interfaces/srv/resume_patrol.hpp"
#include "raisin_interfaces/srv/load_waypoints_file.hpp"
#include "raisin_interfaces/srv/save_waypoints_file.hpp"
#include "raisin_interfaces/srv/list_files.hpp"
#include "raisin_interfaces/srv/save_graph_file.hpp"
#include "raisin_interfaces/srv/load_graph_file.hpp"
#include "raisin_interfaces/srv/load_laser_map.hpp"
#include "raisin_interfaces/srv/refine_waypoints.hpp"
#include "raisin_interfaces/msg/waypoint.hpp"
#include "raisin_interfaces/msg/graph_node.hpp"
#include "raisin_interfaces/msg/graph_edge.hpp"
#include "raisin_interfaces/msg/robot_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace raisin_sdk {

/**
 * @brief CiA402 Status Word values from motor driver
 * These values indicate the motor driver state, not error codes.
 */
enum class CiA402StatusWord : uint16_t {
    NOT_READY_TO_SWITCH_ON = 0,   ///< Initial state - driver not ready (**error**)
    FAULT = 8,                     ///< Fault detected (**error**)
    READY_TO_SWITCH_ON = 33,       ///< Driver ready (normal state before enable)
    SWITCHED_ON = 35,              ///< Driver switched on (normal)
    OPERATION_ENABLED = 39,        ///< Motor operational (normal running state)
    ECAT_CONN_ERROR = 99           ///< EtherCAT connection error (**error**)
};

/**
 * @brief Check if status word indicates an error
 * Error states: NOT_READY(0), FAULT(8), ECAT_ERROR(99)
 * Normal states: READY(33), SWITCHED_ON(35), ENABLED(39)
 */
inline bool isActuatorStatusError(uint16_t status) {
    return status == static_cast<uint16_t>(CiA402StatusWord::FAULT) ||
           status == static_cast<uint16_t>(CiA402StatusWord::ECAT_CONN_ERROR) ||
           status == static_cast<uint16_t>(CiA402StatusWord::NOT_READY_TO_SWITCH_ON);
}

/**
 * @brief Get human-readable name for status word
 */
inline std::string getActuatorStatusName(uint16_t status) {
    switch (status) {
        case 0:  return "NOT_READY";
        case 8:  return "FAULT";
        case 33: return "READY";
        case 35: return "SWITCHED_ON";
        case 39: return "ENABLED";
        case 99: return "ECAT_ERROR";
        default: return "UNKNOWN(" + std::to_string(status) + ")";
    }
}

namespace detail {

/**
 * @brief Get available network interfaces including loopback
 * @return Vector of interface names
 */
inline std::vector<std::string> getNetworkInterfaces() {
    std::vector<std::string> interfaces;

#ifndef _WIN32
    struct ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) != 0) {
        interfaces.push_back("lo");
        return interfaces;
    }

    for (auto* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_name || !(ifa->ifa_flags & IFF_UP)) {
            continue;
        }
        if (!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }
        const std::string name = ifa->ifa_name;
        if (name.rfind("docker", 0) == 0 || name.rfind("veth", 0) == 0 ||
            name.rfind("br-", 0) == 0 || name.rfind("virbr", 0) == 0) {
            continue;
        }
        if (std::find(interfaces.begin(), interfaces.end(), name) == interfaces.end()) {
            interfaces.push_back(name);
        }
    }
    freeifaddrs(ifaddr);

    auto loIt = std::find(interfaces.begin(), interfaces.end(), "lo");
    if (loIt != interfaces.end() && loIt != interfaces.begin()) {
        interfaces.erase(loIt);
        interfaces.insert(interfaces.begin(), "lo");
    } else if (loIt == interfaces.end()) {
        interfaces.insert(interfaces.begin(), "lo");
    }
#else
    interfaces.push_back("lo");
#endif

    return interfaces;
}

}  // namespace detail

/**
 * @brief Waypoint structure for easy manipulation
 */
struct Waypoint {
    std::string frame = "map";  ///< Coordinate frame: "map", "gps", "odom"
    double x = 0.0;             ///< X coordinate (or latitude for GPS)
    double y = 0.0;             ///< Y coordinate (or longitude for GPS)
    double z = 0.0;             ///< Z coordinate (or altitude for GPS)
    bool use_z = false;         ///< Whether to check Z when reaching waypoint

    Waypoint() = default;
    Waypoint(const std::string& f, double x_, double y_, double z_ = 0.0, bool uz = false)
        : frame(f), x(x_), y(y_), z(z_), use_z(uz) {}

    /// Create GPS waypoint from latitude/longitude
    static Waypoint GPS(double latitude, double longitude, double altitude = 0.0) {
        return Waypoint("gps", latitude, longitude, altitude, false);
    }

    /// Create local map waypoint
    static Waypoint Map(double x, double y, double z = 0.0) {
        return Waypoint("map", x, y, z, false);
    }
};

/**
 * @brief Mission status information
 */
struct MissionStatus {
    std::vector<Waypoint> waypoints;  ///< Current waypoint list
    uint8_t current_index = 0;        ///< Index of waypoint being navigated to
    uint8_t repetition = 0;           ///< Remaining patrol laps (ignored if infinite_loop is true)
    bool infinite_loop = false;       ///< Whether patrol repeats indefinitely
    bool valid = false;               ///< Whether status was retrieved successfully
};

/**
 * @brief Result of a service call
 */
struct ServiceResult {
    bool success = false;
    std::string message;
};

/**
 * @brief Result of resume patrol operation
 */
struct ResumePatrolResult {
    bool success = false;
    std::string message;
    uint8_t waypoint_index = 0;   ///< Index of nearest waypoint to resume from
};

/**
 * @brief Result of list files operation
 */
struct ListFilesResult {
    bool success = false;
    std::string message;
    std::vector<std::string> files;  ///< List of available files
};

/**
 * @brief Graph node for path planning
 */
struct GraphNode {
    int32_t id = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief Graph edge for path planning
 */
struct GraphEdge {
    int32_t from_node = 0;
    int32_t to_node = 0;
    double cost = 0.0;
};

/**
 * @brief Result of load graph operation
 */
struct LoadGraphResult {
    bool success = false;
    std::string message;
    std::vector<GraphNode> nodes;
    std::vector<GraphEdge> edges;
};

/**
 * @brief Result of load map operation (includes graph and paths)
 */
struct LoadMapResult {
    bool success = false;
    std::string message;
    std::string mapName;                    ///< Loaded map name
    std::vector<GraphNode> graphNodes;      ///< Auto-loaded graph nodes
    std::vector<GraphEdge> graphEdges;      ///< Auto-loaded graph edges
    std::vector<Waypoint> waypoints;        ///< Auto-loaded default route (route_1)
    std::vector<std::string> availableRoutes; ///< List of available routes
};

/**
 * @brief Result of refine waypoints operation
 */
struct RefineWaypointsResult {
    bool success = false;
    std::string message;
    std::vector<Waypoint> refined_waypoints;  ///< Refined waypoints following graph
    std::vector<int32_t> path_node_ids;       ///< Node IDs forming the path
};

/**
 * @brief Robot state from odometry (map frame)
 */
struct RobotState {
    double x = 0.0;        ///< X position in map frame
    double y = 0.0;        ///< Y position in map frame
    double z = 0.0;        ///< Z position in map frame
    double yaw = 0.0;      ///< Yaw angle in radians
    double vx = 0.0;       ///< Linear velocity X
    double vy = 0.0;       ///< Linear velocity Y
    double omega = 0.0;    ///< Angular velocity
    bool valid = false;    ///< True when odometry is received
};

/**
 * @brief Actuator (motor) information
 */
struct ActuatorInfo {
    std::string name;           ///< Motor name (e.g., "FR_hip", "FL_thigh")
    uint16_t status = 0;        ///< CiA402 status word (see CiA402StatusWord enum)
    double temperature = 0.0;   ///< Motor temperature in Celsius
    double position = 0.0;      ///< Joint position in radians
    double velocity = 0.0;      ///< Joint velocity in rad/s
    double effort = 0.0;        ///< Joint torque in Nm
};

/**
 * @brief Locomotion state enum values
 */
enum class LocomotionState : int32_t {
    COMM_DISABLED = 0,      ///< Communication disabled
    COMM_ENABLED = 1,       ///< Communication enabled
    MOTOR_READY = 2,        ///< Motors ready
    MOTOR_COMMUTATION = 3,  ///< Motor commutation in progress
    MOTOR_ENABLED = 4,      ///< Motors enabled
    IN_TEST_MODE = 5,       ///< Test mode active
    STANDING_MODE = 6,      ///< Robot is standing
    IN_CONTROL = 7,         ///< Robot is under control (walking)
    SITDOWN_MODE = 8,       ///< Robot is sitting down
    MOTOR_DISABLED = 9      ///< Motors disabled
};

/**
 * @brief Joy listen source type
 */
enum class JoySourceType : int32_t {
    JOY = 0,           ///< Manual joystick control
    VEL_CMD = 1,       ///< Autonomous velocity command
    NUM_SOURCES = 2    ///< No source / disabled
};

/**
 * @brief Extended robot state with full information from robot_state topic
 */
struct ExtendedRobotState {
    // Position and velocity
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;

    // Locomotion state
    int32_t locomotion_state = 0;   ///< See LocomotionState enum (0-9)

    // Battery information
    double voltage = 0.0;           ///< Current battery voltage
    double current = 0.0;           ///< Current draw in Amps
    double max_voltage = 0.0;       ///< Maximum voltage
    double min_voltage = 0.0;       ///< Minimum voltage

    // Temperature
    double body_temperature = 0.0;  ///< Body temperature in Celsius

    // Joy control state
    int32_t joy_listen_type = 2;    ///< See JoySourceType enum

    // Actuator states
    std::vector<ActuatorInfo> actuators;

    bool valid = false;

    /// Get locomotion state as string
    std::string getLocomotionStateName() const {
        static const std::vector<std::string> names = {
            "COMM_DISABLED", "COMM_ENABLED", "MOTOR_READY",
            "MOTOR_COMMUTATION", "MOTOR_ENABLED", "IN_TEST_MODE",
            "STANDING_MODE", "IN_CONTROL", "SITDOWN_MODE", "MOTOR_DISABLED"
        };
        if (locomotion_state >= 0 && locomotion_state < static_cast<int32_t>(names.size())) {
            return names[locomotion_state];
        }
        return "UNKNOWN";
    }

    /// Get joy source type as string
    std::string getJoySourceName() const {
        switch (static_cast<JoySourceType>(joy_listen_type)) {
            case JoySourceType::JOY: return "JOY (Manual)";
            case JoySourceType::VEL_CMD: return "VEL_CMD (Autonomous)";
            default: return "NONE";
        }
    }

    /// Check if robot is standing or walking
    bool isOperational() const {
        return locomotion_state == static_cast<int32_t>(LocomotionState::STANDING_MODE) ||
               locomotion_state == static_cast<int32_t>(LocomotionState::IN_CONTROL);
    }

    /// Check if any actuator has an error
    bool hasActuatorError() const {
        for (const auto& act : actuators) {
            if (isActuatorStatusError(act.status)) return true;
        }
        return false;
    }

    /// Get list of actuators with errors
    std::vector<std::string> getActuatorsWithErrors() const {
        std::vector<std::string> errorList;
        for (const auto& act : actuators) {
            if (isActuatorStatusError(act.status)) {
                errorList.push_back(act.name + " (" + getActuatorStatusName(act.status) + ")");
            }
        }
        return errorList;
    }

    /// Check if all actuators are in operational state
    bool allActuatorsOperational() const {
        for (const auto& act : actuators) {
            if (act.status != static_cast<uint16_t>(CiA402StatusWord::OPERATION_ENABLED)) {
                return false;
            }
        }
        return true;
    }
};

/**
 * @brief Point for simple point cloud representation
 */
struct Point3D {
    float x, y, z;
};

// Callback types
using OdometryCallback = std::function<void(const RobotState&)>;
using PointCloudCallback = std::function<void(const std::vector<Point3D>&)>;
using ExtendedRobotStateCallback = std::function<void(const ExtendedRobotState&)>;

/**
 * @brief High-level client for controlling Raisin robot autonomy
 */
class RaisinClient {
public:
    /**
     * @brief Constructor - initializes raisin_network
     * @param client_id Unique identifier for this client
     */
    explicit RaisinClient(const std::string& client_id = "raisin_client")
        : client_id_(client_id), connected_(false) {

        raisin::raisinInit();
        interfaces_ = detail::getNetworkInterfaces();
    }

    ~RaisinClient() {
        disconnect();
    }

    /**
     * @brief Connect to a Raisin robot
     * @param robot_id Robot identifier or IP address
     * @param timeout_sec Connection timeout in seconds
     * @param cancel_token Optional atomic flag to cancel connection attempt
     * @return true if connection successful
     */
    bool connect(const std::string& robot_id, int timeout_sec = 10,
                 std::atomic<bool>* cancel_token = nullptr) {
        std::vector<std::vector<std::string>> threads = {{"main"}};

        auto isCancelled = [cancel_token]() {
            return cancel_token && !cancel_token->load();
        };

        std::vector<std::string> emptyInterfaces;
        network_ = std::make_shared<raisin::Network>(client_id_, "external_sdk", threads, emptyInterfaces);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (isCancelled()) {
            network_->shutdown();
            network_.reset();
            return false;
        }

        auto start_time = std::chrono::steady_clock::now();
        while (!isCancelled()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed >= timeout_sec) {
                break;
            }

            try {
                connection_ = network_->connect(robot_id);
                if (connection_) {
                    break;
                }
            } catch (...) {
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        if (!connection_) {
            std::cerr << "[RaisinClient] Failed to connect to: " << robot_id << std::endl;
            network_->shutdown();
            network_.reset();
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (isCancelled()) {
            network_->shutdown();
            network_.reset();
            connection_.reset();
            return false;
        }

        node_ = std::make_unique<raisin::Node>(network_);
        connected_ = true;
        robotId_ = robot_id;
        std::cout << "[RaisinClient] Connected to robot: " << robot_id << std::endl;
        return true;
    }

    /**
     * @brief Disconnect from robot
     */
    void disconnect() {
        connected_ = false;

        odomSubscriber_.reset();
        cloudSubscriber_.reset();
        robotStateSubscriber_.reset();

        setWaypointsClient_.reset();
        getWaypointsClient_.reset();
        setMapClient_.reset();
        setJoyListenClient_.reset();
        standUpClient_.reset();
        sitDownClient_.reset();
        listWaypointsFilesClient_.reset();
        loadWaypointsFileClient_.reset();
        resumePatrolClient_.reset();

        if (node_) {
            node_->cleanupResources();
            node_.reset();
        }

        connection_.reset();

        if (network_) {
            try {
                network_->shutdown();
            } catch (...) {
            }
            network_.reset();
        }
    }

    bool isConnected() const { return connected_; }

    // ========================================================================
    // Waypoint Navigation
    // ========================================================================

    /**
     * @brief Set waypoints for autonomous navigation
     * @param waypoints List of waypoints to navigate
     * @param repetition Number of times to traverse route (ignored if infinite_loop is true)
     * @param start_index Index of waypoint to start from
     * @param infinite_loop If true, patrol repeats indefinitely
     * @return Result of the operation
     */
    ServiceResult setWaypoints(const std::vector<Waypoint>& waypoints,
                                uint8_t repetition = 1,
                                uint8_t start_index = 0,
                                bool infinite_loop = false) {
        if (!connected_) {
            return {false, "Not connected to robot"};
        }

        ensureWaypointClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::SetWaypoints::Request>();

        for (const auto& wp : waypoints) {
            raisin::raisin_interfaces::msg::Waypoint msg;
            msg.frame = wp.frame;
            msg.x = wp.x;
            msg.y = wp.y;
            msg.z = wp.z;
            msg.use_z = wp.use_z;
            request->waypoints.push_back(msg);
        }

        request->repetition = repetition;
        request->current_index = start_index;
        request->infinite_loop = infinite_loop;

        ServiceResult result;
        auto future = setWaypointsClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Get current mission status (for arrival notification)
     * @return Mission status including waypoints and progress
     */
    MissionStatus getMissionStatus() {
        MissionStatus status;

        if (!connected_) {
            return status;
        }

        ensureWaypointClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::GetWaypoints::Request>();
        auto future = getWaypointsClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();

            if (response->success) {
                status.valid = true;
                status.current_index = response->current_index;
                status.repetition = response->repetition;
                status.infinite_loop = response->infinite_loop;

                for (const auto& wp : response->waypoints) {
                    Waypoint waypoint;
                    waypoint.frame = wp.frame;
                    waypoint.x = wp.x;
                    waypoint.y = wp.y;
                    waypoint.z = wp.z;
                    waypoint.use_z = wp.use_z;
                    status.waypoints.push_back(waypoint);
                }
            }
        }

        return status;
    }

    // ========================================================================
    // Locomotion Control (Stand Up / Sit Down)
    // ========================================================================

    /**
     * @brief Stand up the robot
     * @return Result of the operation
     */
    ServiceResult standUp() {
        if (!connected_) {
            return {false, "Not connected to robot"};
        }

        ensureLocomotionClients();

        auto request = std::make_shared<raisin::std_srvs::srv::Trigger::Request>();
        ServiceResult result;
        auto future = standUpClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Sit down the robot
     * @return Result of the operation
     */
    ServiceResult sitDown() {
        if (!connected_) {
            return {false, "Not connected to robot"};
        }

        ensureLocomotionClients();

        auto request = std::make_shared<raisin::std_srvs::srv::Trigger::Request>();
        ServiceResult result;
        auto future = sitDownClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    // ========================================================================
    // Patrol Route Management
    // ========================================================================

    /**
     * @brief List available waypoint files (saved patrol routes)
     * @param directory Optional directory path (empty for default)
     * @return List of available waypoint files
     */
    ListFilesResult listWaypointsFiles(const std::string& directory = "") {
        ListFilesResult result;

        if (!connected_) {
            result.success = false;
            result.message = "Not connected to robot";
            return result;
        }

        ensurePatrolClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::ListFiles::Request>();
        request->directory = directory;

        auto future = listWaypointsFilesClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
            result.files = response->files;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Load a saved waypoint file (patrol route)
     * @param name Name of the waypoint file to load
     * @return Result of the operation
     */
    ServiceResult loadWaypointsFile(const std::string& name) {
        if (!connected_) {
            return {false, "Not connected to robot"};
        }

        ensurePatrolClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::LoadWaypointsFile::Request>();
        request->name = name;

        ServiceResult result;
        auto future = loadWaypointsFileClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Resume patrol from nearest waypoint
     * Finds the closest waypoint to robot's current position
     * @return Result including the nearest waypoint index
     */
    ResumePatrolResult resumePatrol() {
        ResumePatrolResult result;

        if (!connected_) {
            result.success = false;
            result.message = "Not connected to robot";
            return result;
        }

        ensurePatrolClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::ResumePatrol::Request>();
        auto future = resumePatrolClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
            result.waypoint_index = response->waypoint_index;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Save current waypoints to a file on the robot
     * @param name Name of the waypoint file (without extension)
     * @return Result of the operation
     */
    ServiceResult saveWaypointsFile(const std::string& name) {
        if (!connected_) {
            return {false, "Not connected to robot"};
        }

        ensurePatrolClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::SaveWaypointsFile::Request>();
        request->name = name;

        ServiceResult result;
        auto future = saveWaypointsFileClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    // ========================================================================
    // Graph File Management
    // ========================================================================

    /**
     * @brief Save graph to a file on the robot
     * @param name Name of the graph file
     * @param nodes Vector of graph nodes
     * @param edges Vector of graph edges
     * @return Result of the operation
     */
    ServiceResult saveGraphFile(const std::string& name,
                                 const std::vector<GraphNode>& nodes,
                                 const std::vector<GraphEdge>& edges) {
        if (!connected_) {
            return {false, "Not connected to robot"};
        }

        ensureGraphClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::SaveGraphFile::Request>();
        request->name = name;

        // Convert SDK GraphNode to raisin GraphNode
        for (const auto& node : nodes) {
            raisin::raisin_interfaces::msg::GraphNode msg;
            msg.id = node.id;
            msg.x = node.x;
            msg.y = node.y;
            msg.z = node.z;
            request->nodes.push_back(msg);
        }

        // Convert SDK GraphEdge to raisin GraphEdge
        for (const auto& edge : edges) {
            raisin::raisin_interfaces::msg::GraphEdge msg;
            msg.from_node = edge.from_node;
            msg.to_node = edge.to_node;
            msg.cost = edge.cost;
            request->edges.push_back(msg);
        }

        ServiceResult result;
        auto future = saveGraphFileClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Refine waypoints using A* algorithm on the graph
     *
     * This method calls the robot's planning/refine_waypoints service to
     * generate a refined path through the graph between waypoints.
     *
     * @param waypoints Input waypoints to refine
     * @param nodes Graph nodes for A* pathfinding
     * @param edges Graph edges for A* pathfinding
     * @return RefineWaypointsResult containing refined waypoints and path node IDs
     */
    RefineWaypointsResult refineWaypoints(const std::vector<Waypoint>& waypoints,
                                           const std::vector<GraphNode>& nodes,
                                           const std::vector<GraphEdge>& edges) {
        RefineWaypointsResult result;

        if (!connected_) {
            result.success = false;
            result.message = "Not connected to robot";
            return result;
        }

        ensureRefineWaypointsClient();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::RefineWaypoints::Request>();

        // Convert waypoints
        for (const auto& wp : waypoints) {
            raisin::raisin_interfaces::msg::Waypoint msg;
            msg.frame = wp.frame;
            msg.x = wp.x;
            msg.y = wp.y;
            msg.z = wp.z;
            msg.use_z = wp.use_z;
            request->waypoints.push_back(msg);
        }

        // Convert graph nodes
        for (const auto& node : nodes) {
            raisin::raisin_interfaces::msg::GraphNode msg;
            msg.id = node.id;
            msg.x = node.x;
            msg.y = node.y;
            msg.z = node.z;
            request->nodes.push_back(msg);
        }

        // Convert graph edges
        for (const auto& edge : edges) {
            raisin::raisin_interfaces::msg::GraphEdge msg;
            msg.from_node = edge.from_node;
            msg.to_node = edge.to_node;
            msg.cost = edge.cost;
            request->edges.push_back(msg);
        }

        auto future = refineWaypointsClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;

            if (result.success) {
                // Convert refined waypoints
                for (const auto& wp : response->refined_waypoints) {
                    Waypoint waypoint;
                    waypoint.frame = wp.frame;
                    waypoint.x = wp.x;
                    waypoint.y = wp.y;
                    waypoint.z = wp.z;
                    waypoint.use_z = wp.use_z;
                    result.refined_waypoints.push_back(waypoint);
                }

                // Copy path node IDs
                result.path_node_ids = response->path_node_ids;
            }
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Load graph from a file on the robot
     * @param name Name of the graph file
     * @return Result containing nodes and edges
     */
    LoadGraphResult loadGraphFile(const std::string& name) {
        LoadGraphResult result;

        if (!connected_) {
            result.success = false;
            result.message = "Not connected to robot";
            return result;
        }

        ensureGraphClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::LoadGraphFile::Request>();
        request->name = name;

        auto future = loadGraphFileClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;

            if (result.success) {
                // Convert raisin GraphNode to SDK GraphNode
                for (const auto& msg : response->nodes) {
                    GraphNode node;
                    node.id = msg.id;
                    node.x = msg.x;
                    node.y = msg.y;
                    node.z = msg.z;
                    result.nodes.push_back(node);
                }

                // Convert raisin GraphEdge to SDK GraphEdge
                for (const auto& msg : response->edges) {
                    GraphEdge edge;
                    edge.from_node = msg.from_node;
                    edge.to_node = msg.to_node;
                    edge.cost = msg.cost;
                    result.edges.push_back(edge);
                }
            }
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    // ========================================================================
    // Map Loading (from robot storage)
    // ========================================================================

    /**
     * @brief List available map files on the robot
     * Maps are stored in log/map/ directory on the robot
     * @return List of available map names
     */
    ListFilesResult listMapFiles() {
        ListFilesResult result;

        if (!connected_) {
            result.success = false;
            result.message = "Not connected to robot";
            return result;
        }

        ensureMapLoadClients();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::ListFiles::Request>();
        request->directory = "";  // Default map directory

        auto future = listMapFilesClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
            result.files = response->files;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Load a map from robot storage
     *
     * This method loads a map that was previously saved on the robot via SLAM.
     * It automatically loads the associated graph and default route (route_1).
     *
     * After calling loadMap(), you can:
     * 1. Use setInitialPose() to start localization with the loaded map
     * 2. Access the auto-loaded graph nodes/edges from the result
     * 3. Access the auto-loaded waypoints from the result
     *
     * @param name Map name (e.g., "my_map")
     * @return LoadMapResult containing graph, waypoints, and available routes
     */
    LoadMapResult loadMap(const std::string& name) {
        LoadMapResult result;
        result.mapName = name;

        if (!connected_) {
            result.success = false;
            result.message = "Not connected to robot";
            return result;
        }

        ensureMapLoadClients();

        // Step 1: Load the map (PCD) from robot storage
        auto mapRequest = std::make_shared<raisin::raisin_interfaces::srv::LoadLaserMap::Request>();
        mapRequest->name = name;

        auto mapFuture = loadMapClient_->asyncSendRequest(mapRequest);

        if (mapFuture.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
            auto response = mapFuture.get();
            if (!response->success) {
                result.success = false;
                result.message = response->message;
                return result;
            }
            std::cout << "[RaisinClient] Map loaded from robot: " << name << std::endl;
        } else {
            result.success = false;
            result.message = "Map load request timeout";
            return result;
        }

        // Store map frame name for subscribeMapOdometry()
        mapFrameName_ = name;

        // Step 2: Auto-load graph from robot
        std::string graphName = name + "/graph";
        auto graphResult = loadGraphFile(graphName);
        if (graphResult.success) {
            result.graphNodes = graphResult.nodes;
            result.graphEdges = graphResult.edges;
            std::cout << "[RaisinClient] Graph auto-loaded: " << result.graphNodes.size()
                      << " nodes, " << result.graphEdges.size() / 2 << " edges" << std::endl;
        } else {
            std::cout << "[RaisinClient] No graph found for " << graphName
                      << " (you can create one with graph editor)" << std::endl;
        }

        // Step 3: List available routes
        auto routesResult = listWaypointsFiles();
        if (routesResult.success) {
            // Filter routes for this map
            for (const auto& file : routesResult.files) {
                if (file.find(name + "/paths/") == 0) {
                    result.availableRoutes.push_back(file);
                }
            }
            std::cout << "[RaisinClient] Available routes: " << result.availableRoutes.size() << std::endl;
        }

        // Step 4: Auto-load default route (route_1)
        std::string defaultRouteName = name + "/paths/route_1";
        auto loadRouteResult = loadWaypointsFile(defaultRouteName);
        if (loadRouteResult.success) {
            auto status = getMissionStatus();
            if (status.valid) {
                result.waypoints = status.waypoints;
                std::cout << "[RaisinClient] Default route auto-loaded: "
                          << result.waypoints.size() << " waypoints" << std::endl;
            }
        } else {
            std::cout << "[RaisinClient] No default route found (route_1)" << std::endl;
        }

        result.success = true;
        result.message = "Map loaded: " + name;
        return result;
    }

    /**
     * @brief Set initial pose for localization on the loaded map
     *
     * Call this after loadMap() to initialize localization.
     * This uses the map already loaded on the robot (not sending PCD data).
     *
     * @param x Initial X position
     * @param y Initial Y position
     * @param yaw Initial yaw angle in radians
     * @return Result of the operation
     */
    ServiceResult setInitialPose(double x, double y, double yaw) {
        if (!connected_) {
            return {false, "Not connected to robot"};
        }

        if (mapFrameName_.empty()) {
            return {false, "No map loaded. Call loadMap() first."};
        }

        ensureMapClient();

        // Get the current map from robot and set initial pose
        // We use SetLaserMap but with the map already loaded via LoadLaserMap
        auto request = std::make_shared<raisin::raisin_interfaces::srv::SetLaserMap::Request>();
        request->name = mapFrameName_;

        // Set initial pose
        raisin::geometry_msgs::msg::Pose initial_pose;
        initial_pose.position.x = x;
        initial_pose.position.y = y;
        initial_pose.position.z = 0.0;

        double half_yaw = yaw * 0.5;
        initial_pose.orientation.x = 0.0;
        initial_pose.orientation.y = 0.0;
        initial_pose.orientation.z = std::sin(half_yaw);
        initial_pose.orientation.w = std::cos(half_yaw);

        request->initial_pose = initial_pose;
        // Note: pc field is empty - we're using the map already loaded on robot

        ServiceResult result;
        auto future = setMapClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
            if (result.success) {
                std::cout << "[RaisinClient] Initial pose set for map: " << mapFrameName_ << std::endl;
            }
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    /**
     * @brief Get the currently loaded map name
     * @return Map name or empty string if no map is loaded
     */
    std::string getLoadedMapName() const {
        return mapFrameName_;
    }

    // ========================================================================
    // Control Mode Switching
    // ========================================================================

    /**
     * @brief Find GUI network ID from detected peers
     */
    std::string findGuiNetworkId(const std::string& prefix = "gui") {
        if (!network_) {
            return "";
        }
        auto connections = network_->getAllConnections();
        for (const auto& conn : connections) {
            if (conn.id.find(prefix) == 0) {
                return conn.id;
            }
        }
        return "";
    }

    /**
     * @brief Set manual joystick control mode (gamepad)
     * @param gui_network_id GUI network ID (auto-detected if empty)
     * @return Result of the operation
     */
    ServiceResult setManualControl(const std::string& gui_network_id = "") {
        std::string networkId = gui_network_id;
        if (networkId.empty()) {
            networkId = findGuiNetworkId();
        }
        return setListenSource("joy/gui", networkId);
    }

    /**
     * @brief Set autonomous control mode (for patrol)
     * @return Result of the operation
     */
    ServiceResult setAutonomousControl() {
        return setListenSource("vel_cmd/autonomy");
    }

    /**
     * @brief Release control (set to None)
     * @param source Control source to release
     * @return Result of the operation
     */
    ServiceResult releaseControl(const std::string& source = "joy/gui") {
        return setListenSource(source, "<CLOSE>");
    }

    /**
     * @brief Set listen source (low-level API)
     */
    ServiceResult setListenSource(const std::string& topic_name, const std::string& network_id = "") {
        if (!connected_) {
            return {false, "Not connected to robot"};
        }

        ensureJoyClient();

        auto request = std::make_shared<raisin::raisin_interfaces::srv::String::Request>();
        if (network_id.empty()) {
            request->data = topic_name;
        } else {
            request->data = topic_name + "<&>" + network_id;
        }

        ServiceResult result;
        auto future = setJoyListenClient_->asyncSendRequest(request);

        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto response = future.get();
            result.success = response->success;
            result.message = response->message;
        } else {
            result.success = false;
            result.message = "Request timeout";
        }

        return result;
    }

    // ========================================================================
    // Subscriptions (Real-time Data)
    // ========================================================================

    /**
     * @brief Subscribe to robot odometry in map frame
     * Call this after setMap() succeeds to get robot position in map coordinates.
     * Topic: /{map_name}/{robot_id}/Odometry
     */
    void subscribeMapOdometry(OdometryCallback callback) {
        if (mapFrameName_.empty()) {
            std::cerr << "[RaisinClient] Error: Call setMap() first before subscribeMapOdometry()" << std::endl;
            return;
        }

        odomCallback_ = callback;
        std::string topic = "/" + mapFrameName_ + "/" + robotId_ + "/Odometry";
        odomSubscriber_ = node_->createSubscriber<raisin::nav_msgs::msg::Odometry>(
            topic, connection_,
            [this](const raisin::nav_msgs::msg::Odometry::SharedPtr& msg) {
                RobotState state;
                state.x = msg->pose.pose.position.x;
                state.y = msg->pose.pose.position.y;
                state.z = msg->pose.pose.position.z;

                double qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y;
                double qz = msg->pose.pose.orientation.z;
                double qw = msg->pose.pose.orientation.w;
                state.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                       1.0 - 2.0 * (qy * qy + qz * qz));

                state.vx = msg->twist.twist.linear.x;
                state.vy = msg->twist.twist.linear.y;
                state.omega = msg->twist.twist.angular.z;
                state.valid = true;

                {
                    std::lock_guard<std::mutex> lock(stateMutex_);
                    latestState_ = state;
                }

                if (odomCallback_) {
                    odomCallback_(state);
                }
            });
        std::cout << "[RaisinClient] Subscribed to " << topic << std::endl;
    }

    /**
     * @brief Subscribe to robot odometry in odom frame (raw Fast-LIO output)
     * Use subscribeMapOdometry() instead for map-aligned coordinates.
     */
    void subscribeOdometry(OdometryCallback callback) {
        odomCallback_ = callback;
        odomSubscriber_ = node_->createSubscriber<raisin::nav_msgs::msg::Odometry>(
            "/Odometry", connection_,
            [this](const raisin::nav_msgs::msg::Odometry::SharedPtr& msg) {
                RobotState state;
                state.x = msg->pose.pose.position.x;
                state.y = msg->pose.pose.position.y;
                state.z = msg->pose.pose.position.z;

                double qx = msg->pose.pose.orientation.x;
                double qy = msg->pose.pose.orientation.y;
                double qz = msg->pose.pose.orientation.z;
                double qw = msg->pose.pose.orientation.w;
                state.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                       1.0 - 2.0 * (qy * qy + qz * qz));

                state.vx = msg->twist.twist.linear.x;
                state.vy = msg->twist.twist.linear.y;
                state.omega = msg->twist.twist.angular.z;
                state.valid = true;

                {
                    std::lock_guard<std::mutex> lock(stateMutex_);
                    latestState_ = state;
                }

                if (odomCallback_) {
                    odomCallback_(state);
                }
            });
        std::cout << "[RaisinClient] Subscribed to /Odometry" << std::endl;
    }

    /**
     * @brief Subscribe to live LiDAR point cloud
     */
    void subscribePointCloud(PointCloudCallback callback) {
        cloudCallback_ = callback;
        cloudSubscriber_ = node_->createSubscriber<raisin::sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", connection_,
            [this](const raisin::sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
                std::vector<Point3D> points;

                if (msg->data.empty()) return;

                size_t point_step = msg->point_step;
                size_t num_points = msg->width * msg->height;

                int x_offset = -1, y_offset = -1, z_offset = -1;
                for (const auto& field : msg->fields) {
                    if (field.name == "x") x_offset = field.offset;
                    else if (field.name == "y") y_offset = field.offset;
                    else if (field.name == "z") z_offset = field.offset;
                }

                if (x_offset < 0 || y_offset < 0 || z_offset < 0) return;

                points.reserve(num_points);
                for (size_t i = 0; i < num_points; ++i) {
                    const uint8_t* ptr = msg->data.data() + i * point_step;
                    Point3D p;
                    p.x = *reinterpret_cast<const float*>(ptr + x_offset);
                    p.y = *reinterpret_cast<const float*>(ptr + y_offset);
                    p.z = *reinterpret_cast<const float*>(ptr + z_offset);
                    points.push_back(p);
                }

                {
                    std::lock_guard<std::mutex> lock(cloudMutex_);
                    latestCloud_ = std::move(points);
                }

                if (cloudCallback_) {
                    std::lock_guard<std::mutex> lock(cloudMutex_);
                    cloudCallback_(latestCloud_);
                }
            });
        std::cout << "[RaisinClient] Subscribed to /cloud_registered" << std::endl;
    }

    /**
     * @brief Subscribe to extended robot state (battery, actuators, locomotion state)
     */
    void subscribeRobotState(ExtendedRobotStateCallback callback) {
        extRobotStateCallback_ = callback;
        robotStateSubscriber_ = node_->createSubscriber<raisin::raisin_interfaces::msg::RobotState>(
            "robot_state", connection_,
            [this](const raisin::raisin_interfaces::msg::RobotState::SharedPtr& msg) {
                ExtendedRobotState state;

                state.x = msg->base_pos[0];
                state.y = msg->base_pos[1];
                state.z = msg->base_pos[2];

                double qx = msg->base_quat[0];
                double qy = msg->base_quat[1];
                double qz = msg->base_quat[2];
                double qw = msg->base_quat[3];
                state.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                       1.0 - 2.0 * (qy * qy + qz * qz));

                state.vx = msg->base_lin_vel[0];
                state.vy = msg->base_lin_vel[1];
                state.omega = msg->base_ang_vel[2];

                state.locomotion_state = msg->state;

                state.voltage = msg->voltage;
                state.current = msg->current;
                state.max_voltage = msg->max_voltage;
                state.min_voltage = msg->min_voltage;

                state.body_temperature = msg->body_temperature;

                state.joy_listen_type = msg->joy_listen_type;

                state.actuators.reserve(msg->actuator_states.size());
                for (const auto& act : msg->actuator_states) {
                    ActuatorInfo info;
                    info.name = act.name;
                    info.status = act.status;
                    info.temperature = act.temperature;
                    info.position = act.position;
                    info.velocity = act.velocity;
                    info.effort = act.effort;
                    state.actuators.push_back(info);
                }

                state.valid = true;

                {
                    std::lock_guard<std::mutex> lock(extStateMutex_);
                    latestExtState_ = state;
                }

                if (extRobotStateCallback_) {
                    extRobotStateCallback_(state);
                }
            });
        std::cout << "[RaisinClient] Subscribed to robot_state" << std::endl;
    }

    // ========================================================================
    // Getters (Thread-safe)
    // ========================================================================

    ExtendedRobotState getExtendedRobotState() {
        std::lock_guard<std::mutex> lock(extStateMutex_);
        return latestExtState_;
    }

    RobotState getRobotState() {
        std::lock_guard<std::mutex> lock(stateMutex_);
        return latestState_;
    }

    std::vector<Point3D> getLatestPointCloud() {
        std::lock_guard<std::mutex> lock(cloudMutex_);
        return latestCloud_;
    }

private:
    std::string client_id_;
    bool connected_;

    std::vector<std::string> interfaces_;
    std::shared_ptr<raisin::Network> network_;
    std::shared_ptr<raisin::Remote::Connection> connection_;
    std::unique_ptr<raisin::Node> node_;

    // Service clients
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::SetWaypoints>> setWaypointsClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::GetWaypoints>> getWaypointsClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::SetLaserMap>> setMapClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::String>> setJoyListenClient_;
    std::shared_ptr<raisin::Client<raisin::std_srvs::srv::Trigger>> standUpClient_;
    std::shared_ptr<raisin::Client<raisin::std_srvs::srv::Trigger>> sitDownClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::ListFiles>> listWaypointsFilesClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::LoadWaypointsFile>> loadWaypointsFileClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::SaveWaypointsFile>> saveWaypointsFileClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::ResumePatrol>> resumePatrolClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::SaveGraphFile>> saveGraphFileClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::LoadGraphFile>> loadGraphFileClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::LoadLaserMap>> loadMapClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::ListFiles>> listMapFilesClient_;
    std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::RefineWaypoints>> refineWaypointsClient_;

    // Subscribers
    raisin::Subscriber<raisin::nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
    raisin::Subscriber<raisin::sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber_;
    raisin::Subscriber<raisin::raisin_interfaces::msg::RobotState>::SharedPtr robotStateSubscriber_;

    // Connection info for map odometry topic
    std::string robotId_;
    std::string mapFrameName_;

    // Callbacks
    OdometryCallback odomCallback_;
    PointCloudCallback cloudCallback_;
    ExtendedRobotStateCallback extRobotStateCallback_;

    // Cached data
    mutable std::mutex stateMutex_;
    mutable std::mutex cloudMutex_;
    mutable std::mutex extStateMutex_;
    RobotState latestState_;
    std::vector<Point3D> latestCloud_;
    ExtendedRobotState latestExtState_;

    void ensureWaypointClients() {
        if (!setWaypointsClient_) {
            setWaypointsClient_ = node_->createClient<raisin::raisin_interfaces::srv::SetWaypoints>(
                "planning/set_waypoints", connection_);
        }
        if (!getWaypointsClient_) {
            getWaypointsClient_ = node_->createClient<raisin::raisin_interfaces::srv::GetWaypoints>(
                "planning/get_waypoints", connection_);
        }
    }

    void ensureJoyClient() {
        if (!setJoyListenClient_) {
            setJoyListenClient_ = node_->createClient<raisin::raisin_interfaces::srv::String>(
                "set_listen", connection_);
        }
    }

    void ensureMapClient() {
        if (!setMapClient_) {
            setMapClient_ = node_->createClient<raisin::raisin_interfaces::srv::SetLaserMap>(
                "set_map", connection_);
        }
    }

    void ensureLocomotionClients() {
        if (!standUpClient_) {
            standUpClient_ = node_->createClient<raisin::std_srvs::srv::Trigger>(
                "stand_up", connection_);
        }
        if (!sitDownClient_) {
            sitDownClient_ = node_->createClient<raisin::std_srvs::srv::Trigger>(
                "sit_down", connection_);
        }
    }

    void ensurePatrolClients() {
        if (!listWaypointsFilesClient_) {
            listWaypointsFilesClient_ = node_->createClient<raisin::raisin_interfaces::srv::ListFiles>(
                "planning/list_waypoints_files", connection_);
        }
        if (!loadWaypointsFileClient_) {
            loadWaypointsFileClient_ = node_->createClient<raisin::raisin_interfaces::srv::LoadWaypointsFile>(
                "planning/load_waypoints_file", connection_);
        }
        if (!saveWaypointsFileClient_) {
            saveWaypointsFileClient_ = node_->createClient<raisin::raisin_interfaces::srv::SaveWaypointsFile>(
                "planning/save_waypoints_file", connection_);
        }
        if (!resumePatrolClient_) {
            resumePatrolClient_ = node_->createClient<raisin::raisin_interfaces::srv::ResumePatrol>(
                "planning/resume_patrol", connection_);
        }
    }

    void ensureGraphClients() {
        if (!saveGraphFileClient_) {
            saveGraphFileClient_ = node_->createClient<raisin::raisin_interfaces::srv::SaveGraphFile>(
                "save_graph_file", connection_);
        }
        if (!loadGraphFileClient_) {
            loadGraphFileClient_ = node_->createClient<raisin::raisin_interfaces::srv::LoadGraphFile>(
                "load_graph_file", connection_);
        }
    }

    void ensureMapLoadClients() {
        if (!loadMapClient_) {
            loadMapClient_ = node_->createClient<raisin::raisin_interfaces::srv::LoadLaserMap>(
                "load_laser_map", connection_);
        }
        if (!listMapFilesClient_) {
            listMapFilesClient_ = node_->createClient<raisin::raisin_interfaces::srv::ListFiles>(
                "list_map_files", connection_);
        }
    }

    void ensureRefineWaypointsClient() {
        if (!refineWaypointsClient_) {
            refineWaypointsClient_ = node_->createClient<raisin::raisin_interfaces::srv::RefineWaypoints>(
                "planning/refine_waypoints", connection_);
        }
    }
};

}  // namespace raisin_sdk
