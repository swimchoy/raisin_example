cmake_minimum_required(VERSION 3.16)
project(raisin_autonomy_sdk VERSION 1.0.0 LANGUAGES CXX)

# ============================================================================
# Raisin Autonomy SDK
#
# This SDK provides examples for controlling Raisin robots using the
# raisin_network library. External developers can use this as a starting
# point for integrating autonomous navigation into their applications.
#
# Structure:
#   include/raisin_sdk/
#     - raisin_client.hpp   : SDK client for robot communication
#   examples/
#     - example_*.cpp       : Simple API examples
# ============================================================================

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# SDK Install Path (this is what external developers receive)
# ============================================================================

if(NOT DEFINED RAISIN_MASTER_PATH)
	set(RAISIN_MASTER_PATH "$ENV{HOME}/Desktop/raisin_master" CACHE PATH "Path to Raisin master source")
endif()

# Check for installed SDK first, then fall back to development build
set(RAISIN_SDK_PATH "${RAISIN_MASTER_PATH}/install" CACHE PATH "Path to Raisin SDK install")
set(RAISIN_BUILD_PATH "${RAISIN_MASTER_PATH}/cmake-build-release" CACHE PATH "Path to Raisin build directory")

if(EXISTS "${RAISIN_SDK_PATH}/lib/libraisin_network.so")
    message(STATUS "Using installed Raisin SDK at: ${RAISIN_SDK_PATH}")
    set(USE_DEVELOPMENT_BUILD OFF)
elseif(EXISTS "${RAISIN_BUILD_PATH}/src/raisin/raisin_network/libraisin_network.so")
    message(STATUS "Using development build at: ${RAISIN_BUILD_PATH}")
    set(USE_DEVELOPMENT_BUILD ON)
else()
    message(FATAL_ERROR "Raisin SDK not found. Set RAISIN_MASTER_PATH to your raisin_master directory.")
endif()

# ============================================================================
# Find System Packages
# ============================================================================

find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)

# PCL for PCD file loading (manual config to avoid VTK/MPI issues)
file(GLOB PCL_INCLUDE_HINT "/usr/include/pcl-*")
if(PCL_INCLUDE_HINT)
    list(GET PCL_INCLUDE_HINT 0 PCL_INCLUDE_DIRS)
    message(STATUS "Found PCL include: ${PCL_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "PCL not found. Install with: sudo apt-get install libpcl-dev")
endif()
set(PCL_LIBRARIES pcl_common pcl_io)

# ============================================================================
# SDK Include and Library Paths
# ============================================================================

if(USE_DEVELOPMENT_BUILD)
    # Development build: collect all include directories from source tree
    file(GLOB RAISIN_SRC_INCLUDES "${RAISIN_MASTER_PATH}/src/raisin/*/include")
    file(GLOB RAISIN_ENCRYPTION_INCLUDES "${RAISIN_MASTER_PATH}/src/raisin/raisin_encryption/*/include")

    set(RAISIN_INCLUDE_DIRS
        # Release install headers (generated messages, websocketpp, etc.)
        "${RAISIN_MASTER_PATH}/release/install/raisin_plugin/ubuntu/22.04/x86_64/release/include"
        "${RAISIN_MASTER_PATH}/release/install/raisin_third_party_common/ubuntu/22.04/x86_64/release/include"
        # RaiSim install
        "${RAISIN_MASTER_PATH}/install/include"
        # All source includes
        ${RAISIN_SRC_INCLUDES}
        ${RAISIN_ENCRYPTION_INCLUDES}
    )

    set(RAISIN_LIBRARIES
        "${RAISIN_BUILD_PATH}/src/raisin/raisin_network/libraisin_network.so"
        "${RAISIN_BUILD_PATH}/src/raisin/raisin_thread_pool/libraisin_thread_pool.so"
        "${RAISIN_BUILD_PATH}/src/raisin/raisin_parameter/libraisin_parameter.so"
        "${RAISIN_BUILD_PATH}/src/raisin/raisin_encryption/raisin_aes256_encryption/libraisin_aes256_encryption.so"
        "${RAISIN_BUILD_PATH}/src/raisin/raisin_data_logger/libraisin_data_logger.so"
        "${RAISIN_BUILD_PATH}/src/raisin/raisin_util/libraisin_util.so"
        "${RAISIN_MASTER_PATH}/install/lib/libraisim.so"
        "${RAISIN_BUILD_PATH}/src/raisin_third_party_common/zstd/build/cmake/lib/libzstd.so"
    )

    set(RAISIN_SDK_LIB_DIR "${RAISIN_BUILD_PATH}/src/raisin/raisin_network")
else()
    # Installed SDK: single directory structure
    set(RAISIN_SDK_INCLUDE_DIR "${RAISIN_SDK_PATH}/include")
    set(RAISIN_SDK_LIB_DIR "${RAISIN_SDK_PATH}/lib")

    set(RAISIN_INCLUDE_DIRS ${RAISIN_SDK_INCLUDE_DIR})

    set(RAISIN_LIBRARIES
        "${RAISIN_SDK_LIB_DIR}/libraisin_network.so"
        "${RAISIN_SDK_LIB_DIR}/libraisin_thread_pool.so"
        "${RAISIN_SDK_LIB_DIR}/libraisin_parameter.so"
        "${RAISIN_SDK_LIB_DIR}/libraisin_aes256_encryption.so"
        "${RAISIN_SDK_LIB_DIR}/libraisin_data_logger.so"
        "${RAISIN_SDK_LIB_DIR}/libraisin_util.so"
        "${RAISIN_SDK_LIB_DIR}/libraisim.so"
        "${RAISIN_SDK_LIB_DIR}/libzstd.so"
    )
endif()

set(SYSTEM_LIBRARIES
    Eigen3::Eigen
    OpenSSL::SSL
    OpenSSL::Crypto
    ${PCL_LIBRARIES}
    pthread
    dl
    rt
)

# ============================================================================
# Simple API Examples
# ============================================================================

# Helper function to add simple examples
function(add_simple_example EXAMPLE_NAME)
    add_executable(${EXAMPLE_NAME} examples/${EXAMPLE_NAME}.cpp)
    target_include_directories(${EXAMPLE_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${RAISIN_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    )
    target_link_libraries(${EXAMPLE_NAME} PRIVATE
        ${RAISIN_LIBRARIES}
        ${SYSTEM_LIBRARIES}
    )
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
        BUILD_RPATH "${RAISIN_SDK_LIB_DIR}"
        INSTALL_RPATH "${RAISIN_SDK_LIB_DIR}"
    )
endfunction()

# Monitoring examples
add_simple_example(example_robot_state)
add_simple_example(example_battery)
add_simple_example(example_actuator_status)
add_simple_example(example_odometry)
add_simple_example(example_pointcloud)
add_simple_example(example_ffmpeg_camera)

# Control example
add_simple_example(example_joy_control)

# Network discovery example
add_simple_example(example_connect)

# ============================================================================
# Install Targets
# ============================================================================

install(TARGETS
    example_robot_state example_battery example_actuator_status
    example_odometry example_pointcloud example_joy_control
    example_ffmpeg_camera
    example_connect
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# ============================================================================
# Print Configuration Summary
# ============================================================================

message(STATUS "")
message(STATUS "=== Raisin Autonomy SDK Configuration ===")
message(STATUS "  Version:      ${PROJECT_VERSION}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Build Type:   ${CMAKE_BUILD_TYPE}")
message(STATUS "  SDK Path:     ${RAISIN_SDK_PATH}")
message(STATUS "")
message(STATUS "  Examples:     example_robot_state")
message(STATUS "                example_battery")
message(STATUS "                example_ffmpeg_camera")
message(STATUS "                example_actuator_status")
message(STATUS "                example_odometry")
message(STATUS "                example_pointcloud")
message(STATUS "                example_joy_control")
message(STATUS "                example_connect")
message(STATUS "")
