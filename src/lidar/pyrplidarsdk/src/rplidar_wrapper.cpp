//  PyRPLIDARSDK - Python wrapper for RPLIDAR SDK
//  Copyright (C) 2025 Dexmate Inc.
//  
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//  
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//  
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//  SOFTWARE.
//
//  ============================================================================
//  This software uses the RPLIDAR SDK:
//  Copyright (c) 2009 - 2014 RoboPeak Team (http://www.robopeak.com)
//  Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd. (http://www.slamtec.com)
//  Licensed under the BSD License. See rplidar_sdk/LICENSE for full terms.
//  ============================================================================

// Standard library includes
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

// Third-party includes
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

// Local includes
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace {
// Constants
constexpr double kPi = 3.14159265358979323846;
constexpr size_t kMaxScanNodes = 8192;
constexpr size_t kSerialNumberLength = 16;
constexpr sl_u32 kDefaultBaudrate = 1000000;
constexpr sl_u32 kDefaultUdpPort = 8089;

// Angle conversion factor from Q14 format to radians
constexpr double kAngleQ14ToRadians = (90.0 / (1 << 14)) * (kPi / 180.0);
// Distance conversion factor from Q2 mm to meters
constexpr double kDistanceQ2ToMeters = 1.0 / (4.0 * 1000.0);

/**
 * Converts a serial number byte array to a hexadecimal string.
 * @param serial_num Pointer to the serial number byte array
 * @return Hexadecimal string representation of the serial number
 */
std::string SerialToHex(const sl_u8* serial_num) {
  if (serial_num == nullptr) {
    return {};
  }
  
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < kSerialNumberLength; ++i) {
    oss << std::setw(2) << static_cast<unsigned>(serial_num[i]);
  }
  return oss.str();
}

}  // namespace

/**
 * Structure containing RPLIDAR device information.
 */
struct DeviceInfo {
  sl_u8 model;
  sl_u16 firmware_version;
  sl_u8 hardware_version;
  std::string serial_number;
  
  DeviceInfo() = default;
  DeviceInfo(sl_u8 model_val, sl_u16 firmware_val, sl_u8 hardware_val, 
             std::string serial_val)
      : model(model_val), 
        firmware_version(firmware_val),
        hardware_version(hardware_val),
        serial_number(std::move(serial_val)) {}
};

/**
 * Structure containing RPLIDAR device health information.
 */
struct DeviceHealth {
  sl_u8 status;
  sl_u16 error_code;
  
  DeviceHealth() = default;
  DeviceHealth(sl_u8 status_val, sl_u16 error_val)
      : status(status_val), error_code(error_val) {}
};

/**
 * RPLIDAR driver class that provides a Python interface to the RPLIDAR SDK.
 * 
 * This class manages the connection to RPLIDAR devices via serial or UDP,
 * handles scanning operations, and provides device information.
 * 
 * Thread Safety: This class is not thread-safe. External synchronization
 * is required if used from multiple threads.
 */
class RplidarDriver {
 public:
  /**
   * Constructs an RplidarDriver instance.
   * 
   * @param port Optional serial port path (e.g., "/dev/ttyUSB0")
   * @param ip_address Optional IP address for UDP connection
   * @param baudrate Serial communication baudrate (default: 1000000)
   * @param udp_port UDP port for network communication (default: 8089)
   * 
   * @note Either port or ip_address must be provided, but not both.
   */
  explicit RplidarDriver(const std::optional<std::string>& port = std::nullopt,
                         const std::optional<std::string>& ip_address = std::nullopt,
                         sl_u32 baudrate = kDefaultBaudrate,
                         sl_u32 udp_port = kDefaultUdpPort)
      : port_(port),
        ip_address_(ip_address),
        baudrate_(baudrate),
        udp_port_(udp_port),
        driver_(nullptr),
        channel_(nullptr),
        is_connected_(false),
        is_scanning_(false) {}

  // Disable copy constructor and assignment operator
  RplidarDriver(const RplidarDriver&) = delete;
  RplidarDriver& operator=(const RplidarDriver&) = delete;

  // Enable move constructor and assignment operator
  RplidarDriver(RplidarDriver&& other) noexcept
      : port_(std::move(other.port_)),
        ip_address_(std::move(other.ip_address_)),
        baudrate_(other.baudrate_),
        udp_port_(other.udp_port_),
        driver_(other.driver_),
        channel_(other.channel_),
        is_connected_(other.is_connected_),
        is_scanning_(other.is_scanning_) {
    other.driver_ = nullptr;
    other.channel_ = nullptr;
    other.is_connected_ = false;
    other.is_scanning_ = false;
  }

  RplidarDriver& operator=(RplidarDriver&& other) noexcept {
    if (this != &other) {
      Disconnect();
      port_ = std::move(other.port_);
      ip_address_ = std::move(other.ip_address_);
      baudrate_ = other.baudrate_;
      udp_port_ = other.udp_port_;
      driver_ = other.driver_;
      channel_ = other.channel_;
      is_connected_ = other.is_connected_;
      is_scanning_ = other.is_scanning_;
      
      other.driver_ = nullptr;
      other.channel_ = nullptr;
      other.is_connected_ = false;
      other.is_scanning_ = false;
    }
    return *this;
  }

  /**
   * Destructor that ensures proper cleanup of resources.
   */
  ~RplidarDriver() {
    Disconnect();
  }

  /**
   * Establishes connection to the RPLIDAR device.
   * 
   * @return true if connection is successful, false otherwise
   * 
   * @note This method will fail if neither port nor ip_address is specified,
   *       or if the device is already connected.
   */
  bool Connect() {
    if (is_connected_) {
      std::cerr << "Error: Device is already connected." << std::endl;
      return false;
    }

    if (!ip_address_ && !port_) {
      std::cerr << "Error: Either serial port or IP address must be specified." << std::endl;
      return false;
    }

    if (ip_address_ && port_) {
      std::cerr << "Error: Cannot specify both serial port and IP address." << std::endl;
      return false;
    }

    // Create communication channel
    if (!CreateChannel()) {
      return false;
    }

    // Create lidar driver
    if (!CreateDriver()) {
      CleanupChannel();
      return false;
    }

    // Establish connection
    const sl_result result = driver_->connect(channel_);
    if (SL_IS_FAIL(result)) {
      LogConnectionError();
      Disconnect();
      return false;
    }

    // Verify connection by attempting to get device health
    // This ensures the device is actually reachable and responsive
    sl_lidar_response_device_health_t health_response;
    const sl_result health_result = driver_->getHealth(health_response);
    
    if (SL_IS_FAIL(health_result)) {
      if (ip_address_) {
        std::cerr << "Error: Failed to connect to RPLIDAR. The IP address " 
                  << *ip_address_ << ":" << udp_port_ 
                  << " may be invalid, unreachable, or no RPLIDAR device is listening." << std::endl;
      } else if (port_) {
        std::cerr << "Error: Failed to connect to RPLIDAR on serial port " 
                  << *port_ << ". Device may be disconnected or in use by another process." << std::endl;
      }
      Disconnect();
      return false;
    }

    is_connected_ = true;
    return true;
  }

  /**
   * Disconnects from the RPLIDAR device and cleans up resources.
   * 
   * This method is safe to call multiple times and will handle cleanup
   * even if the connection was not properly established.
   */
  void Disconnect() {
    if (is_scanning_) {
      StopScan();
    }

    if (driver_ != nullptr) {
      driver_->stop();
      delete driver_;
      driver_ = nullptr;
    }

    CleanupChannel();
    is_connected_ = false;
  }

  /**
   * Retrieves device information from the connected RPLIDAR.
   * 
   * @return DeviceInfo structure containing model, firmware, hardware versions,
   *         and serial number, or std::nullopt if the operation fails
   */
  std::optional<DeviceInfo> GetDeviceInfo() const {
    if (!IsConnected() || driver_ == nullptr) {
      std::cerr << "Error: Device is not connected or driver is null." << std::endl;
      return std::nullopt;
    }

    sl_lidar_response_device_info_t device_info_response;
    const sl_result result = driver_->getDeviceInfo(device_info_response);
    
    if (SL_IS_FAIL(result)) {
      std::cerr << "Error: Failed to retrieve device information." << std::endl;
      return std::nullopt;
    }

    return DeviceInfo(
        device_info_response.model,
        device_info_response.firmware_version,
        device_info_response.hardware_version,
        SerialToHex(device_info_response.serialnum)
    );
  }

  /**
   * Retrieves health status from the connected RPLIDAR.
   * 
   * @return DeviceHealth structure containing status and error code,
   *         or std::nullopt if the operation fails
   */
  std::optional<DeviceHealth> GetHealth() const {
    if (!IsConnected() || driver_ == nullptr) {
      std::cerr << "Error: Device is not connected or driver is null." << std::endl;
      return std::nullopt;
    }

    sl_lidar_response_device_health_t health_response;
    const sl_result result = driver_->getHealth(health_response);
    
    if (SL_IS_FAIL(result)) {
      std::cerr << "Error: Failed to retrieve device health." << std::endl;
      return std::nullopt;
    }

    return DeviceHealth(health_response.status, health_response.error_code);
  }
    
  /**
   * Starts the laser scanning operation.
   * 
   * @return true if scanning started successfully, false otherwise
   * 
   * @note The device must be connected before calling this method.
   *       Multiple calls to StartScan() without calling StopScan() will fail.
   */
  bool StartScan() {
    if (!IsConnected() || driver_ == nullptr) {
      std::cerr << "Error: Device is not connected or driver is null." << std::endl;
      return false;
    }

    if (is_scanning_) {
      std::cerr << "Warning: Scanning is already active." << std::endl;
      return true;
    }

    // force=false, use_typical_scan=true for optimal performance
    const sl_result result = driver_->startScan(false, true);
    if (SL_IS_FAIL(result)) {
      std::cerr << "Error: Failed to start scanning." << std::endl;
      return false;
    }

    is_scanning_ = true;
    return true;
  }

  /**
   * Stops the laser scanning operation.
   * 
   * This method is safe to call even if scanning is not active.
   */
  void StopScan() {
    if (driver_ != nullptr) {
      driver_->stop();
    }
    is_scanning_ = false;
  }

  /**
   * Retrieves the latest scan data from the RPLIDAR.
   * 
   * @return Tuple containing (angles_rad, ranges_m, qualities) as vectors,
   *         or std::nullopt if the operation fails
   * 
   * @note Scanning must be started before calling this method.
   *       The returned vectors contain only valid measurement points.
   */
  std::optional<std::tuple<std::vector<float>, std::vector<float>, std::vector<uint8_t>>> 
  GetScanData() {
    if (!IsConnected() || driver_ == nullptr) {
      std::cerr << "Error: Device is not connected or driver is null." << std::endl;
      return std::nullopt;
    }

    if (!is_scanning_) {
      std::cerr << "Error: Scanning is not active. Call StartScan() first." << std::endl;
      return std::nullopt;
    }

    sl_lidar_response_measurement_node_hq_t nodes[kMaxScanNodes];
    size_t node_count = kMaxScanNodes;

    const sl_result result = driver_->grabScanDataHq(nodes, node_count);
    if (SL_IS_FAIL(result)) {
      std::cerr << "Error: Failed to grab scan data." << std::endl;
      return std::nullopt;
    }
    
    // BUG: This will make the sequence of points inconsistent, bad for point-wise timestamp estimation
    // // Sort scan data by angle for consistent output
    // if (driver_ != nullptr) {
    //   driver_->ascendScanData(nodes, node_count);
    // }

    return ProcessScanNodes(nodes, node_count);
  }

  /**
   * Retrieves the latest scan data with timestamp from the RPLIDAR.
   * 
   * @return Tuple containing (angles_rad, ranges_m, qualities, timestamps) as vectors,
   *         or std::nullopt if the operation fails
   * 
   * @note Scanning must be started before calling this method.
   *       The returned vectors contain only valid measurement points.
   */
  std::optional<std::tuple<std::vector<float>, std::vector<float>, std::vector<uint8_t>, uint64_t>>
  GetScanDataWithTimestamp() {
    if (!IsConnected() || driver_ == nullptr) {
      std::cerr << "Error: Device is not connected or driver is null." << std::endl;
      return std::nullopt;
    }

    if (!is_scanning_) {
      std::cerr << "Error: Scanning is not active. Call StartScan() first." << std::endl;
      return std::nullopt;
    }

    sl_lidar_response_measurement_node_hq_t nodes[kMaxScanNodes];
    size_t node_count = kMaxScanNodes;

    sl_u64 timestamp_uS = 0;
    const sl_result result = driver_->grabScanDataHqWithTimeStamp(nodes, node_count, timestamp_uS);
    if (SL_IS_FAIL(result)) {
      std::cerr << "Error: Failed to grab scan data." << std::endl;
      return std::nullopt;
    }

    auto ret_without_ts = ProcessScanNodes(nodes, node_count);
    auto ret_with_ts = std::tuple_cat(
        std::move(ret_without_ts),
        std::make_tuple(timestamp_uS * 1000)  // Convert microseconds to nanoseconds
    );
    return ret_with_ts;
  }

  /**
   * Checks if the device is currently connected.
   * 
   * @return true if connected, false otherwise
   */
  bool IsConnected() const {
    return is_connected_ && driver_ != nullptr && channel_ != nullptr;
  }

  /**
   * Checks if scanning is currently active.
   * 
   * @return true if scanning, false otherwise
   */
  bool IsScanning() const {
    return is_scanning_;
  }


 private:
  /**
   * Creates the appropriate communication channel based on configuration.
   * 
   * @return true if channel creation is successful, false otherwise
   */
  bool CreateChannel() {
    if (ip_address_) {
      channel_ = *sl::createUdpChannel(*ip_address_, udp_port_);
      if (channel_ == nullptr) {
        std::cerr << "Error: Failed to create UDP channel for " << *ip_address_ 
                  << ":" << udp_port_ << std::endl;
        return false;
      }
    } else if (port_) {
      channel_ = *sl::createSerialPortChannel(*port_, baudrate_);
      if (channel_ == nullptr) {
        std::cerr << "Error: Failed to create serial channel for " << *port_ 
                  << " at baudrate " << baudrate_ << std::endl;
        return false;
      }
    }
    return true;
  }

  /**
   * Creates the LIDAR driver instance.
   * 
   * @return true if driver creation is successful, false otherwise
   */
  bool CreateDriver() {
    driver_ = *sl::createLidarDriver();
    if (driver_ == nullptr) {
      std::cerr << "Error: Failed to create LIDAR driver." << std::endl;
      return false;
    }
    return true;
  }

  /**
   * Cleans up the communication channel.
   */
  void CleanupChannel() {
    if (channel_ != nullptr) {
      delete channel_;
      channel_ = nullptr;
    }
  }

  /**
   * Logs connection error messages with appropriate details.
   */
  void LogConnectionError() const {
    if (ip_address_) {
      std::cerr << "Error: Failed to connect to LIDAR via UDP at " 
                << *ip_address_ << ":" << udp_port_ << std::endl;
    } else if (port_) {
      std::cerr << "Error: Failed to connect to LIDAR on serial port " 
                << *port_ << std::endl;
    }
  }

  /**
   * Processes raw scan nodes and converts them to Python-friendly format.
   * 
   * @param nodes Array of scan measurement nodes
   * @param node_count Number of nodes in the array
   * @return Tuple of (angles, ranges, qualities) vectors containing valid points
   */
  std::tuple<std::vector<float>, std::vector<float>, std::vector<uint8_t>>
  ProcessScanNodes(const sl_lidar_response_measurement_node_hq_t* nodes, 
                   size_t node_count) const {
    std::vector<float> angles;
    std::vector<float> ranges;
    std::vector<uint8_t> qualities;

    // Pre-allocate with estimated capacity for better performance
    const size_t estimated_valid_points = node_count * 0.8;  // Assume 80% valid points
    angles.reserve(estimated_valid_points);
    ranges.reserve(estimated_valid_points);
    qualities.reserve(estimated_valid_points);

    for (size_t i = 0; i < node_count; ++i) {
      const auto& node = nodes[i];
      
      // Convert angle from Q14 format to radians
      const double angle_rad = static_cast<double>(node.angle_z_q14) * kAngleQ14ToRadians;
      
      // Convert distance from Q2 mm to meters
      const double range_m = static_cast<double>(node.dist_mm_q2) * kDistanceQ2ToMeters;
      
      // Only include valid measurement points
      if (range_m > 0.0) {
        angles.push_back(static_cast<float>(angle_rad));
        ranges.push_back(static_cast<float>(range_m));
        qualities.push_back(node.quality);
      }
    }

    return std::make_tuple(std::move(angles), std::move(ranges), std::move(qualities));
  }

  // Configuration parameters
  std::optional<std::string> port_;
  std::optional<std::string> ip_address_;
  sl_u32 baudrate_;
  sl_u32 udp_port_;

  // SDK objects (raw pointers managed by SDK)
  sl::ILidarDriver* driver_;
  sl::IChannel* channel_;

  // State tracking
  bool is_connected_;
  bool is_scanning_;
};

NB_MODULE(rplidar_wrapper, m) {
  m.doc() = "High-performance nanobind wrapper for RPLIDAR SDK";
  
  // DeviceInfo structure binding
  nb::class_<DeviceInfo>(m, "DeviceInfo", "RPLIDAR device information structure")
      .def_ro("model", &DeviceInfo::model, "Device model number")
      .def_ro("firmware_version", &DeviceInfo::firmware_version, "Firmware version")
      .def_ro("hardware_version", &DeviceInfo::hardware_version, "Hardware version")
      .def_ro("serial_number", &DeviceInfo::serial_number, "Device serial number (hex string)")
      .def("__repr__", [](const DeviceInfo& info) {
        return "DeviceInfo(model=" + std::to_string(info.model) +
               ", firmware=" + std::to_string(info.firmware_version) +
               ", hardware=" + std::to_string(info.hardware_version) +
               ", serial='" + info.serial_number + "')";
      });

  // DeviceHealth structure binding  
  nb::class_<DeviceHealth>(m, "DeviceHealth", "RPLIDAR device health information structure")
      .def_ro("status", &DeviceHealth::status, "Health status code")
      .def_ro("error_code", &DeviceHealth::error_code, "Error code (0 if no error)")
      .def("__repr__", [](const DeviceHealth& health) {
        std::string status_str = "Unknown";
        switch (health.status) {
          case SL_LIDAR_STATUS_OK:
            status_str = "OK";
            break;
          case SL_LIDAR_STATUS_WARNING:
            status_str = "Warning";
            break;
          case SL_LIDAR_STATUS_ERROR:
            status_str = "Error";
            break;
        }
        return "DeviceHealth(status=" + status_str + 
               ", error_code=" + std::to_string(health.error_code) + ")";
      });

  // RplidarDriver class binding with Python-friendly method names
  nb::class_<RplidarDriver>(m, "RplidarDriver", 
                            "High-level interface to RPLIDAR devices")
      .def(nb::init<std::optional<std::string>, std::optional<std::string>, sl_u32, sl_u32>(),
           "port"_a = std::nullopt,
           "ip_address"_a = std::nullopt,
           "baudrate"_a = kDefaultBaudrate,
           "udp_port"_a = kDefaultUdpPort,
           R"(
           Initialize RPLIDAR driver.
           
           Args:
               port: Serial port path (e.g., '/dev/ttyUSB0', 'COM3')
               ip_address: IP address for UDP connection  
               baudrate: Serial communication baudrate (default: 1000000)
               udp_port: UDP port for network communication (default: 8089)
               
           Note:
               Either port or ip_address must be provided, but not both.
           )")
      // Connection management
      .def("connect", &RplidarDriver::Connect, 
           "Establish connection to RPLIDAR device")
      .def("disconnect", &RplidarDriver::Disconnect,
           "Disconnect from RPLIDAR device and cleanup resources")
      .def("is_connected", &RplidarDriver::IsConnected,
           "Check if device is currently connected")
      
      // Device information
      .def("get_device_info", &RplidarDriver::GetDeviceInfo,
           "Retrieve device information (model, firmware, etc.)")
      .def("get_health", &RplidarDriver::GetHealth,
           "Retrieve device health status")
      
      // Scanning operations  
      .def("start_scan", &RplidarDriver::StartScan,
           "Start laser scanning operation")
      .def("stop_scan", &RplidarDriver::StopScan,
           "Stop laser scanning operation")
      .def("is_scanning", &RplidarDriver::IsScanning,
           "Check if scanning is currently active")
      .def("get_scan_data", &RplidarDriver::GetScanData,
           R"(
           Retrieve latest scan data.
           
           Returns:
               Optional tuple of (angles_rad, ranges_m, qualities) as lists,
               or None if operation fails.
               
           Note:
               Scanning must be started before calling this method.
               Only valid measurement points are included in the results.
           )")
      .def("get_scan_data_with_timestamp", &RplidarDriver::GetScanDataWithTimestamp,
           R"(           
           Retrieve latest scan data with timestamps.

           Returns:
               Optional tuple of (angles_rad, ranges_m, qualities, timestamp) as lists,
               or None if operation fails.

           Note:
               Scanning must be started before calling this method.
               Timestamp is in nanoseconds for the first point. Comes from the hardware.
               Each subsequent point's timestamp can be incremented based on the scan duration (frequency) later on.
               Only valid measurement points are included in the results.
           )");
}
