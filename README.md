# MSP ROS Package

A lightweight ROS package for communicating with Betaflight/INAV flight controllers using the MultiWii Serial Protocol (MSP). This package provides essential functionality similar to mavros but is optimized for Betaflight/INAV with a smaller footprint.

## Features

- **IMU Data Acquisition**: Retrieve accelerometer, gyroscope, and attitude data
- **Custom Debug Information**: Access waypoint and other debug data
- **RC Command Passthrough**: Send RC commands from ROS to flight controller
- **Lightweight Design**: Optimized for Betaflight/INAV with minimal dependencies
- **Configurable Parameters**: Easy to configure through ROS parameters

## Prerequisites

- ROS Noetic (or compatible version)
- C++11 or higher
- Serial port access privileges
- Betaflight/INAV flight controller with MSP support

## Installation

### 1. Clone the Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/JJJJJJJack/msp_ros.git
```

### 2. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Set Serial Port Permissions

```bash
sudo usermod -aG dialout $USER
# Logout and login again for changes to take effect
```

## Usage

### Launch File

The package includes a launch file for easy configuration:

```bash
roslaunch msp_ros msp_ros.launch
```

### Custom Launch Configuration

You can create a custom launch file or modify the parameters directly:

```bash
rosrun msp_ros msp_ros _port:=/dev/ttyACM0 _baudrate:=115200 _LogIMU:=true _SendRC:=true _LogDebug:=true
```

## ROS Topics

### Published Topics

#### `msp_imu` (sensor_msgs/Imu)
- **Description**: IMU data including orientation, linear acceleration, and angular velocity
- **Message Type**: `sensor_msgs/Imu`
- **Frame ID**: `msp_imu`
- **Frequency**: Up to 100 Hz (configurable)

#### `msp_debug` (std_msgs/Float64MultiArray)
- **Description**: Custom debug data (waypoint information)
- **Message Type**: `std_msgs/Float64MultiArray`
- **Data Format**: 6-element array containing debug values
- **Frequency**: Up to 100 Hz (configurable)

### Subscribed Topics

#### `joy_control` (sensor_msgs/Joy)
- **Description**: RC commands from joystick/controller
- **Message Type**: `sensor_msgs/Joy`
- **Channel Mapping**:
  - Axis 0: Roll (converted to 1000-2000 PWM)
  - Axis 1: Pitch (converted to 1000-2000 PWM)
  - Axis 2: Yaw (converted to 1000-2000 PWM)
  - Axis 3: Throttle (converted to 1000-2000 PWM)
  - Buttons 0-3: AUX1-AUX4 channels (1000 to 2000 PWM)

## ROS Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `port` | string | `/dev/ttyACM0` | Serial port device path |
| `baudrate` | int | 115200 | Serial communication baudrate |
| `LogIMU` | bool | false | Enable/disable IMU data publishing |
| `SendRC` | bool | false | Enable/disable RC command sending |
| `LogDebug` | bool | true | Enable/disable debug data publishing |

## Example Configuration

### Example 1: Basic IMU Data Acquisition

```xml
<launch>
  <node name="msp_ros" pkg="msp_ros" type="msp_ros" output="screen">
    <param name="port" value="/dev/ttyACM0"/>
    <param name="baudrate" value="115200"/>
    <param name="LogIMU" value="true"/>
    <param name="SendRC" value="false"/>
    <param name="LogDebug" value="false"/>
  </node>
</launch>
```

### Example 2: Full Configuration with RC Control

```xml
<launch>
  <node name="msp_ros" pkg="msp_ros" type="msp_ros" output="screen">
    <param name="port" value="/dev/ttyACM0"/>
    <param name="baudrate" value="115200"/>
    <param name="LogIMU" value="true"/>
    <param name="SendRC" value="true"/>
    <param name="LogDebug" value="true"/>
  </node>
  
  <!-- Example joy node for RC control -->
  <node name="joy_node" pkg="joy" type="joy_node">
    <param name="dev" value="/dev/input/js0"/>
    <param name="deadzone" value="0.12"/>
  </node>
</launch>
```

## MSP Protocol Support

This package supports the following MSP messages:

- **MSP_ATTITUDE (108)**: Attitude data (roll, pitch, yaw)
- **MSP_RAW_IMU (102)**: Raw IMU data (accelerometer, gyroscope)
- **MSP_SET_RAW_RC (200)**: Send RC commands to flight controller
- **Custom Waypoint Message MSP_WP (118)**: For debug information

## Troubleshooting

### Common Issues

1. **Serial Port Permission Denied**
   ```bash
   sudo chmod -R 777 /dev/ttyACM0
   # Or add user to dialout group permanently
   ```

2. **No Data Received**
   - Check if the correct port is specified
   - Verify baudrate matches flight controller configuration
   - Ensure MSP is enabled in Betaflight/INAV configurator

3. **RC Commands Not Working**
   - Check if `SendRC` parameter is set to true
   - Verify joystick is properly connected and publishing to `joy_control`
   - Ensure joystick is not in arm configuration when connected

### Debugging

Enable debug printing by uncommenting the following line in `msp_ros.cpp`:

```cpp
#define DEBUG_PRINT 0
```

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

### Development Guidelines

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Based on the original MSP library by christianrauch: https://github.com/christianrauch/msp
- Thanks to the Betaflight and INAV communities for their documentation and support
- Inspired by the functionality of mavros but optimized for Betaflight/INAV

## Contact

For questions or support, please open an issue on GitHub or contact the maintainer.

---

**Note**: This package is designed for educational and research purposes. Always ensure safe operation when working with flight controllers.