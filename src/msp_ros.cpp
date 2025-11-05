#include "ros/ros.h"
#include <Client.hpp>
#include <iostream>
#include <time.h>
#include <msp_msg.hpp>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/Imu.h"

// #define DEBUG_PRINT 0

using namespace std;

// Global variables
bool joy_control_ready = false;
sensor_msgs::Joy joy_control;

void joystick_command_callback(const sensor_msgs::Joy& message){
  joy_control_ready = true;
  joy_control = message;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "msp_ros");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);

    string USB_Dev;
    int BAUDRATE;
    bool LogIMU, LogDebug, SendRC;
    // Get parameters
    nh.param<string>("port", USB_Dev, "/dev/ttyACM0");
    nh.param<int>("baudrate", BAUDRATE, 115200);
    nh.param<bool>("LogIMU", LogIMU, false);
    nh.param<bool>("LogDebug", LogDebug, false);
    nh.param<bool>("SendRC", SendRC, false);
    
    // Subscribers and publishers
    ros::Subscriber sub_joy_control  = nh.subscribe("joy_control", 1, joystick_command_callback);
    ros::Publisher imu_pub           = nh.advertise<sensor_msgs::Imu>("msp_imu", 10);
    ros::Publisher debug_pub         = nh.advertise<std_msgs::Float64MultiArray>("msp_debug", 10);

    const string device = USB_Dev;
    const size_t baudrate = BAUDRATE;

    cout<<"Connecting to device: "<<device<<" at baudrate: "<<baudrate<<endl;

    msp::client::Client client;
    client.setLoggingLevel(msp::client::LoggingLevel::WARNING);
    client.setVariant(msp::FirmwareVariant::INAV);
    client.start(device, baudrate);

    msp::FirmwareVariant fw_variant = msp::FirmwareVariant::INAV;

    // Initialize time measurement
    struct timeval tvstart, tvend;
    gettimeofday(&tvstart,NULL);

    // Define messages
    msp::msg::SetWp set_WP(fw_variant);

    int count = 0;
    while(ros::ok()){
        // Get time duration
        gettimeofday(&tvend,NULL);
        double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
        
        // set_WP.msg1 = count;
        // if(client.sendMessage(set_WP) == 1) {
        //     cout<<"sending wp"<<endl;
        // }

        // Request and publish debug data
        if(LogDebug){
            msp::msg::WayPoint waypoint(fw_variant);
            if(client.sendMessage(waypoint) == 1) {
                #ifdef DEBUG_PRINT
                printf("Debug - msg1: %6.2f,     msg2: %6.2f,     msg3: %6.2f,     msg4: %6.2f,     msg5: %6.2f,     msg6: %6.2f\n", (double)waypoint.msg1, (double)waypoint.msg2, (double)waypoint.msg3, (double)waypoint.msg4, (double)waypoint.msg5, (double)waypoint.msg6);
                #endif
                std_msgs::Float64MultiArray waypoint_msg;
                waypoint_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                waypoint_msg.layout.dim[0].label = "debug_data";
                waypoint_msg.layout.dim[0].size = 6;
                waypoint_msg.layout.dim[0].stride = 1;
                waypoint_msg.layout.data_offset = 0;
                waypoint_msg.data.push_back(waypoint.msg1);
                waypoint_msg.data.push_back(waypoint.msg2);
                waypoint_msg.data.push_back(waypoint.msg3);
                waypoint_msg.data.push_back(waypoint.msg4);
                waypoint_msg.data.push_back(waypoint.msg5);
                waypoint_msg.data.push_back(waypoint.msg6);
                debug_pub.publish(waypoint_msg);
            }
        }
        
        // Request and publish IMU data
        if(LogIMU){
            msp::msg::Attitude attitude(fw_variant);
            sensor_msgs::Imu imu_ros_msg;
            imu_ros_msg.header.stamp = ros::Time::now();
            imu_ros_msg.header.frame_id = "msp_imu";
            bool attitude_received = false, imu_received = false;
            if(client.sendMessage(attitude) == 1){
                // Convert Euler angles to quaternion
                double cy = cos(attitude.yaw * 0.5 * M_PI / 180.0);
                double sy = sin(attitude.yaw * 0.5 * M_PI / 180.0);
                double cr = cos(attitude.roll * 0.5 * M_PI / 180.0);
                double sr = sin(attitude.roll * 0.5 * M_PI / 180.0);
                double cp = cos(attitude.pitch * 0.5 * M_PI / 180.0);
                double sp = sin(attitude.pitch * 0.5 * M_PI / 180.0);
                imu_ros_msg.orientation.w = cy * cr * cp + sy * sr * sp;
                imu_ros_msg.orientation.x = cy * sr * cp - sy * cr * sp;
                imu_ros_msg.orientation.y = cy * cr * sp + sy * sr * cp;
                imu_ros_msg.orientation.z = sy * cr * cp - cy * sr * sp;
                #ifdef DEBUG_PRINT
                printf("Attitude - roll: %6.2f, pitch: %6.2f, yaw: %6.2f\n", (float)(attitude.roll), (float)(attitude.pitch), (float)(attitude.yaw));
                #endif
                attitude_received = true;
            }
            msp::msg::RawImu imu_msg(fw_variant);
            if(client.sendMessage(imu_msg) == 1) {
                msp::msg::ImuSI imu_SI = msp::msg::ImuSI(imu_msg, 2048.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
                imu_ros_msg.linear_acceleration.x = imu_SI.acc[0];
                imu_ros_msg.linear_acceleration.y = imu_SI.acc[1];
                imu_ros_msg.linear_acceleration.z = imu_SI.acc[2];
                imu_ros_msg.angular_velocity.x    = imu_SI.gyro[0];
                imu_ros_msg.angular_velocity.y    = imu_SI.gyro[1];
                imu_ros_msg.angular_velocity.z    = imu_SI.gyro[2];
                #ifdef DEBUG_PRINT
                printf("IMU - acc_x: %6.2f, acc_y: %6.2f, acc_z: %6.2f, gyro_x: %6.2f, gyro_y: %6.2f, gyro_z: %6.2f\n", (float)(imu_SI.acc[0]), (float)(imu_SI.acc[1]), (float)(imu_SI.acc[2]), (float)(imu_SI.gyro[0]), (float)(imu_SI.gyro[1]), (float)(imu_SI.gyro[2]));
                #endif
                imu_received = true;
            }
            if(attitude_received || imu_received){
                imu_pub.publish(imu_ros_msg);
            }
        }

        if(SendRC && joy_control_ready){
            msp::msg::SetRawRc RawRC(fw_variant);
            RawRC.channels.push_back(joy_control.axes[0]*500 + 1500); // Roll
            RawRC.channels.push_back(joy_control.axes[1]*500 + 1500); // Pitch
            RawRC.channels.push_back(joy_control.axes[3]*1000);       // Throttle
            RawRC.channels.push_back(joy_control.axes[2]*500 + 1500); // Yaw
            RawRC.channels.push_back((joy_control.buttons[0]>=0.9)?1900:1000); // AUX1
            RawRC.channels.push_back((joy_control.buttons[1]>=0.9)?1900:1000); // AUX2
            RawRC.channels.push_back((joy_control.buttons[2]>=0.9)?1900:1000); // AUX3
            RawRC.channels.push_back((joy_control.buttons[3]>=0.9)?1900:1000); // AUX4
            // Fill remaining channels with defaults
            size_t count = RawRC.channels.size();
            while(RawRC.channels.size() < 8){
                RawRC.channels.push_back(1500);
            }
            if(client.sendMessage(RawRC) == 1) {
                #ifdef DEBUG_PRINT
                cout<<"sending RawRC"<<endl;
                #endif
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    
    client.stop();
}
