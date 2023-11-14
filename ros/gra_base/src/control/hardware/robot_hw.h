#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "gra_base/BaseState.h"
#include "gra_base/BaseSetpoint.h"
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>

#define NUM_DRIVE_JOINTS 1
#define NUM_STEER_JOINTS 1

class RobotHW : public hardware_interface::RobotHW {
private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;

    double cmd_drive[NUM_DRIVE_JOINTS] = {0.0};
    double pos_drive[NUM_DRIVE_JOINTS] = {0.0};
    double vel_drive[NUM_DRIVE_JOINTS] = {0.0};
    double eff_drive[NUM_DRIVE_JOINTS] = {0.0};

    double cmd_steer[NUM_STEER_JOINTS] = {0.0};
    double pos_steer[NUM_STEER_JOINTS] = {0.0};
    double vel_steer[NUM_STEER_JOINTS] = {0.0};
    double eff_steer[NUM_STEER_JOINTS] = {0.0};

    // ROS node
    ros::Publisher wheel_vel_pub;
    ros::Subscriber wheel_state_sub;

    void baseStateCallback(const gra_base::BaseState& msg) {
        // Update joint states from robot feedback
        // This should be adjusted according to how your robot feedback is structured
        for (int i = 0; i < NUM_DRIVE_JOINTS; i++) {
            pos_drive[i] = msg.states[i].position;
            vel_drive[i] = msg.states[i].velocity;
            eff_drive[i] = msg.states[i].output;
        }
        for (int i = 0; i < NUM_STEER_JOINTS; i++) {
            pos_steer[i] = msg.states[NUM_DRIVE_JOINTS + i].position;
            vel_steer[i] = msg.states[NUM_DRIVE_JOINTS + i].velocity;
            eff_steer[i] = msg.states[NUM_DRIVE_JOINTS + i].output;
        }
    }

    float control_frequency;

public:
    ros::NodeHandle nh;
    ros::Rate* control_rate;
    RobotHW() {
        ROS_INFO("Initializing hardware interface");

        // Register drive joints
        for (int i = 0; i < NUM_DRIVE_JOINTS; i++) {
            std::string joint_name = "rear_wheel_joint";
            hardware_interface::JointStateHandle state_handle_drive(joint_name, &pos_drive[i], &vel_drive[i], &eff_drive[i]);
            jnt_state_interface.registerHandle(state_handle_drive);

            hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joint_name), &cmd_drive[i]);
            jnt_vel_interface.registerHandle(vel_handle);
        }

        // Register steering joints
        for (int i = 0; i < NUM_STEER_JOINTS; i++) {
            std::string joint_name = "front_steer_joint";
            hardware_interface::JointStateHandle state_handle_steer(joint_name, &pos_steer[i], &vel_steer[i], &eff_steer[i]);
            jnt_state_interface.registerHandle(state_handle_steer);

            hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_name), &cmd_steer[i]);
            jnt_pos_interface.registerHandle(pos_handle);
        }

        registerInterface(&jnt_state_interface);
        registerInterface(&jnt_vel_interface);
        registerInterface(&jnt_pos_interface);

        // Initialize the publisher and subscriber
        wheel_vel_pub = nh.advertise<gra_base::BaseSetpoint>("base_setpoint", 1000);
        wheel_state_sub = nh.subscribe("base_state", 1000, &RobotHW::baseStateCallback, this);

        // Get the controller frequency
        nh.param<float>("control_frequency", control_frequency, 100.0);

        // Initialize the control rate
        control_rate = new ros::Rate(control_frequency);

        ROS_INFO("Hardware interface initialized");
    }
    void write() {
        // Publish the target wheel velocities and steering commands
        gra_base::BaseSetpoint msg;
        for (int i = 0; i < NUM_DRIVE_JOINTS; i++) {
            msg.setpoints[i].velocity = cmd_drive[i];
            // Add steering commands if needed
        }
        wheel_vel_pub.publish(msg);
    }
    void read() {
        // Read feedback from the robot and update joint states
    }
    ~RobotHW() {
        delete control_rate;
    }
};
