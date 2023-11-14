#include "ros/ros.h"
#include "gra_base/BaseParameters.h"
#include "gra_base/BaseAdaptiveState.h"
#include "gra_base/BaseState.h"

#define NUM_DRIVE_JOINTS 2
#define NUM_STEER_JOINTS 1

class HardwareParameterServer {
    private:
        gra_base::BaseParameters base_parameters_msg;
        gra_base::BaseAdaptiveState base_adaptive_state_msg;
        ros::Publisher base_parameters_pub;
        ros::Publisher base_adaptive_state_pub;
        ros::NodeHandle _nh;
        
        // Generic method for initializing a parameter
        template <typename T>
        void initParam(T& param, std::string name, T default_value) {
            _nh.param<T>(name, param, default_value);
        }
        
        // Parameters for Ackermann steering
        float wheel_radius;
        float wheel_base;
        float steering_limit;

    public:
        HardwareParameterServer() : _nh("hardware_parameter_server") {
            // Initialize the publisher
            base_parameters_pub = _nh.advertise<gra_base::BaseParameters>("/base_parameters", 1000);
            base_adaptive_state_pub = _nh.advertise<gra_base::BaseAdaptiveState>("/base_adaptive_state", 1000);

            // Initialize parameters
            initParam<float>(wheel_radius, "wheel_radius", 0.25); 
            initParam<float>(wheel_base, "wheel_base", 1.54);     // distance between front wheels and rear wheels
            initParam<float>(steering_limit, "steering_limit", 0.785398); // 45 degrees in radians

            // Initialize drive joint PID gains and other parameters
            for (int i = 0; i < NUM_DRIVE_JOINTS; i++) {
                initParam<float>(base_parameters_msg.parameters[i].p_in, "drive_joint_" + std::to_string(i + 1) + "/p_in", 35.0);
                initParam<float>(base_parameters_msg.parameters[i].i_in, "drive_joint_" + std::to_string(i + 1) + "/i_in", 0.0015);
                initParam<float>(base_parameters_msg.parameters[i].d_in, "drive_joint_" + std::to_string(i + 1) + "/d_in", 0.0);
                // Additional parameters as needed
            }

            // Initialize steering joint PID gains and other parameters
            for (int i = 0; i < NUM_STEER_JOINTS; i++) {
                initParam<float>(base_adaptive_state_msg.adaptive_states[i].i_accumulator, "steer_joint_" + std::to_string(i + 1) + "/i_accumulator_initial", 20.0);
                // Additional parameters as needed
            }

            ROS_INFO("Waiting for /base_state to be published...");
            ros::topic::waitForMessage<gra_base::BaseState>("/base_state");
            ROS_INFO("Received /base_state message, publishing parameters /base_parameters and /adaptive_state");

            base_parameters_pub.publish(base_parameters_msg);
            base_adaptive_state_pub.publish(base_adaptive_state_msg);
        }
};
