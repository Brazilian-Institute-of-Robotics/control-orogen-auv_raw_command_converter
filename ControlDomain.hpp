#ifndef AUV_RAW_COMMAND_CONVERTER_CONTROL_DOMAIN_HPP
#define AUV_RAW_COMMAND_CONVERTER_CONTROL_DOMAIN_HPP

#include <base/Eigen.hpp>
#include <vector>
#include <string>

namespace auv_raw_command_converter
{

enum ControlDomain
{
    WorldFrame, // Pose in the world frame of the robot
    WorldFrameDelta, // Position and heading are deltas in the world frame
    AlignedPoseFrame, // Position and heading in the aligned frame of the robot (Pitch and roll are not available)
    AlignedVelocity, // Velocity in m/s or rad/s in the aligned frame of the robot (Pitch and roll are not available)
    Speed, // Speed in m/s or rad/s of the actuators
    Effort, //Torque in N or m of the actuators
    Acceleration, // Acceleration in rad/s^2 or m/s^2 of the actuators
    Raw, // Raw value of the actuators
    Unset //Not a Number
};

struct LinearAngular6DDomain
{
    ControlDomain linear[3];
    ControlDomain angular[3];
    
    LinearAngular6DDomain()
    {
	linear[0] = Raw;
	linear[1] = Raw;
	linear[2] = Raw;
	angular[0] = Raw;
	angular[1] = Raw;
	angular[2] = Raw;
    }
};

struct Scaling
{
    base::Vector3d linear;
    base::Vector3d angular;
    
    Scaling()
    {
	linear = base::Vector3d::Ones();
	angular = base::Vector3d::Ones();
    }
};

struct InputDeviceConfig
{
    std::string device_identifier;
    base::MatrixXd axis_mapping;
    
    bool isValid()
    {
	if(axis_mapping.rows() == 6)
		return true;
	
	return false;
    }
};

}

#endif
