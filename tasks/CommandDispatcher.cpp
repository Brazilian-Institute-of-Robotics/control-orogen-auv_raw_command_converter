/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CommandDispatcher.hpp"

using namespace auv_raw_command_converter;

CommandDispatcher::CommandDispatcher(std::string const& name)
    : CommandDispatcherBase(name)
{
}

CommandDispatcher::CommandDispatcher(std::string const& name, RTT::ExecutionEngine* engine)
    : CommandDispatcherBase(name, engine)
{
}

CommandDispatcher::~CommandDispatcher()
{
}


bool CommandDispatcher::readInputCommand()
{
    // check if port is connected
    if(!_cmd_in.connected())
    {
        if(state() != WAIT_FOR_CONNECTED_INPUT_PORT){
            error(WAIT_FOR_CONNECTED_INPUT_PORT);
        }
        return false;
    }
    
    // read new command
    RTT::FlowStatus status = _cmd_in.readNewest(command);
    if(status == RTT::NoData)
    {
	if(state() != WAIT_FOR_INPUT){
	    error(WAIT_FOR_INPUT);
	}
	return false;
    }
    else if(status == RTT::NewData)
    {
	new_command = true;
	last_cmd_time = base::Time::now();
	return true;
    }
    
    // check for timeout
    if(_timeout_in.value() != 0 && (base::Time::now() - last_cmd_time).toSeconds() > _timeout_in.value())
    {
	if(state() != TIMEOUT){
	    error(TIMEOUT);
	}
    }
    
    return false;
}

void CommandDispatcher::writeOutputCommands()
{
    base::LinearAngular6DCommand world_cmd;
    base::LinearAngular6DCommand aligned_position_cmd;
    base::LinearAngular6DCommand aligned_velocity_cmd;
    base::LinearAngular6DCommand acceleration_cmd;
    
    // handle linear part
    for (unsigned i = 0; i < 3; i++) {
        switch (domain.linear[i]) {
        case WorldFrame:
            world_cmd.time = command.time;
            world_cmd.linear[i] = command.linear[i];
            break;
        case WorldFrameDelta:
            world_cmd.time = command.time;
            if (std::abs(command.linear[i]) < 1.0e-4 && !base::isNaN(last_delta_world_command.linear[i])) {
                if (new_delta_command[i]) {
                    last_delta_world_command.linear[i] = pose_sample.position[i];
                    new_delta_command[i] = false;
                }
                world_cmd.linear[i] = last_delta_world_command.linear[i];
            } else {
                world_cmd.linear[i] = pose_sample.position[i] + command.linear[i];
                last_delta_world_command.linear[i] = world_cmd.linear[i];
                new_delta_command[i] = true;
            }
            break;
        case AlignedPoseFrame:
            aligned_position_cmd.time = command.time;
            aligned_position_cmd.linear[i] = command.linear[i];
            break;
        case AlignedVelocity:
            aligned_velocity_cmd.time = command.time;
            aligned_velocity_cmd.linear[i] = command.linear[i];
            break;
        case Speed:
        case Effort:
        case Acceleration:
        case Raw:
            acceleration_cmd.time = command.time;
            acceleration_cmd.linear[i] = command.linear[i];
            break;
        case Nan:
            acceleration_cmd.time = command.time;
            acceleration_cmd.linear[i] = base::unset<double>();
            break;
        }
    }
    
    // handle angular part
    for (unsigned i = 0; i < 3; i++) {
        switch (domain.angular[i]) {
        case WorldFrame:
            world_cmd.time = command.time;
            world_cmd.angular[i] = command.angular[i];
            break;
        case WorldFrameDelta: {
            world_cmd.time = command.time;
            if (std::abs(command.angular[i]) < 1.0e-4 && !base::isNaN(last_delta_world_command.angular[i])) {
                if (new_delta_command[3 + i]) {
                    last_delta_world_command.linear[i] = pose_sample.position[i];
                    base::Vector3d euler = base::getEuler(pose_sample.orientation);
                    last_delta_world_command.angular[i] = euler[2 - i];
                    new_delta_command[3 + i] = false;
                }
                world_cmd.angular[i] = last_delta_world_command.angular[i];
            } else {
                base::Orientation target_orientation = pose_sample.orientation * Eigen::AngleAxisd(command.angular[i], Eigen::Vector3d::Unit(i));
                base::Vector3d euler = base::getEuler(target_orientation);
                world_cmd.angular[i] = euler[2 - i];
                last_delta_world_command.angular[i] = world_cmd.angular[i];
                new_delta_command[3 + i] = true;
            }
            break;
        }
        case AlignedPoseFrame:
            aligned_position_cmd.time = command.time;
            aligned_position_cmd.angular[i] = command.angular[i];
            break;
        case AlignedVelocity:
            aligned_velocity_cmd.time = command.time;
            aligned_velocity_cmd.angular[i] = command.angular[i];
            break;
        case Speed:
        case Effort:
        case Acceleration:
        case Raw:
            acceleration_cmd.time = command.time;
            acceleration_cmd.angular[i] = command.angular[i];
            break;
        case Nan:
            acceleration_cmd.time = command.time;
            acceleration_cmd.angular[i] = base::unset<double>();
            break;
        }
    }
    
    // write commands to ports
    if(!world_cmd.time.isNull())
	_world_command.write(world_cmd);
    if(!aligned_position_cmd.time.isNull())
	_aligned_position_command.write(aligned_position_cmd);
    if(!aligned_velocity_cmd.time.isNull())
	_aligned_velocity_command.write(aligned_velocity_cmd);
    if(!acceleration_cmd.time.isNull())
	_acceleration_command.write(acceleration_cmd);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CommandDispatcher.hpp for more detailed
// documentation about them.

bool CommandDispatcher::configureHook()
{
    if (! CommandDispatcherBase::configureHook())
        return false;
    
    new_command = false;
    domain = _control_domains.value();
    expecting_pose_samples = false;
    last_delta_world_command.linear = base::NaN<double>() * Eigen::Vector3d::Ones();
    last_delta_world_command.angular = base::NaN<double>() * Eigen::Vector3d::Ones();
    for(unsigned i = 0; i < 6; i++)
        new_delta_command[i] = false;
    
    // check if pose samples are needed
    for(unsigned i = 0; i < 3; i++)
    {
	if(domain.linear[i] == WorldFrameDelta)
	{
	    expecting_pose_samples = true;
	}
    }
    for(unsigned i = 0; i < 3; i++)
    {
	if(domain.angular[i] == WorldFrameDelta)
	{
	    expecting_pose_samples = true;
	}
    }
    
    // check for misconfiguration
    for(unsigned i = 0; i < 2; i++)
    {
	if(domain.angular[i] == AlignedPoseFrame ||
	    domain.angular[i] == AlignedVelocity)
	{
	    RTT::log(RTT::Error) << "Misconfiguration: Roll and pitch can't be controlled in the aligned domains!" << RTT::endlog();
	    return false;
	}
    }
    
    return true;
}
bool CommandDispatcher::startHook()
{
    if (! CommandDispatcherBase::startHook())
        return false;

    if(expecting_pose_samples && !_pose_samples.connected())
        RTT::log(RTT::Warning) << "Pose samples are required, but currently not connected!" << RTT::endlog();

    return true;
}
void CommandDispatcher::updateHook()
{
    CommandDispatcherBase::updateHook();
    
    if(expecting_pose_samples)
    {
        RTT::FlowStatus status = _pose_samples.readNewest(pose_sample);
	if(status == RTT::NoData)
	{
	    if(state() != WAIT_FOR_POSE_SAMPLE)
		error(WAIT_FOR_POSE_SAMPLE);
	}
        else if(status == RTT::NewData)
        {
            if(!pose_sample.hasValidPosition() || !pose_sample.hasValidOrientation())
                exception(POSE_SAMPLE_INVALID);
        }
    }
    
    if(readInputCommand() || new_command)
    {
	new_command = false;
	
	writeOutputCommands();
    }
    
}
void CommandDispatcher::errorHook()
{
    CommandDispatcherBase::errorHook();
    
    States s = state();
    if(s == WAIT_FOR_POSE_SAMPLE)
    {
        if(_pose_samples.readNewest(pose_sample) != RTT::NoData)
            recover();
    }
    else if (s == TIMEOUT ||
        s == WAIT_FOR_CONNECTED_INPUT_PORT ||
        s == WAIT_FOR_INPUT)
    {
        if (readInputCommand())
            recover();
    }
}
void CommandDispatcher::stopHook()
{
    CommandDispatcherBase::stopHook();
}
void CommandDispatcher::cleanupHook()
{
    CommandDispatcherBase::cleanupHook();
}
