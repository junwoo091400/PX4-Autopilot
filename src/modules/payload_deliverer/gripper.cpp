#include "gripper.h"

Gripper::Gripper()
{
	// Do nothing
}

void Gripper::init(const GripperConfig &config)
{
	_config = config;

	switch (_config.type) {
		case GripperConfig::GripperType::SERVO:
			break;

		case GripperConfig::GripperType::UNDEFINED:
		// FALL THROUGH
		default:
			_valid = false;
			return;
	}

	switch (_config.sensor) {
		case GripperConfig::GripperSensorType::ENCODER:
			// Handle encoder sensor setup
			_has_feedback_sensor = true;
			break;

		case GripperConfig::GripperSensorType::NONE:
		default:
			// No feedback sensor
			_has_feedback_sensor = false;
			break;
	}

	_timeout_us = _config.timeout_us;

	// We have valid gripper type & sensor configuration
	_valid = true;

	// Grab in the initialization stage
	grab();
}

void Gripper::grab()
{
	publish_gripper_command(gripper_s::COMMAND_GRAB);
	_state = GripperState::GRABBING;
	_last_command_time = hrt_absolute_time();
}

void Gripper::release()
{
	publish_gripper_command(gripper_s::COMMAND_RELEASE);
	_state = GripperState::RELEASING;
	_last_command_time = hrt_absolute_time();
}

void Gripper::update()
{
	const bool command_timed_out = (hrt_elapsed_time(&_last_command_time) >= _timeout_us);

	// Handle transition from intermediate state to definite state
	switch (_state) {
		case GripperState::GRABBING:
			if (_has_feedback_sensor) {
				// Handle feedback sensor input, return true for now (not supported)
				_state = GripperState::GRABBED;
				break;
			}

			if (command_timed_out) {
				_state = GripperState::GRABBED;
			}
			break;

		case GripperState::RELEASING:
			if (_has_feedback_sensor) {
				// Handle feedback sensor input, return true for now (not supported)
				_released_state_cache = true;
				_state = GripperState::RELEASED;
				break;
			}

			if (command_timed_out) {
				_released_state_cache = true;
				_state = GripperState::RELEASED;
			}
			break;

		default:
			// DO NOTHING
			break;
	}
}

void Gripper::publish_gripper_command(const int8_t gripper_command)
{
	gripper_s gripper{};
	gripper.timestamp = hrt_absolute_time();
	gripper.command = gripper_command;
	_gripper_pub.publish(gripper);
}
