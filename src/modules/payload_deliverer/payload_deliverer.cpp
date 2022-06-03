#include "payload_deliverer.h"

PayloadDeliverer::PayloadDeliverer()
{
	// Close the gripper as initialization
	send_gripper_control(GripperAction::CLOSE);
}

void PayloadDeliverer::run()
{
	static vehicle_command_s vcmd{};

	while (!should_exit()) {
		hrt_abstime now = hrt_absolute_time();

		// Check for a payload drop vehicle command message
		if (_vehicle_command_sub.update(&vcmd) && (vcmd.command == vehicle_command_s::VEHICLE_CMD_DO_WINCH ||
				vcmd.command == vehicle_command_s::VEHICLE_CMD_DO_GRIPPER)) {
			PX4_INFO("Payload Drop Initiated. Gripper Opening!");
			_current_command = vcmd.command;
			_current_payload = PayloadDeployType::GRIPPER;
			send_gripper_control(GripperAction::OPEN);
			// Execute payload drop
			_last_payload_drop_time = now;
			_is_executing_payload_drop = true;
		}

		// Simulate a successful payload drop after certain period
		if (_is_executing_payload_drop &&
		    ((now - _last_payload_drop_time) > PAYLOAD_DROP_TIMEOUT_SIMULATION_US)) {
			vehicle_command_ack_s vcmd_ack{};
			vcmd_ack.timestamp = now;

			vcmd_ack.command = _current_command;
			_current_payload = PayloadDeployType::NONE;

			vcmd_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
			// Publish a acknowledgement of a successful payload drop
			_vehicle_command_ack_pub.publish(vcmd_ack);

			PX4_INFO("Payload Drop Successful Ack Sent. Gripper closing!");
			send_gripper_control(GripperAction::CLOSE);

			// Reset internal flags
			_is_executing_payload_drop = false;
		}

		px4_usleep(100_ms);
	}
}

bool PayloadDeliverer::send_gripper_control(const GripperAction gripper_action)
{
	_actuator_controls_pub.get().timestamp = hrt_absolute_time();

	switch (gripper_action) {
	case GripperAction::OPEN:
		_actuator_controls_pub.get().control[GRIPPER_ACTUATOR_CONTROLS_FUNCTION] = GRIPPER_OPEN_ACTUATOR_CONTROLS_VAL;
		break;

	case GripperAction::CLOSE:
		_actuator_controls_pub.get().control[GRIPPER_ACTUATOR_CONTROLS_FUNCTION] = GRIPPER_CLOSE_ACTUATOR_CONTROLS_VAL;
		break;

	// Unsupported Gripper Action
	default:
		return false;
	}

	return _actuator_controls_pub.update();
}

// ModuleBase related functions (below)
int PayloadDeliverer::task_spawn(int argc, char *argv[])
{
	px4_main_t entry_point = (px4_main_t)&run_trampoline;

	int _task_id = px4_task_spawn_cmd("payload_deliverer", SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT, 1500, entry_point, (char *const *)argv);

	if (_task_id < 0) {
		PX4_INFO("Payload Drop module instantiation Failed!");
		_task_id = -1;
		return -errno;

	} else {
		return PX4_OK;
	}
}

PayloadDeliverer *PayloadDeliverer::instantiate(int argc, char *argv[])
{
	return new PayloadDeliverer();
}

int PayloadDeliverer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Implementation of a simple simulated payload drop module that sends a successful payload drop
acknowledgement message after a certain timeout period.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("payload_deliverer", "auxilary");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Tests the Payload deploy & retraction sequence");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void PayloadDeliverer::test_payload()
{
	PX4_INFO("Test: Opening the Gripper!");
	send_gripper_control(GripperAction::OPEN);
	px4_usleep(5_s);
	PX4_INFO("Test: Closing the Gripper!");
	send_gripper_control(GripperAction::CLOSE);
}

int PayloadDeliverer::custom_command(int argc, char *argv[])
{
	if (argc >= 1) {
		// Tests the basic payload open / close ability
		if (strcmp(argv[0], "test") == 0) {
			get_instance() -> test_payload();
			return 0;
		}
	}

	return print_usage("Unrecognized command");
}

// Main Entry point function for PX4 NuttX System
int payload_deliverer_main(int argc, char *argv[])
{
	return PayloadDeliverer::main(argc, argv);
}
