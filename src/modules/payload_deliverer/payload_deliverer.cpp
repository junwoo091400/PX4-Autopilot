#include "payload_deliverer.h"

PayloadDeliverer::PayloadDeliverer()
{
	// Configure the Gripper
	GripperConfig config{};
	config.type = GripperConfig::GripperType::SERVO;
	config.sensor = GripperConfig::GripperSensorType::NONE;
	config.timeout_us = GRIPPER_TIMEOUT_US;
	_gripper.init(config);
	if (!_gripper.is_valid()) {
		PX4_INFO("Gripper object initialization invalid!");
	}
}

void PayloadDeliverer::run()
{
	static vehicle_command_s vcmd{};

	while (!should_exit()) {
		hrt_abstime now = hrt_absolute_time();

		if (_vehicle_command_sub.update(&vcmd)) {
			update_gripper(now, &vcmd);

		} else {
			update_gripper(now);

		}

		px4_usleep(100_ms);
	}
}

void PayloadDeliverer::update_gripper(const hrt_abstime &now,  const vehicle_command_s *vehicle_command)
{
	_gripper.update();

	// Process successful release command acknowledgement
	if (_gripper.released_readOnce()) {
		vehicle_command_ack_s vcmd_ack{};
		vcmd_ack.timestamp = now;

		vcmd_ack.command = vehicle_command_s::VEHICLE_CMD_DO_GRIPPER;
		vcmd_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
		_vehicle_command_ack_pub.publish(vcmd_ack);

		PX4_INFO("Payload Drop Successful Ack Sent!");
	}

	// If there's no vehicle command to process, just return
	if (vehicle_command == nullptr) {
		return;
	}

	// Process if we received DO_GRIPPER vehicle command
	if (vehicle_command -> command == vehicle_command_s::VEHICLE_CMD_DO_GRIPPER) {
		PX4_INFO("Gripper command received!");
		const int32_t gripper_action = *(int32_t*)&vehicle_command -> param2; // Convert the action to integer
		switch (gripper_action) {
			case GRIPPER_ACTION_GRAB:
				_gripper.grab();
				break;

			case GRIPPER_ACTION_RELEASE:
				_gripper.release();
				break;
		}
	}
}

// ModuleBase related functions (below)
int PayloadDeliverer::task_spawn(int argc, char *argv[])
{
	px4_main_t entry_point = (px4_main_t)&run_trampoline;

	int _task_id = px4_task_spawn_cmd("payload_deliverer", SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT, 1500, entry_point, (char *const *)argv);

	if (_task_id < 0) {
		PX4_INFO("Payload Deliverer module instantiation Failed!");
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
Implementation of a simple payload deliverer module that processes gripper and winch control,
activated by vehicle commands

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("payload_deliverer", "auxilary");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Tests the Gripper's release & grabbing sequence");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void PayloadDeliverer::test_payload()
{
	PX4_INFO("Test: Opening the Gripper!");
	_gripper.release();
	px4_usleep(5_s);
	PX4_INFO("Test: Closing the Gripper!");
	_gripper.grab();
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
