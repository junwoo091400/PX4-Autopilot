/**
 * @file payload_deliverer.h
 * @author Junwoo Hwang (junwoo091400@gmail.com)
 * @brief
 *
 */
#include <drivers/drv_hrt.h>

#include <px4_platform_common/module.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/vehicle_command.h>

#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/actuator_controls.h>

using namespace time_literals; // To use time literals like "100_ms"

/**
 * Payload Deliverer app start / stop handling function
 */
extern "C" __EXPORT int payload_deliverer_main(int argc, char *argv[]);

// Timeout to simulate a successful payload drop, after which vechicle_command_ack message gets published
static constexpr hrt_abstime PAYLOAD_DROP_TIMEOUT_SIMULATION_US = 5000000;

// Types of Payloads supported by the module
enum class PayloadDeployType {
	NONE,
	GRIPPER,
	WINCH
};

// GRIPPER Actuator Controls Information
static constexpr uint8_t GRIPPER_ACTUATOR_CONTROLS_GROUP = 1;
static constexpr uint8_t GRIPPER_ACTUATOR_CONTROLS_FUNCTION = 4;

// GRIPPER retract / open values for actuator controls message
// To CLOSE, we need to EXTEND the linear servo, so we need to command positive value.
static constexpr float GRIPPER_CLOSE_ACTUATOR_CONTROLS_VAL = 0.75f;
static constexpr float GRIPPER_OPEN_ACTUATOR_CONTROLS_VAL = -1.0f;

enum class GripperAction {
	NONE,
	OPEN,
	CLOSE
};

/**
 * @brief Payload Deliverer Module
 *
 * Activates a winch / gripper when the DO_WINCH or DO_GRIPPER vehicle command is received,
 * after which the vehicle_command_ack command gets sent upon successful confirmation
 * of the payload deployment.
 *
 * This module communicates with the Navigator which handles publishing such vehicle commands.
 */
class PayloadDeliverer : public ModuleBase<PayloadDeliverer>
{
public:
	PayloadDeliverer();

	/** @see ModuleBase **/
	static int task_spawn(int argc, char *argv[]);
	static PayloadDeliverer *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Main run function that runs on a thread
	 */
	void run() override;

	void test_payload();

private:
	bool send_gripper_control(const GripperAction gripper_action);

	PayloadDeployType _current_payload{PayloadDeployType::NONE};
	uint16_t _current_command{0}; 		// Current vehicle command for payload deployment that is being executed

	bool _is_executing_payload_drop{false}; // Indicates whether the module is in process of dropping the payload
	hrt_abstime _last_payload_drop_time{0}; // [us] Last time that payload drop was commanded

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::PublicationData<actuator_controls_s> _actuator_controls_pub{ORB_ID(actuator_controls_1)};
};
