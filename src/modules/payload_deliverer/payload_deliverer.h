/**
 * @file payload_deliverer.h
 * @author Junwoo Hwang (junwoo091400@gmail.com)
 * @brief
 *
 */
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>

#include "gripper.h"

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/actuator_controls.h>

using namespace time_literals;

/**
 * Payload Deliverer app start / stop handling function
 */
extern "C" __EXPORT int payload_deliverer_main(int argc, char *argv[]);

// Timeout to simulate a successful payload drop, after which vechicle_command_ack message gets published
static constexpr hrt_abstime GRIPPER_TIMEOUT_US = 2_s;

// Types of Payloads supported by the module
enum class PayloadDeployType {
	NONE,
	GRIPPER,
	WINCH
};

// GRIPPER retract / open values for actuator controls message
// To CLOSE, we need to EXTEND the linear servo, so we need to command positive value.
static constexpr float GRIPPER_CLOSE_ACTUATOR_CONTROLS_VAL = 0.75f;
static constexpr float GRIPPER_OPEN_ACTUATOR_CONTROLS_VAL = -1.0f;

// Vehicle command constants defined in MAV_CMD documentation of MAVLink
static constexpr int32_t GRIPPER_ACTION_RELEASE = 0;
static constexpr int32_t GRIPPER_ACTION_GRAB = 1;

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
	// Update the state of the gripper
	void update_gripper(const hrt_abstime &now, const vehicle_command_s *vehicle_command = nullptr);

	// Gripper object to handle gripper action
	Gripper _gripper;

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::PublicationData<actuator_controls_s> _actuator_controls_pub{ORB_ID(actuator_controls_1)};
};
