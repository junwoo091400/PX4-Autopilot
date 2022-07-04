#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/gripper.h>

using namespace time_literals;

/**
 * Griper class to handle functionality of using a Gripper
 */

// Configuration
typedef struct GripperConfig {
	typedef enum GripperType {
		UNDEFINED = -1,
		SERVO = 0,
	} GripperType;

	GripperType type{GripperType::UNDEFINED};

	// Gripper state feedback sensor type
	typedef enum GripperSensorType {
		NONE = -1,
		ENCODER,
	} GripperSensorType;

	GripperSensorType sensor{GripperSensorType::NONE};

	hrt_abstime timeout_us{0};

} GripperConfig;

// State definitino
typedef enum GripperState {
		IDLE = 0,
		GRABBING,
		GRABBED,
		RELEASING,
		RELEASED
} GripperState;

class Gripper
{
public:
	// Constructor
	Gripper();

	// Initialize the gripper
	void init(const GripperConfig &config);

	// Actuate the gripper to the grab position
	void grab();

	// Actuate the gripper to release position
	void release();

	// Update gripper state
	void update();

	// Returns true if in grabbed position either sensed or timeout based
	bool grabbed() { return _state == GripperState::GRABBED; }

	// Returns true if in released position either sensed or timeout based
	bool released() { return _state == GripperState::RELEASED; }

	// Returns true only once after the state transitioned into released
	// Used for sending vehicle command ack only when the gripper actually releases
	// in payload deliverer
	bool released_readOnce() {
		if (_released_state_cache) {
			_released_state_cache = false;
			return true;
		} else {
			return false;
		}
	}

	// Returns true if Gripper output function is assigned properly
	bool is_valid() const { return _valid; }

private:
	// Internal function to send gripper command via uORB
	void publish_gripper_command(const int8_t gripper_command);

	GripperConfig _config{};

	// Internal settings
	bool _valid{false};
	bool _has_feedback_sensor{false};
	hrt_abstime _timeout_us{0};

	// Internal state
	GripperState _state{GripperState::IDLE};
	hrt_abstime _last_command_time{0};
	bool _released_state_cache{false}; //

	uORB::Publication<gripper_s> _gripper_pub{ORB_ID(gripper)};
};
