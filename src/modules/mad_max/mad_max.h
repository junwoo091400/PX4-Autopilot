/**
 * @file mad_max.h
 * @author Junwoo Hwang (junwoo091400@gmail.com)
 *
 * Simple module that prints out different messages depending on the current
 * speed of the vehicle.
 *
 * The inspiration comes from MAD-MAX, where they love going speedy ;)
 *
 */

// For Hardware Time related functions like hrt_abs_time()
#include <drivers/drv_hrt.h>

// General Math library functions (e.g. Matrix library, etc)
#include <mathlib/mathlib.h>

#include <px4_platform_common/module.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_local_position.h>

using namespace matrix;		// To use Vector2f
using namespace time_literals;	// To use time literals like "100_ms"

/**
 * Module start / stop handling function that gets called by NuttX
 */
extern "C" __EXPORT int mad_max_main(int argc, char *argv[]);

class MadMax : public ModuleBase<MadMax>
{
public:
	// Default Constructor
	MadMax();

	// Methods requried by the ModuleBase inheritance
	static int task_spawn(int argc, char *argv[]);
	static MadMax *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/**
	 * @brief Main run function that runs on a thread
	 */
	void run() override;

private:
	// Subscription to the 'vehicle_local_position' uORB topic
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
};
