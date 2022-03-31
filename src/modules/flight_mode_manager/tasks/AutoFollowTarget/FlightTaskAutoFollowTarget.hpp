/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file FlightTaskAutoFollowTarget.hpp
 *
 * Flight task for autonomous, gps driven follow-me mode.
 *
 * @author Alessandro Simovic <potaito-dev@protonmail.com>
 * @author Junwoo Hwang <junwoo091400@gmail.com>
 */

#pragma once

#include "FlightTaskAuto.hpp"
#include "follow_target_estimator/TargetEstimator.hpp"
#include "Sticks.hpp"

#include <parameters/param.h>
#include <mathlib/mathlib.h>

#include <uORB/Subscription.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/follow_target_status.h>
#include <uORB/topics/follow_target_estimator.h>
#include <uORB/topics/gimbal_manager_set_attitude.h>

#include <lib/mathlib/math/filter/second_order_reference_model.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/helper_functions.hpp>
#include <motion_planning/VelocitySmoothing.hpp>

// [m] Minimum safety altitude above home (or bottom distance sensor)
// underneath which the flight task will stop moving horizontally
static constexpr float MINIMUM_SAFETY_ALTITUDE = 1.0f;

// [m/s] Vertical ascent speed when the drone detects that it is too close
// to the ground (below MINIMUM_SAFETY_ALTITUDE).
static constexpr float EMERGENCY_ASCENT_SPEED = 0.5f;

// [m] max vertical deviation from position setpoint, above
// which no horizontal control is done
static constexpr float ALT_ACCEPTANCE_THRESHOLD = 3.0f;

// [m] Minimum distance between drone and target for the drone to do any yaw control.
static constexpr float MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL = 1.0f;

// Second order filter parameter for target position filter
static constexpr float TARGET_POS_VEL_FILTER_NATURAL_FREQUENCY = 1.0f; // [rad/s]
static constexpr float TARGET_POS_VEL_FILTER_DAMPING_RATIO = 0.7071;

// [us] If the target estimator output isn't updated longer than this, reset pose filter.
static constexpr uint64_t TARGET_ESTIMATOR_TIMEOUT_US = 1500000UL;

// [m/s] Speed deadzone for which, under this velocity, the target orientation
// tracking will freeze, since orientation can be noisy in low velocities
static constexpr float TARGET_SPEED_DEADZONE_FOR_ORIENTATION_TRACKING = 1.0;

// If we use the raw MPC_XY_VEL_MAX and MPC_ACC_HOR_MAX parameters for setting
// orbit angle trajectory constraints, it will generate the maximum constraints-hitting
// trajectory that the MPC can handle. But considering target's movements, this trajectory
// will almost always be infeasible & too aggressive. So we apply this ratio for the
// constraints to create feasible orbit angle trajectory.
static constexpr float ORBIT_ANGLE_TRAJECTORY_GENERATOR_CONSTRAINTS_RATIO = 0.5f;

// [m/s] Speed with which the follow distance will be adjusted by when commanded with deflection via RC command
static constexpr float FOLLOW_DISTANCE_USER_ADJUST_SPEED = 2.0;
// [m] Maximum follow distance that can be set by user's RC adjustment
static constexpr float FOLLOW_DISTANCE_MAX = 100.f;

// [m/s] Speed with which the follow height will be adjusted by when commanded with deflection via RC command
static constexpr float FOLLOW_HEIGHT_USER_ADJUST_SPEED = 1.5;
// [m] Maximum follow height that can be set by user's RC adjustment
static constexpr float FOLLOW_HEIGHT_MAX = 100.f;

// [rad/s] Angular rate with which the follow distance will be adjusted by when commanded with full deflection via RC command
static constexpr float FOLLOW_ANGLE_USER_ADJUST_SPEED = 1.5;

// [seconds] Arbitrary time window constant that gets multiplied to user adjustment speed, to calculate the
// 'acceptable' error in orbit angle / height and distance, to which if the difference between the setpoint
// and actual state is smaller than this error, RC adjustments get applied.
// This is introduced to prevent setpoint adjustments becoming too diverged from the vehicle's actual position
static constexpr float USER_ADJUSTMENT_ERROR_TIME_WINDOW = 0.5f;

// Deadzone on both +/- direction for normalized stick input (-1, +1) where user adjustment will be ignored
static constexpr float USER_ADJUSTMENT_DEADZONE = 0.1f;


class FlightTaskAutoFollowTarget : public FlightTask
{
public:
	FlightTaskAutoFollowTarget();
	virtual ~FlightTaskAutoFollowTarget();

	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool update() override;

	// Override parameter update function to check when Follow Target properties are changed
	void updateParams() override;

protected:
	// Follow Perspectives set by the parameter FLW_TGT_FS
	enum FollowPerspective
	{
		FOLLOW_PERSPECTIVE_NONE,
		FOLLOW_PERSPECTIVE_BEHIND,
		FOLLOW_PERSPECTIVE_FRONT,
		FOLLOW_PERSPECTIVE_FRONT_RIGHT,
		FOLLOW_PERSPECTIVE_FRONT_LEFT,
		FOLLOW_PERSPECTIVE_MID_RIGHT,
		FOLLOW_PERSPECTIVE_MID_LEFT,
		FOLLOW_PERSPECTIVE_BEHIND_RIGHT,
		FOLLOW_PERSPECTIVE_BEHIND_LEFT,
		FOLLOW_PERSPECTIVE_MIDDLE_FOLLOW,
		FOLLOW_PERSPECTIVE_INVALID  // Leave this as last!
	};

	// Angles [deg] for the different follow-me perspectives
	enum {
		FOLLOW_PERSPECTIVE_BEHIND_ANGLE_DEG = 180,
		FOLLOW_PERSPECTIVE_FRONT_ANGLE_DEG = 0,
		FOLLOW_PERSPECTIVE_FRONT_RIGHT_ANGLE_DEG = 45,
		FOLLOW_PERSPECTIVE_FRONT_LEFT_ANGLE_DEG = -45,
		FOLLOW_PERSPECTIVE_MID_RIGHT_ANGLE_DEG = 90,
		FOLLOW_PERSPECTIVE_MID_LEFT_ANGLE_DEG = -90,
		FOLLOW_PERSPECTIVE_BEHIND_RIGHT_ANGLE_DEG = 135,
		FOLLOW_PERSPECTIVE_BEHIND_LEFT_ANGLE_DEG = -135
	};

	// Follow Altitude modes set by the parameter FLW_TGT_ALT_M
	enum FollowAltitudeMode
	{
		FOLLOW_ALTITUDE_MODE_CONSTANT,
		FOLLOW_ALTITUDE_MODE_TRACK_TERRAIN,
		FOLLOW_ALTITUDE_MODE_TRACK_TARGET
	};

	/**
	 * Update the Follow height based on RC commands
	 *
	 * If the drone's height error to the setpoint is within the an user adjustment error time window
	 * follow_height will be adjusted with a speed proportional to user RC command
	 *
	 * @param sticks Sticks object to get RC commanded values for adjustments
	 */
	void update_rc_adjusted_follow_height(const Sticks &sticks);

	/**
	 * Update the Follow distance based on RC commands
	 *
	 * If the drone's distance error to the setpoint is within the an user adjustment error time window
	 * follow_distance will be adjusted with a speed proportional to user RC command
	 *
	 * @param sticks Sticks object to get RC commanded values for adjustments
	 * @param drone_to_target_vector Tracked follow distance variable reference which will be updated to the new value
	 */
	void update_rc_adjusted_follow_distance(const Sticks &sticks, const Vector2f &drone_to_target_vector);

	/**
	 * Update the Follow angle based on RC commands
	 *
	 * If the drone's orbit angle in relation to target is within the an user adjustment error time window
	 * away from the orbit angle setpoint, follow_angle will be adjusted with a speed proportional to user RC command
	 *
	 * @param sticks Sticks object to get RC commanded values for adjustments
	 * @param measured_angle Measured current drone's orbit angle around the target (depends on tracked target orientation for reference)
	 * @param tracked_orbit_angle_setpoint Rate constrained orbit angle setpoint value from last command
	 */
	void update_rc_adjusted_follow_angle(const Sticks &sticks, const float measured_orbit_angle, const float tracked_orbit_angle_setpoint);

	/**
	 * Update the Second Order Target Position + Velocity Filter to track kinematically feasible target position and velocity
	 *
	 * @param follow_target_estimator Received value from alpha-beta-gamma target estimator filter output
	 */
	void update_target_position_velocity_filter(const follow_target_estimator_s &follow_target_estimator);

	/**
	 * Calculate the tracked target orientation and overwrite the tracked target orientation if necessary
	 *
	 * @param target_velocity Filtered Target velocity from which we will calculate target orientation
	 * @param current_target_orientation  Tracked target orientation value that will be over-written with new orientation
	 */
	void update_target_orientation(const Vector2f &target_velocity, float &current_target_orientation);

	/**
	 * Returns the orbit angle setpoint, taking into account the maximal orbit tangential speed.
	 * While setting the orbital velocity setpoint vector for the according position setpoint
	 *
	 * @param target_orientation Tracked target orientation
	 * @param previous_orbit_angle_setpoint Previous orbit angle setpoint
	 *
	 * @return Angle [rad] Next feasible orbit angle setpoint
	 */
	float update_orbit_angle_trajectory(const float target_orientation, const float previous_orbit_angle_setpoint);

	/**
	 * Returns the orbit angle setpoint, taking into account the maximal orbit tangential speed.
	 *
	 * @param orbit_angle_setpoint Orbit angle setpoint
	*/
	Vector2f get_orbit_tangential_velocity(const float orbit_angle_setpoint) const;

	/**
	 * Calculates desired drone position taking into account orbit angle and the follow target altitude mode
	 *
	 * @param target_position Tracked target position Vector3f reference
	 * @param orbit_angle_setpoint Current orbit angle setpoint around the target
	 *
	 * @return Position [m,m,m] Final position setpoint for the drone
	 */
	Vector3f calculate_desired_drone_position(const Vector3f &target_position, const float orbit_angle_setpoint) const;

	/**
	 * Calculate the gimbal height offset to the target to calculate the pitch angle command
	 *
	 * @param altitude_mode Current Follow Target Altitude mode
	 * @param target_pos_z Target's local position z value to use for 3D tracking
	 *
	 * @return Height [m] Difference between the target and the drone
	 */
	float calculate_gimbal_height(const FollowAltitudeMode altitude_mode, const float target_pos_z) const;

	/**
	 * Publishes gimbal control command to track the target, given xy distance and z (height) difference
	 *
	 * @param xy_distance Horizontal distance to target
	 * @param z_distance Vertical distance to target
	 *
	 * @return Angle [rad] Gimbal pitch setpoint, for logging in follow_target_status uORB message
	 */
	float point_gimbal_at(const float xy_distance, const float z_distance);

	/**
	 * Get the current follow-me perspective angle setting from PX4 parameters
	 *
	 * @param follow_perspective value of the parameter FLW_TGT_FS
	 * @return Angle [deg] from which the drone should view the target while following it, with zero degrees indicating the target's 12 o'clock
	 */
	float get_follow_angle_setting_deg(const FollowPerspective follow_perspective) const;

	// Sticks object to read in stick commands from the user
	Sticks _sticks;

	// Estimator for target position and velocity
	TargetEstimator _target_estimator;
	follow_target_estimator_s _follow_target_estimator;

	// Last target estimator timestamp to handle timeout filter reset
	uint64_t _last_valid_target_estimator_timestamp{0};

	// Second Order Filter to calculate kinematically feasible target position
	SecondOrderReferenceModel<matrix::Vector3f> _target_position_velocity_filter;

	// Internally tracked Follow Target characteristics, to allow RC control input adjustments
	float _follow_distance{8.0f}; // [m]
	float _follow_height{10.0f}; // [m]
	float _follow_angle_rad{0.0f}; // [rad]

	// Tracked estimate of target's course (where velocity vector is pointing)
	float _target_course_rad{0.0f};
	// Tracked orbit angle setpoint
	float _orbit_angle_setpoint_rad{0.0f};

	// Angular acceleration limited orbit angle setpoint curve trajectory generator
	VelocitySmoothing _orbit_angle_traj_generator;

	// Yaw setpoint filter to remove jitter-ness
	AlphaFilter<float> _yaw_setpoint_filter;

	// Variable to remember the home position's z coordinate, which will be baseline for the position z setpoint
	float _home_position_z;

	DEFINE_PARAMETERS_CUSTOM_PARENT(
		FlightTask,
		(ParamFloat<px4::params::FLW_TGT_HT>) _param_flw_tgt_ht,
		(ParamFloat<px4::params::FLW_TGT_DST>) _param_flw_tgt_dst,
		(ParamInt<px4::params::FLW_TGT_FS>) _param_flw_tgt_fs,
		(ParamInt<px4::params::FLW_TGT_ALT_M>) _param_flw_tgt_alt_m,
		(ParamFloat<px4::params::FLW_TGT_YAW_T>) _param_ft_yaw_t,
		(ParamFloat<px4::params::MPC_ACC_HOR_MAX>) _param_mpc_acc_hor_max,
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max
	)

	uORB::Subscription _follow_target_estimator_sub{ORB_ID(follow_target_estimator)};

	uORB::PublicationMulti<follow_target_status_s> _follow_target_status_pub{ORB_ID(follow_target_status)};
	uORB::PublicationMulti<gimbal_manager_set_attitude_s> _gimbal_manager_set_attitude_pub{ORB_ID(gimbal_manager_set_attitude)};
};
