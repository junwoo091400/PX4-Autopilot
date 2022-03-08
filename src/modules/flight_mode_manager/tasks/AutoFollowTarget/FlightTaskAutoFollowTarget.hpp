/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 */

#pragma once

#include "FlightTaskAuto.hpp"
#include "follow_target_estimator/TargetEstimator.hpp"

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

// Minimum distance between drone and target for the drone to do any yaw control.
static constexpr float MINIMUM_DISTANCE_TO_TARGET_FOR_YAW_CONTROL = 1.0f;

// Minimum safety altitude above home (or bottom distance sensor)
// underneath which the flight task will stop moving horizontally
static constexpr float MINIMUM_SAFETY_ALTITUDE = 1.0f;

// [m] max vertical deviation from position setpoint, above
// which no horizontal control is done
static constexpr float ALT_ACCEPTANCE_THRESHOLD = 3.0f;

// Vertical ascent speed when the drone detects that it
// is too close to the ground (below MINIMUM_SAFETY_ALTITUDE)
static constexpr float EMERGENCY_ASCENT_SPEED = 0.2f;

// [s] If the target estimator output isn't updated longer than this, reset pose filter.
static constexpr float TARGET_ESTIMATOR_TIMEOUT_SECONDS = 1.5;

// Second order filter parameter for target position filter
static constexpr float TARGET_POSE_FILTER_NATURAL_FREQUENCY = 1.0f; // [rad/s]
static constexpr float TARGET_POSE_FILTER_DAMPING_RATIO = 0.7071;

// [m/s] Velocity deadzone for which, under this velocity, the target orientation
// tracking will freeze, since orientation can be noisy in low velocities
static constexpr float TARGET_VELOCITY_DEADZONE_FOR_ORIENTATION_TRACKING = 0.5;

// [m/s] Velocity limit to limit orbital angular rate depending on follow distance
static constexpr float MAXIMUM_TANGENTIAL_ORBITING_SPEED = 5.0;

// Yaw setpoint filter to avoid jitter-ness, which can happen because the yaw is
// calculated off of position offset between target & drone, which updates very frequently.
static constexpr float YAW_SETPOINT_FILTER_ALPHA = 1.5;


class FlightTaskAutoFollowTarget : public FlightTask
{
public:
	FlightTaskAutoFollowTarget();
	virtual ~FlightTaskAutoFollowTarget();

	bool activate(const vehicle_local_position_setpoint_s &last_setpoint) override;
	bool update() override;

protected:
	enum {
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
		FOLLOW_PERSPECTIVE_FRONT_LEFT_ANGLE_DEG = 315,
		FOLLOW_PERSPECTIVE_MID_RIGHT_ANGLE_DEG = 90,
		FOLLOW_PERSPECTIVE_MID_LEFT_ANGLE_DEG = 270,
		FOLLOW_PERSPECTIVE_BEHIND_RIGHT_ANGLE_DEG = 135,
		FOLLOW_PERSPECTIVE_BEHIND_LEFT_ANGLE_DEG = 225
	};

	enum {
		FOLLOW_ALTITUDE_MODE_CONSTANT,
		FOLLOW_ALTITUDE_MODE_TRACK_TARGET
	};

	enum {
		FOLLOW_GIMBAL_MODE_2D,
		FOLLOW_GIMBAL_MODE_2D_WITH_TERRAIN,
		FOLLOW_GIMBAL_MODE_3D
	};

	void update_target_pose_filter(follow_target_estimator_s follow_target_estimator);
	float update_target_orientation(Vector2f target_velocity);
	float update_orbit_angle(float target_orientation, float fllow_angle, float max_orbital_rate);
	Vector3f calculate_drone_desired_position(Vector3f target_position);

	void point_gimbal_at(float xy_distance, float z_distance);

	/**
	 * Get the current follow-me perspective setting from PX4 parameters
	 *
	 * @param param_nav_ft_fs value of the parameter NAV_FT_FS
	 * @return Angle [deg] from which the drone should view the target while following it, with zero degrees indicating the target's 12 o'clock
	 */
	float update_follow_me_angle_setting(int param_nav_ft_fs) const;

	// Estimator for target position and velocity
	TargetEstimator _target_estimator;
	follow_target_estimator_s _follow_target_estimator;

	// Last target estimator timestamp to handle timeout filter reset
	uint16_t _last_target_estimator_timestamp{0};

	// Second Order Filter to calculate kinematically feasible target position
	SecondOrderReferenceModel<matrix::Vector3f> _target_pose_filter;

	// Estimated (Filtered) target orientation setpoint
	float _target_orientation_rad{0.0f};

	// Follow angle is defined with 0 degrees following from front, and then clockwise rotation
	float _follow_angle_rad{0.0f};

	// Current orbit angle measured in global frame, against the target
	float _current_orbit_angle{0.0f};

	// If target speed is too low (below deadzone), don't set velocity setpoints since it's estimate can be noisy
	bool _dont_follow_target_velocity{true};

	// NOTE: If more of these internal state variables come into existence, it
	// would make sense to create an internal state machine with a single enum
	bool _emergency_ascent = false;

	// Yaw setpoint filter to remove jitter-ness
	AlphaFilter<float> _yaw_setpoint_filter;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_MIN_FT_HT>) _param_nav_min_ft_ht,
		(ParamFloat<px4::params::NAV_FT_DST>) _param_nav_ft_dst,
		(ParamInt<px4::params::NAV_FT_FS>) _param_nav_ft_fs,
		(ParamInt<px4::params::NAV_FT_ALT_M>) _param_nav_ft_alt_m,
		(ParamInt<px4::params::NAV_FT_GMB_M>) _param_nav_ft_gmb_m,
		(ParamInt<px4::params::NAV_FT_DELC>) _param_nav_ft_delc
	)

	uORB::Subscription _follow_target_estimator_sub{ORB_ID(follow_target_estimator)};

	uORB::PublicationMulti<follow_target_status_s> _follow_target_status_pub{ORB_ID(follow_target_status)};
	uORB::PublicationMulti<gimbal_manager_set_attitude_s> _gimbal_manager_set_attitude_pub{ORB_ID(gimbal_manager_set_attitude)};

	// Debugging
	float _gimbal_pitch{0};
};
