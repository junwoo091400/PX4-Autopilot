/****************************************************************************
 *
 *   Copyright (c) 2013-2021 PX4 Development Team. All rights reserved.
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
 * @file mission_feasibility_checker.cpp
 * Provides checks if mission is feasible given the navigation capabilities
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include "mission_feasibility_checker.h"

#include "mission_block.h"
#include "navigator.h"

#include <drivers/drv_pwm_output.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/landing_slope/Landingslope.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/position_controller_landing_status.h>
#include <px4_platform_common/events.h>

bool
MissionFeasibilityChecker::checkMissionFeasible(const mission_s &mission,
		float max_distance_to_1st_waypoint, float max_distance_between_waypoints,
		int max_flight_time)
{
	// Reset warning flag
	_navigator->get_mission_result()->warning = false;

	// trivial case: A mission with length zero cannot be valid
	if ((int)mission.count <= 0) {
		return false;
	}

	bool failed = false;

	// first check if we have a valid position
	const bool home_valid = _navigator->home_position_valid();
	const bool home_alt_valid = _navigator->home_alt_valid();

	if (!home_alt_valid) {
		failed = true;
		mavlink_log_info(_navigator->get_mavlink_log_pub(), "Not yet ready for mission, no position lock.\t");
		events::send(events::ID("navigator_mis_no_pos_lock"), events::Log::Info, "Not yet ready for mission, no position lock");

	} else {
		failed = failed || !checkDistanceToFirstWaypoint(mission, max_distance_to_1st_waypoint);
	}

	const float home_alt = _navigator->get_home_position()->alt;

	// check if all mission item commands are supported
	failed = failed || !checkMissionItemValidity(mission);
	failed = failed || !checkDistancesBetweenWaypoints(mission, max_distance_between_waypoints);
	failed = failed || !checkGeofence(mission, home_alt, home_valid);
	failed = failed || !checkHomePositionAltitude(mission, home_alt, home_alt_valid);

	// reset for next check
	_has_takeoff = false;
	_has_landing = false;

	if (max_flight_time > 0) {
		failed = failed || !checkFlightTime(mission, max_flight_time);
	}

	if (_navigator->get_vstatus()->is_vtol) {
		failed = failed || !checkVTOL(mission, home_alt);

	} else if (_navigator->get_vstatus()->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
		failed = failed || !checkRotarywing(mission, home_alt);

	} else {
		failed = failed || !checkFixedwing(mission, home_alt);
	}

	return !failed;
}

bool
MissionFeasibilityChecker::checkRotarywing(const mission_s &mission, float home_alt)
{
	/*
	 * Perform check and issue feedback to the user
	 * Mission is only marked as feasible if takeoff check passes
	 */
	bool resTakeoff = checkTakeoff(mission, home_alt);
	bool resVTOLLanding = checkVTOLLanding(mission);
	bool hasVTOLLanding = _has_landing && resVTOLLanding;

	bool res = resTakeoff && !_has_vtol_takeoff && !hasVTOLLanding;

	if (_has_vtol_takeoff || hasVTOLLanding) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(),
				     "Mission rejected: plan has VTOL items but vehicle isn't a VTOL.\t");
		events::send(events::ID("navigator_vtol_mis_on_mc"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: plan has VTOL items but vehicle isn't a VTOL");
	}

	return res;
}

bool
MissionFeasibilityChecker::checkFixedwing(const mission_s &mission, float home_alt)
{
	/* Perform checks and issue feedback to the user for all checks */
	bool resTakeoff = checkTakeoff(mission, home_alt);
	bool resLanding = checkFixedWingLanding(mission);

	bool res = resTakeoff && !_has_vtol_takeoff && resLanding;

	if (_has_vtol_takeoff) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(),
				     "Mission rejected: plan has VTOL items but vehicle isn't a VTOL.\t");
		events::send(events::ID("navigator_vtol_mis_on_fw"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: plan has VTOL items but vehicle isn't a VTOL");
	}

	/* Mission is only marked as feasible if all checks return true */
	return res;
}

bool
MissionFeasibilityChecker::checkVTOL(const mission_s &mission, float home_alt)
{
	/* Perform checks and issue feedback to the user for all checks */
	bool resTakeoff = checkTakeoff(mission, home_alt);
	bool resLanding = checkVTOLLanding(mission);
	bool resTakeoffLandReq = checkTakeoffLandAvailable();

	/* Mission is only marked as feasible if all checks return true */
	return (resTakeoff && resLanding && resTakeoffLandReq);
}

bool
MissionFeasibilityChecker::checkGeofence(const mission_s &mission, float home_alt, bool home_valid)
{
	if (_navigator->get_geofence().isHomeRequired() && !home_valid) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position\t");
		events::send(events::ID("navigator_mis_geofence_no_home"), {events::Log::Error, events::LogInternal::Info},
			     "Geofence requires a valid home position");
		return false;
	}

	/* Check if all mission items are inside the geofence (if we have a valid geofence) */
	if (_navigator->get_geofence().valid()) {
		for (size_t i = 0; i < mission.count; i++) {
			struct mission_item_s missionitem = {};
			const ssize_t len = sizeof(missionitem);

			if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
				/* not supposed to happen unless the datamanager can't access the SD card, etc. */
				return false;
			}

			if (missionitem.altitude_is_relative && !home_valid) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence requires valid home position\t");
				events::send(events::ID("navigator_mis_geofence_no_home2"), {events::Log::Error, events::LogInternal::Info},
					     "Geofence requires a valid home position");
				return false;
			}

			// Geofence function checks against home altitude amsl
			missionitem.altitude = missionitem.altitude_is_relative ? missionitem.altitude + home_alt : missionitem.altitude;

			if (MissionBlock::item_contains_position(missionitem) && !_navigator->get_geofence().check(missionitem)) {

				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Geofence violation for waypoint %zu\t", i + 1);
				events::send<int16_t>(events::ID("navigator_mis_geofence_violation"), {events::Log::Error, events::LogInternal::Info},
						      "Geofence violation for waypoint {1}",
						      i + 1);
				return false;
			}
		}
	}

	return true;
}

bool
MissionFeasibilityChecker::checkHomePositionAltitude(const mission_s &mission, float home_alt, bool home_alt_valid)
{
	/* Check if all waypoints are above the home altitude */
	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			_navigator->get_mission_result()->warning = true;
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		/* reject relative alt without home set */
		if (missionitem.altitude_is_relative && !home_alt_valid && MissionBlock::item_contains_position(missionitem)) {

			_navigator->get_mission_result()->warning = true;

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: No home pos, WP %zu uses rel alt\t", i + 1);
			events::send<int16_t>(events::ID("navigator_mis_no_home_rel_alt"), {events::Log::Error, events::LogInternal::Info},
					      "Mission rejected: No home position, waypoint {1} uses relative altitude",
					      i + 1);
			return false;

		}

		/* calculate the global waypoint altitude */
		float wp_alt = (missionitem.altitude_is_relative) ? missionitem.altitude + home_alt : missionitem.altitude;

		if (home_alt_valid && home_alt > wp_alt && MissionBlock::item_contains_position(missionitem)) {

			_navigator->get_mission_result()->warning = true;

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Warning: Waypoint %zu below home\t", i + 1);
			events::send<int16_t>(events::ID("navigator_mis_wp_below_home"), {events::Log::Warning, events::LogInternal::Info},
					      "Waypoint {1} below home", i + 1);
		}
	}

	return true;
}

bool
MissionFeasibilityChecker::checkMissionItemValidity(const mission_s &mission)
{
	// do not allow mission if we find unsupported item
	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			// not supposed to happen unless the datamanager can't access the SD card, etc.
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Cannot access SD card\t");
			events::send(events::ID("navigator_mis_sd_failure"), events::Log::Error,
				     "Mission rejected: Cannot access mission storage");
			return false;
		}

		// check if we find unsupported items and reject mission if so
		if (missionitem.nav_cmd != NAV_CMD_IDLE &&
		    missionitem.nav_cmd != NAV_CMD_WAYPOINT &&
		    missionitem.nav_cmd != NAV_CMD_LOITER_UNLIMITED &&
		    missionitem.nav_cmd != NAV_CMD_LOITER_TIME_LIMIT &&
		    missionitem.nav_cmd != NAV_CMD_RETURN_TO_LAUNCH &&
		    missionitem.nav_cmd != NAV_CMD_LAND &&
		    missionitem.nav_cmd != NAV_CMD_TAKEOFF &&
		    missionitem.nav_cmd != NAV_CMD_LOITER_TO_ALT &&
		    missionitem.nav_cmd != NAV_CMD_VTOL_TAKEOFF &&
		    missionitem.nav_cmd != NAV_CMD_VTOL_LAND &&
		    missionitem.nav_cmd != NAV_CMD_DELAY &&
		    missionitem.nav_cmd != NAV_CMD_CONDITION_GATE &&
		    missionitem.nav_cmd != NAV_CMD_DO_JUMP &&
		    missionitem.nav_cmd != NAV_CMD_DO_CHANGE_SPEED &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_HOME &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_SERVO &&
		    missionitem.nav_cmd != NAV_CMD_DO_LAND_START &&
		    missionitem.nav_cmd != NAV_CMD_DO_TRIGGER_CONTROL &&
		    missionitem.nav_cmd != NAV_CMD_DO_DIGICAM_CONTROL &&
		    missionitem.nav_cmd != NAV_CMD_IMAGE_START_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_IMAGE_STOP_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_VIDEO_START_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_VIDEO_STOP_CAPTURE &&
		    missionitem.nav_cmd != NAV_CMD_DO_CONTROL_VIDEO &&
		    missionitem.nav_cmd != NAV_CMD_DO_MOUNT_CONFIGURE &&
		    missionitem.nav_cmd != NAV_CMD_DO_MOUNT_CONTROL &&
		    missionitem.nav_cmd != NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW &&
		    missionitem.nav_cmd != NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_LOCATION &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_NONE &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_DIST &&
		    missionitem.nav_cmd != NAV_CMD_OBLIQUE_SURVEY &&
		    missionitem.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL &&
		    missionitem.nav_cmd != NAV_CMD_SET_CAMERA_MODE &&
		    missionitem.nav_cmd != NAV_CMD_SET_CAMERA_ZOOM &&
		    missionitem.nav_cmd != NAV_CMD_SET_CAMERA_FOCUS &&
		    missionitem.nav_cmd != NAV_CMD_DO_VTOL_TRANSITION &&
		    missionitem.nav_cmd != NAV_CMD_WAYPOINT_USER_1) {

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: item %i: unsupported cmd: %d\t",
					     (int)(i + 1),
					     (int)missionitem.nav_cmd);
			events::send<uint16_t, uint16_t>(events::ID("navigator_mis_unsup_cmd"), {events::Log::Error, events::LogInternal::Warning},
							 "Mission rejected: item {1}: unsupported command: {2}", i + 1, missionitem.nav_cmd);
			return false;
		}

		/* Check non navigation item */
		if (missionitem.nav_cmd == NAV_CMD_DO_SET_SERVO) {

			/* check actuator number */
			if (missionitem.params[0] < 0 || missionitem.params[0] > 5) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Actuator number %d is out of bounds 0..5\t",
						     (int)missionitem.params[0]);
				events::send<uint32_t>(events::ID("navigator_mis_act_index"), {events::Log::Error, events::LogInternal::Warning},
						       "Actuator number {1} is out of bounds 0..5", (int)missionitem.params[0]);
				return false;
			}

			/* check actuator value */
			if (missionitem.params[1] < -PWM_DEFAULT_MAX || missionitem.params[1] > PWM_DEFAULT_MAX) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Actuator value %d is out of bounds -PWM_DEFAULT_MAX..PWM_DEFAULT_MAX\t", (int)missionitem.params[1]);
				events::send<uint32_t, uint32_t>(events::ID("navigator_mis_act_range"), {events::Log::Error, events::LogInternal::Warning},
								 "Actuator value {1} is out of bounds -{2}..{2}", (int)missionitem.params[1], PWM_DEFAULT_MAX);
				return false;
			}
		}

		// check if the mission starts with a land command while the vehicle is landed
		if ((i == 0) && missionitem.nav_cmd == NAV_CMD_LAND && _navigator->get_land_detected()->landed) {

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: starts with landing\t");
			events::send(events::ID("navigator_mis_starts_w_landing"), {events::Log::Error, events::LogInternal::Info},
				     "Mission rejected: starts with landing");
			return false;
		}
	}

	return true;
}

bool
MissionFeasibilityChecker::checkTakeoff(const mission_s &mission, float home_alt)
{
	bool takeoff_first = false;
	int takeoff_index = -1;

	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(struct mission_item_s);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		// look for a takeoff waypoint
		if (missionitem.nav_cmd == NAV_CMD_TAKEOFF || missionitem.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {
			// make sure that the altitude of the waypoint is at least one meter larger than the acceptance radius
			// this makes sure that the takeoff waypoint is not reached before we are at least one meter in the air

			float takeoff_alt = missionitem.altitude_is_relative
					    ? missionitem.altitude
					    : missionitem.altitude - home_alt;

			// check if we should use default acceptance radius
			float acceptance_radius = _navigator->get_default_acceptance_radius();

			if (missionitem.acceptance_radius > NAV_EPSILON_POSITION) {
				acceptance_radius = missionitem.acceptance_radius;
			}

			if (takeoff_alt - 1.0f < acceptance_radius) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Takeoff altitude too low!\t");
				/* EVENT
				 * @description The minimum takeoff altitude is the acceptance radius plus 1m.
				 */
				events::send<float>(events::ID("navigator_mis_takeoff_too_low"), {events::Log::Error, events::LogInternal::Info},
						    "Mission rejected: takeoff altitude too low! Minimum: {1:.1m_v}", acceptance_radius + 1.f);
				return false;
			}

			// tell that mission has a takeoff waypoint
			_has_takeoff = true;
			_has_vtol_takeoff = (missionitem.nav_cmd == NAV_CMD_VTOL_TAKEOFF);

			// tell that a takeoff waypoint is the first "waypoint"
			// mission item
			if (i == 0) {
				takeoff_first = true;

			} else if (takeoff_index == -1) {
				// stores the index of the first takeoff waypoint
				takeoff_index = i;
			}
		}
	}

	if (takeoff_index != -1) {
		// checks if all the mission items before the first takeoff waypoint
		// are not waypoints or position-related items;
		// this means that, before a takeoff waypoint, one can set
		// one of the bellow mission items
		for (size_t i = 0; i < (size_t)takeoff_index; i++) {
			struct mission_item_s missionitem = {};
			const ssize_t len = sizeof(struct mission_item_s);

			if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
				/* not supposed to happen unless the datamanager can't access the SD card, etc. */
				return false;
			}

			takeoff_first = !(missionitem.nav_cmd != NAV_CMD_IDLE &&
					  missionitem.nav_cmd != NAV_CMD_DELAY &&
					  missionitem.nav_cmd != NAV_CMD_DO_JUMP &&
					  missionitem.nav_cmd != NAV_CMD_DO_CHANGE_SPEED &&
					  missionitem.nav_cmd != NAV_CMD_DO_SET_HOME &&
					  missionitem.nav_cmd != NAV_CMD_DO_SET_SERVO &&
					  missionitem.nav_cmd != NAV_CMD_DO_LAND_START &&
					  missionitem.nav_cmd != NAV_CMD_DO_TRIGGER_CONTROL &&
					  missionitem.nav_cmd != NAV_CMD_DO_DIGICAM_CONTROL &&
					  missionitem.nav_cmd != NAV_CMD_IMAGE_START_CAPTURE &&
					  missionitem.nav_cmd != NAV_CMD_IMAGE_STOP_CAPTURE &&
					  missionitem.nav_cmd != NAV_CMD_VIDEO_START_CAPTURE &&
					  missionitem.nav_cmd != NAV_CMD_VIDEO_STOP_CAPTURE &&
					  missionitem.nav_cmd != NAV_CMD_DO_CONTROL_VIDEO &&
					  missionitem.nav_cmd != NAV_CMD_DO_MOUNT_CONFIGURE &&
					  missionitem.nav_cmd != NAV_CMD_DO_MOUNT_CONTROL &&
					  missionitem.nav_cmd != NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW &&
					  missionitem.nav_cmd != NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE &&
					  missionitem.nav_cmd != NAV_CMD_DO_SET_ROI &&
					  missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_LOCATION &&
					  missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET &&
					  missionitem.nav_cmd != NAV_CMD_DO_SET_ROI_NONE &&
					  missionitem.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_DIST &&
					  missionitem.nav_cmd != NAV_CMD_OBLIQUE_SURVEY &&
					  missionitem.nav_cmd != NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL &&
					  missionitem.nav_cmd != NAV_CMD_SET_CAMERA_MODE &&
					  missionitem.nav_cmd != NAV_CMD_SET_CAMERA_ZOOM &&
					  missionitem.nav_cmd != NAV_CMD_SET_CAMERA_FOCUS &&
					  missionitem.nav_cmd != NAV_CMD_SET_CAMERA_FOCUS &&
					  missionitem.nav_cmd != NAV_CMD_DO_VTOL_TRANSITION &&
					  missionitem.nav_cmd != NAV_CMD_WAYPOINT_USER_1);
		}
	}


	if (_has_takeoff && !takeoff_first) {
		// check if the takeoff waypoint is the first waypoint item on the mission
		// i.e, an item with position/attitude change modification
		// if it is not, the mission should be rejected
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: takeoff not first waypoint item\t");
		events::send(events::ID("navigator_mis_takeoff_not_first"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: takeoff is not the first waypoint item");
		return false;
	}


	// all checks have passed
	return true;
}

bool
MissionFeasibilityChecker::checkFixedWingLanding(const mission_s &mission)
{
	/* Go through all mission items and search for a landing waypoint
	 * if landing waypoint is found: the previous waypoint is checked to be at a feasible distance and altitude given the landing slope */

	bool landing_valid = false;

	bool land_start_found = false;
	size_t do_land_start_index = 0;
	size_t landing_approach_index = 0;

	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(missionitem);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		// if DO_LAND_START found then require valid landing AFTER
		if (missionitem.nav_cmd == NAV_CMD_DO_LAND_START) {
			if (land_start_found) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: more than one land start.\t");
				events::send(events::ID("navigator_mis_multiple_land"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: more than one land start commands");
				return false;

			} else {
				land_start_found = true;
				do_land_start_index = i;
			}
		}

		if (missionitem.nav_cmd == NAV_CMD_LAND) {
			mission_item_s missionitem_previous {};

			if (i > 0) {
				landing_approach_index = i - 1;

				if (dm_read((dm_item_t)mission.dataman_id, landing_approach_index, &missionitem_previous, len) != len) {
					/* not supposed to happen unless the datamanager can't access the SD card, etc. */
					return false;
				}

				if (MissionBlock::item_contains_position(missionitem_previous)) {

					uORB::SubscriptionData<position_controller_landing_status_s> landing_status{ORB_ID(position_controller_landing_status)};

					const bool landing_status_valid = (landing_status.get().timestamp > 0);
					const float wp_distance = get_distance_to_next_waypoint(missionitem_previous.lat, missionitem_previous.lon,
								  missionitem.lat, missionitem.lon);

					if (landing_status_valid && (wp_distance > landing_status.get().flare_length)) {
						/* Last wp is before flare region */

						const float delta_altitude = missionitem.altitude - missionitem_previous.altitude;

						if (delta_altitude < 0) {

							const float horizontal_slope_displacement = landing_status.get().horizontal_slope_displacement;
							const float slope_angle_rad = landing_status.get().slope_angle_rad;
							const float slope_alt_req = Landingslope::getLandingSlopeAbsoluteAltitude(wp_distance, missionitem.altitude,
										    horizontal_slope_displacement, slope_angle_rad);

							if (missionitem_previous.altitude > slope_alt_req + 1.0f) {
								/* Landing waypoint is above altitude of slope at the given waypoint distance (with small tolerance for floating point discrepancies) */
								const float wp_distance_req = Landingslope::getLandingSlopeWPDistance(missionitem_previous.altitude,
											      missionitem.altitude, horizontal_slope_displacement, slope_angle_rad);

								mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: adjust landing approach.\t");
								mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Move down %d m or move further away by %d m.\t",
										     (int)ceilf(slope_alt_req - missionitem_previous.altitude),
										     (int)ceilf(wp_distance_req - wp_distance));
								/* EVENT
								 * @description
								 * The landing waypoint must be above the altitude of slope at the given waypoint distance.
								 * Move it down {1m_v} or move it further away by {2m}.
								 */
								events::send<int32_t, int32_t>(events::ID("navigator_mis_land_approach"), {events::Log::Error, events::LogInternal::Info},
											       "Mission rejected: adjust landing approach",
											       (int)ceilf(slope_alt_req - missionitem_previous.altitude),
											       (int)ceilf(wp_distance_req - wp_distance));

								return false;
							}

						} else {
							/* Landing waypoint is above last waypoint */
							mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: landing above last waypoint.\t");
							events::send(events::ID("navigator_mis_land_too_high"), {events::Log::Error, events::LogInternal::Info},
								     "Mission rejected: landing waypoint is above the last waypoint");
							return false;
						}

					} else {
						/* Last wp is in flare region */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: waypoint within landing flare.\t");
						events::send(events::ID("navigator_mis_land_within_flare"), {events::Log::Error, events::LogInternal::Info},
							     "Mission rejected: waypoint is within landing flare");
						return false;
					}

					landing_valid = true;

				} else {
					// mission item before land doesn't have a position
					mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: need landing approach.\t");
					events::send(events::ID("navigator_mis_req_landing_approach"), {events::Log::Error, events::LogInternal::Info},
						     "Mission rejected: landing approach is required");
					return false;
				}

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: starts with land waypoint.\t");
				events::send(events::ID("navigator_mis_starts_w_landing2"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: starts with landing");
				return false;
			}

		} else if (missionitem.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
			if (land_start_found && do_land_start_index < i) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Mission rejected: land start item before RTL item not possible.\t");
				events::send(events::ID("navigator_mis_land_before_rtl"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: land start item before RTL item is not possible");
				return false;
			}
		}
	}

	if (land_start_found && (!landing_valid || (do_land_start_index > landing_approach_index))) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: invalid land start.\t");
		events::send(events::ID("navigator_mis_invalid_land"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: invalid land start");
		return false;
	}

	/* No landing waypoints or no waypoints */
	return true;
}

bool
MissionFeasibilityChecker::checkVTOLLanding(const mission_s &mission)
{
	/* Go through all mission items and search for a landing waypoint
	 * if landing waypoint is found: the previous waypoint is checked to be at a feasible distance and altitude given the landing slope */

	size_t do_land_start_index = 0;
	size_t landing_approach_index = 0;

	for (size_t i = 0; i < mission.count; i++) {
		struct mission_item_s missionitem;
		const ssize_t len = sizeof(missionitem);

		if (dm_read((dm_item_t)mission.dataman_id, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			return false;
		}

		// if DO_LAND_START found then require valid landing AFTER
		if (missionitem.nav_cmd == NAV_CMD_DO_LAND_START) {
			if (_has_landing) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: more than one land start.\t");
				events::send(events::ID("navigator_mis_multi_land"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: more than one land start commands");
				return false;

			} else {
				_has_landing = true;
				do_land_start_index = i;
			}
		}

		if (missionitem.nav_cmd == NAV_CMD_LAND || missionitem.nav_cmd == NAV_CMD_VTOL_LAND) {
			mission_item_s missionitem_previous {};

			if (i > 0) {
				landing_approach_index = i - 1;

				if (dm_read((dm_item_t)mission.dataman_id, landing_approach_index, &missionitem_previous, len) != len) {
					/* not supposed to happen unless the datamanager can't access the SD card, etc. */
					return false;
				}


			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: starts with land waypoint.\t");
				events::send(events::ID("navigator_mis_starts_w_land"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: starts with land waypoint");
				return false;
			}

		} else if (missionitem.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
			if (_has_landing && do_land_start_index < i) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Mission rejected: land start item before RTL item not possible.\t");
				events::send(events::ID("navigator_mis_land_before_rtl2"), {events::Log::Error, events::LogInternal::Info},
					     "Mission rejected: land start item before RTL item is not possible");
				return false;
			}
		}
	}

	if (_has_landing && (do_land_start_index > landing_approach_index)) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: invalid land start.\t");
		events::send(events::ID("navigator_mis_invalid_land2"), {events::Log::Error, events::LogInternal::Info},
			     "Mission rejected: invalid land start");
		return false;
	}

	/* No landing waypoints or no waypoints */
	return true;
}

bool
MissionFeasibilityChecker::checkTakeoffLandAvailable()
{
	bool resTakeoffLandReq = true;

	switch (_navigator->get_takeoff_land_required()) {
	case 0:
		resTakeoffLandReq = true;
		break;

	case 1:
		resTakeoffLandReq = _has_takeoff || !_navigator->get_land_detected()->landed;

		if (!resTakeoffLandReq) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Takeoff item missing.");
		}

		break;

	case 2:
		resTakeoffLandReq = _has_landing;

		if (!resTakeoffLandReq) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Landing item missing.");
		}

		break;

	case 3:
		resTakeoffLandReq = _has_takeoff && _has_landing;

		if (!resTakeoffLandReq) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Takeoff or Landing item missing.");
		}

		break;

	case 4:
		resTakeoffLandReq = _has_takeoff == _has_landing;

		if (!resTakeoffLandReq && (_has_takeoff)) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Add Landing item or remove Takeoff.");

		} else if (!resTakeoffLandReq && (_has_landing)) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Add Takeoff item or remove Landing.");
		}

		break;

	case 5:
		_navigator->readVtolHomeLandApproachesFromStorage();

		if (!_navigator->get_land_detected()->landed && !_navigator->hasVtolHomeLandApproach()) {
			resTakeoffLandReq = _has_landing;

			if (!resTakeoffLandReq) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Landing item missing.");
			}

		} else {
			resTakeoffLandReq = _has_takeoff == _has_landing;

			if (!resTakeoffLandReq && (_has_takeoff)) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Add Landing item or remove Takeoff.");

			} else if (!resTakeoffLandReq && (_has_landing)) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission rejected: Add Takeoff item or remove Landing.");
			}
		}

		break;

	default:
		resTakeoffLandReq = true;
		break;
	}

	return resTakeoffLandReq;
}

bool
MissionFeasibilityChecker::checkDistanceToFirstWaypoint(const mission_s &mission, float max_distance)
{
	if (max_distance <= 0.0f) {
		/* param not set, check is ok */
		return true;
	}

	/* find first waypoint (with lat/lon) item in datamanager */
	for (size_t i = 0; i < mission.count; i++) {

		struct mission_item_s mission_item {};

		if (!(dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s))) {
			/* error reading, mission is invalid */
			mavlink_log_info(_navigator->get_mavlink_log_pub(), "Error reading offboard mission.\t");
			events::send(events::ID("navigator_mis_storage_failure"), events::Log::Error,
				     "Error reading mission storage");
			return false;
		}

		/* check only items with valid lat/lon */
		if (!MissionBlock::item_contains_position(mission_item)) {
			continue;
		}

		/* check distance from current position to item */
		float dist_to_1wp = get_distance_to_next_waypoint(
					    mission_item.lat, mission_item.lon,
					    _navigator->get_home_position()->lat, _navigator->get_home_position()->lon);

		if (dist_to_1wp < max_distance) {

			return true;

		} else {
			/* item is too far from home */
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
					     "First waypoint too far away: %dm, %d max\t",
					     (int)dist_to_1wp, (int)max_distance);
			events::send<uint32_t, uint32_t>(events::ID("navigator_mis_first_wp_too_far"), {events::Log::Error, events::LogInternal::Info},
							 "First waypoint too far away: {1m} (maximum: {2m})", (uint32_t)dist_to_1wp, (uint32_t)max_distance);

			_navigator->get_mission_result()->warning = true;
			return false;
		}
	}

	/* no waypoints found in mission, then we will not fly far away */
	return true;
}

bool
MissionFeasibilityChecker::checkDistancesBetweenWaypoints(const mission_s &mission, float max_distance)
{
	if (max_distance <= 0.0f) {
		/* param not set, check is ok */
		return true;
	}

	double last_lat = (double)NAN;
	double last_lon = (double)NAN;
	int last_cmd = 0;

	/* Go through all waypoints */
	for (size_t i = 0; i < mission.count; i++) {

		struct mission_item_s mission_item {};

		if (!(dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s))) {
			/* error reading, mission is invalid */
			mavlink_log_info(_navigator->get_mavlink_log_pub(), "Error reading offboard mission.\t");
			events::send(events::ID("navigator_mis_storage_failure2"), events::Log::Error,
				     "Error reading mission storage");
			return false;
		}

		/* check only items with valid lat/lon */
		if (!MissionBlock::item_contains_position(mission_item)) {
			continue;
		}

		/* Compare it to last waypoint if already available. */
		if (PX4_ISFINITE(last_lat) && PX4_ISFINITE(last_lon)) {

			/* check distance from current position to item */
			const float dist_between_waypoints = get_distance_to_next_waypoint(
					mission_item.lat, mission_item.lon,
					last_lat, last_lon);


			if (dist_between_waypoints > max_distance) {
				/* distance between waypoints is too high */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Distance between waypoints too far: %d meters, %d max.\t",
						     (int)dist_between_waypoints, (int)max_distance);
				events::send<uint32_t, uint32_t>(events::ID("navigator_mis_wp_dist_too_far"), {events::Log::Error, events::LogInternal::Info},
								 "Distance between waypoints too far: {1m}, (maximum: {2m})", (uint32_t)dist_between_waypoints, (uint32_t)max_distance);

				_navigator->get_mission_result()->warning = true;
				return false;

				/* do not allow waypoints that are literally on top of each other */

				/* and do not allow condition gates that are at the same position as a navigation waypoint */

			} else if (dist_between_waypoints < 0.05f &&
				   (mission_item.nav_cmd == NAV_CMD_CONDITION_GATE || last_cmd == NAV_CMD_CONDITION_GATE)) {

				/* Waypoints and gate are at the exact same position, which indicates an
				 * invalid mission and makes calculating the direction from one waypoint
				 * to another impossible. */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Distance between waypoint and gate too close: %d meters\t",
						     (int)dist_between_waypoints);
				events::send<float, float>(events::ID("navigator_mis_wp_gate_too_close"), {events::Log::Error, events::LogInternal::Info},
							   "Distance between waypoint and gate too close: {1:.3m} (minimum: {2:.3m})", dist_between_waypoints, 0.05f);

				_navigator->get_mission_result()->warning = true;
				return false;
			}
		}

		last_lat = mission_item.lat;
		last_lon = mission_item.lon;
		last_cmd = mission_item.nav_cmd;
	}

	/* We ran through all waypoints and have not found any distances between waypoints that are too far. */
	return true;
}

bool
MissionFeasibilityChecker::checkFlightTime(const mission_s &mission, int max_flight_time)
{
	bool success = true;

	float hover_speed = 0.0f;
	float cruise_speed = 0.0f;
	float takeoff_speed = 0.0f;
	float land_speed = 0.0f;

	uint8_t vehicle_type = _navigator->get_vstatus()->vehicle_type;

	if (_navigator->get_vstatus()->is_vtol) {
		param_get(param_find("FW_AIRSPD_TRIM"), &cruise_speed);
		param_get(param_find("MPC_XY_CRUISE"), &hover_speed);
		param_get(param_find("MPC_Z_VEL_MAX_UP"), &takeoff_speed);
		param_get(param_find("MPC_Z_VEL_MAX_DN"), &land_speed);

		if ((cruise_speed <= 0) || (hover_speed <= 0) || (takeoff_speed <= 0) || (land_speed <= 0)) {
			success = false;
		}

	} else {
		switch (vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
			param_get(param_find("MPC_XY_CRUISE"), &hover_speed);
			param_get(param_find("MPC_Z_VEL_MAX_UP"), &takeoff_speed);
			param_get(param_find("MPC_Z_VEL_MAX_DN"), &land_speed);

			if ((hover_speed <= 0) || (takeoff_speed <= 0) || (land_speed <= 0)) {
				success = false;
			}

			break;

		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
			param_get(param_find("FW_AIRSPD_TRIM"), &cruise_speed);
			param_get(param_find("FW_T_CLMB_MAX"), &takeoff_speed);
			param_get(param_find("FW_T_SINK_MIN"), &land_speed);

			if ((cruise_speed <= 0) || (takeoff_speed <= 0) || (land_speed <= 0)) {
				success = false;
			}

			break;
		}
	}

	if (!success) {

		PX4_ERR("mission feasibility could not be determined due to parameter problem!");

	} else {

		float last_atlitude = (float)NAN;
		float rtl_atlitude = (float)NAN;
		double last_lat = (double)NAN;
		double last_lon = (double)NAN;

		float hover_time = 0.0f;
		float cruise_time = 0.0f;
		float takeoff_time = 0.0f;
		float land_time = 0.0f;
		float total_distance = 0.0f;
		float loiter_radius = 0.0f;

		bool is_first_waypoint = true;
		bool is_received_rtl_cmd = false;
		bool vtol_in_hover = false;

		for (size_t i = 0; i < mission.count; i++) {

			float distance = 0.0f;

			struct mission_item_s mission_item {};

			if (!(dm_read((dm_item_t)mission.dataman_id, i, &mission_item, sizeof(mission_item_s)) == sizeof(mission_item_s))) {
				/* error reading, mission is invalid */
				mavlink_log_info(_navigator->get_mavlink_log_pub(), "Error reading offboard mission.");
				return false;
			}

			/* check only items with valid lat/lon */
			if (MissionBlock::item_contains_position(mission_item)) {

				/* check VTOL transition */
				if (_navigator->get_vstatus()->is_vtol && (mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION)) {
					vtol_in_hover = !vtol_in_hover;
				}

				if ((mission_item.nav_cmd == NAV_CMD_TAKEOFF) || (mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)) {
					float takeoff_altitude = mission_item.altitude;
					takeoff_time = takeoff_altitude / takeoff_speed;
				}

				if (is_first_waypoint) {
					/* check distance from current position to item */
					distance = get_distance_to_next_waypoint(
							   mission_item.lat, mission_item.lon,
							   _navigator->get_home_position()->lat, _navigator->get_home_position()->lon);

					is_first_waypoint = false;

				} else if (mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT) {
					distance = get_distance_to_next_waypoint(
							   mission_item.lat, mission_item.lon, last_lat, last_lon);

					loiter_radius = mission_item.loiter_radius;

				} else if ((mission_item.nav_cmd == NAV_CMD_LAND)  || (mission_item.nav_cmd == NAV_CMD_VTOL_LAND)) {

					float distance_to_landpoint = get_distance_to_next_waypoint(
									      mission_item.lat, mission_item.lon, last_lat, last_lon);

					if (vtol_in_hover || (vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)) {

						land_time =  mission_item.altitude / land_speed;
						distance = distance_to_landpoint;

					} else {

						if (loiter_radius > distance_to_landpoint) {
							distance = distance_to_landpoint;

						} else {
							distance = sqrtf(powf(distance_to_landpoint, 2) - powf(loiter_radius, 2));
							distance += loiter_radius;
						}
					}

				} else {
					distance += get_distance_to_next_waypoint(
							    mission_item.lat, mission_item.lon, last_lat, last_lon);
				}

				total_distance += distance;
				last_atlitude = mission_item.altitude;
				last_lat = mission_item.lat;
				last_lon = mission_item.lon;

				if (_navigator->get_vstatus()->is_vtol) {

					if (vtol_in_hover) {
						hover_time += (distance / hover_speed);

					} else {
						cruise_time += (distance / cruise_speed);
					}

				} else {
					hover_time += (distance / hover_speed);
					cruise_time += (distance / cruise_speed);
				}
			}

			if (mission_item.nav_cmd == NAV_CMD_RETURN_TO_LAUNCH) {
				is_received_rtl_cmd = true;
				rtl_atlitude = mission_item.altitude;
			}
		}

		if (is_received_rtl_cmd) {
			float diff_altitude = abs(last_atlitude - rtl_atlitude);

			total_distance += get_distance_to_next_waypoint(
						  _navigator->get_home_position()->lat, _navigator->get_home_position()->lon,
						  last_lat, last_lon);

			land_time = diff_altitude / land_speed;
		}

		float total_time = 0.0f;

		if (_navigator->get_vstatus()->is_vtol) {

			total_time = takeoff_time + hover_time + cruise_time + land_time;

		} else {

			switch (vehicle_type) {
			case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:

				hover_time = total_distance / hover_speed;
				total_time = takeoff_time + hover_time + land_time;
				break;

			case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
				total_time = cruise_time;
				break;
			}
		}

		float in_air_time = 0.0f;

		if (_navigator->get_vstatus()->takeoff_time != 0) {
			in_air_time = static_cast<float>(hrt_elapsed_time(&(_navigator->get_vstatus()->takeoff_time))) / 1000000.0f;;
			total_time += in_air_time;
		}

		if (static_cast<int>(total_time) > max_flight_time) {
			/* total time is over the limit */
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
					     "Total flight time: %d min and %d sec. \nFlight time is limited to: %d min and %d sec.",
					     ((int)total_time / 60), ((int)total_time % 60), ((int)max_flight_time / 60), ((int)max_flight_time % 60));

			_navigator->get_mission_result()->warning = true;
			success = false;
		}

		/*
		// helpful for development
		mavlink_log_info(_navigator->get_mavlink_log_pub(),
					"Total distance: %d m \nTotal flight time: %02d:%02d:%02d \nIn the air: %02d:%02d:%02d",
					(int)round(total_distance),
					(int)total_time / 3600, (int)total_time / 60, (int)total_time % 60,
					(int)in_air_time / 3600, (int)in_air_time / 60, (int)in_air_time % 60);
		*/

	}

	return success;
}
