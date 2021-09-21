
/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
* @file FailureDetector.hpp
* Base class for failure detection logic based on vehicle states
* for failsafe triggering.
*
* @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
*
*/

#pragma once

#include <mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <hysteresis/hysteresis.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/wind.h>

typedef enum {
	FAILURE_NONE = vehicle_status_s::FAILURE_NONE,
	FAILURE_ROLL = vehicle_status_s::FAILURE_ROLL,
	FAILURE_PITCH = vehicle_status_s::FAILURE_PITCH,
	FAILURE_ALT = vehicle_status_s::FAILURE_ALT,
	FAILURE_EXT = vehicle_status_s::FAILURE_EXT,
	FAILURE_ARM_ESCS = vehicle_status_s::FAILURE_ARM_ESC,
	FAILURE_HIGH_WIND = vehicle_status_s::FAILURE_HIGH_WIND,
	FAILURE_BATTERY = vehicle_status_s::FAILURE_BATTERY,
	FAILURE_IMBALANCED_PROP = vehicle_status_s::FAILURE_IMBALANCED_PROP
} failure_detector_bitmak;

using uORB::SubscriptionData;

class FailureDetector : public ModuleParams
{
public:
	FailureDetector(ModuleParams *parent);

	bool update(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &vehicle_control_mode);
	uint8_t getStatus() const { return _status; }
	float getImbalancedPropMetric() const { return _imbalanced_prop_lpf.getState(); }

private:
	void updateAttitudeStatus();
	void updateExternalAtsStatus();
	void updateBatteryStatus(const vehicle_status_s &vehicle_status);
	void updateEscsStatus(const vehicle_status_s &vehicle_status);
	void updateHighWindStatus();
	void updateImbalancedPropStatus();

	uint8_t _status{FAILURE_NONE};

	systemlib::Hysteresis _roll_failure_hysteresis{false};
	systemlib::Hysteresis _pitch_failure_hysteresis{false};
	systemlib::Hysteresis _ext_ats_failure_hysteresis{false};
	systemlib::Hysteresis _esc_failure_hysteresis{false};
	systemlib::Hysteresis _battery_failure_hysteresis{false};

	static constexpr float _imbalanced_prop_lpf_time_constant{5.f};
	AlphaFilter<float> _imbalanced_prop_lpf{};
	uint32_t _selected_accel_device_id{0};
	hrt_abstime _imu_status_timestamp_prev{0};

	uORB::SubscriptionMultiArray<battery_status_s> _battery_status_subs{ORB_ID::battery_status};

	uORB::Subscription _vehicule_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _pwm_input_sub{ORB_ID(pwm_input)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};
	uORB::Subscription _wind_sub{ORB_ID(wind)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FD_FAIL_P>) _param_fd_fail_p,
		(ParamInt<px4::params::FD_FAIL_R>) _param_fd_fail_r,
		(ParamFloat<px4::params::FD_FAIL_R_TTRI>) _param_fd_fail_r_ttri,
		(ParamFloat<px4::params::FD_FAIL_P_TTRI>) _param_fd_fail_p_ttri,
		(ParamBool<px4::params::FD_EXT_ATS_EN>) _param_fd_ext_ats_en,
		(ParamInt<px4::params::FD_EXT_ATS_TRIG>) _param_fd_ext_ats_trig,
		(ParamInt<px4::params::FD_ESCS_EN>) _param_fd_escs_en,
		(ParamInt<px4::params::FD_BAT_EN>) _param_fd_bat_en,
		(ParamFloat<px4::params::FD_WIND_MAX>) _param_fd_wind_max,
		(ParamInt<px4::params::FD_IMB_PROP_THR>) _param_fd_imb_prop_thr
	)
};
