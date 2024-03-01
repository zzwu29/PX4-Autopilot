/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file RpmControl.hpp
 *
 * Control rpm of a helicopter rotor.
 * Input: PWM input pulse period from an rpm sensor
 * Output: Duty cycle command for the ESC
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/pid/PID.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/rpm_control_status.h>

class RpmControl : public ModuleParams
{
public:
	RpmControl(ModuleParams *parent) :  ModuleParams(parent) {};
	~RpmControl() = default;

	float getActuatorCorrection()
	{
		if (_pwm_input_sub.updated()) {
			pwm_input_s pwm_input{};

			if (_pwm_input_sub.copy(&pwm_input)) {
				// 1'000'000 / [us] -> pulses per second * 60 -> pulses per minute
				if ((1 < pwm_input.period) && (pwm_input.period < 10000)) {
					_rpm_raw = 60.f * 1e6f / (static_cast<float>(pwm_input.period) * 18.f);

				} else {
					_rpm_raw = 0;
				}

				const float dt = math::constrain((pwm_input.timestamp - _timestamp_last_rpm_measurement) * 1e-6f, 0.01f, 1.f);
				_timestamp_last_rpm_measurement = pwm_input.timestamp;
				_rpm_filter.setParameters(dt, 0.5f);
				_rpm_filter.update(_rpm_raw);
			}
		}

		hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain((now - _timestamp_last_update) * 1e-6f, 0.01f, 1.f);
		_timestamp_last_update = now;

		_pid.setGains(_param_ca_heli_rpm_p.get() * 1e-3f, _param_ca_heli_rpm_i.get() * 1e-3f, 0.f);
		_pid.setOutputLimit(.5f);
		_pid.setIntegralLimit(.5f);
		_pid.setSetpoint(_param_ca_heli_rpm_sp.get());
		float output = _pid.update(_rpm_filter.getState(), dt, true);
		//float output = _param_ca_heli_rpm_p.get() * (_rpm_setpoint - _rpm_filter.getState()) * 1e-3f;

		rpm_control_status_s rpm_control_status{};
		rpm_control_status.rpm_raw = _rpm_raw;
		rpm_control_status.rpm_estimate = _rpm_filter.getState();
		rpm_control_status.rpm_setpoint = _param_ca_heli_rpm_sp.get();
		rpm_control_status.output = output;
		rpm_control_status.timestamp = hrt_absolute_time();
		_rpm_control_status_pub.publish(rpm_control_status);

		return output;
	}

private:
	uORB::Subscription _pwm_input_sub{ORB_ID(pwm_input)};
	uORB::Publication<rpm_control_status_s> _rpm_control_status_pub{ORB_ID(rpm_control_status)};

	float _rpm_raw{0.f};
	AlphaFilter<float> _rpm_filter;
	PID _pid;
	hrt_abstime _timestamp_last_update{0};
	hrt_abstime _timestamp_last_rpm_measurement{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_HELI_RPM_SP>) _param_ca_heli_rpm_sp,
		(ParamFloat<px4::params::CA_HELI_RPM_P>) _param_ca_heli_rpm_p,
		(ParamFloat<px4::params::CA_HELI_RPM_I>) _param_ca_heli_rpm_i
	)
};
