/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file terrain_estimator.cpp
 * Function for fusing rangefinder and optical flow measurements
 * to estimate terrain vertical position
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/terr_est_compute_flow_xy_innov_var_and_hx.h"
#include "python/ekf_derivation/generated/terr_est_compute_flow_y_innov_var_and_h.h"

#include <mathlib/mathlib.h>

void Ekf::initHagl()
{

#if defined(CONFIG_EKF2_RANGE_FINDER)
	_aid_src_terrain_range_finder.time_last_fuse = _time_delayed_us;
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
	_aid_src_terrain_optical_flow.time_last_fuse = _time_delayed_us;
#endif // CONFIG_EKF2_OPTICAL_FLOW
}

void Ekf::runTerrainEstimator(const imuSample &imu_delayed)
{
	// If we are on ground, store the local position and time to use as a reference
	if (!_control_status.flags.in_air) {
		_last_on_ground_posD = _state.pos(2);
		_control_status.flags.rng_fault = false;

	} else if (!_control_status_prev.flags.in_air) {
		// Let the estimator run freely before arming for bench testing purposes, but reset on takeoff
		// because when using optical flow measurements, it is safer to start with a small distance to ground
		// as an overestimated distance leads to an overestimated velocity, causing a dangerous behavior.
		initHagl();
	}
}

#if defined(CONFIG_EKF2_RANGE_FINDER)
void Ekf::controlHaglRngFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseRangeFinder)
	    || _control_status.flags.rng_fault) {

		stopHaglRngFusion();
		return;
	}

	if (_range_sensor.isDataReady()) {
		updateHaglRng(_aid_src_terrain_range_finder);
	}

	if (_range_sensor.isDataHealthy()) {

		const bool continuing_conditions_passing = _rng_consistency_check.isKinematicallyConsistent();
				//&& !_control_status.flags.rng_hgt // TODO: should not be fused when using range height

		const bool starting_conditions_passing = continuing_conditions_passing
				&& _range_sensor.isRegularlySendingData()
				&& (_rng_consistency_check.getTestRatio() < 1.f);

		_time_last_healthy_rng_data = _time_delayed_us;

		if (_hagl_sensor_status.flags.range_finder) {
			if (continuing_conditions_passing) {
				fuseHaglRng(_aid_src_terrain_range_finder);

				// We have been rejecting range data for too long
				const uint64_t timeout = static_cast<uint64_t>(_params.terrain_timeout * 1e6f);
				const bool is_fusion_failing = isTimedOut(_aid_src_terrain_range_finder.time_last_fuse, timeout);

				if (is_fusion_failing) {
					if (_range_sensor.getDistBottom() > 2.f * _params.rng_gnd_clearance) {
						// Data seems good, attempt a reset
						resetHaglRng();

					} else if (starting_conditions_passing) {
						// The sensor can probably not detect the ground properly
						// declare the sensor faulty and stop the fusion
						_control_status.flags.rng_fault = true;
						_range_sensor.setFaulty(true);
						stopHaglRngFusion();

					} else {
						// This could be a temporary issue, stop the fusion without declaring the sensor faulty
						stopHaglRngFusion();
					}
				}

			} else {
				stopHaglRngFusion();
			}

		} else {
			if (starting_conditions_passing) {
				_hagl_sensor_status.flags.range_finder = true;

				// Reset the state to the measurement only if the test ratio is large,
				// otherwise let it converge through the fusion
				if (_hagl_sensor_status.flags.flow && (_aid_src_terrain_range_finder.test_ratio < 0.2f)) {
					fuseHaglRng(_aid_src_terrain_range_finder);

				} else {
					resetHaglRng();
				}
			}
		}

	} else if (_hagl_sensor_status.flags.range_finder && isTimedOut(_time_last_healthy_rng_data, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopHaglRngFusion();
	}
}

void Ekf::resetHaglRng()
{
	_terrain_vpos = _state.pos(2) + _range_sensor.getDistBottom();
	_terrain_var = getRngVar();
	_terrain_vpos_reset_counter++;

	_aid_src_terrain_range_finder.time_last_fuse = _time_delayed_us;
}

void Ekf::stopHaglRngFusion()
{
	if (_hagl_sensor_status.flags.range_finder) {
		ECL_INFO("stopping HAGL range fusion");

		// reset flags
		resetEstimatorAidStatus(_aid_src_terrain_range_finder);

		_innov_check_fail_status.flags.reject_hagl = false;

		_hagl_sensor_status.flags.range_finder = false;
	}
}

void Ekf::updateHaglRng(estimator_aid_source1d_s &aid_src) const
{
	// get a height above ground measurement from the range finder assuming a flat earth
	const float meas_hagl = _range_sensor.getDistBottom();

	// predict the hagl from the vehicle position and terrain height
	const float pred_hagl = _terrain_vpos - _state.pos(2);

	// calculate the innovation
	const float hagl_innov = pred_hagl - meas_hagl;

	// calculate the observation variance adding the variance of the vehicles own height uncertainty
	const float obs_variance = getRngVar();

	// calculate the innovation variance - limiting it to prevent a badly conditioned fusion
	const float hagl_innov_var = fmaxf(_terrain_var + obs_variance, obs_variance);

	// perform an innovation consistency check and only fuse data if it passes
	const float innov_gate = fmaxf(_params.range_innov_gate, 1.0f);


	aid_src.timestamp_sample = _time_delayed_us; // TODO

	aid_src.observation = meas_hagl;
	aid_src.observation_variance = obs_variance;

	aid_src.innovation = hagl_innov;
	aid_src.innovation_variance = hagl_innov_var;

	setEstimatorAidStatusTestRatio(aid_src, innov_gate);

	aid_src.fused = false;
}

void Ekf::fuseHaglRng(estimator_aid_source1d_s &aid_src)
{
	if (!aid_src.innovation_rejected) {
		// calculate the Kalman gain
		const float gain = _terrain_var / aid_src.innovation_variance;

		// correct the state
		_terrain_vpos -= gain * aid_src.innovation;

		// correct the variance
		_terrain_var = fmaxf(_terrain_var * (1.0f - gain), 0.0f);

		// record last successful fusion event
		_innov_check_fail_status.flags.reject_hagl = false;

		aid_src.time_last_fuse = _time_delayed_us;
		aid_src.fused = true;

	} else {
		_innov_check_fail_status.flags.reject_hagl = true;
		aid_src.fused = false;
	}
}
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
void Ekf::controlHaglFlowFusion()
{
	if (!(_params.terrain_fusion_mode & TerrainFusionMask::TerrainFuseOpticalFlow)) {
		stopHaglFlowFusion();
		return;
	}

	if (_flow_data_ready) {
		updateOptFlow(_aid_src_terrain_optical_flow);

		const bool continuing_conditions_passing = _control_status.flags.in_air
				&& !_control_status.flags.opt_flow
				&& _control_status.flags.gps
				&& !_hagl_sensor_status.flags.range_finder;

		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_hagl_sensor_status.flags.flow) {
			if (continuing_conditions_passing) {

				// TODO: wait until the midpoint of the flow sample has fallen behind the fusion time horizon
				fuseFlowForTerrain(_aid_src_terrain_optical_flow);

				// TODO: do something when failing continuously the innovation check
				/* const bool is_fusion_failing = isTimedOut(_aid_src_optical_flow.time_last_fuse, _params.reset_timeout_max); */

				/* if (is_fusion_failing) { */
				/* 	resetHaglFlow(); */
				/* } */

			} else {
				stopHaglFlowFusion();
			}

		} else {
			if (starting_conditions_passing) {
				_hagl_sensor_status.flags.flow = true;
				// TODO: do a reset instead of trying to fuse the data?
				fuseFlowForTerrain(_aid_src_terrain_optical_flow);
			}
		}

		_flow_data_ready = false;

	} else if (_hagl_sensor_status.flags.flow && (_time_delayed_us > _flow_sample_delayed.time_us + (uint64_t)5e6)) {
		// No data anymore. Stop until it comes back.
		stopHaglFlowFusion();
	}
}

void Ekf::stopHaglFlowFusion()
{
	if (_hagl_sensor_status.flags.flow) {
		ECL_INFO("stopping HAGL flow fusion");

		_hagl_sensor_status.flags.flow = false;
		resetEstimatorAidStatus(_aid_src_terrain_optical_flow);
	}
}

void Ekf::resetHaglFlow()
{
	// TODO: use the flow data
	_terrain_vpos = fmaxf(0.0f, _state.pos(2));
	_terrain_var = 100.0f;
	_terrain_vpos_reset_counter++;

	_aid_src_terrain_optical_flow.time_last_fuse = _time_delayed_us;
}

void Ekf::controlHaglFakeFusion()
{
	if (!_control_status.flags.in_air
	    && !_hagl_sensor_status.flags.range_finder
	    && !_hagl_sensor_status.flags.flow) {

		bool recent_terrain_aiding = false;

#if defined(CONFIG_EKF2_RANGE_FINDER)
		recent_terrain_aiding |= isRecent(_aid_src_terrain_range_finder.time_last_fuse, (uint64_t)1e6);
#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
		recent_terrain_aiding |= isRecent(_aid_src_terrain_optical_flow.time_last_fuse, (uint64_t)1e6);
#endif // CONFIG_EKF2_OPTICAL_FLOW

		if (_control_status.flags.vehicle_at_rest || !recent_terrain_aiding) {
			initHagl();
		}
	}
}

bool Ekf::isTerrainEstimateValid() const
{
	bool valid = false;

#if defined(CONFIG_EKF2_RANGE_FINDER)

	// we have been fusing range finder measurements in the last 5 seconds
	if (isRecent(_aid_src_terrain_range_finder.time_last_fuse, (uint64_t)5e6)) {
		if (_hagl_sensor_status.flags.range_finder || !_control_status.flags.in_air) {
			valid = true;
		}
	}

#endif // CONFIG_EKF2_RANGE_FINDER

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

	// we have been fusing optical flow measurements for terrain estimation within the last 5 seconds
	// this can only be the case if the main filter does not fuse optical flow
	if (_hagl_sensor_status.flags.flow && isRecent(_aid_src_terrain_optical_flow.time_last_fuse, (uint64_t)5e6)) {
		valid = true;
	}

#endif // CONFIG_EKF2_OPTICAL_FLOW

	return valid;
}
