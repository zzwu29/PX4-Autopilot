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
 * @file CollisionPrevention.cpp
 * CollisionPrevention controller.
 *
 */

#include "CollisionPrevention.hpp"
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;

namespace
{
static constexpr int INTERNAL_MAP_INCREMENT_DEG = 10; //cannot be lower than 5 degrees, should divide 360 evenly
static constexpr int INTERNAL_MAP_USED_BINS = 360 / INTERNAL_MAP_INCREMENT_DEG;

static float wrap_360(float f)
{
	return wrap(f, 0.f, 360.f);
}

static int wrap_bin(int i)
{
	i = i % INTERNAL_MAP_USED_BINS;

	while (i < 0) {
		i += INTERNAL_MAP_USED_BINS;
	}

	return i;
}

} // namespace

CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{
	static_assert(INTERNAL_MAP_INCREMENT_DEG >= 5, "INTERNAL_MAP_INCREMENT_DEG needs to be at least 5");
	static_assert(360 % INTERNAL_MAP_INCREMENT_DEG == 0, "INTERNAL_MAP_INCREMENT_DEG should divide 360 evenly");

	// initialize internal obstacle map
	_obstacle_map_body_frame.timestamp = getTime();
	_obstacle_map_body_frame.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map_body_frame.increment = INTERNAL_MAP_INCREMENT_DEG;
	_obstacle_map_body_frame.min_distance = UINT16_MAX;
	_obstacle_map_body_frame.max_distance = 0;
	_obstacle_map_body_frame.angle_offset = 0.f;
	uint32_t internal_bins = sizeof(_obstacle_map_body_frame.distances) / sizeof(_obstacle_map_body_frame.distances[0]);
	uint64_t current_time = getTime();

	for (uint32_t i = 0 ; i < internal_bins; i++) {
		_data_timestamps[i] = current_time;
		_data_maxranges[i] = 0;
		_data_fov[i] = 0;
		_obstacle_map_body_frame.distances[i] = UINT16_MAX;
	}
}

hrt_abstime CollisionPrevention::getTime()
{
	return hrt_absolute_time();
}

hrt_abstime CollisionPrevention::getElapsedTime(const hrt_abstime *ptr)
{
	return hrt_absolute_time() - *ptr;
}

bool CollisionPrevention::is_active()
{
	bool activated = _param_cp_dist.get() > 0;

	if (activated && !_was_active) {
		_time_activated = getTime();
	}

	_was_active = activated;
	return activated;
}

void
CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle, const matrix::Quatf &vehicle_attitude)
{
	int msg_index = 0;
	float vehicle_orientation_deg = math::degrees(Eulerf(vehicle_attitude).psi());
	float increment_factor = 1.f / obstacle.increment;

	if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL || obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED) {
		// Obstacle message arrives in local_origin frame (north aligned)
		// corresponding data index (convert to world frame and shift by msg offset)
		for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
			float bin_angle_deg = (float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset;
			msg_index = ceil(wrap_360(vehicle_orientation_deg + bin_angle_deg - obstacle.angle_offset) * increment_factor);

			//add all data points inside to FOV
			if (obstacle.distances[msg_index] != UINT16_MAX) {
				if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[msg_index] * 0.01f)) {
					_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
					_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
					_data_maxranges[i] = obstacle.max_distance;
					_data_fov[i] = 1;
				}
			}
		}

	} else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD) {
		// Obstacle message arrives in body frame (front aligned)
		// corresponding data index (shift by msg offset)
		for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
			float bin_angle_deg = (float)i * INTERNAL_MAP_INCREMENT_DEG +
					      _obstacle_map_body_frame.angle_offset;
			msg_index = ceil(wrap_360(bin_angle_deg - obstacle.angle_offset) * increment_factor);

			//add all data points inside to FOV
			if (obstacle.distances[msg_index] != UINT16_MAX) {

				if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[msg_index] * 0.01f)) {
					_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
					_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
					_data_maxranges[i] = obstacle.max_distance;
					_data_fov[i] = 1;
				}
			}
		}

	} else {
		mavlink_log_critical(&_mavlink_log_pub, "Obstacle message received in unsupported frame %i\t",
				     obstacle.frame);
		events::send<uint8_t>(events::ID("col_prev_unsup_frame"), events::Log::Error,
				      "Obstacle message received in unsupported frame {1}", obstacle.frame);
	}
}

bool
CollisionPrevention::_enterData(int map_index, float sensor_range, float sensor_reading)
{
	//use data from this sensor if:
	//1. this sensor data is in range, the bin contains already valid data and this data is coming from the same or less range sensor
	//2. this sensor data is in range, and the last reading was out of range
	//3. this sensor data is out of range, the last reading was as well and this is the sensor with longest range
	//4. this sensor data is out of range, the last reading was valid and coming from the same sensor

	uint16_t sensor_range_cm = static_cast<uint16_t>(100.0f * sensor_range + 0.5f); //convert to cm

	if (sensor_reading < sensor_range) {
		if ((_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
		     && sensor_range_cm <= _data_maxranges[map_index])
		    || _obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]) {

			return true;
		}

	} else {
		if ((_obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]
		     && sensor_range_cm >= _data_maxranges[map_index])
		    || (_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
			&& sensor_range_cm == _data_maxranges[map_index])) {

			return true;
		}
	}

	return false;
}

void
CollisionPrevention::_updateObstacleMap()
{
	_sub_vehicle_attitude.update();

	// add distance sensor data
	for (auto &dist_sens_sub : _distance_sensor_subs) {
		distance_sensor_s distance_sensor;

		if (dist_sens_sub.update(&distance_sensor)) {
			// consider only instances with valid data and orientations useful for collision prevention
			if ((getElapsedTime(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING)) {

				// update message description
				_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, distance_sensor.timestamp);
				_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
									(uint16_t)(distance_sensor.max_distance * 100.0f));
				_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
									(uint16_t)(distance_sensor.min_distance * 100.0f));

				_addDistanceSensorData(distance_sensor, Quatf(_sub_vehicle_attitude.get().q));
			}
		}
	}

	// add obstacle distance data
	if (_sub_obstacle_distance.update()) {
		const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance.get();

		// Update map with obstacle data if the data is not stale
		if (getElapsedTime(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US && obstacle_distance.increment > 0.f) {
			//update message description
			_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, obstacle_distance.timestamp);
			_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
								obstacle_distance.max_distance);
			_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
								obstacle_distance.min_distance);
			_addObstacleSensorData(obstacle_distance, Quatf(_sub_vehicle_attitude.get().q));
		}
	}

	// publish fused obtacle distance message with data from offboard obstacle_distance and distance sensor
	_obstacle_distance_pub.publish(_obstacle_map_body_frame);
}

void
CollisionPrevention::_addDistanceSensorData(distance_sensor_s &distance_sensor, const matrix::Quatf &vehicle_attitude)
{
	// clamp at maximum sensor range
	float distance_reading = math::min(distance_sensor.current_distance, distance_sensor.max_distance);

	// discard values below min range
	if ((distance_reading > distance_sensor.min_distance)) {

		float sensor_yaw_body_rad = _sensorOrientationToYawOffset(distance_sensor, _obstacle_map_body_frame.angle_offset);
		float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

		// calculate the field of view boundary bin indices
		int lower_bound = (int)floor((sensor_yaw_body_deg  - math::degrees(distance_sensor.h_fov / 2.0f)) /
					     INTERNAL_MAP_INCREMENT_DEG);
		int upper_bound = (int)floor((sensor_yaw_body_deg  + math::degrees(distance_sensor.h_fov / 2.0f)) /
					     INTERNAL_MAP_INCREMENT_DEG);

		// floor values above zero, ceil values below zero
		if (lower_bound < 0) { lower_bound++; }

		if (upper_bound < 0) { upper_bound++; }

		// rotate vehicle attitude into the sensor body frame
		matrix::Quatf attitude_sensor_frame = vehicle_attitude;
		attitude_sensor_frame.rotate(Vector3f(0.f, 0.f, sensor_yaw_body_rad));
		float sensor_dist_scale = cosf(Eulerf(attitude_sensor_frame).theta());

		if (distance_reading < distance_sensor.max_distance) {
			distance_reading = distance_reading * sensor_dist_scale;
		}

		uint16_t sensor_range = static_cast<uint16_t>(100.0f * distance_sensor.max_distance + 0.5f); // convert to cm

		for (int bin = lower_bound; bin <= upper_bound; ++bin) {
			int wrapped_bin = wrap_bin(bin);

			if (_enterData(wrapped_bin, distance_sensor.max_distance, distance_reading)) {
				_obstacle_map_body_frame.distances[wrapped_bin] = static_cast<uint16_t>(100.0f * distance_reading + 0.5f);
				_data_timestamps[wrapped_bin] = _obstacle_map_body_frame.timestamp;
				_data_maxranges[wrapped_bin] = sensor_range;
				_data_fov[wrapped_bin] = 1;
			}
		}
	}
}

void
CollisionPrevention::_adaptSetpointDirection(Vector2f &setpoint_dir, int &setpoint_index, float vehicle_yaw_angle_rad)
{
	const float col_prev_d = _param_cp_dist.get();
	const int guidance_bins = floor(_param_cp_guide_ang.get() / INTERNAL_MAP_INCREMENT_DEG);
	const int sp_index_original = setpoint_index;
	float best_cost = 9999.f;
	int new_sp_index = setpoint_index;

	for (int i = sp_index_original - guidance_bins; i <= sp_index_original + guidance_bins; i++) {

		// apply moving average filter to the distance array to be able to center in larger gaps
		const int filter_size = 1;
		float mean_dist = 0;

		for (int j = i - filter_size; j <= i + filter_size; j++) {
			int bin = wrap_bin(j);

			if (_obstacle_map_body_frame.distances[bin] == UINT16_MAX) {
				mean_dist += col_prev_d * 100.f;

			} else {
				mean_dist += _obstacle_map_body_frame.distances[bin];
			}
		}

		const int bin = wrap_bin(i);
		mean_dist = mean_dist / (2.f * filter_size + 1.f);
		const float deviation_cost = col_prev_d * 50.f * abs(i - sp_index_original);
		const float bin_cost = deviation_cost - mean_dist - _obstacle_map_body_frame.distances[bin];

		if (bin_cost < best_cost && _obstacle_map_body_frame.distances[bin] != UINT16_MAX) {
			best_cost = bin_cost;
			new_sp_index = bin;
		}
	}

	//only change setpoint direction if it was moved to a different bin
	if (new_sp_index != setpoint_index) {
		float angle = math::radians((float)new_sp_index * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset);
		angle = wrap_2pi(vehicle_yaw_angle_rad + angle);
		setpoint_dir = {cosf(angle), sinf(angle)};
		setpoint_index = new_sp_index;
	}
}

float
CollisionPrevention::_sensorOrientationToYawOffset(const distance_sensor_s &distance_sensor, float angle_offset) const
{
	float offset = angle_offset > 0.0f ? math::radians(angle_offset) : 0.0f;

	switch (distance_sensor.orientation) {
	case distance_sensor_s::ROTATION_YAW_0:
		offset = 0.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_45:
		offset = M_PI_F / 4.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_90:
		offset = M_PI_F / 2.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_135:
		offset = 3.0f * M_PI_F / 4.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_180:
		offset = M_PI_F;
		break;

	case distance_sensor_s::ROTATION_YAW_225:
		offset = -3.0f * M_PI_F / 4.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_270:
		offset = -M_PI_F / 2.0f;
		break;

	case distance_sensor_s::ROTATION_YAW_315:
		offset = -M_PI_F / 4.0f;
		break;

	case distance_sensor_s::ROTATION_CUSTOM:
		offset = matrix::Eulerf(matrix::Quatf(distance_sensor.q)).psi();
		break;
	}

	return offset;
}

void
CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint, const Vector2f &curr_pos,
		const Vector2f &curr_vel)
{
	_updateObstacleMap();

	// read parameters
	const float col_prev_d = _param_cp_dist.get();
	const float col_prev_dly = _param_cp_delay.get();
	const bool move_no_data = _param_cp_go_nodata.get();
	const float xy_p = _param_mpc_xy_p.get();
	const float max_jerk = _param_mpc_jerk_max.get();
	const float max_accel = _param_mpc_acc_hor.get();
	const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
	const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();

	const float setpoint_length = setpoint.norm();

	const hrt_abstime constrain_time = getTime();
	int num_fov_bins = 0;

	if ((constrain_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US) {
		if (setpoint_length > 0.001f) {

			Vector2f setpoint_dir = setpoint / setpoint_length;
			float vel_max = setpoint_length;
			const float min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, col_prev_d);

			const float sp_angle_body_frame = atan2f(setpoint_dir(1), setpoint_dir(0)) - vehicle_yaw_angle_rad;
			const float sp_angle_with_offset_deg = wrap_360(math::degrees(sp_angle_body_frame) -
							       _obstacle_map_body_frame.angle_offset);
			int sp_index = floor(sp_angle_with_offset_deg / INTERNAL_MAP_INCREMENT_DEG);

			// change setpoint direction slightly (max by _param_cp_guide_ang degrees) to help guide through narrow gaps
			_adaptSetpointDirection(setpoint_dir, sp_index, vehicle_yaw_angle_rad);

			// limit speed for safe flight
			for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) { // disregard unused bins at the end of the message

				// delete stale values
				const hrt_abstime data_age = constrain_time - _data_timestamps[i];

				if (data_age > RANGE_STREAM_TIMEOUT_US) {
					_obstacle_map_body_frame.distances[i] = UINT16_MAX;
				}

				const float distance = _obstacle_map_body_frame.distances[i] * 0.01f; // convert to meters
				const float max_range = _data_maxranges[i] * 0.01f; // convert to meters
				float angle = math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset);

				// convert from body to local frame in the range [0, 2*pi]
				angle = wrap_2pi(vehicle_yaw_angle_rad + angle);

				// get direction of current bin
				const Vector2f bin_direction = {cosf(angle), sinf(angle)};

				//count number of bins in the field of valid_new
				if (_obstacle_map_body_frame.distances[i] < UINT16_MAX) {
					num_fov_bins ++;
				}

				if (_obstacle_map_body_frame.distances[i] > _obstacle_map_body_frame.min_distance
				    && _obstacle_map_body_frame.distances[i] < UINT16_MAX) {

					if (setpoint_dir.dot(bin_direction) > 0) {
						// calculate max allowed velocity with a P-controller (same gain as in the position controller)
						const float curr_vel_parallel = math::max(0.f, curr_vel.dot(bin_direction));
						float delay_distance = curr_vel_parallel * col_prev_dly;

						if (distance < max_range) {
							delay_distance += curr_vel_parallel * (data_age * 1e-6f);
						}

						const float stop_distance = math::max(0.f, distance - min_dist_to_keep - delay_distance);
						const float vel_max_posctrl = xy_p * stop_distance;

						const float vel_max_smooth = math::trajectory::computeMaxSpeedFromDistance(max_jerk, max_accel, stop_distance, 0.f);
						const float projection = bin_direction.dot(setpoint_dir);
						float vel_max_bin = vel_max;

						if (projection > 0.01f) {
							vel_max_bin = math::min(vel_max_posctrl, vel_max_smooth) / projection;
						}

						// constrain the velocity
						if (vel_max_bin >= 0) {
							vel_max = math::min(vel_max, vel_max_bin);
						}
					}

				} else if (_obstacle_map_body_frame.distances[i] == UINT16_MAX && i == sp_index) {
					if (!move_no_data || (move_no_data && _data_fov[i])) {
						vel_max = 0.f;
					}
				}
			}

			//if the sensor field of view is zero, never allow to move (even if move_no_data=1)
			if (num_fov_bins == 0) {
				vel_max = 0.f;
			}

			setpoint = setpoint_dir * vel_max;
		}

	} else {
		//allow no movement
		float vel_max = 0.f;
		setpoint = setpoint * vel_max;

		// if distance data is stale, switch to Loiter
		if (getElapsedTime(&_last_timeout_warning) > 1_s && getElapsedTime(&_time_activated) > 1_s) {

			if ((constrain_time - _obstacle_map_body_frame.timestamp) > TIMEOUT_HOLD_US
			    && getElapsedTime(&_time_activated) > TIMEOUT_HOLD_US) {
				_publishVehicleCmdDoLoiter();
			}

			_last_timeout_warning = getTime();
		}


	}
}

void CollisionPrevention::_constrainAccelerationSetpoint(matrix::Vector2f &setpoint_accel,
		const matrix::Vector2f &setpoint_vel)
{
	_updateObstacleMap();

	const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
	const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();

	const float setpoint_length = setpoint_accel.norm();

	const hrt_abstime constrain_time = getTime();
	int num_fov_bins = 0;

	float acc_vel_constraint = INFINITY;
	matrix::Vector2f acc_vel_constraint_dir = {0.f, 0.f};
	matrix::Vector2f acc_vel_constraint_setpoint = {0.f, 0.f};

	if ((constrain_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US) {

		const float min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, _param_cp_dist.get());
		bool setpoint_possible = true;
		matrix::Vector2f new_setpoint = {0.f, 0.f};

		if (setpoint_length > 0.001f) {
			Vector2f setpoint_dir = setpoint_accel / setpoint_length;

			const float sp_angle_body_frame = atan2f(setpoint_dir(1), setpoint_dir(0)) - vehicle_yaw_angle_rad;
			const float sp_angle_with_offset_deg = wrap_360(math::degrees(sp_angle_body_frame) -
							       _obstacle_map_body_frame.angle_offset);
			int sp_index = floor(sp_angle_with_offset_deg / INTERNAL_MAP_INCREMENT_DEG);

			// change setpoint direction slightly (max by _param_cp_guide_ang degrees) to help guide through narrow gaps
			_adaptSetpointDirection(setpoint_dir, sp_index, vehicle_yaw_angle_rad);

			float closest_distance = INFINITY;
			matrix::Vector2f bin_closest_dist = {0.f, 0.f};


			for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
				// delete stale values
				const hrt_abstime data_age = constrain_time - _data_timestamps[i];
				const float max_range = _data_maxranges[i] * 0.01f;

				if (data_age > RANGE_STREAM_TIMEOUT_US) {
					_obstacle_map_body_frame.distances[i] = UINT16_MAX;
				}

				if (_obstacle_map_body_frame.distances[i] < UINT16_MAX) {
					num_fov_bins ++;
				}

				float angle = math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset);
				angle = wrap_2pi(vehicle_yaw_angle_rad + angle);

				// get direction of current bin
				const Vector2f bin_direction = {cosf(angle), sinf(angle)};

				// only consider bins which are between min and max values
				if (_obstacle_map_body_frame.distances[i] > _obstacle_map_body_frame.min_distance
				    && _obstacle_map_body_frame.distances[i] < UINT16_MAX) {
					const float distance = _obstacle_map_body_frame.distances[i] * 0.01f;

					// Assume current velocity is sufficiently close to the setpoint velocity
					const float curr_vel_parallel = math::max(0.f, setpoint_vel.dot(bin_direction));
					float delay_distance = curr_vel_parallel * _param_cp_delay.get();

					if (distance < max_range) {
						delay_distance += curr_vel_parallel * (data_age * 1e-6f);
					}

					const float stop_distance = math::max(0.f, distance - min_dist_to_keep - delay_distance);
					const float vel_max_posctrl = _param_mpc_xy_p.get() * stop_distance;
					const float vel_max_smooth = math::trajectory::computeMaxSpeedFromDistance(_param_mpc_jerk_max.get(),
								     _param_mpc_acc_hor.get(), stop_distance, 0.f);

					const float vel_max = math::min(vel_max_posctrl, vel_max_smooth);
					const float acc_max_postrl = _param_mpc_vel_p_acc.get() * math::min((vel_max - curr_vel_parallel), 0.f);

					if (acc_max_postrl < acc_vel_constraint) {
						acc_vel_constraint = acc_max_postrl;
						acc_vel_constraint_dir = bin_direction;
					}

					if (distance < closest_distance) {
						closest_distance = distance;
						bin_closest_dist = bin_direction;
					}

					// calculate closest distance for acceleration constraint


				} else if (_obstacle_map_body_frame.distances[i] == UINT16_MAX && i == sp_index) {
					if (!_param_cp_go_nodata.get() || (_param_cp_go_nodata.get() && _data_fov[i])) {
						setpoint_possible = false;
					}
				}
			}

			const Vector2f normal_component = bin_closest_dist * (setpoint_dir.dot(bin_closest_dist));
			const Vector2f tangential_component = setpoint_dir - normal_component;


			if (closest_distance < min_dist_to_keep && setpoint_possible) {
				float scale = (closest_distance - min_dist_to_keep); // always negative meaning it will push us away from the obstacle
				new_setpoint = tangential_component * setpoint_length  + bin_closest_dist * _param_mpc_xy_p.get() *
					       _param_mpc_vel_p_acc.get() *
					       scale; // scale is on the closest distance vector, as thats the critical direction

			} else if (closest_distance >= min_dist_to_keep && setpoint_possible) {
				const float scale_distance = math::max(min_dist_to_keep, _param_mpc_vel_manual.get() / _param_mpc_xy_p.get());
				float scale = (closest_distance - min_dist_to_keep) / scale_distance;
				scale *= scale; // square the scale to lower commanded accelerations close to the obstacle
				scale = math::min(scale, 1.0f);


				// only scale accelerations towards the obstacle
				if (bin_closest_dist.dot(setpoint_dir) > 0) {
					new_setpoint = (tangential_component + normal_component * scale) * setpoint_length;

				} else {
					new_setpoint = setpoint_dir * setpoint_length ;
				}

			}

			if (num_fov_bins == 0) {
				setpoint_accel.setZero();
				PX4_WARN("No fov bins");

			} else {
				acc_vel_constraint_setpoint = acc_vel_constraint_dir * acc_vel_constraint;
				setpoint_accel = new_setpoint + acc_vel_constraint_setpoint;
			}


		} else {
			// If no setpoint is provided, still apply force when you are close to an obstacle
			float closest_distance = INFINITY;
			matrix::Vector2f bin_closest_dist = {0.f, 0.f};

			for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
				if (constrain_time - _data_timestamps[i] > RANGE_STREAM_TIMEOUT_US) {
					_obstacle_map_body_frame.distances[i] = UINT16_MAX;
				}

				//count number of bins in the field of valid_new
				if (_obstacle_map_body_frame.distances[i] < UINT16_MAX) {
					num_fov_bins ++;
				}

				float angle = math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset);
				angle = wrap_2pi(vehicle_yaw_angle_rad + angle);

				// get direction of current bin
				const Vector2f bin_direction = {cosf(angle), sinf(angle)};

				if (_obstacle_map_body_frame.distances[i] > _obstacle_map_body_frame.min_distance
				    && _obstacle_map_body_frame.distances[i] < UINT16_MAX) {
					const float distance = _obstacle_map_body_frame.distances[i] * 0.01f;

					if (distance < closest_distance) {
						closest_distance = distance;
						bin_closest_dist = bin_direction;
					}
				}
			}

			if (closest_distance < min_dist_to_keep) {
				float scale = (closest_distance - min_dist_to_keep);
				new_setpoint = bin_closest_dist * _param_mpc_xy_p.get() * _param_mpc_vel_p_acc.get() * scale;
			}

			if (num_fov_bins == 0) {
				setpoint_accel.setZero();
				PX4_WARN("No fov bins");

			} else {
				setpoint_accel = new_setpoint;
			}
		}

	} else {
		//allow no movement
		PX4_WARN("No movement");
		setpoint_accel.setZero();

		// if distance data is stale, switch to Loiter
		if (getElapsedTime(&_last_timeout_warning) > 1_s && getElapsedTime(&_time_activated) > 1_s) {

			if ((constrain_time - _obstacle_map_body_frame.timestamp) > TIMEOUT_HOLD_US
			    && getElapsedTime(&_time_activated) > TIMEOUT_HOLD_US) {
				_publishVehicleCmdDoLoiter();
			}

			_last_timeout_warning = getTime();
		}
	}

}
void
CollisionPrevention::modifySetpoint(matrix::Vector2f &setpoint_accel, const matrix::Vector2f &setpoint_vel)
{
	//calculate movement constraints based on range data
	matrix::Vector2f new_setpoint = setpoint_accel;
	matrix::Vector2f original_setpoint = setpoint_accel;
	_constrainAccelerationSetpoint(new_setpoint, setpoint_vel);

	//warn user if collision prevention starts to interfere
	bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * _param_mpc_acc_hor.get()
				      || new_setpoint(0) > original_setpoint(0) + 0.05f * _param_mpc_acc_hor.get()
				      || new_setpoint(1) < original_setpoint(1) - 0.05f * _param_mpc_acc_hor.get()
				      || new_setpoint(1) > original_setpoint(1) + 0.05f * _param_mpc_acc_hor.get());

	_interfering = currently_interfering;

	// publish constraints
	collision_constraints_s	constraints{};
	constraints.timestamp = getTime();
	original_setpoint.copyTo(constraints.original_setpoint);
	new_setpoint.copyTo(constraints.adapted_setpoint);
	_constraints_pub.publish(constraints);

	setpoint_accel = new_setpoint;
}

void CollisionPrevention::_publishVehicleCmdDoLoiter()
{
	vehicle_command_s command{};
	command.timestamp = getTime();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;
	command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
	command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;

	// publish the vehicle command
	_vehicle_command_pub.publish(command);
}
