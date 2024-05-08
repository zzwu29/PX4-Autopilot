/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module_params.h>

#include <matrix/matrix/math.hpp>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <math.h>

#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/boat_drive_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_position.h>

#include <lib/pid/pid.h>
#include <lib/l1/ECL_L1_Pos_Controller.hpp>



/**
 * @brief Enum class for the different states of guidance.
 */
enum class GuidanceState {
	// TURNING, ///< The vehicle is currently turning.
	DRIVING, ///< The vehicle is currently driving straight.
	GOAL_REACHED ///< The vehicle has reached its goal.
};

/**
 * @brief Class for boat drive guidance.
 */
class BoatDriveGuidance : public ModuleParams
{
public:
	/**
	 * @brief Constructor for BoatDriveGuidance.
	 * @param parent The parent ModuleParams object.
	 */
	BoatDriveGuidance(ModuleParams *parent);
	~BoatDriveGuidance() = default;

	/**
	 * @brief Compute guidance for the vehicle.
	 * @param global_pos The global position of the vehicle in degrees.
	 * @param current_waypoint The current waypoint the vehicle is heading towards in degrees.
	 * @param next_waypoint The next waypoint the vehicle will head towards after reaching the current waypoint in degrees.
	 * @param vehicle_yaw The yaw orientation of the vehicle in radians.
	 * @param body_velocity The velocity of the vehicle in m/s.
	 * @param angular_velocity The angular velocity of the vehicle in rad/s.
	 * @param dt The time step in seconds.
	 */
	void computeGuidance(float yaw, float angular_velocity, float dt);

	/**
	 * @brief Set the maximum speed for the vehicle.
	 * @param max_speed The maximum speed in m/s.
	 * @return The set maximum speed in m/s.
	 */
	float setMaxSpeed(float max_speed) { return _max_speed = max_speed; }


	/**
	 * @brief Set the maximum angular velocity for the vehicle.
	 * @param max_angular_velocity The maximum angular velocity in rad/s.
	 * @return The set maximum angular velocity in rad/s.
	 */
	float setMaxAngularVelocity(float max_angular_velocity) { return _max_angular_velocity = max_angular_velocity; }

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};

	uORB::Publication<boat_drive_setpoint_s> _boat_drive_setpoint_pub{ORB_ID(boat_drive_setpoint)};

	position_setpoint_triplet_s _position_setpoint_triplet{};
	vehicle_global_position_s _vehicle_global_position{};

	GuidanceState _currentState; ///< The current state of guidance.

	float _desired_angular_velocity; ///< The desired angular velocity.

	float _max_speed; ///< The maximum speed.
	float _max_angular_velocity; ///< The maximum angular velocity.

	matrix::Vector2d _current_waypoint; ///< The current waypoint.

	VelocitySmoothing _forwards_velocity_smoothing; ///< The velocity smoothing for forward motion.
	PositionSmoothing _position_smoothing; ///< The position smoothing.

	PID_t _heading_p_controller; ///< The PID controller for yaw rate.

	enum WAYPOINT_CONTROL_STATE {
		NONE,
		LOITERING,
		BOATING_TO_A_POINT,
		BOATING_ON_A_LINE
	} _wp_ctrl_state{WAYPOINT_CONTROL_STATE::NONE};

	ECL_L1_Pos_Controller _l1_guidance;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RDD_P_HEADING>) _param_rdd_p_gain_heading,
		(ParamFloat<px4::params::NAV_ACC_RAD>) _param_nav_acc_rad,
		(ParamFloat<px4::params::RDD_MAX_JERK>) _param_rdd_max_jerk,
		(ParamFloat<px4::params::RDD_MAX_ACCEL>) _param_rdd_max_accel,

		(ParamFloat<px4::params::BT_L1_PERIOD>) _param_bt_l1_period,
		(ParamFloat<px4::params::BT_L1_DAMPING>) _param_bt_l1_damping,

		(ParamFloat<px4::params::BT_SPD_CRUISE>) _param_bt_spd_cruise,
		(ParamFloat<px4::params::BT_SPD_LIM>) _param_bt_spd_lim,
		(ParamFloat<px4::params::BT_SPD_MAX>) _param_bt_spd_max,
		(ParamFloat<px4::params::BT_SPD_IDLE>) _param_bt_spd_idle,
		(ParamFloat<px4::params::BT_SPD_SLEW>) _param_bt_spd_slew,

		(ParamFloat<px4::params::BT_SPD_P>) _param_bt_spd_p,
		(ParamFloat<px4::params::BT_SPD_I>) _param_bt_spd_i,
		(ParamFloat<px4::params::BT_SPD_IMAX>) _param_bt_spd_imax,
		(ParamFloat<px4::params::BT_SPD_OUTLIM>) _param_bt_spd_outlim,

		(ParamFloat<px4::params::BT_THR_LIM>) _param_bt_thr_lim,
		(ParamFloat<px4::params::BT_THR_SLEW>) _param_bt_thr_slew,

		(ParamFloat<px4::params::BT_LACC_LIM>) _param_bt_lacc_lim,
		(ParamFloat<px4::params::BT_LACC_SLEW>) _param_bt_lacc_slew,

		(ParamFloat<px4::params::BT_LACC_P>) _param_bt_lacc_p,
		(ParamFloat<px4::params::BT_LACC_I>) _param_bt_lacc_i,
		(ParamFloat<px4::params::BT_LACC_IMAX>) _param_bt_lacc_imax,
		(ParamFloat<px4::params::BT_YAWRATE_FF>) _param_bt_yawrate_ff,
		(ParamFloat<px4::params::BT_LACC_OUTLIM>) _param_bt_lacc_outlim,

		(ParamFloat<px4::params::BT_STR_GAIN>) _param_bt_str_gain,
		(ParamFloat<px4::params::BT_STR_SLEW>) _param_bt_str_slew,
		(ParamFloat<px4::params::BT_STR_DZ>) _param_bt_str_dz,

		(ParamBool<px4::params::BT_CCTRL_EN>) _param_bt_cctrl_en,
		(ParamInt<px4::params::BT_CCTRL_BTN>) _param_bt_cctrl_btn,

		(ParamFloat<px4::params::BT_MAX_HERR>) _param_bt_max_heading_error,
		(ParamFloat<px4::params::BT_MIN_HERR>) _param_bt_min_heading_error,
		(ParamFloat<px4::params::BT_MISSION_THR>) _param_bt_mission_throttle,
		(ParamFloat<px4::params::BT_MIN_THR>) _param_bt_min_throttle,

		(ParamFloat<px4::params::NAV_LOITER_RAD>) _param_nav_loiter_rad	/**< loiter radius for Rover */
	)
};
