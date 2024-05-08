/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#include "GZMixingInterfaceBoatMotor.hpp"

bool GZMixingInterfaceBoatMotor::init(const std::string &model_name)
{

	std::string boat_motor_speed_topic = "/model/" + model_name + "/command/motor_speed";

	if (!_node.Subscribe(boat_motor_speed_topic, &GZMixingInterfaceBoatMotor::boat_motorSpeedCallback, this)) {
		PX4_ERR("failed to subscribe to %s", boat_motor_speed_topic.c_str());
		return false;
	}

	// cmd_thrust

	// std::string boat_left_motor_topic = "/model/boat_0/joint/left_propeller_joint/cmd_vel";
	// std::string boat_right_motor_topic = "/model/boat_0/joint/right_propeller_joint/cmd_vel";

	std::string boat_left_motor_topic = "/model/boat_wam_0/joint/left_propeller_joint/cmd_thrust";
	std::string boat_right_motor_topic = "/model/boat_wam_0/joint/right_propeller_joint/cmd_thrust";

	_actuators_pub_left = _node.Advertise<gz::msgs::Double>(boat_left_motor_topic);
	_actuators_pub_right = _node.Advertise<gz::msgs::Double>(boat_right_motor_topic);

	if (!_actuators_pub_left.Valid()) {
		PX4_ERR("failed to advertise %s", boat_left_motor_topic.c_str());
		return false;
	}

	// _boat_motor_encoders_pub.advertise();

	ScheduleNow();

	return true;
}

bool GZMixingInterfaceBoatMotor::updateOutputs(bool stop_boat_motors, uint16_t outputs[MAX_ACTUATORS],
		unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	unsigned active_output_count = 0;

	// printf("outputs: %f\n", (double)outputs[0]);
	// printf("outputs: %f\n", (double)outputs[1]);
	// printf("num_outputs: %d\n", (int)num_outputs);
	// printf(" \n");

	for (unsigned i = 0; i < num_outputs; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			active_output_count++;

		} else {
			break;
		}
	}

	if (active_output_count > 0) {
		gz::msgs::Actuators boat_motor_velocity_message;
		boat_motor_velocity_message.mutable_velocity()->Resize(active_output_count, 0);

		// for (unsigned i = 0; i < active_output_count; i++) {
		// 	// Offsetting the output allows for negative values despite unsigned integer to reverse the boat_motors
		// 	static constexpr float output_offset = 100.0f;
		// 	float scaled_output = (float)outputs[i] - output_offset;
		// 	// printf("scaled_output: %f\n", (double)scaled_output);
		// 	boat_motor_velocity_message.set_velocity(i, scaled_output);
		// }

		// printf("boat_motor_velocity_message: %f\n", (double)boat_motor_velocity_message.velocity(0));

		gz::msgs::Double boat_motor_velocity_message_right;
		boat_motor_velocity_message_right.set_data(outputs[0]);

		// printf("outputs[0]: %f\n", (double)(outputs[0] - 100.0f));
		// printf("boat_motor_velocity_message_right: %f\n", (double)boat_motor_velocity_message_right.data());

		gz::msgs::Double boat_motor_velocity_message_left;
		boat_motor_velocity_message_left.set_data(outputs[0]);

		// printf("boat_motor_velocity_message_left: %f\n", (double)boat_motor_velocity_message_left.data());
		// printf(" \n");



		if (_actuators_pub_left.Valid() && _actuators_pub_right.Valid()) {
			return (_actuators_pub_right.Publish(boat_motor_velocity_message_right)
				&& _actuators_pub_left.Publish(boat_motor_velocity_message_left));
		}

	}

	return false;
}

void GZMixingInterfaceBoatMotor::Run()
{
	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);
}

void GZMixingInterfaceBoatMotor::boat_motorSpeedCallback(const gz::msgs::Actuators &actuators)
{
	if (hrt_absolute_time() == 0) {
		return;
	}

	pthread_mutex_lock(&_node_mutex);

	// boat_motor_encoders_s boat_motor_encoders{};

	// for (int i = 0; i < actuators.velocity_size(); i++) {
	// 	// Convert from RPM to rad/s
	// 	boat_motor_encoders.boat_motor_speed[i] = (float)actuators.velocity(i) * (2.0f * M_PI_F / 60.0f);
	// }

	// if (actuators.velocity_size() > 0) {
	// 	boat_motor_encoders.timestamp = hrt_absolute_time();
	// 	_boat_motor_encoders_pub.publish(boat_motor_encoders);
	// }

	pthread_mutex_unlock(&_node_mutex);
}
