// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* header */
#include <kalman_filter/kf_pos_vel_acc_plugin.h>

namespace kf_plugin
{

  void KalmanFilterPosVelAcc::initialize(ros::NodeHandle nh, string suffix, int id)
  {
    KalmanFilter::initialize(nh, suffix, id);
    input_name_v_ = {"acc"};
    measure_name_v_ = {"pos", "vel"};
  }

  /* be sure that the first parma should be timestamp */
    void KalmanFilterPosVelAcc::getPredictModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& state_transition_model, MatrixXd& control_input_model) const
  {
    assert(params.size() == 1);

    float dt = params[0];

    assert(dt >= 0);

    Matrix2d state_transition_model_temp;
    state_transition_model_temp << 1, dt, 0, 1;
    state_transition_model = state_transition_model_temp;

    Matrix<double, 2, 1> control_input_model_temp;
    control_input_model_temp << (dt * dt)/2, dt;
    control_input_model = control_input_model_temp;
  }

  /* be sure that the first parma is timestamp */
    void KalmanFilterPosVelAcc::getCorrectModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& observation_model) const
  {
    /* params: correct mode */
    assert(params.size() == 1);
    assert((int)params[0] <= VEL);

    Matrix<double, 1, 2> observation_model_temp;
    switch((int)params[0])
      {
      case POS:
        {
          observation_model_temp << 1, 0;
          break;
        }
      case VEL:
        {
          observation_model_temp << 0, 1;
          break;
        }
      }
    observation_model = observation_model_temp;
  }

  void KalmanFilterPosVelAccBias::initialize(ros::NodeHandle nh, string suffix, int id)
  {
    KalmanFilter::initialize(nh, suffix, id);

    input_name_v_ = {"acc", "bias"};
    measure_name_v_ = {"pos", "vel"};
  }

  void KalmanFilterPosVelAccBias::getPredictModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& state_transition_model, MatrixXd& control_input_model) const
  {
    assert(params.size() == 1);

    float dt = params[0];

    Matrix3d state_transition_model_temp;
    state_transition_model_temp << 1, dt, -dt*dt/2, 0, 1, -dt, 0, 0, 1;
    state_transition_model = state_transition_model_temp;

    Matrix<double, 3, 2> control_input_model_temp;
    control_input_model_temp << (dt * dt)/2, 0, dt, 0, 0, 1;
    control_input_model = control_input_model_temp;
  }

  void KalmanFilterPosVelAccBias::getCorrectModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& observation_model) const
  {
    /* params: correct mode */
    assert(params.size() == 1);
    assert((int)params[0] <= VEL);

    Matrix<double, 1, 3> observation_model_temp;
    switch((int)params[0])
      {
      case POS:
        {
          observation_model_temp << 1, 0, 0;
          break;
        }
      case VEL:
        {
          observation_model_temp << 0, 1, 0;
          break;
        }
      }
    observation_model = observation_model_temp;
  }
};


PLUGINLIB_EXPORT_CLASS(kf_plugin::KalmanFilterPosVelAcc, kf_plugin::KalmanFilter);
PLUGINLIB_EXPORT_CLASS(kf_plugin::KalmanFilterPosVelAccBias, kf_plugin::KalmanFilter);
