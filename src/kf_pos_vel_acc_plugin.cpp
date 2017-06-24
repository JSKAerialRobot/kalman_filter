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

    /* cfg init */
    server_ = new dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>(nhp_);
    dynamic_reconf_func_ = boost::bind(&KalmanFilterPosVelAcc::cfgCallback, this, _1, _2);
    server_->setCallback(dynamic_reconf_func_);
  }

  /* be sure that the first parma should be timestamp */
  void KalmanFilterPosVelAcc::updatePredictModel(const vector<double>& params)
  {
    assert(params.size() == 1);

    float dt = params[0];

    assert(dt >= 0);

    Matrix2d state_transition_model;
    state_transition_model << 1, dt, 0, 1;
    setStateTransitionModel(state_transition_model);

    Matrix<double, 2, 1> control_input_model;
    control_input_model << (dt * dt)/2, dt;
    setControlInputModel(control_input_model);
  }

  /* be sure that the first parma is timestamp */
  void KalmanFilterPosVelAcc::updateCorrectModel(const vector<double>& params)
  {
    /* params: correct mode */
    assert(params.size() == 1);
    assert((int)params[0] <= VEL);

    Matrix<double, 1, 2> observation_model;
    switch((int)params[0])
      {
      case POS:
        {
          observation_model << 1, 0;
          break;
        }
      case VEL:
        {
          observation_model << 0, 1;
          break;
        }
      }
    setObservationModel(observation_model);
  }

  void KalmanFilterPosVelAcc::cfgCallback(kalman_filter::KalmanFilterPosVelAccConfig &config, uint32_t level)
  {
    if(config.kalmanFilterFlag == true)
      {
        printf("cfg update, node: %s ", (nhp_.getNamespace()).c_str());

        switch(level)
          {
          case 1:  // INPUT_SIGMA = 1
            input_sigma_(0) = config.inputSigma;
            setPredictionNoiseCovariance();
            printf("change the input sigma\n");
            break;
          case 3:  // MEASURE_SIGMA = 3
            measure_sigma_(0) = config.measureSigma;
            setMeasurementNoiseCovariance();
            printf("change the measure sigma\n");
            break;
          default :
            printf("\n");
            break;
          }
      }
  }


  void KalmanFilterPosVelAccBias::initialize(ros::NodeHandle nh, string suffix, int id)
  {
    KalmanFilter::initialize(nh, suffix, id);

    //cfg init
    server_ = new dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>(nhp_);
    dynamic_reconf_func_ = boost::bind(&KalmanFilterPosVelAccBias::cfgCallback, this, _1, _2);
    server_->setCallback(dynamic_reconf_func_);
  }

  void KalmanFilterPosVelAccBias::updatePredictModel(const vector<double>& params)
  {
    assert(params.size() == 1);

    float dt = params[0];

    Matrix3d state_transition_model;
    state_transition_model << 1, dt, -dt*dt/2, 0, 1, -dt, 0, 0, 1;
    setStateTransitionModel(state_transition_model);

    Matrix<double, 3, 2> control_input_model;
    control_input_model << (dt * dt)/2, 0, dt, 0, 0, 1;
    setControlInputModel(control_input_model);
  }

  void KalmanFilterPosVelAccBias::updateCorrectModel(const vector<double>& params)
  {
    /* params: correct mode */
    assert(params.size() == 1);
    assert((int)params[0] <= VEL);

    Matrix<double, 1, 3> observation_model;
    switch((int)params[0])
      {
      case POS:
        {
          observation_model << 1, 0, 0;
          break;
        }
      case VEL:
        {
          observation_model << 0, 1, 0;
          break;
        }
      }
    setObservationModel(observation_model);
  }


  void KalmanFilterPosVelAccBias::cfgCallback(kalman_filter::KalmanFilterPosVelAccBiasConfig &config, uint32_t level)
  {
    if(config.kalmanFilterFlag == true)
      {
        printf("cfg update, node: %s", (nhp_.getNamespace()).c_str());

        switch(level)
          {
          case 1:  // INPUT_SIGMA = 1
            input_sigma_(0) = config.input1Sigma;
            setPredictionNoiseCovariance();
            printf("change the input1 sigma\n");
            break;
          case 2:  // BIAS_SIGMA = 2
            input_sigma_(1) = config.input2Sigma;
            setPredictionNoiseCovariance();
            printf("change the input2 sigma\n");
            break;
          case 3:  // MEASURE_SIGMA = 3
            measure_sigma_(0)  = config.measureSigma;
            setMeasurementNoiseCovariance();
            printf("change the measure sigma\n");
            break;
          default :
            printf("\n");
            break;
          }
      }
  }

};


PLUGINLIB_EXPORT_CLASS(kf_plugin::KalmanFilterPosVelAcc, kf_plugin::KalmanFilter);
PLUGINLIB_EXPORT_CLASS(kf_plugin::KalmanFilterPosVelAccBias, kf_plugin::KalmanFilter);
