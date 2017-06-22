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

/* base class */
#include <kalman_filter/kf_base_plugin.h>

/* plugin */
#include <pluginlib/class_list_macros.h>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <kalman_filter/KalmanFilterPosVelAccConfig.h>
#include <kalman_filter/KalmanFilterPosVelAccBiasConfig.h>

namespace kf_plugin
{
  enum CORRECT_MODE{POS = 0, VEL = 1, POS_VEL = 2,};

  class KalmanFilterPosVelAcc : public kf_plugin::KalmanFilter
  {
  public:
    KalmanFilterPosVelAcc(): KalmanFilter(2 /* state dim */,
                                          1 /* input dim */,
                                          1 /* measure dim */) {}

    ~KalmanFilterPosVelAcc() {}

    void initialize(ros::NodeHandle nh, string suffix, int id);

    void updatePredictModel(double dt);

    void updateCorrectModel(const vector<double>& params);

    bool prediction(double acc);

    bool correction(vector<double>& meas, uint8_t correct_mode);

  private:
    //dynamic reconfigure
    dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>* server_;
    dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccConfig>::CallbackType dynamic_reconf_func_;

    void cfgCallback(kalman_filter::KalmanFilterPosVelAccConfig &config, uint32_t level);

  };

  class KalmanFilterPosVelAccBias : public kf_plugin::KalmanFilter
  {
  public:
    KalmanFilterPosVelAccBias(): KalmanFilter(3 /* state dim */,
                                              2 /* input dim */,
                                              1 /* measure dim */) {}
    ~KalmanFilterPosVelAccBias() {}

    void initialize(ros::NodeHandle nh, string suffix, int id);

    void updatePredictModel(double dt);

    void updateCorrectModel(const vector<double>& params);

    bool prediction(double acc);

    bool correction(vector<double>& meas, uint8_t correct_mode);

  private:
    //dynamic reconfigure
    dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>* server_;
    dynamic_reconfigure::Server<kalman_filter::KalmanFilterPosVelAccBiasConfig>::CallbackType dynamic_reconf_func_;

    void cfgCallback(kalman_filter::KalmanFilterPosVelAccBiasConfig &config, uint32_t level);

  };
};

