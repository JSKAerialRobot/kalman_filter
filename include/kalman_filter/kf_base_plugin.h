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

#ifndef KALMAN_FILTER_PLUGIN_H
#define KALMAN_FILTER_PLUGIN_H

/* ros */
#include <ros/ros.h>

/* math */
#include <Eigen/Core>
#include <Eigen/Dense>

/* util */
#include <iostream>
#include <vector>

/* for mutex */
#include <boost/version.hpp>
#include <boost/thread/mutex.hpp>
#if BOOST_VERSION>105200
 #include <boost/thread/lock_guard.hpp>
#endif

using namespace Eigen;
using namespace std;

namespace kf_plugin
{

  class KalmanFilter
  {
  public:
    KalmanFilter(int state_dim, int input_dim, int measure_dim):
      state_dim_(state_dim), input_dim_(input_dim), measure_dim_(measure_dim),
      input_start_flag_(false), measure_start_flag_(false)
    {
      state_transition_model_ = MatrixXd::Zero(state_dim_, state_dim_);
      control_input_model_ = MatrixXd::Zero(state_dim_, input_dim_);
      observation_model_ = MatrixXd::Zero(measure_dim_, state_dim_);

      input_sigma_ = VectorXd::Zero(input_dim_);
      measure_sigma_ = VectorXd::Zero(measure_dim_);

      estimate_state_ = VectorXd::Zero(state_dim_);

      estimate_covariance_ = MatrixXd::Zero(state_dim_, state_dim_);
      prediction_noise_covariance_ = MatrixXd::Zero(state_dim_, state_dim_);

      measurement_noise_covariance_ = MatrixXd::Zero(measure_dim_, measure_dim_);
      inovation_covariance_ = MatrixXd::Zero(measure_dim_, measure_dim_);
      kalman_gain_  = MatrixXd::Zero(state_dim_, measure_dim_);

    }

    virtual ~KalmanFilter(){}

    virtual void initialize(ros::NodeHandle nh, string suffix, int id )
    {
      id_ = id;
      nh_ = nh;
      nhp_ = ros::NodeHandle(nh, "kf/" + suffix);

      rosParamInit();

      setPredictionNoiseCovariance();
      setMeasurementNoiseCovariance();
    }

    virtual bool prediction(VectorXd  input, bool debug = false)
    {
      if(!getFilteringFlag()) return false;

      /* lock */
      {
        boost::lock_guard<boost::mutex> lock(kf_mutex_);

        statePropagation(input);
        covariancePropagation();

        if(debug)
          {
            std::cout << "prediction: estimate_state" << estimate_state_ << std::endl;
            std::cout << "prediction: estimate_covariance" << estimate_covariance_ << std::endl;
          }

      }
      return true;
    }

    virtual bool correction(VectorXd measurement, bool debug = false)
    {
      if(!getFilteringFlag()) return false;

      /* lock */
      {
        boost::lock_guard<boost::mutex> lock(kf_mutex_);

        kalmanGain();
        stateCorrection(measurement);
        covarianceCorrection();
        if(debug)
          {
            std::cout << "estimate_state_" << std::endl <<  estimate_state_ << std::endl;
            std::cout << "kalman_gain_" << std::endl << kalman_gain_  << std::endl;

            std::cout << "state_transition_model_" << std::endl <<  state_transition_model_ << std::endl;
            std::cout << "control_input_model_" << std::endl << control_input_model_  << std::endl;
            std::cout << "observation_model_" << std::endl << observation_model_  << std::endl;

            std::cout << "estimate_covariance_" << std::endl << estimate_covariance_  << std::endl;
            std::cout << "measurement_noise_covariance_" << std::endl << measurement_noise_covariance_  << std::endl;
            std::cout << "inovation_covariance_" << std::endl << inovation_covariance_  << std::endl;

          }

      }
      return true;
    }

    /* default: linear propagation */
    virtual void statePropagation(VectorXd input)
    {
      VectorXd estimate_hat_state = state_transition_model_ * estimate_state_ + control_input_model_ * input;
      estimate_state_ = estimate_hat_state;
    }

    virtual void covariancePropagation()
    {
      MatrixXd estimate_bar_covariance
        = state_transition_model_ * estimate_covariance_ * state_transition_model_.transpose()
        + prediction_noise_covariance_;
      estimate_covariance_ = estimate_bar_covariance;
    }

    virtual void kalmanGain()
    {
      inovation_covariance_ = observation_model_ * estimate_covariance_ * observation_model_.transpose()
        + measurement_noise_covariance_;
      kalman_gain_ = estimate_covariance_ * observation_model_.transpose() * inovation_covariance_.inverse();
    }

    virtual void covarianceCorrection()
    {
      MatrixXd estimate_covariance_tmp = (MatrixXd::Identity(state_dim_, state_dim_) - kalman_gain_ * observation_model_) * estimate_covariance_;
      estimate_covariance_ = estimate_covariance_tmp;
    }

    /* default: linear propagation */
    virtual void stateCorrection(VectorXd measurement)
    {
      VectorXd estimate_state = estimate_state_ + kalman_gain_ * (measurement - observation_model_ * estimate_state_);
      estimate_state_ = estimate_state;
    }

    void setEstimateState(const VectorXd& state)
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);
      estimate_state_ = state;
    }

    void resetState()
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);
      estimate_state_ = VectorXd::Zero(state_dim_);
    }

    void setInputSigma( VectorXd input_sigma)
    {
      assert(input_sigma_.size() == input_sigma.size());
      input_sigma_ = input_sigma;
      setPredictionNoiseCovariance();
    }

    void setMeasureSigma( VectorXd measure_sigma)
    {
      assert(measure_sigma_.size() == measure_sigma.size());
      measure_sigma_ = measure_sigma;
      setMeasurementNoiseCovariance();
    }

    void setPredictionNoiseCovariance()
    {
      assert(input_sigma_.size() == input_dim_);

      MatrixXd input_sigma_m = (input_sigma_).asDiagonal();
      prediction_noise_covariance_ = control_input_model_ * (input_sigma_m * input_sigma_m) * control_input_model_.transpose();
    }

    void setMeasurementNoiseCovariance()
    {
      assert(measure_sigma_.size() == measure_dim_);
      MatrixXd measure_sigma_m = (measure_sigma_).asDiagonal();
      measurement_noise_covariance_ = measure_sigma_m * measure_sigma_m;
    }

    inline int getId() { return id_;}
    inline void setId(int id) { id_ = id;}

    VectorXd getEstimateState()
      {
        boost::lock_guard<boost::mutex> lock(kf_mutex_);
        return estimate_state_;
      }

    void getEstimateState(vector<double>& estiamte_state )
      {
        boost::lock_guard<boost::mutex> lock(kf_mutex_);
         estimate_state_;
      }

    inline MatrixXd getEstimateCovariance(){ return estimate_covariance_;}

    inline int getStateDim(){return state_dim_;}
    inline int getInputDim(){return input_dim_;}
    inline int getMeasureDim(){return measure_dim_;}

    inline void setStateTransitionModel(MatrixXd state_transition_model){state_transition_model_ = state_transition_model;}
    inline void setControlInputModel(MatrixXd control_input_model){control_input_model_ = control_input_model;}
    inline void setObservationModel(MatrixXd observation_model){observation_model_ = observation_model;}

    inline void setInputFlag(bool flag = true){input_start_flag_ = flag; }
    inline void setMeasureFlag(bool flag = true){ measure_start_flag_ = flag;}

    inline bool getFilteringFlag()
    {
      if(input_start_flag_ && measure_start_flag_)
        return true;
      else
        return false;
    }

    void setInitState(double state_value, int no)
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);
      estimate_state_(no) = state_value;
    }

    inline void setInitState(VectorXd init_state)
    {
      boost::lock_guard<boost::mutex> lock(kf_mutex_);
      estimate_state_ = init_state;
    }

    virtual void updatePredictModel(const vector<double>& params){}
    virtual void updateCorrectModel(const vector<double>& params){}

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    bool param_verbose_;
    int state_dim_, input_dim_,  measure_dim_;
    int id_;

    VectorXd input_sigma_,   measure_sigma_;
    VectorXd estimate_state_;

    MatrixXd prediction_noise_covariance_, estimate_covariance_;

    MatrixXd measurement_noise_covariance_;
    MatrixXd inovation_covariance_;
    MatrixXd kalman_gain_;

    MatrixXd state_transition_model_;
    MatrixXd control_input_model_;
    MatrixXd observation_model_;

    //filtering start flag
    bool input_start_flag_;
    bool measure_start_flag_;

    //for mutex
    boost::mutex kf_mutex_;

    void rosParamInit()
    {
      std::string ns = nhp_.getNamespace();

      nhp_.param("param_verbose", param_verbose_, true);

      for(int i = 0; i < input_dim_; i ++)
        {
          std::stringstream input_sigma_no;
          input_sigma_no << i + 1;
          nhp_.param(std::string("input_sigma") + input_sigma_no.str(), input_sigma_(i), 0.0);
        }

      for(int i = 0; i < measure_dim_; i ++)
        {
          std::stringstream measure_sigma_no;
          measure_sigma_no << i + 1;
          nhp_.param(std::string("measure_sigma") + measure_sigma_no.str(), measure_sigma_(i), 0.0);
        }

      if(param_verbose_)
        {
          cout << ns << ": measure_dim  is " << measure_dim_ << endl;
          cout << ns << ": input_dim  is " << input_dim_ << endl;
          cout << ns << ": state_dim  is " << state_dim_ << endl;
          cout << ns << ": input_sigma  is " << input_sigma_ << endl;
          cout << ns << ": measure_sigma  is " << measure_sigma_ << endl;
        }

    }

  };
};
#endif
