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

/* for dynamic reconfigure */
#include <dynamic_reconfigure/server.h>
#include <kalman_filter/KalmanFilterConfig.h>

/* math */
#include <Eigen/Core>
#include <Eigen/Dense>

/* util */
#include <iostream>
#include <vector>
#include <deque>

/* for mutex */
#include <mutex>
#include <thread>

using namespace Eigen;
using namespace std;

namespace kf_plugin
{

  class KalmanFilter
  {
  public:
    KalmanFilter():
      state_dim_(0), input_start_flag_(false), measure_start_flag_(false),
      est_state_buf_(0), est_cov_buf_(0), input_buf_(0), params_buf_(0), timestamp_buf_(0)
    {
    }

    virtual ~KalmanFilter(){}

    virtual void initialize(ros::NodeHandle nh, string suffix, int id )
    {
      id_ = id;
      nh_ = nh;
      nhp_ = ros::NodeHandle(nh, "kf/" + suffix);

      if(state_dim_ == 0)
        throw std::runtime_error("the state dimension is zero");

      /* init state and covariance */
      estimate_state_ = VectorXd::Zero(state_dim_);
      estimate_covariance_ = MatrixXd::Zero(state_dim_, state_dim_);

      rosParamInit();

      /* cfg init */
      cfg_server_ptr_ = boost::shared_ptr< dynamic_reconfigure::Server<kalman_filter::KalmanFilterConfig> >(new dynamic_reconfigure::Server<kalman_filter::KalmanFilterConfig>(nhp_));
      cfg_server_ptr_->setCallback(boost::bind(&KalmanFilter::cfgCallback, this, _1, _2));
    }

    virtual bool prediction(const VectorXd& input, /* the vector of input */
                            const double timestamp, /* the timestamp of prediction state(which will be the timestamp for whole system) */
                            const vector<double>& params = vector<double>(0) /* the vector of param for predict model */)
    {
      if(!getFilteringFlag()) return false;

      /* update the model */
      updatePredictModel(params);
      /* propagation of state */
      statePropagation(input);
      /* propagation of covariance */
      /* for time sync */
      if(time_sync_)
        {
          if(debug_verbose_) std::cout  << nhp_.getNamespace() << ": prediction: time sync" << std::endl;

          /*************************************************/
          // Stephan Weiss, et.al,
          // "Versatile Distributed Pose Estimation and Sensor
          // Self-Calibration for an Autonomous MAV"
          /*************************************************/

          std::lock_guard<std::recursive_mutex> lock(kf_mutex_);

          /* add the latest information to the ring buffer */
          est_state_buf_.push_back(estimate_state_);
          input_buf_.push_back(input);
          params_buf_.push_back(params); // the predict model may change due to the repropagation
          timestamp_buf_.push_back(timestamp);

          /*
            Propagation of convariance has delay compared with state.
            Convariance propagation starts to the last measurement correction
          */
          MatrixXd state_transition_model;
          MatrixXd control_input_model;
          getPredictModel(params_buf_.at(est_cov_buf_.size()),
                          est_state_buf_.at(est_cov_buf_.size()),
                          state_transition_model, control_input_model);
          estimate_covariance_ = covariancePropagation(estimate_covariance_,
                                                       state_transition_model,
                                                       control_input_model,
                                                       input_noise_covariance_);
          est_cov_buf_.push_back(estimate_covariance_);
        }
      else
        covariancePropagation();

      if(debug_verbose_)
        {
          std::lock_guard<std::recursive_mutex> lock(kf_mutex_);

          cout << "state transition model: \n" << state_transition_model_ << endl;
          cout << "control input model: \n" << control_input_model_ << endl;

          cout << "prediction: estimate_state: \n" << estimate_state_.transpose() << endl;
          cout << "prediction: estimate_covariance: \n" << estimate_covariance_ << endl;
        }

      return true;
    }

    virtual bool correction(const VectorXd& measurement, /* the vector of measurement */
                            const double timestamp, /* timestamp of the measure state */
                            const vector<double>& params = vector<double>(0) /* the vector of param for correct model, the first param should be timestamp */)
    {
      if(!getFilteringFlag()) return false;

      /* update the model */
      updateCorrectModel(params);

      /* correction */
      VectorXd estimate_state;
      MatrixXd estimate_covariance;
      getTimeSyncPropagationResult(estimate_state, estimate_covariance, timestamp);

      MatrixXd kalman_gain;
      MatrixXd inovation_covariance;
      { //lock
        std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
        inovation_covariance = observation_model_ * estimate_covariance * observation_model_.transpose() + measurement_noise_covariance_;
        kalman_gain = estimate_covariance * observation_model_.transpose() * inovation_covariance.inverse();
        estimate_covariance_ = (MatrixXd::Identity(estimate_state.size(), estimate_state.size()) - kalman_gain * observation_model_) * estimate_covariance;
        estimate_state_ = estimate_state + kalman_gain * (measurement - observation_model_ * estimate_state);
      }

      if(time_sync_) rePropagation();

      if(debug_verbose_)
        {
          std::lock_guard<std::recursive_mutex> lock(kf_mutex_);

          cout << "estimate_state" << endl <<  estimate_state_ << endl;
          cout << "kalman_gain" << endl << kalman_gain  << endl;

          cout << "state_transition_model" << endl <<  state_transition_model_ << endl;
          cout << "control_input_model" << endl << control_input_model_  << endl;
          cout << "observation_model" << endl << observation_model_  << endl;

          cout << "estimate_covariance" << endl << estimate_covariance_  << endl;
          cout << "inovation_covariance" << endl << inovation_covariance  << endl;
        }

      return true;
    }

    void setEstimateState(const VectorXd& state)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      estimate_state_ = state;
    }

    void resetState()
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      estimate_state_.setZero();
    }

    virtual void setInputSigma( VectorXd input_sigma_v)
    {
      input_sigma_v_ = input_sigma_v;
      setPredictionNoiseCovariance();
    }

    virtual void setMeasureSigma( VectorXd measure_sigma_v)
    {
      measure_sigma_v_ = measure_sigma_v;
      setMeasurementNoiseCovariance();
    }

    void setPredictionNoiseCovariance()
    {
      MatrixXd input_sigma_m = (input_sigma_v_).asDiagonal();
      input_noise_covariance_ = input_sigma_m * input_sigma_m;
    }

    void setMeasurementNoiseCovariance()
    {
      MatrixXd measure_sigma_m = (measure_sigma_v_).asDiagonal();
      measurement_noise_covariance_ = measure_sigma_m * measure_sigma_m;
    }

    inline const int getId() const { return id_;}
    inline void setId(int id) { id_ = id;}

    const VectorXd& getEstimateState()
    {
      /* can not add const suffix for this function, because of the lock_guard */
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      return estimate_state_;
    }

    inline const MatrixXd getEstimateCovariance() const
    {
      /* TODO: this not correct, if we consider the delay (latency) in time_sync mode */
      return estimate_covariance_;
    }

    inline const VectorXd& getInputSigma()
    {
      return input_sigma_v_;
    }

    inline const VectorXd& setMeasureSigma()
    {
      return measure_sigma_v_;
    }

    inline const int getStateDim() const {return state_dim_;}
    inline const int getInputDim() const {return input_sigma_v_.size();}
    inline const int getMeasureDim() const {return measure_sigma_v_.size();}

    inline void setStateTransitionModel(MatrixXd state_transition_model){state_transition_model_ = state_transition_model;}
    inline void setControlInputModel(MatrixXd control_input_model){control_input_model_ = control_input_model;}
    inline void setObservationModel(MatrixXd observation_model){observation_model_ = observation_model;}

    inline void setInputFlag(bool flag = true) { input_start_flag_ = flag; }
    inline void setMeasureFlag(bool flag = true) { measure_start_flag_ = flag; }

    inline const bool getFilteringFlag() const
    {
      if(input_start_flag_ && measure_start_flag_) return true;
      else return false;
    }

    void setInitState(double state_value, int no)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      estimate_state_(no) = state_value;
    }

    inline void setInitState(VectorXd init_state)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      estimate_state_ = init_state;
    }

    virtual void getPredictModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& state_transition_model, MatrixXd& control_input_model) const = 0;
    virtual void getCorrectModel(const vector<double>& params, const VectorXd& estimate_state, MatrixXd& observation_model) const = 0;

    inline void setTimeSync(const bool flag){time_sync_ = flag;}
    inline double getTimestamp()
    {
      if(timestamp_buf_.size() == 0) return 0;
      return timestamp_buf_.back();
    }
    inline void setDebugVerbose(bool flag)
    {
      debug_verbose_ = flag;
    }

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    bool param_verbose_;
    bool debug_verbose_;
    int id_;
    bool time_sync_;

    int state_dim_;
    VectorXd estimate_state_;
    MatrixXd estimate_covariance_;

    vector<string> input_name_v_, measure_name_v_;
    VectorXd input_sigma_v_, measure_sigma_v_;
    MatrixXd input_noise_covariance_, measurement_noise_covariance_;

    MatrixXd state_transition_model_;
    MatrixXd control_input_model_;
    MatrixXd observation_model_;

    /* buffer for time sync */
    deque<Eigen::VectorXd> est_state_buf_;
    deque<Eigen::MatrixXd> est_cov_buf_;
    deque<Eigen::VectorXd> input_buf_;
    deque< vector<double> > params_buf_;
    deque<double> timestamp_buf_;
    //int convariance_propagate_index_;

    /* filtering start flag */
    bool input_start_flag_;
    bool measure_start_flag_;

    /* for mutex */
    std::recursive_mutex kf_mutex_;

    /* dynamic reconfigure */
    boost::shared_ptr< dynamic_reconfigure::Server<kalman_filter::KalmanFilterConfig> > cfg_server_ptr_;

    void rosParamInit()
    {
      string ns = nhp_.getNamespace();

      ros::NodeHandle global_nh("~");

      global_nh.param("param_verbose", param_verbose_, true);
      global_nh.param("debug_verbose", debug_verbose_, false);

      nhp_.param("time_sync", time_sync_, false);

      if(param_verbose_)
        {
          cout << ns << ": time sync is " << time_sync_ << endl;
        }
    }

    void updatePredictModel(const vector<double>& params)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      MatrixXd state_transition_model, control_input_model;
      getPredictModel(params, estimate_state_, state_transition_model, control_input_model);

      state_transition_model_ = state_transition_model;
      control_input_model_ = control_input_model;
    }

    void updateCorrectModel(const vector<double>& params)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      MatrixXd observation_model;
      getCorrectModel(params, estimate_state_, observation_model);

      observation_model_ = observation_model;
    }

    virtual const VectorXd statePropagation(const VectorXd& estimate_state,
                                            const VectorXd& input,
                                            const MatrixXd& state_transition_model,
                                            const MatrixXd& control_input_model) const
    {
      return state_transition_model * estimate_state + control_input_model * input;
    }

    void statePropagation(VectorXd input)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      estimate_state_ = statePropagation(estimate_state_, input, state_transition_model_, control_input_model_);
    }

    virtual const MatrixXd covariancePropagation(const MatrixXd& estimate_covariance,
                                                 const MatrixXd& state_transition_model,
                                                 const MatrixXd& control_input_model,
                                                 const MatrixXd& input_noise_covariance) const
    {
      return state_transition_model * estimate_covariance * state_transition_model.transpose() + control_input_model * input_noise_covariance * control_input_model.transpose();
    }

    void covariancePropagation()
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
      estimate_covariance_ = covariancePropagation(estimate_covariance_, state_transition_model_,
                                                   control_input_model_, input_noise_covariance_);
    }

    // virtual const VectorXd getResidual(VectorXd measurement, VectorXd estimate_state) {}

    /* Stephan Weiss, et.al,
       "Versatile Distributed Pose Estimation and Sensor
       Self-Calibration for an Autonomous MAV"
    */
    void getTimeSyncPropagationResult(VectorXd& estimate_state, MatrixXd& estimate_covariance, double timestamp)
    {
      std::lock_guard<std::recursive_mutex> lock(kf_mutex_);

      estimate_state = estimate_state_;
      estimate_covariance = estimate_covariance_;

      if(time_sync_)
        {
          if(debug_verbose_) std::cout << nhp_.getNamespace() << ": correction time sync" << std::endl;

          /* no predict state stored: measurement is faster than input */
          if(timestamp_buf_.size() == 0)
            {
              assert(est_cov_buf_.size() == 0);
              return;
            }

          size_t index = timestamp_buf_.size() - 1; // max

          if(debug_verbose_)
            cout << nhp_.getNamespace() << ": start timestamp searching from end index: " << timestamp_buf_.size() << endl;
          for(auto it = timestamp_buf_.begin(); it != timestamp_buf_.end(); ++it)
            {
              /* future timestamp, escape */
              if(*it > timestamp)
                {
                  index = distance(timestamp_buf_.begin(), it);

                  /* should assign the state before the measure timestamp */
                  if(index > 0) index--;

                  if(debug_verbose_)
                    cout << nhp_.getNamespace() << ": correction, time_sync: exit from timestamp searching, index: " << index << ", min_diff of timestamp: " << timestamp - *it << endl;
                  break;
                }
            }

          estimate_state = est_state_buf_.at(index);
          if(est_cov_buf_.size() > 0) estimate_covariance = est_cov_buf_.back();

          /* find the propagated covariance in correct time stamp */
          if(est_cov_buf_.size() < index + 1)
            {
              if(debug_verbose_)
                cout << nhp_.getNamespace()  << ": correction, time_sync: no enough covariance propagation before correction: " << index + 1 - est_cov_buf_.size() << endl;

              /* propagate residual convariance */
              MatrixXd state_transition_model;
              MatrixXd control_input_model;
              for(size_t i = est_cov_buf_.size(); i <  index + 1; i++)
                {
                  getPredictModel(params_buf_.at(i), est_state_buf_.at(i),
                                  state_transition_model, control_input_model);
                  estimate_covariance = covariancePropagation(estimate_covariance,
                                                              state_transition_model,
                                                              control_input_model,
                                                              input_noise_covariance_);
                }
            }

          if(est_cov_buf_.size() > index + 1)
            {
              if(debug_verbose_)
                cout << nhp_.getNamespace() << "correction, time_sync: exceeded covariance propagation than correction timestamp: " << est_cov_buf_.size() - (index + 1) << endl;
              estimate_covariance = est_cov_buf_.at(index);
            }

          if(debug_verbose_)
            {
              if(est_cov_buf_.size() == index + 1) std::cout << nhp_.getNamespace()  << ": correction, time_sync:  same size" << std::endl;
            }

          /* remove unnecessary part */
          est_state_buf_.erase(est_state_buf_.begin(), est_state_buf_.begin() + index + 1);
          input_buf_.erase(input_buf_.begin(), input_buf_.begin() + index + 1);
          params_buf_.erase(params_buf_.begin(), params_buf_.begin() + index + 1);
          timestamp_buf_.erase(timestamp_buf_.begin(), timestamp_buf_.begin() + index + 1);
          est_cov_buf_.clear();
        }
    }

    void rePropagation()
    {
      {//lock
        std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
        assert(est_state_buf_.size() == params_buf_.size());
        if(params_buf_.size() == 0) return;

        /* iteration re-propagation */
        if(debug_verbose_)
          cout << nhp_.getNamespace() << ": start re-propagation for " << params_buf_.size() << endl;
      }

      for(auto it = params_buf_.begin(); it != params_buf_.end(); ++it)
        {
          size_t index = distance(params_buf_.begin(), it);

          /* update the model */
          updatePredictModel(*it);

          /* only state propagation */
          statePropagation(input_buf_.at(index));

          /* update the buffer */
          est_state_buf_.at(index) = estimate_state_;
          //std::cout << nhp_.getNamespace() << ": estimate state: " << estimate_state_.transpose() << std::endl;
        }
    }

    virtual void cfgCallback(kalman_filter::KalmanFilterConfig &config, uint32_t level)
    {
      if(config.kalman_filter_flag == true)
        {
          ROS_INFO_STREAM(nhp_.getNamespace() << " cfg update");

          switch(level)
            {
            case 1:  // hard code: INPUT_ID = 1
              {
                // check whether have the desired input id
                if (config.input_id > input_sigma_v_.size() - 1)
                  {
                    ROS_ERROR_STREAM(nhp_.getNamespace() << ": the input id from cfg does not exist, the max id in kalman filter is " << input_sigma_v_.size() - 1);
                    break;
                  }

                ROS_INFO_STREAM(nhp_.getNamespace() << ": check the input id: " << config.input_id << " which is " << input_name_v_.at(config.input_id));
                break;
              }
            case 2:  // hard code: INPUT_SIGMA = 2
              {
                if (config.input_id > input_sigma_v_.size() - 1)
                  {
                    ROS_ERROR_STREAM(nhp_.getNamespace() << ": the input id from cfg does not exist, the max id in kalman filter is " << input_sigma_v_.size() - 1);
                    break;
                  }

                ROS_INFO_STREAM(nhp_.getNamespace() << ": change the sigma of " << input_name_v_.at(config.input_id));
                input_sigma_v_(config.input_id) = config.input_sigma;
                setPredictionNoiseCovariance();
              break;
              }
            case 3:  // hard code: MEASURE_ID = 3
              {
                // check whether have the desired measure id
                if (config.measure_id > measure_sigma_v_.size() - 1)
                  {
                    ROS_ERROR_STREAM(nhp_.getNamespace() << ": the measure id from cfg does not exist, the max id in kalman filter is " << measure_sigma_v_.size() - 1);
                    break;
                  }

                ROS_INFO_STREAM(nhp_.getNamespace() << ": check the measure id: " << config.measure_id << " which is " << measure_name_v_.at(config.measure_id));
                break;
              }
            case 4:  // hard code: MEASURE_SIGMA = 4
              {
                if (config.measure_id > measure_sigma_v_.size() - 1)
                  {
                    ROS_ERROR_STREAM(nhp_.getNamespace() << ": the measure id from cfg does not exist, the max id in kalman filter is " << measure_sigma_v_.size() - 1);
                    break;
                  }

                ROS_INFO_STREAM(nhp_.getNamespace() << ": change the sigma of " << measure_name_v_.at(config.measure_id));
                measure_sigma_v_(config.measure_id) = config.measure_sigma;
                setMeasurementNoiseCovariance();
              break;
              }
            default :
              break;
            }
        }
    }
  };
};
#endif
