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
#include <deque>

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
      input_start_flag_(false), measure_start_flag_(false),
      est_state_buf_(0), est_cov_buf_(0), input_buf_(0), params_buf_(0), timestamp_buf_(0)
    {
      state_transition_model_ = MatrixXd::Zero(state_dim_, state_dim_);
      control_input_model_ = MatrixXd::Zero(state_dim_, input_dim_);
      observation_model_ = MatrixXd::Zero(measure_dim_, state_dim_);

      input_sigma_ = VectorXd::Zero(input_dim_);
      measure_sigma_ = VectorXd::Zero(measure_dim_);

      estimate_state_ = VectorXd::Zero(state_dim_);
      estimate_covariance_ = MatrixXd::Zero(state_dim_, state_dim_);

      input_noise_covariance_ = MatrixXd::Zero(input_dim_, input_dim_);
      measurement_noise_covariance_ = MatrixXd::Zero(measure_dim_, measure_dim_);
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

    virtual bool prediction(const VectorXd& input, /* the vector of input */
                            const vector<double>& params = vector<double>(0), /* the vector of param for predict model */
                            const double timestamp = 0 /* the timestamp of prediction state(which will be the timestamp for whole system) */
                            )
    {
      if(!getFilteringFlag()) return false;

      /* lock */
      {
        boost::lock_guard<boost::mutex> lock(kf_mutex_);

        /* update the model */
        updatePredictModel(params);

        /* propagation */
        estimate_state_ = statePropagation(input, estimate_state_);
        estimate_covariance_ = covariancePropagation(input, estimate_covariance_);

        /* for time sync */
        if(time_sync_)
          {
            /* update the timestamp  */
            est_state_buf_.push_back(estimate_state_);
            est_cov_buf_.push_back(estimate_covariance_);
            input_buf_.push_back(input);
            params_buf_.push_back(params);
            timestamp_buf_.push_back(timestamp);
           }

        if(debug_verbose_)
          {
            cout << "state transition model: " << state_transition_model_ << endl;
            cout << "control input model:" << control_input_model_ << endl;

            cout << "prediction: estimate_state" << estimate_state_ << endl;
            cout << "prediction: estimate_covariance" << estimate_covariance_ << endl;
          }

      }
      return true;
    }

    virtual bool correction(const VectorXd& measurement, /* the vector of measurement */
                            const vector<double>& params = vector<double>(0), /* the vector of param for correct model, the first param should be timestamp */
                            const double timestamp = 0/* timestamp of the measure state */
                            )
    {
      if(!getFilteringFlag()) return false;

      /* lock */
      {
        boost::lock_guard<boost::mutex> lock(kf_mutex_);

        /* update the model */
        updateCorrectModel(params);

        /* correction */
        VectorXd estimate_state = estimate_state_;
        MatrixXd estimate_covariance = estimate_covariance_;

        if(time_sync_)
          {
            if(timestamp <= 0) return false;
            getTimeSyncState(estimate_state, estimate_covariance, timestamp);
          }

        MatrixXd inovation_covariance = observation_model_ * estimate_covariance * observation_model_.transpose() + measurement_noise_covariance_;
        MatrixXd kalman_gain = estimate_covariance * observation_model_.transpose() * inovation_covariance.inverse();
        estimate_covariance_ = (MatrixXd::Identity(state_dim_, state_dim_) - kalman_gain * observation_model_) * estimate_covariance;
        estimate_state_ = estimate_state + kalman_gain * getResidual(measurement, estimate_state);

        if(time_sync_) rePropagation();

        if(debug_verbose_)
          {
            cout << "estimate_state" << endl <<  estimate_state_ << endl;
            cout << "kalman_gain" << endl << kalman_gain  << endl;

            cout << "state_transition_model" << endl <<  state_transition_model_ << endl;
            cout << "control_input_model" << endl << control_input_model_  << endl;
            cout << "observation_model" << endl << observation_model_  << endl;

            cout << "estimate_covariance" << endl << estimate_covariance_  << endl;
            cout << "inovation_covariance" << endl << inovation_covariance  << endl;

          }

      }
      return true;
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
      input_noise_covariance_ = input_sigma_m * input_sigma_m;
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

    virtual void updatePredictModel(const vector<double>& params = vector<double>(0)) = 0;
    virtual void updateCorrectModel(const vector<double>& params = vector<double>(0)) = 0;

    inline void setTimeSync(bool flag){time_sync_ = flag;}
    inline double getTimestamp()
    {
      if(timestamp_buf_.size() == 0) return 0;
      return timestamp_buf_.back();
    }

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    bool param_verbose_;
    bool debug_verbose_;
    int state_dim_, input_dim_,  measure_dim_;
    int id_;
    bool time_sync_;

    VectorXd input_sigma_,  measure_sigma_;
    VectorXd estimate_state_;

    MatrixXd estimate_covariance_;
    MatrixXd input_noise_covariance_, measurement_noise_covariance_;

    MatrixXd state_transition_model_;
    MatrixXd control_input_model_;
    MatrixXd observation_model_;

    /* buffer for time sync */
    deque<MatrixXd> est_state_buf_;
    deque<MatrixXd> est_cov_buf_;
    deque<VectorXd> input_buf_;
    deque< vector<double> > params_buf_;
    deque<double> timestamp_buf_;

    /* filtering start flag */
    bool input_start_flag_;
    bool measure_start_flag_;

    /* for mutex */
    boost::mutex kf_mutex_;

    void rosParamInit()
    {
      string ns = nhp_.getNamespace();

      nhp_.param("param_verbose", param_verbose_, true);
      nhp_.param("debug_verbose", debug_verbose_, false);

      nhp_.param("time_sync", time_sync_, false);

      for(int i = 0; i < input_dim_; i ++)
        {
          stringstream input_sigma_no;
          input_sigma_no << i + 1;
          nhp_.param(string("input_sigma") + input_sigma_no.str(), input_sigma_(i), 0.0);
        }

      for(int i = 0; i < measure_dim_; i ++)
        {
          stringstream measure_sigma_no;
          measure_sigma_no << i + 1;
          nhp_.param(string("measure_sigma") + measure_sigma_no.str(), measure_sigma_(i), 0.0);
        }

      if(param_verbose_)
        {
          cout << ns << ": measure_dim  is " << measure_dim_ << endl;
          cout << ns << ": input_dim  is " << input_dim_ << endl;
          cout << ns << ": state_dim  is " << state_dim_ << endl;
          cout << ns << ": input_sigma  is " << input_sigma_ << endl;
          cout << ns << ": measure_sigma  is " << measure_sigma_ << endl;

          cout << ns << ": time sync is " << time_sync_ << endl;
        }
    }

    /* default: linear propagation */
    virtual VectorXd statePropagation(VectorXd input, VectorXd estimate_state)
    {
      return state_transition_model_ * estimate_state + control_input_model_ * input;
    }

    /* default: linear subtraction */
    virtual VectorXd getResidual(VectorXd measurement, VectorXd estimate_state)
    {
      return (measurement - observation_model_ * estimate_state);
    }

    MatrixXd covariancePropagation(VectorXd input, MatrixXd estimate_covariance)
    {
      return  state_transition_model_ * estimate_covariance * state_transition_model_.transpose() + control_input_model_ * input_noise_covariance_ * control_input_model_.transpose();
    }

    void getTimeSyncState(VectorXd& estimate_state, MatrixXd& estimate_covariance, double timestamp)
    {
      /* no predict state stored */
      if(timestamp_buf_.size() == 0) return;

      size_t index = timestamp_buf_.size() - 1; // max

      if(debug_verbose_)
        cout << "start timestamp searching from " << timestamp_buf_.size() << endl;
      for(auto it = timestamp_buf_.begin(); it != timestamp_buf_.end(); ++it)
        {
          /* future timestamp, escape */
          if(*it > timestamp)
            {
              index = distance(timestamp_buf_.begin(), it);

              /* should assign the state before the measure timestamp */
              if(index > 0) index--;

              if(debug_verbose_)
                cout << "correction, time_sync: exit from timestamp searching, index: " << index << ", min_diff of timestamp: " << timestamp - *it << endl;
              break;
            }
        }

       estimate_state = est_state_buf_[index];
       estimate_covariance = est_cov_buf_[index];

      /* 1. time sync: remove unnecessary part */
      for(int i = 0; i <= index; i++)
        {
          est_state_buf_.pop_front();
          est_cov_buf_.pop_front();
          input_buf_.pop_front();
          params_buf_.pop_front();
          timestamp_buf_.pop_front();
        }
    }

    void rePropagation()
    {
      assert(est_state_buf_.size() == params_buf_.size());

      if(params_buf_.size() == 0) return;
      /* iteration re-propagation */
      if(debug_verbose_)
        cout << "start re-propagation for " << params_buf_.size() << endl;

      for(auto it = params_buf_.begin(); it != params_buf_.end(); ++it)
        {
          size_t index = distance(params_buf_.begin(), it);

          /* update the model */
          updatePredictModel(*it);

          /* propagation */
          estimate_state_ = statePropagation(input_buf_[index], estimate_state_);
          estimate_covariance_ = covariancePropagation(input_buf_[index], estimate_covariance_);
          /* update the buffer */
          est_state_buf_[index] = estimate_state_;
          est_cov_buf_[index] = estimate_covariance_;
        }
    }
  };
};
#endif
