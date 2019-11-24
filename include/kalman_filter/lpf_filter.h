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


#ifndef DIGITAL_FILTER_H
#define DIGITAL_FILTER_H

#include <iostream>

/* math */
#include <Eigen/Core>
#include <Eigen/Dense>

//* ros
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

class LowPassFilter
{
public:
  LowPassFilter() {}
  ~LowPassFilter() {}

  virtual void setInitValues(const Eigen::VectorXd& init_value) = 0;

  /* overwrite function for 1 dimension */
  virtual void setInitValues(const double& init_value)
  {
    Eigen::VectorXd init_vec(1);
    init_vec << init_value;
    setInitValues(init_vec);
  }

  /* overwrite function for 3 dimension: tf::Vector3 */
  virtual void setInitValues(const tf::Vector3& init_value)
  {
    Eigen::Vector3d init_vec;
    tf::vectorTFToEigen(init_value, init_vec);
    setInitValues(init_vec);
  }

  /* overwrite function for 3 dimension: geometry_msgs::Point */
  virtual void setInitValues(const geometry_msgs::Point& init_value)
  {
    Eigen::Vector3d init_vec;
    tf::pointMsgToEigen(init_value, init_vec);
    setInitValues(init_vec);
  }

  virtual const Eigen::VectorXd filterFunction(const Eigen::VectorXd& input) = 0;

  /* overwrite function for 1 dimension */
  virtual const double filterFunction(const double& input)
  {
    Eigen::VectorXd input_vec(1);
    input_vec << input;
    return filterFunction(input_vec)(0);
  }

  /* overwrite function for 3 dimension: tf::Vector3 */
  virtual const tf::Vector3 filterFunction(const tf::Vector3& input)
  {
    Eigen::Vector3d input_vec;
    tf::vectorTFToEigen(input, input_vec);
    tf::Vector3 output;
    tf::vectorEigenToTF(filterFunction(input_vec), output);
    return output;
  }

  /* overwrite function for 3 dimension: geometry_msgs::Point */
  virtual const geometry_msgs::Point filterFunction(const geometry_msgs::Point& input)
  {
    Eigen::Vector3d input_vec;
    tf::pointMsgToEigen(input, input_vec);
    geometry_msgs::Point output;
    tf::pointEigenToMsg(filterFunction(input_vec), output);
    return output;
  }

};

class IirFilter : public LowPassFilter
{
 private:
  double sample_freq_;
  double cutoff_freq_;

  double a_;
  double w0_;
  double a1_;
  double a2_;
  double b0_;
  double b1_;
  double b2_;

  Eigen::VectorXd pre1_;
  Eigen::VectorXd pre2_;

 public:
  IirFilter(): LowPassFilter() {}

  IirFilter(double sample_freq, double cutoff_freq, int dimension = 1, bool verbose = false)
    {
      setSampleFreq(sample_freq);
      setCutoffFreq(cutoff_freq);
      setInitParam(verbose);

      pre1_ = Eigen::VectorXd::Zero(dimension);
      pre2_ = Eigen::VectorXd::Zero(dimension);
    }

  ~IirFilter(){}

  inline void setSampleFreq(double sample_freq){ sample_freq_ = sample_freq; }
  inline void setCutoffFreq(double cutoff_freq){ cutoff_freq_ = cutoff_freq; }

  void setInitParam(bool verbose = false)
  {
    assert(sample_freq_ > cutoff_freq_);

    w0_ = tan(M_PI * cutoff_freq_ / sample_freq_);
    a_  = sin(w0_) / 0.707;
    a1_ = 2 * cos(w0_) / (1 + a_ );
    a2_ = (a_ - 1) / (a_ + 1);
    b0_ = (1 - cos(w0_)) / 2 / (1 + a_);
    b1_ =  (1 - cos(w0_)) / (1 + a_);
    b2_ = (1 - cos(w0_)) / 2 / (1 + a_);

    if(verbose)
      std::cout << "IIR Filter: w0: " <<  w0_ << ", a: " << a_ << ", a1: " << a1_ << ", a2 " <<  a2_ << ", b0 " << b0_ << ", b1 " <<  b1_ << ", b2 " << b2_ << std::endl;
  }

  void setInitValues(const Eigen::VectorXd& init_value)
  {
    assert(init_value.size() == pre1_.size());

    pre2_ = init_value / (2 - 2 * cos(w0_)) * (a_ + 1);
    pre1_ = init_value / (2 - 2 * cos(w0_)) * (a_ + 1);
  }

  void setInitValues(const double& init_value) { LowPassFilter::setInitValues(init_value); }
  void setInitValues(const tf::Vector3& init_value) { LowPassFilter::setInitValues(init_value); }

  const Eigen::VectorXd filterFunction(const Eigen::VectorXd& input)
  {
    assert(input.size() == pre1_.size());

    Eigen::VectorXd reg_pos = input + a1_ * pre1_ + a2_ * pre2_;
    Eigen::VectorXd output = b0_ * reg_pos + b1_ * pre1_ + b2_ * pre2_;
    pre2_ = pre1_;
    pre1_ = reg_pos;
    return output;
  }

  const tf::Vector3 filterFunction(const tf::Vector3& input) {return LowPassFilter::filterFunction(input); }
  const double filterFunction(const double& input) {return  LowPassFilter::filterFunction(input); }

};

class FirFilter : public LowPassFilter
{
 private:
  float filter_factor_;
  Eigen::VectorXd output_val_;

 public:
  FirFilter(): LowPassFilter()
  {
    filter_factor_ = 1;
  }

  FirFilter(const double& filter_factor, int dimension = 1)
    {
      setFilterFactor(filter_factor);
      output_val_ = Eigen::VectorXd::Zero(dimension);
    }

  ~FirFilter(){}

  inline void setFilterFactor(const double& filter_factor)
  {
    assert(filter_factor <= 1);
    filter_factor_ = filter_factor;
  }

  inline const double getFilterFactor() const
  {
    return filter_factor_;
  }

  void setInitValues(const Eigen::VectorXd& init_value)
  {
    assert(init_value.size() == output_val_.size());

    output_val_ = init_value;
  }

  void setInitValues(const double& init_value) { LowPassFilter::setInitValues(init_value); }
  void setInitValues(const tf::Vector3& init_value) { LowPassFilter::setInitValues(init_value); }

  const Eigen::VectorXd filterFunction(const Eigen::VectorXd& input)
  {
    assert(input.size() == output_val_.size());
    output_val_ += filter_factor_ * (input - output_val_);

    return output_val_;
  }

  const tf::Vector3 filterFunction(const tf::Vector3& input) {return  LowPassFilter::filterFunction(input); }
  const double filterFunction(const double& input) {return  LowPassFilter::filterFunction(input); }

};

class FirFilterQuaternion
{
 private:
  float filter_factor_;
  Eigen::Quaterniond output_val_;

 public:
  FirFilterQuaternion()
  {
    filter_factor_ = 1;
  }

  FirFilterQuaternion(const double& filter_factor)
    {
      setFilterFactor(filter_factor);
    }

  ~FirFilterQuaternion(){}

  inline const double getFilterFactor() const
  {
    return filter_factor_;
  }

  inline void setFilterFactor(const double& filter_factor)
  {
    assert(filter_factor <= 1);
    filter_factor_ = filter_factor;
  }

  void setInitValues(const Eigen::Quaterniond& init_value)
  {
    output_val_ = init_value;
  }

  void setInitValues(const tf::Quaternion& init_value)
  {
    Eigen::Quaterniond init_value_q;
    tf::quaternionTFToEigen(init_value, init_value_q);
    setInitValues(init_value_q);
  }

  void setInitValues(const geometry_msgs::Quaternion& init_value)
  {
    Eigen::Quaterniond init_value_q;
    tf::quaternionMsgToEigen(init_value, init_value_q);
    setInitValues(init_value_q);
  }

  const Eigen::Quaterniond filterFunction(const Eigen::Quaterniond& input)
  {
    //linear interporation: output_val_ += filter_factor_ * (input - output_val_);

    output_val_ = output_val_.slerp(filter_factor_, input);

    return output_val_;
  }

  const tf::Quaternion filterFunction(const tf::Quaternion& input)
  {
    Eigen::Quaterniond input_q;
    tf::quaternionTFToEigen(input, input_q);
    tf::Quaternion output;
    tf::quaternionEigenToTF(filterFunction(input_q), output);
    return output;
  }

  const geometry_msgs::Quaternion filterFunction(const geometry_msgs::Quaternion& input)
  {
    Eigen::Quaterniond input_q;
    tf::quaternionMsgToEigen(input, input_q);
    geometry_msgs::Quaternion output;
    tf::quaternionEigenToMsg(filterFunction(input_q), output);
    return output;
  }

};


#endif
