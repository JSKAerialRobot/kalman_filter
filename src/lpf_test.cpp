#include <boost/shared_ptr.hpp>
#include <kalman_filter/lpf_filter.h>

int main(int argc, char** argv)
{
  ros::init (argc, argv, "lpf_test");

  ros::NodeHandle nh;

  IirFilter iir_filter_1d(100, 10, 1);
  iir_filter_1d.setInitValues(10.0);
  double temp_1d = iir_filter_1d.filterFunction(10.0);
  /* TODO: write more test code */

  IirFilter iir_filter_3d(100, 10, 3);
  iir_filter_3d.setInitValues(tf::Vector3(1, 1, 1));
  tf::Vector3 temp_3d = iir_filter_1d.filterFunction(tf::Vector3(1, 1, 1));
  /* TODO: write more test code */

  FirFilter fir_filter_1d(10, 1);
  fir_filter_1d.setInitValues(10.0);
  temp_1d = iir_filter_1d.filterFunction(10.0);
  /* TODO: write more test code */

  FirFilter fir_filter_3d(10, 3);
  fir_filter_3d.setInitValues(tf::Vector3(1, 1, 1));
  temp_3d = iir_filter_1d.filterFunction(tf::Vector3(1, 1, 1));
  /* TODO: write more test code */


  return 0;
}
