#ifndef _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_CONVERSION_H_
#define _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_CONVERSION_H_

#include <pointmatcher/PointMatcher.h>

#include <Eigen/Eigen>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

namespace tf
{
  struct Transform;
} // namespace tf

namespace laser_odometry {
namespace conversion {

  template<typename T>
  typename PointMatcher<T>::TransformationParameters
  eigenMatrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix, const int dim);

  template<typename T>
  tf::Transform toRos(const typename PointMatcher<T>::TransformationParameters& t);

  template<typename T>
  typename PointMatcher<T>::DataPoints fromRos(const sensor_msgs::PointCloud2& rosMsg);

  template<typename T>
  typename PointMatcher<T>::DataPoints fromRos(const sensor_msgs::LaserScan& rosMsg,
                                               const bool addTimestamps = false);
} // namespace conversion
} // namespace laser_odometry

#endif // _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_CONVERSION_H_
