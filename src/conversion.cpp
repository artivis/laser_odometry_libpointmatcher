#include <laser_odometry_libpointmatcher/conversion.h>

namespace laser_odometry {
namespace conversion {

template<>
typename PointMatcher<float>::TransformationParameters
eigenMatrixToDim<float>(const PointMatcher<float>::TransformationParameters& matrix, const int dim);
template<>
typename PointMatcher<double>::TransformationParameters
eigenMatrixToDim<double>(const PointMatcher<double>::TransformationParameters& matrix, const int dim);


template<>
tf::Transform toRos<float>(const PointMatcher<float>::TransformationParameters& t);
template<>
tf::Transform toRos<double>(const PointMatcher<double>::TransformationParameters& t);


template
PointMatcher<float>::DataPoints fromRos<float>(const sensor_msgs::PointCloud2& rosMsg);
template
PointMatcher<double>::DataPoints fromRos<double>(const sensor_msgs::PointCloud2& rosMsg);


template
PointMatcher<float>::DataPoints fromRos<float>(const sensor_msgs::LaserScan& rosMsg, const bool addTimestamps);
template
PointMatcher<double>::DataPoints fromRos<double>(const sensor_msgs::LaserScan& rosMsg, const bool addTimestamps);


} //namespace conversion
} // namespace laser_odometry


