#ifndef _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_
#define _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_

#include <laser_odometry_core/laser_odometry_core.h>

//PointMatcher library headers
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>

namespace laser_odometry
{

  class LaserOdometryLibPointMatcher : public LaserOdometryBase
  {
    using Base = LaserOdometryBase;

    using Matcher = PointMatcher<double>;
    using DataPoints = Matcher::DataPoints;
    using DataPointsPtr = boost::shared_ptr<DataPoints>;

  public:

    LaserOdometryLibPointMatcher()  = default;
    ~LaserOdometryLibPointMatcher() = default;

  protected:

    bool process_impl(const sensor_msgs::LaserScanConstPtr& scan_msg,
                      const tf::Transform& prediction) override;

    bool process_impl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                      const tf::Transform& prediction) override;

    bool icp(const DataPointsPtr& src_cloud,
             const tf::Transform& prediction);

    void isKeyFrame() override;

  public:

    OdomType odomType() const noexcept override;

  protected:

    double kf_dist_angular_;
    double kf_dist_linear_;
    double kf_dist_linear_sq_;

    double estimated_overlap_th_;
    double match_ratio_th_;

    Matcher::ICP icp_;

    Matcher::TransformationParameters transform_ =
        Matcher::TransformationParameters::Identity(4,4);

    DataPointsPtr source_cloud_;
    DataPointsPtr ref_cloud_;

    template <class Msg>
    void convert(const typename Msg::ConstPtr& msg,
                 DataPointsPtr& lpm_scan);

    bool configureImpl() override;

    bool initialize(const sensor_msgs::LaserScanConstPtr& scan_msg) override;

    bool initialize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) override;

    bool isKeyFrame(const tf::Transform& tf) override;

    tf::Transform toTf(const Matcher::TransformationParameters& transform)
    {
      return PointMatcher_ros::eigenMatrixToTransform<double>(transform);
    }
  };

  template <>
  void LaserOdometryLibPointMatcher::
  convert<sensor_msgs::LaserScan>(const sensor_msgs::LaserScan::ConstPtr& msg,
                                  DataPointsPtr& lpm_scan)
  {
    constexpr bool force_3d = true;

    lpm_scan = boost::make_shared<DataPoints>(
          PointMatcher_ros::rosMsgToPointMatcherCloud<double>(*msg, nullptr, "", force_3d) );
  }

  template <>
  void LaserOdometryLibPointMatcher::
  convert<sensor_msgs::PointCloud2>(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                    DataPointsPtr& lpm_scan)
  {
    lpm_scan = boost::make_shared<DataPoints>(
          PointMatcher_ros::rosMsgToPointMatcherCloud<double>(*msg) );
  }

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_ */
