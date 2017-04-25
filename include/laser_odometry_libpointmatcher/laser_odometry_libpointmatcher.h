#ifndef _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_
#define _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_

#include <laser_odometry_core/laser_odometry_core.h>

//PointMatcher library headers
#include <pointmatcher_ros/point_cloud.h>

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

    ProcessReport process(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr,
                          geometry_msgs::Pose2DPtr pose_ptr,
                          geometry_msgs::Pose2DPtr relative_pose_ptr = nullptr) override;

    ProcessReport process(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr,
                          nav_msgs::OdometryPtr odom_ptr,
                          nav_msgs::OdometryPtr relative_odom_ptr = nullptr) override;

    OdomType odomType() const override;

  protected:

    bool initialized_ = false;

    double kf_dist_angular_;
    double kf_dist_linear_;
    double kf_dist_linear_sq_;

    double estimated_overlap_th_;
    double match_ratio_th_;

    // pose of the last keyframe scan in fixed frame
    tf::Transform world_to_base_kf_;

    Matcher::ICP icp_;

    Matcher::TransformationParameters transform_ =
        Matcher::TransformationParameters::Identity(4,4);

    DataPointsPtr ref_cloud_;

    void convert(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                 DataPointsPtr& lpm_scan);

    bool configureImpl() override;

    tf::Transform predict(const tf::Transform& tf) override;

    void initialize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    bool isKeyFrame(const tf::Transform& tf);
  };

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_ */
