#ifndef _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_
#define _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_

#include <laser_odometry_core/laser_odometry_core.h>
#include <laser_odometry_libpointmatcher/conversion.h>

namespace laser_odometry
{

  class LaserOdometryLibPointMatcher : public LaserOdometryBase
  {
    using Base = LaserOdometryBase;

    using Scalar = double;
    using Matcher = PointMatcher<Scalar>;
    using DataPoints = Matcher::DataPoints;
    using DataPointsPtr = boost::shared_ptr<DataPoints>;

  public:

    LaserOdometryLibPointMatcher()  = default;
    ~LaserOdometryLibPointMatcher() = default;

  protected:

    bool processImpl(const sensor_msgs::LaserScanConstPtr& scan_msg,
                     const Transform& prediction) override;

    bool processImpl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                     const Transform& prediction) override;

    bool icp(const DataPointsPtr& src_cloud,
             const Transform& prediction);

    void isKeyFrame() override;

  public:

    OdomType odomType() const noexcept override;

  protected:

    Scalar kf_dist_angular_;
    Scalar kf_dist_linear_sq_;

    Scalar estimated_overlap_th_;
    Scalar match_ratio_th_;

    Matcher::ICP icp_;

    Matcher::TransformationParameters transform_ =
        Matcher::TransformationParameters::Identity(4,4);

    DataPointsPtr source_cloud_;
    DataPointsPtr ref_cloud_;

    template <class Msg>
    void convert(const typename Msg::ConstPtr& msg,
                 DataPointsPtr& lpm_scan)
    {
      lpm_scan = boost::make_shared<DataPoints>(conversion::fromRos<Scalar>(*msg));
    }

    bool configureImpl() override;

    bool initialize(const sensor_msgs::LaserScanConstPtr& scan_msg) override;

    bool initialize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) override;

    bool isKeyFrame(const Transform& tf) override;
  };

} /* namespace laser_odometry */

#endif /* _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_H_ */
