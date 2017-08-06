#include <pluginlib/class_list_macros.h>

#include <laser_odometry_core/laser_odometry_utils.h>
#include <laser_odometry_libpointmatcher/laser_odometry_libpointmatcher.h>

#include <pointmatcher_ros/transform.h>
#include <tf_conversions/tf_eigen.h>

#include <fstream>

namespace laser_odometry {

bool LaserOdometryLibPointMatcher::configureImpl()
{
  if (private_nh_.hasParam("icp_config"))
  {
    std::string icp_config;
    private_nh_.getParam("icp_config", icp_config);

    std::ifstream ifs(icp_config.c_str());
    if (ifs.good())
    {
      icp_.loadFromYaml(ifs);
      ROS_INFO_STREAM("Load config from YAML file " << icp_config);
    }
    else
    {
      ROS_ERROR_STREAM("Cannot load config from YAML file " << icp_config);
      icp_.setDefault();
    }

    ifs.close();
  }
  else
  {
    ROS_WARN_STREAM("No config file specified, using default ICP chain.");
    icp_.setDefault();
  }

  Scalar kf_dist_linear;
  kf_dist_linear        = private_nh_.param("dist_threshold",    0.25);
  kf_dist_angular_      = private_nh_.param("rot_threshold",     0.17);
  estimated_overlap_th_ = private_nh_.param("overlap_threshold", 0.85);
  match_ratio_th_       = private_nh_.param("ratio_threshold",   0.65);

  kf_dist_linear_sq_ = kf_dist_linear * kf_dist_linear;

  return true;
}

bool LaserOdometryLibPointMatcher::process_impl(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                                const tf::Transform& prediction)
{
  convert<sensor_msgs::LaserScan>(scan_msg, source_cloud_);

  return icp(source_cloud_, prediction);
}

bool LaserOdometryLibPointMatcher::process_impl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                                const tf::Transform& prediction)
{
  convert<sensor_msgs::PointCloud2>(cloud_msg, source_cloud_);

  return icp(source_cloud_, prediction);
}

bool LaserOdometryLibPointMatcher::icp(const DataPointsPtr& src_cloud,
                                       const tf::Transform& prediction)
{
  if (src_cloud->features.cols() == 0)
  {
    ROS_ERROR("No good points in the cloud");
    return false;
  }

  Matcher::TransformationParameters initial_guess;

  tf::Transform pred = prediction;
  pred.setRotation( pred.getRotation().normalize() );

  Eigen::Affine3d tmp;
  tf::transformTFToEigen(prediction, tmp);

  initial_guess = tmp.matrix();

  bool icp_valid = false;

  // Call ICP
  Matcher::TransformationParameters transform;
  try
  {
    transform = icp_(*src_cloud, *ref_cloud_, initial_guess);

    icp_valid = true;
  }
  catch (const Matcher::ConvergenceError& error)
  {
    ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
    return false;
  }

  if (icp_valid)
  {
    increment_ = toTf(transform);

//    ROS_WARN_STREAM("transform :\n" << transform);
//    utils::print(increment_, "increment\n", 25);

    const Matcher::Matrix cov = icp_.errorMinimizer->getCovariance();

    assert(cov.rows()==6 && cov.cols()==6);

//    increment_covariance_ =
//        boost::assign::list_of
//          (cov(0, 0)) (cov(0, 1)) (cov(0, 2)) (cov(0, 3)) (cov(0, 4)) (cov(0, 5))
//          (cov(1, 0)) (cov(1, 1)) (cov(1, 2)) (cov(1, 3)) (cov(1, 4)) (cov(1, 5))
//          (cov(2, 0)) (cov(2, 1)) (cov(2, 2)) (cov(2, 3)) (cov(2, 4)) (cov(2, 5))
//          (cov(3, 0)) (cov(3, 1)) (cov(3, 2)) (cov(3, 3)) (cov(3, 4)) (cov(3, 5))
//          (cov(4, 0)) (cov(4, 1)) (cov(4, 2)) (cov(4, 3)) (cov(4, 4)) (cov(4, 5))
//          (cov(5, 0)) (cov(5, 1)) (cov(5, 2)) (cov(5, 3)) (cov(5, 4)) (cov(5, 5));
  }
  else
  {
    increment_.setIdentity();
    ROS_WARN("libpointmatcher could not align scans.");
  }

  return true;
}

void LaserOdometryLibPointMatcher::isKeyFrame()
{
  /// @todo what's best?
//  std::swap(ref_cloud_, source_cloud_);
  Matcher::swapDataPoints(*ref_cloud_, *source_cloud_);
}

bool LaserOdometryLibPointMatcher::initialize(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  convert<sensor_msgs::LaserScan>(scan_msg, ref_cloud_);

  if (ref_cloud_->features.cols() == 0)
  {
    ROS_ERROR("initialize: No good points in the cloud");
    return false;
  }

  current_time_ = scan_msg->header.stamp;

  return true;
}

bool LaserOdometryLibPointMatcher::initialize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  convert<sensor_msgs::PointCloud2>(cloud_msg, ref_cloud_);

  if (ref_cloud_->features.cols() == 0)
  {
    ROS_ERROR("initialize: No good points in the cloud");
    return false;
  }

  current_time_ = cloud_msg->header.stamp;

  return true;
}

bool LaserOdometryLibPointMatcher::isKeyFrame(const tf::Transform& tf)
{
  if (std::abs(tf::getYaw(tf.getRotation())) > kf_dist_angular_) return true;

  const Scalar x = tf.getOrigin().getX();
  const Scalar y = tf.getOrigin().getY();
  const Scalar z = tf.getOrigin().getZ();

  if ( (x*x + y*y + z*z) > kf_dist_linear_sq_ ) return true;

  if (icp_.errorMinimizer->getOverlap() < estimated_overlap_th_) return true;

  return false;
}

OdomType LaserOdometryLibPointMatcher::odomType() const noexcept
{
  /// @todo @note depending on the icp_->errorMinimizer
  /// the returned covariance is either an actual covariance
  /// or identity.
  return OdomType::Odom3DCov;
}

} /* namespace laser_odometry */

PLUGINLIB_EXPORT_CLASS(laser_odometry::LaserOdometryLibPointMatcher, laser_odometry::LaserOdometryBase);
