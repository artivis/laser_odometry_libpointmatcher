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

  kf_dist_linear_       = private_nh_.param("dist_threshold",    0.5);
  kf_dist_angular_      = private_nh_.param("rot_threshold",     0.17);
  estimated_overlap_th_ = private_nh_.param("overlap_threshold", 0.6);
  match_ratio_th_       = private_nh_.param("ratio_threshold",   0.65);

  world_to_base_kf_ = tf::Transform::getIdentity();

  return true;
}

bool LaserOdometryLibPointMatcher::process(const sensor_msgs::PointCloud2ConstPtr cloud_ptr,
                                        geometry_msgs::Pose2DPtr pose_ptr,
                                        geometry_msgs::Pose2DPtr /*relative_pose_ptr*/)
{
  nav_msgs::OdometryPtr odom_ptr = boost::make_shared<nav_msgs::Odometry>();

  bool processed = process(cloud_ptr, odom_ptr);

  // Retrieve Odometry
  fillPose2DMsg(pose_ptr);

  return processed;
}

bool LaserOdometryLibPointMatcher::process(const sensor_msgs::PointCloud2ConstPtr cloud_ptr,
                                        nav_msgs::OdometryPtr odom_ptr,
                                        nav_msgs::OdometryPtr /*relative_odom_ptr*/)
{
  assert(odom_ptr != nullptr &&
   "LaserOdometryLibPointMatcher::process OdometryPtr odom is nullptr!");

  if (!initialized_)
  {
    initialize(cloud_ptr);
    world_origin_to_base_ = world_origin_ * world_to_base_;
    fillOdomMsg(odom_ptr);

    ROS_INFO_STREAM("LaserOdometryLibPointMatcher Initialized!");

    return false;
  }

  DataPointsPtr sourceDP;
  convert(cloud_ptr, sourceDP);
  current_time_ = cloud_ptr->header.stamp;

  // the predicted change of the laser's position, in the fixed frame
  tf::Transform pr_ch;
  pr_ch = predict(tf::Transform::getIdentity());

  // account for the change since the last kf, in the fixed frame
  pr_ch = pr_ch * (world_to_base_ * world_to_base_kf_.inverse());

  // the predicted change of the laser's position, in the laser frame
  tf::Transform pr_ch_l;
  pr_ch_l = laser_to_base_ * world_to_base_.inverse() *
              pr_ch * world_to_base_ * base_to_laser_ ;

  Matcher::TransformationParameters initial_guess;

  Eigen::Affine3d tmp;
  tf::transformTFToEigen(pr_ch_l, tmp);

  initial_guess = tmp.matrix();

  bool icp_valid = false;

  //Call ICP
  Matcher::TransformationParameters transform;
  try
  {
    transform = icp_(*sourceDP, *ref_cloud_, initial_guess);

    icp_valid = true;
  }
  catch (const Matcher::ConvergenceError& error)
  {
    ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
    return false;
  }

  tf::Transform corr_ch;
  if (icp_valid)
  {
    // the correction of the laser's position, in the laser frame
    tf::Transform corr_ch_l = PointMatcher_ros::eigenMatrixToTransform<double>(transform);

    // the correction of the base's position, in the base frame
    corr_ch = expressFromLaserToBase(corr_ch_l);

    // update the pose in the world frame
    world_to_base_ = world_to_base_kf_ * corr_ch;

    // update the pose in the world 'origin' frame
    world_origin_to_base_ = world_origin_ * world_to_base_;

    // Retrieve odom
    fillOdomMsg(odom_ptr);

    const Matcher::Matrix cov = icp_.errorMinimizer->getCovariance();

    assert(cov.rows()==6 && cov.cols()==6);

    odom_ptr->pose.covariance =
        boost::assign::list_of
          (cov(0, 0)) (cov(0, 1)) (cov(0, 2)) (cov(0, 3)) (cov(0, 4)) (cov(0, 5))
          (cov(1, 0)) (cov(1, 1)) (cov(1, 2)) (cov(1, 3)) (cov(1, 4)) (cov(1, 5))
          (cov(2, 0)) (cov(2, 1)) (cov(2, 2)) (cov(2, 3)) (cov(2, 4)) (cov(2, 5))
          (cov(3, 0)) (cov(3, 1)) (cov(3, 2)) (cov(3, 3)) (cov(3, 4)) (cov(3, 5))
          (cov(4, 0)) (cov(4, 1)) (cov(4, 2)) (cov(4, 3)) (cov(4, 4)) (cov(4, 5))
          (cov(5, 0)) (cov(5, 1)) (cov(5, 2)) (cov(5, 3)) (cov(5, 4)) (cov(5, 5));
  }
  else
  {
    corr_ch.setIdentity();
    fillCovariance(odom_ptr->pose.covariance);
    ROS_WARN("Error in scan matching");
  }

  if (isKeyFrame(corr_ch))
  {
    // generate a keyframe
    ref_cloud_        = sourceDP;
    world_to_base_kf_ = world_to_base_;
  }

  return true;
}

void LaserOdometryLibPointMatcher::convert(const sensor_msgs::PointCloud2ConstPtr cloud_msg,
                                        DataPointsPtr& lpm_scan)
{
  lpm_scan = boost::make_shared<DataPoints>(
        PointMatcher_ros::rosMsgToPointMatcherCloud<double>(*cloud_msg) );
}

void LaserOdometryLibPointMatcher::initialize(const sensor_msgs::PointCloud2ConstPtr cloud_msg)
{
  convert(cloud_msg, ref_cloud_);

  current_time_ = cloud_msg->header.stamp;

  initialized_ = true;
}

bool LaserOdometryLibPointMatcher::isKeyFrame(const tf::Transform& tf)
{
  if (fabs(tf::getYaw(tf.getRotation())) > kf_dist_angular_) return true;

  double x = tf.getOrigin().getX();
  double y = tf.getOrigin().getY();

  if (x*x + y*y > kf_dist_linear_sq_) return true;

  return false;
}

tf::Transform LaserOdometryLibPointMatcher::predict(const tf::Transform& /*tf*/)
{
  return guess_relative_tf_;
}

} /* namespace laser_odometry */

PLUGINLIB_EXPORT_CLASS(laser_odometry::LaserOdometryLibPointMatcher, laser_odometry::LaserOdometryBase);
