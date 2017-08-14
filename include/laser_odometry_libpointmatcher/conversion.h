#ifndef _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_CONVERSION_H_
#define _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_CONVERSION_H_

#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pointmatcher/PointMatcher.h>

namespace laser_odometry {
namespace conversion {

template<typename T>
typename PointMatcher<T>::TransformationParameters
eigenMatrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix, const int dim)
{
  typedef typename PointMatcher<T>::TransformationParameters M;
  assert(matrix.rows() == matrix.cols());
  assert((matrix.rows() == 3) || (matrix.rows() == 4));
  assert((dim == 3) || (dim == 4));

  if (matrix.rows() == dim) return matrix;

  M out(M::Identity(dim,dim));

  out.topLeftCorner(2,2)  = matrix.topLeftCorner(2,2);
  out.topRightCorner(2,1) = matrix.topRightCorner(2,1);

  return out;
}

template<typename T>
tf::Transform toRos(const typename PointMatcher<T>::TransformationParameters& t)
{
  tf::Transform tf;

  const Eigen::Isometry3d iso3d(
        Eigen::Matrix4d(eigenMatrixToDim<double>(t.template cast<double>(), 4)));

  tf::transformEigenToTF(iso3d, tf);
  return tf;
}

template<typename T>
typename PointMatcher<T>::DataPoints fromRos(const sensor_msgs::PointCloud2& rosMsg)
{
  typedef PointMatcher<T> PM;
  typedef typename PM::DataPoints DataPoints;
  typedef typename DataPoints::Label Label;
  typedef typename DataPoints::Labels Labels;
  typedef typename DataPoints::View View;

  if (rosMsg.fields.empty())
    return DataPoints();

  // fill labels
  // conversions of descriptor fields from pcl
  // see http://www.ros.org/wiki/pcl/Overview
  Labels featLabels;
  Labels descLabels;
  std::vector<bool> isFeature;

  for (auto it = rosMsg.fields.begin(); it != rosMsg.fields.end(); ++it)
  {
    const std::string name(it->name);
    const std::size_t count(std::max<std::size_t>(it->count, 1));
    if (name == "x" || name == "y" || name == "z")
    {
      featLabels.emplace_back(name, count);
      isFeature.push_back(true);
    }
    else if (name == "rgb" || name == "rgba")
    {
      descLabels.emplace_back("color", (name == "rgba") ? 4 : 3);
      isFeature.push_back(false);
    }
    else if ((it+1) != rosMsg.fields.end() && it->name == "normal_x" && (it+1)->name == "normal_y")
    {
      if ((it+2) != rosMsg.fields.end() && (it+2)->name == "normal_z")
      {
        descLabels.emplace_back("normals", 3);
        it += 2;
        isFeature.push_back(false);
        isFeature.push_back(false);
      }
      else
      {
        descLabels.emplace_back("normals", 2);
        it += 1;
        isFeature.push_back(false);
      }
      isFeature.push_back(false);
    }
    else
    {
      descLabels.emplace_back(name, count);
      isFeature.push_back(false);
    }
  }
  featLabels.emplace_back("pad", 1);
  assert(isFeature.size() == rosMsg.fields.size());

  // create cloud
  const unsigned pointCount(rosMsg.width * rosMsg.height);
  DataPoints cloud(featLabels, descLabels, pointCount);
  cloud.getFeatureViewByName("pad").setConstant(1);

  // fill cloud
  // TODO: support big endian, pass through endian-swapping
  // method just after the *reinterpret_cast
  typedef sensor_msgs::PointField PF;
  size_t fieldId = 0;
  for (auto it = rosMsg.fields.begin(); it != rosMsg.fields.end(); ++it, ++fieldId)
  {
    if (it->name == "rgb" || it->name == "rgba")
    {
      // special case for colors
      if (((it->datatype != PF::UINT32) && (it->datatype != PF::INT32) &&
           (it->datatype != PF::FLOAT32)) || (it->count != 1))
        throw std::runtime_error(
            (boost::format("Colors in a point cloud must be a single element of"
                           "size 32 bits, found %1% elements of type %2%") % it->count % unsigned(it->datatype)).str() );

      View view(cloud.getDescriptorViewByName("color"));
      int ptId(0);
      for (std::size_t y(0); y < rosMsg.height; ++y)
      {
        const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
        for (size_t x(0); x < rosMsg.width; ++x)
        {
          const uint32_t rgba(*reinterpret_cast<const uint32_t*>(dataPtr + it->offset));
          const T colorA(T((rgba >> 24) & 0xff) / 255.);
          const T colorR(T((rgba >> 16) & 0xff) / 255.);
          const T colorG(T((rgba >> 8)  & 0xff) / 255.);
          const T colorB(T((rgba >> 0)  & 0xff) / 255.);
          view(0, ptId) = colorR;
          view(1, ptId) = colorG;
          view(2, ptId) = colorB;

          if (view.rows() > 3) view(3, ptId) = colorA;

          dataPtr += rosMsg.point_step;
          ptId += 1;
        }
      }
    }
    else
    {
      // get view for editing data
      View view(
            (it->name == "normal_x") ? cloud.getDescriptorRowViewByName("normals", 0) :
                                       ((it->name == "normal_y") ? cloud.getDescriptorRowViewByName("normals", 1) :
                                                                   ((it->name == "normal_z") ? cloud.getDescriptorRowViewByName("normals", 2) :
                                                                                               ((isFeature[fieldId]) ? cloud.getFeatureViewByName(it->name) :
                                                                                                                       cloud.getDescriptorViewByName(it->name))))
                                       );
      // use view to read data
      int ptId(0);
      const std::size_t count(std::max<size_t>(it->count, 1));
      for (std::size_t y(0); y < rosMsg.height; ++y)
      {
        const uint8_t* dataPtr(&rosMsg.data[0] + rosMsg.row_step*y);
        for (std::size_t x(0); x < rosMsg.width; ++x)
        {
          const uint8_t* fPtr(dataPtr + it->offset);
          for (unsigned dim(0); dim < count; ++dim)
          {
            switch (it->datatype)
            {
              case PF::INT8:    view(dim, ptId) = T(*reinterpret_cast<const int8_t*>(fPtr));   fPtr += 1; break;
              case PF::UINT8:   view(dim, ptId) = T(*reinterpret_cast<const uint8_t*>(fPtr));  fPtr += 1; break;
              case PF::INT16:   view(dim, ptId) = T(*reinterpret_cast<const int16_t*>(fPtr));  fPtr += 2; break;
              case PF::UINT16:  view(dim, ptId) = T(*reinterpret_cast<const uint16_t*>(fPtr)); fPtr += 2; break;
              case PF::INT32:   view(dim, ptId) = T(*reinterpret_cast<const int32_t*>(fPtr));  fPtr += 4; break;
              case PF::UINT32:  view(dim, ptId) = T(*reinterpret_cast<const uint32_t*>(fPtr)); fPtr += 4; break;
              case PF::FLOAT32: view(dim, ptId) = T(*reinterpret_cast<const float*>(fPtr));    fPtr += 4; break;
              case PF::FLOAT64: view(dim, ptId) = T(*reinterpret_cast<const double*>(fPtr));   fPtr += 8; break;
              default: abort();
            }
          }
          dataPtr += rosMsg.point_step;
          ptId += 1;
        }
      }
    }
  }

  boost::shared_ptr<typename PM::DataPointsFilter> filter(PM::get().DataPointsFilterRegistrar.create("RemoveNaNDataPointsFilter"));
  return filter->filter(cloud);
}

template<typename T>
typename PointMatcher<T>::DataPoints fromRos(const sensor_msgs::LaserScan& rosMsg,
                                             const bool addTimestamps = false)
{
  typedef PointMatcher<T> PM;
  typedef typename PM::DataPoints DataPoints;
//  typedef typename DataPoints::Label Label;
  typedef typename DataPoints::Labels Labels;
//  typedef typename DataPoints::View View;

  Labels featLabels;
  featLabels.emplace_back("x", 1);
  featLabels.emplace_back("y", 1);
  featLabels.emplace_back("z", 1);

  featLabels.emplace_back("pad", 1);

  // Build descriptors
  Labels descLabels;
  if (!rosMsg.intensities.empty())
  {
    descLabels.emplace_back("intensity", 1);
    assert(rosMsg.intensities.size() == rosMsg.ranges.size());
  }
  if (addTimestamps)
  {
    descLabels.emplace_back("timestamp", 3);
  }

  // filter points based on range
  std::vector<std::size_t> ids(rosMsg.ranges.size());
  std::vector<double> ranges(rosMsg.ranges.size());
  std::vector<double> intensities(rosMsg.intensities.size());

  std::size_t goodCount(0);
  for (std::size_t i = 0; i < rosMsg.ranges.size(); ++i)
  {
    const float range(rosMsg.ranges[i]);
    if (range >= rosMsg.range_min && range <= rosMsg.range_max)
    {
      ranges[goodCount] = range;
      ids[goodCount] = i;

      if (!rosMsg.intensities.empty())
      {
        intensities[goodCount] = rosMsg.intensities[i];
      }

      ++goodCount;
    }
  }

  if (goodCount == 0) return DataPoints();

  ids.resize(goodCount);
  ranges.resize(goodCount);

  if (!rosMsg.intensities.empty()) intensities.resize(goodCount);

  DataPoints cloud(featLabels, descLabels, goodCount);
  cloud.getFeatureViewByName("pad").setConstant(1);

  // fill features
  for (std::size_t i = 0; i < ranges.size(); ++i)
  {
    const T angle = rosMsg.angle_min + ids[i] * rosMsg.angle_increment;
    const T range(ranges[i]);
    const T x = std::cos(angle) * range;
    const T y = std::sin(angle) * range;

    cloud.features(0,i) = x;
    cloud.features(1,i) = y;
    cloud.features(2,i) = 0;
  }

  // fill descriptors
  if (!rosMsg.intensities.empty())
  {
    auto is = cloud.getDescriptorViewByName("intensity");
    for (size_t i = 0; i < intensities.size(); ++i)
    {
      is(0,i) = intensities[i];
    }
  }

  if (addTimestamps)
  {
    auto is = cloud.getDescriptorViewByName("timestamp");

    for (size_t i = 0; i < ranges.size(); ++i)
    {
      const ros::Time curTime(rosMsg.header.stamp + ros::Duration(ids[i] * rosMsg.time_increment));

      const T Msec = round(curTime.sec/1e6);
      const T sec  = round(curTime.sec - Msec*1e6);
      const T nsec = round(curTime.nsec);

      is(0,i) = Msec;
      is(1,i) = sec;
      is(2,i) = nsec;
    }
  }

  return cloud;
}

} // namespace conversion
} // namespace laser_odometry

#endif // _LASER_ODOMETRY_LIBPOINTMATCHER_LASER_ODOMETRY_LIBPOINTMATCHER_CONVERSION_H_
