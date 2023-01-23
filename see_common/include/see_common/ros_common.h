// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef ROS_COMMON_H_
#define ROS_COMMON_H_

#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <see_common/common_structs.h>
#include <see_common/pcl_common.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

using namespace pcl_ros;

void connected(const ros::SingleSubscriberPublisher &) {}
void disconnected(const ros::SingleSubscriberPublisher &) {}

//! Convert Affine Transformation Matrix to SeeView Pose
/*!
  \param transform Affine transformation matrix
  \param view SeeView pose
  \param f_axis Forward axis of sensor
*/
void AffineTransformToSeeView(Eigen::Affine3f transform, SeeView &view,
                              int f_axis) {
  Eigen::Vector3f axis(0, 0, 0);
  axis(f_axis) = 1;

  view.getVector3fMap() = transform.translation();
  view.getViewVector3fMap() = (transform.rotation() * axis).normalized();
}

//! Convert SeeView Pose to Affine Transformation Matrix
/*!
  \param view SeeView pose
  \param transform Affine transformation matrix
  \param f_axis Forward axis of sensor
*/
void SeeViewToAffineTransform(SeeView view, Eigen::Affine3f &transform,
                              int f_axis) {
  Eigen::Vector3f axis(0, 0, 0);
  axis(f_axis) = 1;

  transform =
      Eigen::Translation3f(view.getVector3fMap()) *
      Eigen::Quaternionf::FromTwoVectors(axis, view.getViewVector3fMap());
}

//! Get ROS boolean parameter
/*!
  \param param Name of parameter to get
  \param default_value value to return if parameter is not set
  \return The value of the parameter if set or the default value
*/
bool CheckParam(std::string param, bool default_value) {
  bool param_value = default_value;
  if (ros::param::has(param)) {
    ros::param::get(param, param_value);
  }
  return param_value;
}

//! Set ROS boolean parameter to specified value
/*!
  \param param Name of parameter to set
  \param value boolean value to set
*/
void SetParam(std::string param, bool value) { ros::param::set(param, value); }

//! Sensor Pointcloud Publisher
/*! ROS publisher for sending sensor pointcloud messages */
struct SenCloudPublisher {
  SensorParams s_p;
  ros::Publisher pub;
  ros::AdvertiseOptions a_o;
  tf::TransformListener tf_tl;
  sensor_msgs::PointCloud2 cloud_out;

  //! Initialise ROS publisher for sensor pointclouds
  /*!
    \param n ROS node handle
    \param topic Name of sensor pointcloud topic
    \param sensor_params Sensor parameters
  */
  SenCloudPublisher(ros::NodeHandle &n, std::string topic,
                    SensorParams sensor_params) {
    s_p = sensor_params;
    a_o = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
        topic, 1, &connected, &disconnected, ros::VoidPtr(), NULL);
    a_o.latch = true;
    a_o.has_header = false;
    pub = n.advertise(a_o);
  }

  //! Publishing function
  /*!
    \param cloud_in Pointer to SEE pointcloud being published
  */
  void SendMessage(SeePointCloudConstPtr cloud_in, int cloud_num) {
    sensor_msgs::PointCloud2 cld_tmp;
    pcl::toROSMsg(*cloud_in, cld_tmp);
    while (!transformPointCloud(s_p.sensor_frame, cld_tmp, cloud_out, tf_tl)) {
    }
    cloud_out.header.seq = cloud_num;
    pub.publish(cloud_out);
  }
};

//! Sensor Pointcloud Subscriber
/*! Subscribe to ROS pointcloud messages from external sensor */
struct SenCloudSubscriber {
  SeeView view;
  SensorParams s_p;
  int cloud_num = -1;
  SeePointCloudPtr cloud;
  bool has_cloud = false;
  ros::Subscriber subscriber;
  tf::TransformListener tf_tl;

  //! Initialise ROS subscriber to sensor pointclouds
  /*!
    \param n ROS node handle
    \param topic Name of sensor pointcloud topic
    \param sensor_params Sensor parameters
  */
  SenCloudSubscriber(ros::NodeHandle &n, std::string topic,
                     SensorParams sensor_params) {
    s_p = sensor_params;
    cloud = SeePointCloudPtr(new pcl::PointCloud<SeePoint>);

    subscriber = n.subscribe<sensor_msgs::PointCloud2>(
        topic, 1, &SenCloudSubscriber::OnSensorMessage, this);
  }

  //! Get the latest pointcloud and corresponding sensor pose
  /*!
    \param cloud_out Pointer to pointcloud
    \param view_out Sensor pose associated with pointcloud
    \return Does the pointcloud and corresponding view exist
  */
  bool GetViewandCloud(SeePointCloudPtr cloud_out, SeeView &view_out) {
    if (has_cloud) {
      view_out = view;
      pcl::copyPointCloud(*cloud, *cloud_out);
      has_cloud = false;
      return true;
    }
    return false;
  }

  //! Callback function for processing new sensor pointcloud messages
  /*!
    \param cloud_in Pointer to new pointcloud message
  */
  void OnSensorMessage(const sensor_msgs::PointCloud2ConstPtr &cloud_in) {
    Eigen::Affine3d tf_affine;
    Eigen::Matrix4f tf_matrix;
    tf::StampedTransform transform;
    sensor_msgs::PointCloud2 cloud_tmp;

    if (!(tf_tl.frameExists(s_p.world_frame) &&
          tf_tl.frameExists(cloud_in->header.frame_id))) {
      ROS_ERROR("Coordinate frame initialisation failed.");
      ros::shutdown();
    }

    if (static_cast<int>(cloud_in->header.seq) > cloud_num) {
      if (transformPointCloud(s_p.world_frame, *cloud_in, cloud_tmp, tf_tl)) {
        tf_tl.lookupTransform(s_p.world_frame, cloud_in->header.frame_id,
                              cloud_in->header.stamp, transform);
        tf::transformTFToEigen(transform, tf_affine);
        AffineTransformToSeeView(tf_affine.cast<float>(), view, s_p.f_axis);
        pcl::fromROSMsg(cloud_tmp, *cloud);

        has_cloud = true;
        cloud_num = static_cast<int>(cloud_in->header.seq);
        ros::param::set("/see/sen_cloud_num", cloud_num);
      }
    }
  }
};

//! SEE Pointcloud Subscriber
/*! ROS subscriber for SEE pointcloud messages */
struct SeeCloudSubscriber {
  ros::Subscriber s;
  int cloud_num = -1;
  bool has_cloud = false;
  SeePointCloudPtr cloud;

  //! Initialise ROS subscriber to SEE pointclouds
  /*!
    \param n ROS node handle
    \param topic Name of SEE pointcloud topic
  */
  SeeCloudSubscriber(ros::NodeHandle &n, std::string topic) {
    cloud = SeePointCloudPtr(new pcl::PointCloud<SeePoint>);
    s = n.subscribe<SeePointCloud>(topic, 1, &SeeCloudSubscriber::OnMessage,
                                   this);
  }

  //! Get the latest pointcloud
  /*!
    \param cloud_out Pointer to pointcloud
    \return Does the pointcloud exist
  */
  bool GetCloud(SeePointCloudPtr cloud_out, int &cloud_num_out) {
    if (has_cloud) {
      pcl::copyPointCloud(*cloud, *cloud_out);
      cloud_num_out = cloud_num;
      has_cloud = false;
      return true;
    }
    return false;
  }

  //! Callback function for processing SEE pointcloud messages
  /*!
    \param cloud_in Pointer to SEE pointcloud message
  */
  void OnMessage(const SeePointCloudConstPtr &cloud_in) {
    if (static_cast<int>(cloud_in->header.seq) > cloud_num) {
      pcl::copyPointCloud(*cloud_in, *cloud);
      cloud_num = static_cast<int>(cloud_in->header.seq);
      has_cloud = true;
      ros::param::set("/see/see_cloud_num", cloud_num);
    }
  }
};

//! SEE View Subscriber
/*! ROS subscriber for SEE view messages */
struct SeeViewSubscriber {
  int view_num;
  SeeView view;
  ros::Subscriber s;
  bool has_view = false;

  //! Initialise ROS subscriber to SEE views
  /*!
    \param n ROS node handle
    \param topic Name of SEE view cloud topic
  */
  SeeViewSubscriber(ros::NodeHandle &n, std::string topic) {
    s = n.subscribe<SeeViewCloud>(topic, 1, &SeeViewSubscriber::OnMessage,
                                  this);
  }

  //! Get the latest view
  /*!
    \param view_out View
    \return Does the view exist
  */
  bool GetView(SeeView &view_out) {
    if (has_view) {
      view_out = view;
      has_view = false;
      return true;
    }
    return false;
  }

  //! Get the latest view and its number
  /*!
    \param view_out View
    \param view_num_out View number
    \return Does the view exist
  */
  bool GetViewandNum(SeeView &view_out, int &view_num_out) {
    if (has_view) {
      view_out = view;
      view_num_out = view_num;
      has_view = false;
      return true;
    }
    return false;
  }

  //! Callback function for processing SEE view messages
  /*!
    \param cloud Pointer to SEE view cloud message
  */
  void OnMessage(const SeeViewCloudConstPtr &cloud_in) {
    view = cloud_in->points[0];
    view_num = static_cast<int>(cloud_in->header.seq);
    has_view = true;
  }
};

//! SEE Pointcloud Publisher
/*! ROS publisher for sending SEE pointcloud messages */
struct SeeCloudPublisher {
  ros::Publisher pub;
  ros::AdvertiseOptions a_o;

  //! Initialise ROS publisher for SEE pointclouds
  /*!
    \param n ROS node handle
    \param topic Name of SEE pointcloud topic
  */
  SeeCloudPublisher(ros::NodeHandle &n, std::string topic) {
    a_o = ros::AdvertiseOptions::create<SeePointCloud>(
        topic, 1, &connected, &disconnected, ros::VoidPtr(), NULL);
    a_o.latch = true;
    a_o.has_header = false;
    pub = n.advertise(a_o);
  }

  //! Publishing function
  /*!
    \param cloud Pointer to SEE pointcloud being published
    \param view_num The view number associated with the pointcloud
  */
  void SendMessage(SeePointCloudConstPtr cloud) { pub.publish(*cloud); }
};

//! SEE View Publisher
/*! ROS publisher for sending SEE view cloud messages */
struct SeeViewPublisher {
  ros::Publisher pub;
  ros::AdvertiseOptions a_o;

  //! Initialise ROS publisher for SEE view clouds
  /*!
    \param n ROS node handle
    \param topic Name of SEE view cloud topic
  */
  SeeViewPublisher(ros::NodeHandle &n, std::string topic) {
    a_o = ros::AdvertiseOptions::create<SeeViewCloud>(
        topic, 1, &connected, &disconnected, ros::VoidPtr(), NULL);
    a_o.latch = true;
    a_o.has_header = false;
    pub = n.advertise(a_o);
  }

  //! Publishing function
  /*!
    \param view SEE view being published
    \param view_num The view number
  */
  void SendMessage(SeeView view, int view_num) {
    SeeViewCloudPtr view_cloud(new pcl::PointCloud<SeeView>);

    view_cloud->push_back(view);
    view_cloud->header.seq = view_num;
    pub.publish(view_cloud);
  }
};

//! SEE Transform Listener
/*! Listens for tf transform messages and converts them into SEE views */
struct SenViewListener {
  SensorParams s_p;
  tf::TransformListener tf_tl;

  //! Initialise SEE transform listener
  /*!
    \param sensor_params Sensor parameters
  */
  SenViewListener(SensorParams sensor_params) { s_p = sensor_params; }

  //! Get the transform from the sensor to world frame from tf as a SeeView
  /*!
    \param view SeeView for the transform from the sensor to world frame
    \return Was the transform obtained successfully
  */
  bool GetView(SeeView &view) {
    Eigen::Affine3d tf_affine;
    tf::StampedTransform transform;

    if (!(tf_tl.frameExists(s_p.world_frame) &&
          tf_tl.frameExists(s_p.sensor_frame))) {
      ROS_ERROR("Coordinate frame initialisation failed.");
      ros::shutdown();
    }

    if (tf_tl.canTransform(s_p.world_frame, s_p.sensor_frame, ros::Time(0))) {
      tf_tl.lookupTransform(s_p.world_frame, s_p.sensor_frame, ros::Time(0),
                            transform);
      tf::transformTFToEigen(transform, tf_affine);
      AffineTransformToSeeView(tf_affine.cast<float>(), view, s_p.f_axis);

      return true;
    } else {
      return false;
    }
  }
};

//! SEE Transform Broadcaster
/*! Broadcast SEE views as tf transform messages */
struct SenViewBroadcaster {
  SensorParams s_p;
  tf::TransformBroadcaster tf_tb;

  //! Initialise SEE transform broadcaster
  /*!
    \param sensor_params Sensor parameters
  */
  SenViewBroadcaster(SensorParams sensor_params) { s_p = sensor_params; }

  //! Broadcast the sensor to world frame transform to tf from a SeeView
  /*!
    \param view SeeView for the transform from the sensor to world frame
  */
  void PublishView(SeeView &view) {
    tf::Transform transform;
    Eigen::Affine3f tf_affine;

    SeeViewToAffineTransform(view, tf_affine, s_p.f_axis);
    tf::transformEigenToTF(tf_affine.cast<double>(), transform);

    tf::StampedTransform st(transform, ros::Time::now(), s_p.world_frame,
                            s_p.sensor_frame);
    tf_tb.sendTransform(st);
  }
};

typedef boost::shared_ptr<SeeViewPublisher> SeeViewPublisherSPtr;
typedef boost::shared_ptr<SeeViewSubscriber> SeeViewSubscriberSPtr;
typedef boost::shared_ptr<SeeCloudPublisher> SeeCloudPublisherSPtr;
typedef boost::shared_ptr<SeeCloudSubscriber> SeeCloudSubscriberSPtr;
typedef boost::shared_ptr<SenViewListener> SenViewListenerSPtr;
typedef boost::shared_ptr<SenViewBroadcaster> SenViewBroadcasterSPtr;
typedef boost::shared_ptr<SenCloudPublisher> SenCloudPublisherSPtr;
typedef boost::shared_ptr<SenCloudSubscriber> SenCloudSubscriberSPtr;

#endif // ROS_COMMON_H_
