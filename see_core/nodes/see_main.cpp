// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#include <see_core/see_core.h>
#include <see_core/see_params.h>

#include <see_common/common_params.h>
#include <see_common/ros_common.h>

using namespace std::chrono;
using namespace ori::see::core;

//! Remove all points from the pointcloud that are outside the bounding box
/*!
  \param cloud Pointcloud result
  \param sensor Sensor parameters
*/
void ApplyBoundingBox(SeePointCloudPtr cloud, SensorParams sensor_params) {
  Eigen::Vector4f min_pt, max_pt;
  pcl::CropBox<SeePoint> crop_box;
  SeePointCloudPtr cloud_temp(new pcl::PointCloud<SeePoint>);

  ROS_INFO_STREAM("Applying Bounding Box");

  min_pt[0] = sensor_params.bounds[0];
  min_pt[1] = sensor_params.bounds[1];
  min_pt[2] = sensor_params.bounds[2];
  max_pt[0] = sensor_params.bounds[3];
  max_pt[1] = sensor_params.bounds[4];
  max_pt[2] = sensor_params.bounds[5];

  pcl::copyPointCloud(*cloud, *cloud_temp);

  crop_box.setInputCloud(cloud_temp);
  crop_box.setMin(min_pt);
  crop_box.setMax(max_pt);
  crop_box.filter(*cloud);
}

//! Publish NBV algorithm parameters to ROS
/*!
  \param core Pointer to core NBV algorithm
*/
void PublishParams(AbstractCoreSPtr core) {
  ros::param::set("/see/r", core->GetParams().r);
  ros::param::set("/see/rho", core->GetParams().rho);
  ros::param::set("/see/d", core->GetParams().d);
}

//! Publish NBV algorithm metrics to ROS
/*!
  \param core Pointer to core NBV algorithm
  \param view_time Computational time for processing the latest view
*/
void PublishMetrics(AbstractCoreSPtr core, float view_time) {
  ros::param::set("/see/view_time", view_time);
  ros::param::set("/see/view_num", core->GetViewNum());
  ros::param::set("/see/front_idx", core->GetFrontierIndex());
  ros::param::set("/see/front_num", core->GetObservedFrontierCount());
}

//! Check if the NBV has been reached within specified thresholds
/*!
  \param view The sensor view
  \param nbv The NBV
  \param nbv_p The NBV threshold parameters
  \return Has the sensor reached the NBV
*/
bool ReachedNBV(SeeView view, SeeView nbv, NBVParams nbv_p, bool &first_view) {
  float dst_off, ort_off;

  if (first_view) {
    first_view = false;
    return true;
  }

  dst_off = (view.getVector3fMap() - nbv.getVector3fMap()).norm();
  ort_off = view.getViewVector3fMap().dot(nbv.getViewVector3fMap());
  ort_off /= view.getViewVector3fMap().norm() * nbv.getViewVector3fMap().norm();
  ort_off = acos(fmin(fmax(ort_off, -1.0), 1.0));

  return dst_off < nbv_p.dst_thres && (ort_off * 180 / M_PI) < nbv_p.ort_thres;
}

//! Initialise SEE and process views until completion
/*!
  \param n The ROS node for SEE
*/
void SeeMain(ros::NodeHandle &n) {
  int cloud_num = 0;
  SeeTopics topics;
  SeeView nbv, view;
  SeeExtern ext_params;
  SeeParams see_params;
  NBVParams nbv_params;
  bool first_view = true;
  SensorParams sensor_params;

  LoadSeeTopics(topics);
  LoadSeeExtern(ext_params);
  LoadSeeParams(see_params);
  LoadNBVParams(nbv_params);
  LoadSensorParams(sensor_params);

  AbstractCoreSPtr core(new SeeCore(see_params, sensor_params));
  SenViewListenerSPtr view_listener(new SenViewListener(sensor_params));
  SeeViewPublisherSPtr view_pub(new SeeViewPublisher(n, topics.see_nbv));
  SeeViewSubscriberSPtr view_sub(new SeeViewSubscriber(n, topics.upd_nbv));
  SeeCloudPublisherSPtr cloud_pub(new SeeCloudPublisher(n, topics.see_pts));
  SenCloudSubscriberSPtr cloud_sub(
      new SenCloudSubscriber(n, topics.sen_pts, sensor_params));

  PublishParams(core);
  while (!CheckParam("/see/ready", true)) {
    ros::spinOnce();
  }

  ROS_INFO_STREAM("Starting observation");
  ros::spinOnce();
  ros::Rate rate(sensor_params.fps);
  while (ros::ok() && !core->IsDone()) {
    SeePointCloudPtr cloud(new pcl::PointCloud<SeePoint>);

    if (ReachedNBV(cloud_sub->view, core->GetNBV(), nbv_params, first_view)) {

      ROS_INFO_STREAM("Capturing view");
      while (!cloud_sub->GetViewandCloud(cloud, view)) {
        ros::spinOnce();
      }
      ApplyBoundingBox(cloud, sensor_params);
      ROS_INFO_STREAM("New View Obtained");

      ROS_INFO_STREAM("Updating Pointcloud");
      auto t1 = high_resolution_clock::now();
      core->UpdatePointCloud(cloud, view);
      auto t2 = high_resolution_clock::now();
      auto ts = duration_cast<std::chrono::duration<float>>(t2 - t1);

      PublishMetrics(core, ts.count());
      ros::spinOnce();

      view_pub->SendMessage(core->GetNBV(), core->GetViewNum());
      ros::spinOnce();

      while (cloud_num < core->GetViewNum()) {
        cloud_pub->SendMessage(core->GetPointCloud());
        ros::spinOnce();
        if (ext_params.use_receipts) {
          ros::param::get("/see/see_cloud_num", cloud_num);
        } else {
          cloud_num = core->GetViewNum();
        }
      }

      if (ext_params.use_constraints) {
        SetParam("/see/nbv_valid", false);
        while (!CheckParam("/see/nbv_valid", true) && !core->IsDone()) {
          ros::spinOnce();
          if (view_sub->GetView(nbv)) {
            core->UpdateNBV(nbv);
            PublishMetrics(core, ts.count());
            view_pub->SendMessage(core->GetNBV(), core->GetViewNum());
            ros::spinOnce();
          }
        }
      }
    }

    SetParam("/see/done", core->IsDone());
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO_STREAM("SEE Main Finished");
}

//! SEE Algorithm Main
/*!
  Run SEE
*/
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "see");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  ros::NodeHandle n;
  SeeMain(n);

  return 0;
}
