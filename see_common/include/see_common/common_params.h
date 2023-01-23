// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef COMMON_PARAMS_H
#define COMMON_PARAMS_H

#include <ros/ros.h>
#include <see_common/common_structs.h>

using namespace ros::param;

//! Load external tool parameters from ROS
/*!
  \param see_ext Struct containing sensor parameters
*/
void LoadSeeExtern(SeeExtern &see_ext) {
  param<bool>("/see/use_rviz", see_ext.use_rviz, false);
  param<bool>("/see/use_moveit", see_ext.use_moveit, false);
  param<bool>("/see/use_receipts", see_ext.use_receipts, false);
  param<bool>("/see/use_constraints", see_ext.use_constraints, false);
}

//! Load NBV parameters from ROS
/*!
  \param nbv Struct containing NBV parameters
*/
void LoadNBVParams(NBVParams &nbv) {
  param<float>("~/nbv/dst_thres", nbv.dst_thres, 0.01);
  param<float>("~/nbv/ort_thres", nbv.ort_thres, 1.00);
}

//! Load SEE topics from ROS
/*!
  \param topics Struct containing SEE topics
*/
void LoadSeeTopics(SeeTopics &topics) {
  param<std::string>("/see/topics/upd_nbv", topics.upd_nbv, "/see/upd_nbv");
  param<std::string>("/see/topics/sim_nbv", topics.sim_nbv, "/see/sim_nbv");
  param<std::string>("/see/topics/see_nbv", topics.see_nbv, "/see/see_nbv");
  param<std::string>("/see/topics/see_pts", topics.see_pts, "/see/see_pts");
  param<std::string>("/see/topics/sen_pts", topics.sen_pts, "/see/sen_pts");
}

//! Load sensor parameters from ROS
/*!
  \param sensor Struct containing sensor parameters
*/
void LoadSensorParams(SensorParams &sensor) {
  param<int>("/see/sensor/fps", sensor.fps, 30);
  param<int>("/see/sensor/f_axis", sensor.f_axis, 2);
  param<int>("/see/sensor/pix_x", sensor.pix_x, 848);
  param<int>("/see/sensor/pix_y", sensor.pix_y, 480);
  param<float>("/see/sensor/fov_x", sensor.fov_x, 69.4);
  param<float>("/see/sensor/fov_y", sensor.fov_y, 42.5);
  param<float>("/see/sensor/noise", sensor.noise, 0.01);
  param<std::string>("/see/sensor/sensor_frame", sensor.sensor_frame, "sensor");
  param<std::string>("/see/sensor/world_frame", sensor.world_frame, "world");
  param<std::vector<float>>("/see/sensor/bounds", sensor.bounds,
                            std::vector<float>());
}

#endif // COMMON_PARAMS_H
