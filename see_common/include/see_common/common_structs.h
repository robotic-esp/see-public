// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

#include <string>
#include <vector>

//! Algorithm Parameters
/*! Parameters common to all algorithms */
struct AlgParams {
  int k;     /*!< The number of points in a radius r to obtain a density rho */
  int rho;   /*!< The target point density per cubic unit (e.g., meters) */
  float r;   /*!< The radius for computing local point density */
  float d;   /*!< The view distance */
  float eps; /*!< The distance tolerance to stop adding 'duplicate' points */
};

//! External Parameters
/*! Parameters defining use of external tools */
struct SeeExtern {
  bool use_rviz;        /*!< Use rviz to visualise SEE output */
  bool use_moveit;      /*!< Use moveit to plan paths to views for a robot */
  bool use_receipts;    /*!< Will SEE require receipts for sent pointclouds  */
  bool use_constraints; /*!< Will views have external constraints applied */
};

//! NBV Parameters
/*! Parameters for reaching and constraining views */
struct NBVParams {
  float dst_thres; /*!< The distance threshold to see if the NBV is reached */
  float ort_thres; /*!< The angular threshold to see if the NBV is reached */
};

//! SEE Topics
/*! Topics used for receiving data input to and publishing output from SEE */
struct SeeTopics {
  std::string upd_nbv; /*!< Topic for receiving externally constrained views */
  std::string sim_nbv; /*!< Topic for sending views to a simulation engine */
  std::string see_nbv; /*!< Topic for publishing SEE NBVs */
  std::string see_pts; /*!< Topic for publishing the SEE pointcloud */
  std::string sen_pts; /*!< Topic for receiving sensor pointclouds */
};

//! Sensor Parameters
/*! Parameters of the sensor */
struct SensorParams {
  int fps;     /*!< Sensor FPS (Frames per Second) */
  int pix_x;   /*!< Number of pixels (sensor measurements) along the x-axis */
  int pix_y;   /*!< Number of pixels (sensor measurements) along the y-axis */
  int f_axis;  /*!< Forward axis of sensor (usually z for RGBD, x for LIDAR) */
  float fov_x; /*!< Field of view (degrees) on the x-axis */
  float fov_y; /*!< Field of view (degrees) on the y-axis */
  float noise; /*!< Std dev of noise on point observations (sim only) */
  std::string sensor_frame;  /*!< Frame for sensor */
  std::string world_frame;   /*!< World frame for the sensor */
  std::vector<float> bounds; /*!< Bounding box for scene */
};

#endif // COMMON_STRUCTS_H
