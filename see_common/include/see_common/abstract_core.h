// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef ABSTRACT_CORE_H
#define ABSTRACT_CORE_H

#include <see_common/common_structs.h>
#include <see_common/pcl_common.h>

//! Abstract core class for all NBV Planning algorithms
/*! Class with virtual methods common to all NBV Planning algorithms */
class AbstractCore {
public:
  //! Update the NBV Planner with a new pointcloud observation
  /*!
    \param pc_dash_ptr Pointer to the new pointcloud.
    \param v_p The sensor pose from which the pointcloud was obtained
  */
  virtual void UpdatePointCloud(SeePointCloudPtr pc_dash_ptr, SeeView &v_p) = 0;

  //! Get the algorithm parameters
  /*!
    \return The parameters used by the NBV planner
  */
  virtual const AlgParams &GetParams() = 0;

  //! Get the NBV
  /*!
    \return The NBV proposed by the NBV planner
  */
  virtual const SeeView &GetNBV() = 0;

  //! Get the latest view
  /*!
    \return Get the sensor view associated with the latest observation
  */
  virtual const SeeView &GetCurrentView() = 0;

  //! Get the Pointcloud
  /*!
    \return Get the combined pointcloud from all previous observations
  */
  virtual SeePointCloudConstPtr GetPointCloud() = 0;

  //! Get the NBV Proposals
  /*!
    \return Get the set of proposed NBVs from the NBV planning algorithm
  */
  virtual SeeViewCloudConstPtr GetViewCloud() = 0;

  //! Get the Octree Search Structure
  /*!
    \return Get the octree used for searching pointcloud observations
  */
  virtual SeePointOctreeConstPtr GetOctree() = 0;

  //! NBV Completion Status
  /*!
    \return Whether the NBV observation is complete
  */
  virtual bool IsDone() = 0;

  //! View Count
  /*!
    \return The number of views obtained
  */
  virtual int GetViewNum() = 0;

  //! NBV Frontier Index
  /*!
    \return Index of the frontier point associated with the chosen NBV
  */
  virtual int GetFrontierIndex() = 0;

  //! Number of Observed Frontiers
  /*!
    \return The number of frontier points observed by the latest view
  */
  virtual int GetObservedFrontierCount() = 0;

  //! Update the NBV to account for external view constraints
  /*!
    \param new_view The constrained NBV
  */
  virtual void UpdateNBV(SeeView new_view) = 0;

  //! Compute the value of parameter k
  /*!
    \param alg The parameters of the NBV algorithm
    \return Is the computed k value less than 3
  */
  bool ComputeK(AlgParams &alg);

  //! Compute the optimal search radius
  /*!
    \param alg The parameters of the NBV algorithm
    \return Was a new search radius set
  */
  bool SetRadius(AlgParams &alg);

  //! Compute the optimal view distance
  /*!
    \param alg The parameters of the NBV algorithm
    \param sen The parameters of the sensor
    \return Was a new view distance set
  */
  bool SetViewDistance(AlgParams &alg, SensorParams &sen);

  //! Compute the optimal target density
  /*!
    \param alg The parameters of the NBV algorithm
    \param sen The parameters of the sensor
    \return Was a new target density set
  */
  bool SetDensity(AlgParams &alg, SensorParams &sen);
};

//! Compute the value of parameter k
/*!
  \param alg The parameters of the NBV algorithm
  \return Is the computed k value less than 3
*/
inline bool AbstractCore::ComputeK(AlgParams &alg) {

  if (alg.r && alg.rho) {
    alg.k = ceil(((4.0 / 3.0) * M_PI * pow(alg.r, 3.0)) * alg.rho);
    alg.eps = pow((3.0 * alg.r) / (2.0 * M_PI * alg.rho), 1.0 / 3.0);

    if (alg.k < 3) {
      float r = alg.r;
      int rho = alg.rho;
      alg.rho = ceil(9.0 / (4.0 * M_PI * pow(r, 3.0)));
      alg.r = pow(9.0 / (4.0 * M_PI * rho), 1.0 / 3.0);
      return false;
    }
    return true;
  }
  return false;
}

//! Compute the value of parameter k
/*!
  \param alg The parameters of the NBV algorithm
  \return Is the computed k value less than 3
*/
inline bool AbstractCore::SetRadius(AlgParams &alg) {

  if (!alg.r && alg.rho) {
    alg.r = pow(9.0 / (4.0 * M_PI * alg.rho), 1.0 / 3.0);
    return true;
  }
  return false;
}

//! Compute the optimal view distance
/*!
  \param alg The parameters of the NBV algorithm
  \param sen The parameters of the sensor
  \return Was a new view distance set
*/
inline bool AbstractCore::SetViewDistance(AlgParams &alg, SensorParams &sen) {
  int pix = sen.pix_x * sen.pix_y;
  float f_x = M_PI * sen.fov_x / 360.0;
  float f_y = M_PI * sen.fov_y / 360.0;

  if (!alg.d && alg.rho && alg.r) {
    // alg.d = pow((3.0 * pix) / (4.0 * alg.rho * tan(f_x) * tan(f_y)), 1.0
    // / 3.0);
    float a = pix / (12.0 * alg.rho * tan(f_x) * tan(f_y));
    alg.d = sqrt(a - ((2 * pow(alg.r, 2)) / 3.0));
    return true;
  }
  return false;
}

//! Compute the optimal target density
/*!
  \param alg The parameters of the NBV algorithm
  \param sen The parameters of the sensor
  \return Was a new target density set
*/
inline bool AbstractCore::SetDensity(AlgParams &alg, SensorParams &sen) {
  int pix = sen.pix_x * sen.pix_y;
  float f_x = M_PI * sen.fov_x / 360.0;
  float f_y = M_PI * sen.fov_y / 360.0;

  if (!alg.rho && alg.d && alg.r) {
    float a = 4 * tan(f_x) * tan(f_y) * (3 * pow(alg.d, 2) + 2 * pow(alg.r, 2));
    alg.rho = pix / a;
    return true;
  }
  return false;
}

typedef boost::shared_ptr<AbstractCore> AbstractCoreSPtr;

#endif // ABSTRACT_CORE_H
