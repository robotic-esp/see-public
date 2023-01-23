// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef SEE_CORE_H
#define SEE_CORE_H

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <omp.h>
#include <ros/ros.h>
#include <set>
#include <time.h>
#include <vector>

#include <boost/container/map.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <lemon/adaptors.h>
#include <lemon/bfs.h>
#include <lemon/connectivity.h>
#include <lemon/list_graph.h>

#include <see_common/abstract_core.h>
#include <see_common/common_structs.h>

#include <see_core/see_structs.h>
#include <see_core/view_opt.h>

typedef std::vector<std::vector<int>> PointIdxMap;
typedef std::vector<std::vector<float>> PointDstMap;
typedef lemon::ListDigraph::NodeMap<int> SeeViewMap;
typedef boost::shared_ptr<std::vector<int>> SeePointIdxPtr;
typedef boost::shared_ptr<lemon::ListDigraph> SeeViewGraphPtr;
typedef boost::shared_ptr<lemon::ListDigraph::NodeMap<int>> SeeViewMapPtr;

//! View Controller Parameters
/*! Parameters storing the state of the view controller */
struct ViewControllerParams {
  std::vector<int> scale;      /*!< Scaling multiplier for view adjustment */
  std::vector<bool> switched;  /*!< Has the view been switched */
  std::vector<float> distance; /*!< Distance between point and center-of-mass */
};

namespace ori {
namespace see {
namespace core {

//! Core SEE(++) Algorithm
/*! Core SEE(++) algorithm that inherits member functions from abstract core */
class SeeCore : public AbstractCore {
public:
  //! Initialise SEE(++) with SEE and sensor parameters
  /*!
    \param see Parameters for SEE (r, rho, upsilon, psi, ...)
    \param sensor The parameters (FoV, resolution, noise) of the sensor
  */
  SeeCore(SeeParams see, SensorParams sensor);

  //! Update SEE(++) with a new sensor observation
  /*!
    \param see_pt_dash_ptr New pointcloud observation to be processed
    \param v_p Sensor pose from which new observation was obtained
  */
  void UpdatePointCloud(SeePointCloudPtr see_pt_dash_ptr, SeeView &v_p);

  //! Update the NBV to account for external view constraints
  /*!
    \param new_view The constrained NBV
  */
  void UpdateNBV(SeeView new_view);

  //! Get the SEE algorithm parameters
  /*!
    \return The parameters used by SEE
  */
  const AlgParams &GetParams();

  //! Get the NBV
  /*!
    \return The NBV proposed by SEE
  */
  const SeeView &GetNBV();

  //! Get the latest view
  /*!
    \return Get the sensor view associated with the latest observation
  */
  const SeeView &GetCurrentView();

  //! Get the Pointcloud
  /*!
    \return Get the combined pointcloud from all previous observations
  */
  SeePointCloudConstPtr GetPointCloud();

  //! Get the NBV Proposals
  /*!
    \return Get the set of proposed NBVs from SEE
  */
  SeeViewCloudConstPtr GetViewCloud();

  //! Get octree structure used for pointcloud search
  /*!
    \return Octree used for pointcloud search and classification
  */
  SeePointOctreeConstPtr GetOctree();

  //! NBV Frontier Index
  /*!
    \return Index of the frontier point associated with the chosen NBV
  */
  int GetFrontierIndex();

  //! Get Number of Frontiers Observed
  /*!
    \return Number of frontier points observed by the latest view
  */
  int GetObservedFrontierCount();

  //! Get View Number
  /*!
    \return Get number of views processed by SEE
  */
  int GetViewNum();

  //! Get SEE completion status
  /*!
    \return SEE completion state
  */
  bool IsDone();

private:
  bool done_; /*!< SEE completion state */
  int v_num_; /*!< View number */
  int f_idx_; /*!< Index of frontier point associated with current view */
  int f_cnt_; /*!< Number of frontier points */
  int r_num_; /*!< Number of reprocessed points */

  SeeView nbv_; /*!< Next best view sensor pose */
  SeeView c_v_; /*!< Sensor pose of latest view observation */
  SeeView i_v_; /*!< Sensor pose of initial view */

  SeeParams see_;           /*!< SEE parameters (r, rho, upsilon, psi, ...) */
  SensorParams sensor_;     /*!< Sensor parameters (FoV, resolution, noise) */
  ViewControllerParams vw_ctrl_; /*!< View controller state of each frontier point */

  SeePointIdxPtr F_; /*!< Set of frontier point indices */
  SeePointIdxPtr C_; /*!< Set of core point indices */
  SeePointIdxPtr O_; /*!< Set of outlier point indices */
  SeePointIdxPtr S_; /*!< Set of sparse point indices */

  omp_lock_t core_lock_;     /*!< Core lock for OMP Parrallelism */
  omp_lock_t queue_lock_;    /*!< Queue lock for OMP Parrallelism */
  omp_lock_t outlier_lock_;  /*!< Outlier lock for OMP Parrallelism */
  omp_lock_t frontier_lock_; /*!< Frontier lock for OMP Parrallelism */

  Eigen::ArrayXf P_dst_; /*!< Array of points projected by HPR */

  std::vector<int> G_idx_; /*!< Set of indices associated with graph vertices */
  std::vector<int> queue_; /*!< Last view which processed each frontier point */

  SeePointCloudPtr see_pt_ptr_; /*!< Pointcloud of all sensor observations */
  SeePointOctreePtr see_apt_oct_ptr_; /*!< Octree used for pointcloud search */
  SeePointOctreePtr see_fpt_oct_ptr_; /*!< Octree used for pointcloud search */
  SeePointOctreePtr see_cpt_oct_ptr_; /*!< Octree used for pointcloud search */
  SeePointOctreePtr see_opt_oct_ptr_; /*!< Octree used for pointcloud search */
  SeePointOctreePtr see_spt_oct_ptr_; /*!< Octree used for pointcloud search */

  XYZPointCloudPtr see_new_ptr_; /*!< Pointcloud of new sensor observations */
  XYZPointOctreePtr see_new_oct_ptr_; /*!< Octree used for pointcloud search */

  SeeViewCloudPtr see_vw_ptr_; /*!< Pointcloud containing SEE view proposals */
  SeeViewOctreePtr see_vw_oct_ptr_; /*!< Octree used for viewcloud search */

  SeeViewMapPtr see_vw_map_ptr_;   /*!< Node map for view visibility graph */
  SeeViewGraphPtr see_vw_gph_ptr_; /*!< View visibility graph */

  //! Initialise SEE parameters
  /*!
    Initialise parameters used by SEE
   */
  void InitialiseParameters();

  //! Initialise containers for new points
  /*!
    \param size Number of new points
  */
  void InitialisePoints(int size);

  //! Apply HPR to input pointcloud
  /*!
    \param see_pt_dash_ptr Input pointcloud
  */
  void ApplyHPR(SeePointCloudPtr see_pt_dash_ptr);

  //! Process new sensor observation
  /*!
    \param see_pt_dash_ptr Pointcloud from new observation
  */
  void ProcessNewPointCloud(SeePointCloudPtr see_pt_dash_ptr);

  //! Add a point observation to the SEE pointcloud
  /*!
    \param see_pt Point observation to be added
    \return If point was added to the pointcloud
  */
  bool AddPoint(SeePoint see_pt);

  //! Add all neighbouring points with a given label to the reprocessing queue
  /*!
    \param pt_idx Index of point being processed
    \param lbl Label of point class to be processed
    \param R_idx Set of points for reprocessing
  */
  void QueueNeighbours(int pt_idx, int lbl, std::vector<int> &R_idx);

  //! Process point classification
  /*!
    \param pt_idx Index of point being processed
    \return If the point class/label has changed
  */
  bool ProcessPoint(int pt_idx);

  //! Apply a new classification label to a given point
  /*!
    \param pt_idx Index of the point being (re)classified
    \param new_label New classification label for point
    \return If the point class/label has changed
  */
  bool ClassifyPoint(int pt_idx, int new_label);

  //! Check if a point should be classified as a core point
  /*!
    \param pt_idx Index of the point
    \return If the point meets the criteria for a core point
  */
  bool IsCorePoint(int pt_idx);

  //! Check if a point should be classified as a frontier point
  /*!
    \param pt_idx Index of the point
    \return If the point meets the criteria for a frontier point
  */
  bool IsFrontierPoint(int pt_idx);

  //! Add a point to the set of core points
  /*!
    \param pt_idx Index of the point
  */
  void AddCorePoint(int pt_idx);

  //! Add a point to the set of frontier points
  /*!
    \param pt_idx Index of the point
  */
  void AddFrontierPoint(int pt_idx);

  //! Add a point to the set of outlier points
  /*!
    \param pt_idx Index of the point
  */
  void AddOutlierPoint(int pt_idx);

  //! Remove a point from the set of frontier points
  /*!
    \param pt_idx Index of the point
  */
  void RemoveFrontierPoint(int pt_idx);

  //! Remove a point from the set of outlier points
  /*!
    \param pt_idx Index of the point
  */
  void RemoveOutlierPoint(int pt_idx);

  //! Check if a point is classified as a frontier
  /*!
    \param pt_idx Index of the point
    \return If pt_idx is a frontier point
  */
  bool CheckFrontierPoint(int pt_idx);

  //! Process a new frontier point
  /*!
    \param pt_idx Index of the point
  */
  void ProcessFrontierPoint(int pt_idx);

  //! Find direction of frontier normal
  /*!
    \param pt_idx Index of the point
    \param normal Normal for frontier point
  */
  void DirectNormals(int pt_idx, Eigen::Vector3f &normal);

  //! Add a view proposal to the frontier-view visibility graph
  /*!
    \param vw_idx Index of the view proposal
  */
  void AddViewGraph(int vw_idx);

  //! Remove a view proposal from the frontier-view visibility graph
  /*!
    \param vw_idx Index of the view proposal
  */
  void RemoveViewGraph(int vw_idx);

  //! Update the connectivity of a view in the frontier-view visibility graph
  /*!
    \param vw_idx Index of the view proposal
  */
  void UpdateViewGraph(int vw_idx);

  //! Update the view proposal associated with a frontier point
  /*!
    \param pt_idx Index of the frontier point
    \return If the view proposal was changed
  */
  bool UpdateView(int pt_idx);

  //! Walk along view vector from frontier to find occlusions
  /*!
    \param pt_idx Index of the frontier point
    \param delta Offset towards observing view
  */
  void WalkAlongVector(int pt_idx, int &delta);

  //! Check if a frontier point is visible from a proposed view
  /*!
    \param pt_idx Index of the frontier point
    \param vw_idx Index of the view proposal
    \param delta Distance offset from the frontier point
    \return If the frontier point passed the visibility check
  */
  bool IsVisible(int pt_idx, int vw_idx, int &delta);

  //! Project occluding points around a frontier for view optimisation
  /*!
    \param pt_idx Index of the frontier point
    \param delta Distance offset (as a multiplier of r) from the frontier point
    \param P The matrix of points to be projected
  */
  void ProjPoints(int pt_idx, int delta, Eigen::MatrixXf &P);

  //! Apply view optimisation if the visibility of a frontier is occluded
  /*!
    \param pt_idx Index of the frontier point
    \param delta Distance offset (as a multiplier of r) from the frontier point
    \param opt_view The optimised view proposal
    \return If the view optimisation was successful
  */
  bool OptimiseView(int pt_idx, int delta, Eigen::Vector3f &opt_view);

  //! Apply view controller using a new sensor observation
  /*!
    \param see_pt_dash_ptr New pointcloud from sensor observation
  */
  void ViewController(SeePointCloudPtr see_pt_dash_ptr);

  //! Switch view proposal to the view from which the point was first observed
  /*!
    \param pt_idx Index of the point
  */
  void SwitchView(int pt_idx);

  //! Adjust the view proposal associated with a given point
  /*!
    \param pt_idx Index of the point
    \param new_view New view proposal for pt_idx
  */
  void AdjustView(int pt_idx, Eigen::Vector3f new_view);

  //! Update the view cloud octree
  /*!
    Update the octree used to search the set of view proposals with new views
   */
  void UpdateViewOctree();

  //! Update the frontier cloud octree
  /*!
    Update the octree used to search the set of frontier points
   */
  void UpdateFrontierOctree();

  //! Update the outlier cloud octree
  /*!
    Update the octree used to search the set of outlier points
   */
  void UpdateOutlierOctree();

  //! Select a NBV
  /*!
    Selected a next best view from the set of view proposals
   */
  void SelectNBV();

  //! Select a NBV from the frontier-view visibility graph
  /*!
    Select a NBV using the frontier-view visibility graph
   */
  void SelectNBVFromGraph();
};

} // namespace core
} // namespace see
} // namespace ori

#endif // SEE_CORE_H
