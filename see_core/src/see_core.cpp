// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#include <see_core/see_core.h>

namespace ori {
namespace see {
namespace core {

//! Initialise SEE(++) with SEE and sensor parameters
/*!
  \param see Parameters for SEE (r, rho, tau, psi, ...)
  \param sensor The parameters (FoV, resolution, noise) of the sensor
*/
SeeCore::SeeCore(SeeParams see, SensorParams sensor)
    : see_(see), sensor_(sensor), f_idx_(INT_MAX), v_num_(0),
      F_(new std::vector<int>), C_(new std::vector<int>),
      O_(new std::vector<int>), S_(new std::vector<int>),
      see_pt_ptr_(new SeePointCloud), see_new_ptr_(new XYZPointCloud),
      see_vw_ptr_(new SeeViewCloud), see_vw_gph_ptr_(new lemon::ListDigraph),
      see_vw_oct_ptr_(new SeeViewOctree(see.psi)),
      see_apt_oct_ptr_(new SeePointOctree(see.r)),
      see_fpt_oct_ptr_(new SeePointOctree(see.r)),
      see_cpt_oct_ptr_(new SeePointOctree(see.r)),
      see_opt_oct_ptr_(new SeePointOctree(see.r)),
      see_spt_oct_ptr_(new SeePointOctree(see.ups)) {

  InitialiseParameters();
  see_pt_ptr_->header.frame_id = sensor_.world_frame;
  see_vw_oct_ptr_->setInputCloud(see_vw_ptr_);
  see_apt_oct_ptr_->setInputCloud(see_pt_ptr_);
  see_cpt_oct_ptr_->setInputCloud(see_pt_ptr_, C_);
  see_fpt_oct_ptr_->setInputCloud(see_pt_ptr_, F_);
  see_opt_oct_ptr_->setInputCloud(see_pt_ptr_, O_);
  see_spt_oct_ptr_->setInputCloud(see_pt_ptr_, S_);
  see_vw_map_ptr_ = boost::make_shared<SeeViewMap>(*see_vw_gph_ptr_);
}

//! Initialise SEE parameters
/*!
  Initialise parameters used by SEE
 */
void SeeCore::InitialiseParameters() {
  SetViewDistance(see_, sensor_);
  SetDensity(see_, sensor_);
  SetRadius(see_);

  see_apt_oct_ptr_->setResolution(see_.r);
  see_cpt_oct_ptr_->setResolution(see_.r);
  see_fpt_oct_ptr_->setResolution(see_.r);
  see_opt_oct_ptr_->setResolution(see_.r);

  if (ComputeK(see_)) {
    ROS_WARN_STREAM("k: " << see_.k << " r: " << see_.r << " d: " << see_.d
                          << " tau: " << see_.tau << " psi: " << see_.psi
                          << " ups: " << see_.ups << " rho: " << see_.rho
                          << " eps: " << see_.eps);
    done_ = false;
  } else {
    ROS_ERROR_STREAM("k < 3, set rho > " << std::setprecision(3) << see_.rho
                                         << " or r > " << see_.r);
    done_ = true;
  }
}

//! Update SEE(++) with a new sensor observation
/*!
  \param see_pt_dash_ptr New pointcloud observation to be processed
  \param v_p Sensor pose from which new observation was obtained
*/
void SeeCore::UpdatePointCloud(SeePointCloudPtr see_pt_dash_ptr, SeeView &v_p) {
  std::vector<int> N_idx;
  std::vector<float> N_dst;
  int opt_views = 0, gph_views = 0;

  f_cnt_ = 0;
  c_v_ = v_p;
  if (!v_num_) {
    i_v_ = v_p;
  }
  v_num_++;

  see_pt_ptr_->header.seq = v_num_;
  ROS_INFO_STREAM("View " << v_num_);
  ApplyHPR(see_pt_dash_ptr);

  if (!see_pt_ptr_->size() && !see_pt_dash_ptr->size()) {
    ROS_ERROR("No points visible, choose a different initial view");
    done_ = true;
    return;
  } else {
    ProcessNewPointCloud(see_pt_dash_ptr);
  }

  if (CheckFrontierPoint(f_idx_)) {
    ROS_INFO_STREAM("Applying View Controller");
    ViewController(see_pt_dash_ptr);
  }

  UpdateViewOctree();
  see_vw_oct_ptr_->nearestKSearch(c_v_, see_.tau, N_idx, N_dst);
  for (size_t i = 0; i < N_idx.size(); i++) {
    opt_views += UpdateView(N_idx[i]);
  }
  ROS_INFO_STREAM("Optimised " << opt_views << " views");

  UpdateViewOctree();
  see_vw_oct_ptr_->nearestKSearch(c_v_, see_.tau, N_idx, N_dst);
  for (size_t i = 0; i < N_idx.size(); i++) {
    UpdateViewGraph(N_idx[i]);
    gph_views++;
  }
  ROS_INFO_STREAM("Graphed " << gph_views << " views");

  SelectNBV();
}

//! Apply HPR to input pointcloud
/*!
  \param see_pt_dash_ptr Input pointcloud
*/
void SeeCore::ApplyHPR(SeePointCloudPtr see_pt_dash_ptr) {
  Eigen::MatrixXf P;
  pcl::demeanPointCloud(*see_pt_dash_ptr, c_v_.getVector4fMap(), P);
  P_dst_ = P.colwise().norm().array();

  see_new_ptr_->resize(P.cols());
  see_new_ptr_->getMatrixXfMap() = P.colwise().normalized();

  see_new_oct_ptr_ = boost::make_shared<XYZPointOctree>(0.01);
  see_new_oct_ptr_->setInputCloud(see_new_ptr_);
  see_new_oct_ptr_->addPointsFromInputCloud();
}

//! Process new sensor observation
/*!
  \param see_pt_dash_ptr Pointcloud from new observation
*/
void SeeCore::ProcessNewPointCloud(SeePointCloudPtr see_pt_dash_ptr) {
  std::vector<int> R_idx;
  int start_idx, new_count = 0;

  see_pt_ptr_->reserve(see_pt_ptr_->size() + see_pt_dash_ptr->size());
  see_vw_ptr_->reserve(see_pt_ptr_->size() + see_pt_dash_ptr->size());

  for (size_t i = 0; i < see_pt_dash_ptr->size(); i++) {
    new_count += AddPoint(see_pt_dash_ptr->points[i]);
  }
  InitialisePoints(see_pt_ptr_->size());
  start_idx = see_pt_ptr_->size() - new_count;

  omp_init_lock(&core_lock_);
  omp_init_lock(&queue_lock_);
  omp_init_lock(&outlier_lock_);
  omp_init_lock(&frontier_lock_);

  ROS_INFO_STREAM("Classifying " << new_count << " new points");
#pragma omp parallel for
  for (size_t i = start_idx; i < see_pt_ptr_->size(); i++) {
    if (ProcessPoint(i)) {
      QueueNeighbours(i, frontier_pt, R_idx);
      QueueNeighbours(i, outlier_pt, R_idx);
    }
  }
  r_num_ = R_idx.size();

  ROS_INFO_STREAM("Reclassifying " << R_idx.size() << " existing points");
#pragma omp parallel for
  for (size_t i = 0; i < R_idx.size(); i++) {
    ProcessPoint(R_idx[i]);
  }

  omp_destroy_lock(&core_lock_);
  omp_destroy_lock(&queue_lock_);
  omp_destroy_lock(&outlier_lock_);
  omp_destroy_lock(&frontier_lock_);

  ROS_INFO_STREAM("Core points: " << C_->size());
  ROS_INFO_STREAM("Frontier points: " << F_->size());
  ROS_INFO_STREAM("Outlier points: " << O_->size());
}

//! Initialise containers for new points
/*!
  \param size Number of new points
*/
void SeeCore::InitialisePoints(int size) {
  queue_.resize(size, v_num_ - 1);
  vw_ctrl_.distance.resize(size, FLT_MAX);
  vw_ctrl_.switched.resize(size, true);
  vw_ctrl_.scale.resize(size, 1);
  G_idx_.resize(size, 0);
}

//! Add a point observation to the SEE pointcloud
/*!
  \param see_pt Point observation to be added
  \return If point was added to the pointcloud
*/
bool SeeCore::AddPoint(SeePoint see_pt) {
  int n_idx;
  float n_dst;
  bool add_pt;
  std::vector<int> N_idx;
  std::vector<float> N_dst;

  n_dst = 1;
  if (see_pt_ptr_->size()) {
    see_apt_oct_ptr_->approxNearestSearch(see_pt, n_idx, n_dst);
  }

  add_pt = sqrt(n_dst) > see_.eps;
  if (add_pt) {
    see_pt.label = null_pt;
    see_pt.getViewVector3fMap() = c_v_.getVector3fMap();
    see_apt_oct_ptr_->addPointToCloud(see_pt, see_pt_ptr_);
    see_vw_ptr_->push_back(c_v_);

    n_dst = 1;
    if (S_->size()) {
      see_spt_oct_ptr_->approxNearestSearch(see_pt, n_idx, n_dst);
    }
    if (sqrt(n_dst) > see_.ups) {
      see_spt_oct_ptr_->addPointFromCloud(see_pt_ptr_->size() - 1, S_);
    }
  }

  return add_pt;
}

//! Add all neighbouring points with a given label to the reprocessing queue
/*!
  \param pt_idx Index of point being processed
  \param lbl Label of point class to be processed
  \param R_idx Set of points for reprocessing
*/
void SeeCore::QueueNeighbours(int pt_idx, int lbl, std::vector<int> &R_idx) {
  std::vector<int> N_idx;
  std::vector<float> N_dst;

  switch (lbl) {
  case frontier_pt:
    omp_set_lock(&frontier_lock_);
    see_fpt_oct_ptr_->radiusSearch(pt_idx, see_.r, N_idx, N_dst);
    omp_unset_lock(&frontier_lock_);
    break;
  case outlier_pt:
    omp_set_lock(&outlier_lock_);
    see_opt_oct_ptr_->radiusSearch(pt_idx, see_.r, N_idx, N_dst);
    omp_unset_lock(&outlier_lock_);
    break;
  }

  for (size_t i = 0; i < N_idx.size(); i++) {
    omp_set_lock(&queue_lock_);
    if (queue_[N_idx[i]] != v_num_) {
      R_idx.push_back(N_idx[i]);
      queue_[N_idx[i]] = v_num_;
    }
    omp_unset_lock(&queue_lock_);
  }
}

//! Process point classification
/*!
  \param pt_idx Index of point being processed
  \return If the point class/label has changed
*/
bool SeeCore::ProcessPoint(int pt_idx) {
  if (IsCorePoint(pt_idx)) {
    return ClassifyPoint(pt_idx, core_pt);
  } else if (IsFrontierPoint(pt_idx)) {
    return ClassifyPoint(pt_idx, frontier_pt);
  } else {
    return ClassifyPoint(pt_idx, outlier_pt);
  }
}

//! Apply a new classification label to a given point
/*!
  \param pt_idx Index of the point being (re)classified
  \param new_label New classification label for point
  \return If the point class/label has changed
*/
bool SeeCore::ClassifyPoint(int pt_idx, int new_label) {
  int current_label = see_pt_ptr_->points[pt_idx].label;

  if (current_label != new_label) {
    switch (current_label) {
    case outlier_pt:
      omp_set_lock(&outlier_lock_);
      RemoveOutlierPoint(pt_idx);
      omp_unset_lock(&outlier_lock_);
      break;
    case frontier_pt:
      omp_set_lock(&frontier_lock_);
      RemoveFrontierPoint(pt_idx);
      omp_unset_lock(&frontier_lock_);
      break;
    }

    switch (new_label) {
    case core_pt:
      omp_set_lock(&core_lock_);
      AddCorePoint(pt_idx);
      omp_unset_lock(&core_lock_);
      break;
    case frontier_pt:
      ProcessFrontierPoint(pt_idx);
      omp_set_lock(&frontier_lock_);
      AddFrontierPoint(pt_idx);
      omp_unset_lock(&frontier_lock_);
      break;
    case outlier_pt:
      omp_set_lock(&outlier_lock_);
      AddOutlierPoint(pt_idx);
      omp_unset_lock(&outlier_lock_);
      break;
    }
  }

  return current_label != new_label;
}

//! Check if a point should be classified as a core point
/*!
  \param pt_idx Index of the point
  \return If the point meets the criteria for a core point
*/
bool SeeCore::IsCorePoint(int pt_idx) {
  std::vector<int> N_idx;
  std::vector<float> N_dst;

  see_apt_oct_ptr_->radiusSearch(pt_idx, see_.r, N_idx, N_dst, see_.k);

  return N_idx.size() == see_.k;
}

//! Check if a point should be classified as a frontier point
/*!
  \param pt_idx Index of the point
  \return If the point meets the criteria for a frontier point
*/
bool SeeCore::IsFrontierPoint(int pt_idx) {
  SeePoint f;
  std::vector<int> C_idx, O_idx;
  std::vector<float> C_dst, O_dst;

  f = see_pt_ptr_->points[pt_idx];

  omp_set_lock(&core_lock_);
  see_cpt_oct_ptr_->radiusSearch(pt_idx, see_.r, C_idx, C_dst, 1);
  omp_unset_lock(&core_lock_);

  omp_set_lock(&outlier_lock_);
  see_opt_oct_ptr_->radiusSearch(pt_idx, see_.r, O_idx, O_dst, 1);
  omp_unset_lock(&outlier_lock_);

  return C_idx.size() && O_idx.size();
}

//! Add a point to the set of core points
/*!
  \param pt_idx Index of the point
*/
void SeeCore::AddCorePoint(int pt_idx) {
  if (see_pt_ptr_->points[pt_idx].label != core_pt) {
    see_pt_ptr_->points[pt_idx].label = core_pt;
    see_pt_ptr_->points[pt_idx].intensity = 90;
    see_cpt_oct_ptr_->addPointFromCloud(pt_idx, C_);
  }
}

//! Add a point to the set of frontier points
/*!
  \param pt_idx Index of the point
*/
void SeeCore::AddFrontierPoint(int pt_idx) {
  if (see_pt_ptr_->points[pt_idx].label != frontier_pt) {
    see_pt_ptr_->points[pt_idx].label = frontier_pt;
    see_pt_ptr_->points[pt_idx].intensity = 180;
    see_fpt_oct_ptr_->addPointFromCloud(pt_idx, F_);
    AddViewGraph(pt_idx);
  }
}

//! Add a point to the set of outlier points
/*!
  \param pt_idx Index of the point
*/
void SeeCore::AddOutlierPoint(int pt_idx) {
  if (see_pt_ptr_->points[pt_idx].label != outlier_pt) {
    see_pt_ptr_->points[pt_idx].label = outlier_pt;
    see_pt_ptr_->points[pt_idx].intensity = 0;
    see_opt_oct_ptr_->addPointFromCloud(pt_idx, O_);
  }
}

//! Remove a point from the set of frontier points
/*!
  \param pt_idx Index of the point
*/
void SeeCore::RemoveFrontierPoint(int pt_idx) {
  if (see_pt_ptr_->points[pt_idx].label == frontier_pt) {
    F_->erase(std::next(std::find(F_->rbegin(), F_->rend(), pt_idx)).base());
    UpdateFrontierOctree();
    RemoveViewGraph(pt_idx);
    f_cnt_++;
  }
}

//! Remove a point from the set of outlier points
/*!
  \param pt_idx Index of the point
*/
void SeeCore::RemoveOutlierPoint(int pt_idx) {
  if (see_pt_ptr_->points[pt_idx].label == outlier_pt) {
    O_->erase(std::next(std::find(O_->rbegin(), O_->rend(), pt_idx)).base());
    UpdateOutlierOctree();
  }
}

//! Check if a point is classified as a frontier
/*!
  \param pt_idx Index of the point
  \return If pt_idx is a frontier point
*/
bool SeeCore::CheckFrontierPoint(int pt_idx) {
  if (pt_idx < see_pt_ptr_->size()) {
    return see_pt_ptr_->points[pt_idx].label == frontier_pt;
  } else {
    return false;
  }
}

//! Process a point as a frontier point
/*!
  \param pt_idx Index of the point
*/
void SeeCore::ProcessFrontierPoint(int pt_idx) {
  SeePoint f_p;
  float max_v, min_v;
  Eigen::Vector4f N_bar;
  Eigen::Matrix3f A, Phi;
  std::vector<int> N_idx;
  std::vector<float> N_dst;
  Eigen::Matrix3f::Index max_i, min_i;
  Eigen::Vector3f f_v, f_m, Lambda, normal, frontier, view, pose, ratio;

  see_apt_oct_ptr_->nearestKSearch(pt_idx, see_.k, N_idx, N_dst);
  pcl::computeMeanAndCovarianceMatrix(*see_pt_ptr_, N_idx, A, N_bar);

  f_p = see_pt_ptr_->points[pt_idx];
  f_m = (f_p.getVector3fMap() - N_bar.head(3)).normalized();
  f_v = (f_p.getViewVector3fMap() - f_p.getVector3fMap()).normalized();

  Eigen::EigenSolver<Eigen::Matrix3f> eig(A);
  Phi = eig.eigenvectors().real();
  Lambda = eig.eigenvalues().real();

  min_v = Lambda.minCoeff(&min_i);
  normal = Phi.col(min_i).normalized();
  Phi.col(min_i) = Eigen::Vector3f::Zero();
  max_v = (f_m.asDiagonal() * Phi).colwise().sum().cwiseAbs().maxCoeff(&max_i);
  frontier = Phi.col(max_i).normalized();

  normal *= boost::math::sign(normal.dot(f_v) + FLT_MIN);
  frontier *= boost::math::sign(frontier.dot(f_m) + FLT_MIN);

  DirectNormals(pt_idx, normal);

  see_pt_ptr_->points[pt_idx].getNormalVector3fMap() = normal;
  see_pt_ptr_->points[pt_idx].getFrontierVector3fMap() = frontier;
  see_pt_ptr_->points[pt_idx].getBoundaryVector3fMap() = normal.cross(frontier);

  pose = -normal;
  view = f_p.getVector3fMap() - see_.d * pose;

  see_vw_ptr_->points[pt_idx].getVector3fMap() = view;
  see_vw_ptr_->points[pt_idx].getViewVector3fMap() = pose;
}

//! Find direction of frontier normal
/*!
  \param pt_idx Index of the point
  \param normal Normal for frontier point
*/
void SeeCore::DirectNormals(int pt_idx, Eigen::Vector3f &normal) {
  int i;
  pcl::PointXYZ p_pos, p_neg;
  Eigen::Vector3f v_pos, v_neg;
  std::vector<int> N_idx_pos, N_idx_neg;
  std::vector<float> N_dst_pos, N_dst_neg;
  bool free_pos = false, free_neg = false;

  v_pos = see_pt_ptr_->points[pt_idx].getVector3fMap() - c_v_.getVector3fMap();
  v_neg = see_pt_ptr_->points[pt_idx].getVector3fMap() - c_v_.getVector3fMap();

  while (!free_pos && !free_neg) {
    v_pos = v_pos + (see_.ups * normal);
    v_neg = v_neg - (see_.ups * normal);
    p_pos.getVector3fMap() = v_pos.normalized();
    p_neg.getVector3fMap() = v_neg.normalized();
    see_new_oct_ptr_->radiusSearch(p_pos, 0.01, N_idx_pos, N_dst_pos);
    see_new_oct_ptr_->radiusSearch(p_neg, 0.01, N_idx_neg, N_dst_neg);

    i = 0;
    free_pos = true;
    while (free_pos && i < N_idx_pos.size()) {
      if (P_dst_(N_idx_pos[i]) < v_pos.norm())
        free_pos = false;
      i++;
    }

    i = 0;
    free_neg = true;
    while (free_neg && i < N_idx_neg.size()) {
      if (P_dst_(N_idx_neg[i]) < v_neg.norm())
        free_neg = false;
      i++;
    }
  }

  if (!free_pos && free_neg) {
    normal *= -1;
  }
}

//! Add a view proposal to the frontier-view visibility graph
/*!
  \param vw_idx Index of the view proposal
*/
void SeeCore::AddViewGraph(int vw_idx) {
  G_idx_[vw_idx] = see_vw_gph_ptr_->id(see_vw_gph_ptr_->addNode());
  (*see_vw_map_ptr_)[see_vw_gph_ptr_->nodeFromId(G_idx_[vw_idx])] = vw_idx;
}

//! Remove a view proposal from the frontier-view visibility graph
/*!
  \param vw_idx Index of the view proposal
*/
void SeeCore::RemoveViewGraph(int vw_idx) {
  see_vw_gph_ptr_->erase(see_vw_gph_ptr_->nodeFromId(G_idx_[vw_idx]));
}

//! Update the connectivity of a view in the frontier-view visibility graph
/*!
  \param vw_idx Index of the view proposal
*/
void SeeCore::UpdateViewGraph(int vw_idx) {
  int delta;
  std::vector<int> N_idx;
  std::vector<float> N_dst;
  lemon::ListDigraph::Node s, t;
  std::vector<lemon::ListDigraph::Arc> arcs;

  s = see_vw_gph_ptr_->nodeFromId(G_idx_[vw_idx]);

  lemon::ListDigraph::OutArcIt out_arc(*see_vw_gph_ptr_, s);
  for (; out_arc != lemon::INVALID; ++out_arc) {
    arcs.push_back(out_arc);
  }
  for (size_t i = 0; i < arcs.size(); i++) {
    see_vw_gph_ptr_->erase(arcs[i]);
  }

  see_vw_oct_ptr_->nearestKSearch(vw_idx, see_.tau, N_idx, N_dst);
  for (size_t i = 0; i < N_idx.size(); i++) {
    if (IsVisible(N_idx[i], vw_idx, delta) && N_idx[i] != vw_idx) {
      t = see_vw_gph_ptr_->nodeFromId(G_idx_[N_idx[i]]);
      see_vw_gph_ptr_->addArc(s, t);
    }
  }
}

//! Update the view proposal associated with a frontier point
/*!
  \param pt_idx Index of the frontier point
  \return If the view proposal was changed
*/
bool SeeCore::UpdateView(int pt_idx) {
  int delta;
  Eigen::Vector3f view, pose;

  if (!IsVisible(pt_idx, pt_idx, delta)) {
    if (OptimiseView(pt_idx, delta, pose)) {
      view = see_pt_ptr_->points[pt_idx].getVector3fMap() - see_.d * pose;
      see_vw_ptr_->points[pt_idx].getVector3fMap() = view;
      see_vw_ptr_->points[pt_idx].getViewVector3fMap() = pose;
      if (!IsVisible(pt_idx, pt_idx, delta)) {
        RemoveFrontierPoint(pt_idx);
        AddOutlierPoint(pt_idx);
      }
      return true;
    }
  }
  return false;
}

//! Walk along view vector from frontier to find occlusions
/*!
  \param pt_idx Index of the frontier point
  \param delta Offset towards observing view
*/
void SeeCore::WalkAlongVector(int pt_idx, int &delta) {
  SeePoint f;
  Eigen::Vector3f n;
  std::vector<float> N_dst;
  std::vector<int> N_idx(1);

  f = see_pt_ptr_->points[pt_idx];
  n = f.getNormalVector3fMap();

  delta = -1;
  while (N_idx.size()) {
    see_apt_oct_ptr_->radiusSearch(f, see_.ups, N_idx, N_dst, 1);
    f.getVector3fMap() += see_.ups * n;
    delta++;
  }
}

//! Check if a frontier point is visible from a proposed view
/*!
  \param pt_idx Index of the frontier point
  \param vw_idx Index of the view proposal
  \param delta Distance offset from the frontier point
  \return If the frontier point passed the visibility check
*/
bool SeeCore::IsVisible(int pt_idx, int vw_idx, int &delta) {
  float dist = 0;
  Eigen::Vector3f vis_vec;
  std::vector<int> N_idx;
  std::vector<float> N_dst;

  SeeView v = see_vw_ptr_->points[vw_idx];
  SeePoint f = see_pt_ptr_->points[pt_idx];

  vis_vec = (v.getVector3fMap() - f.getVector3fMap()).normalized();
  WalkAlongVector(pt_idx, delta);
  f.getVector3fMap() += delta * see_.ups * vis_vec;

  while (!N_idx.size() && dist < see_.psi) {
    see_apt_oct_ptr_->radiusSearch(f, see_.ups, N_idx, N_dst, 1);
    f.getVector3fMap() += see_.ups * vis_vec;
    dist += see_.ups;
  }

  if (N_idx.size()) {
    return false;
  }
  return true;
}

//! Project occluding points around a frontier for view optimisation
/*!
  \param pt_idx Index of the frontier point
  \param delta Distance offset (as a multiplier of r) from the frontier point
  \param P The matrix of points to be projected
*/
void SeeCore::ProjPoints(int pt_idx, int delta, Eigen::MatrixXf &P) {
  SeePoint f;
  Eigen::Vector3f v;
  std::vector<int> N_idx;
  std::vector<float> N_dst;

  f = see_pt_ptr_->points[pt_idx];
  v = (f.getViewVector3fMap() - f.getVector3fMap()).normalized();

  see_spt_oct_ptr_->radiusSearch(f, see_.psi, N_idx, N_dst);
  f.getVector3fMap() += delta * see_.ups * v;

  pcl::demeanPointCloud(*see_pt_ptr_, N_idx, f.getVector4fMap(), P);
  P.colwise().normalize();
}

//! Apply view optimisation if the visibility of a frontier is occluded
/*!
  \param pt_idx Index of the frontier point
  \param delta Distance offset (as a multiplier of r) from the frontier point
  \param opt_view The optimised view proposal
  \return If the view optimisation was successful
*/
bool SeeCore::OptimiseView(int pt_idx, int delta, Eigen::Vector3f &opt_view) {
  int result;
  SeePoint f;
  Eigen::Vector3f v;
  Eigen::MatrixXf P;

  f = see_pt_ptr_->points[pt_idx];
  v = (f.getVector3fMap() - f.getViewVector3fMap()).normalized();

  ProjPoints(pt_idx, delta, P);
  if (ViewOpt(v, P, opt_view, result)) {
  } else if (ViewOptHemi(v, P, opt_view, result)) {
  } else {
    return false;
  }

  return true;
}

//! Apply view controller using a new sensor observation
/*!
  \param see_pt_dash_ptr New pointcloud from sensor observation
*/
void SeeCore::ViewController(SeePointCloudPtr see_pt_dash_ptr) {
  float theta_f, theta_b;
  Eigen::Matrix3f u_x_f, u_x_b, R_f, R_b, R;
  Eigen::Vector3f f_p, f_n, f_b, f_f, f_d, v_p, v_s;
  Eigen::Vector4f centroid = Eigen::Vector4f::Zero();

  SeeView v = see_vw_ptr_->points[f_idx_];
  SeePoint f = see_pt_ptr_->points[f_idx_];

  v_p = v.getVector3fMap();
  f_p = f.getVector3fMap();
  f_n = f.getNormalVector3fMap();
  f_f = f.getFrontierVector3fMap();
  f_b = f.getBoundaryVector3fMap();

  u_x_f << 0, -f_f(2), f_f(1), f_f(2), 0, -f_f(0), -f_f(1), f_f(0), 0;
  u_x_b << 0, -f_b(2), f_b(1), f_b(2), 0, -f_b(0), -f_b(1), f_b(0), 0;
  R << f_n(0), f_f(0), f_b(0), f_n(1), f_f(1), f_b(1), f_n(2), f_f(2), f_b(2);

  pcl::compute3DCentroid(*see_pt_dash_ptr, centroid);
  f_d = R.transpose() * (f.getVector3fMap() - centroid.head(3));

  if (!see_pt_dash_ptr->size() || !r_num_) {
    ROS_INFO_STREAM("Freeing point");
    RemoveFrontierPoint(f_idx_);
    AddOutlierPoint(f_idx_);
  } else if (f_d.norm() < vw_ctrl_.distance[f_idx_]) {
    theta_f = vw_ctrl_.scale[f_idx_] * f_d(2) * see_.d;
    theta_f /= pow(see_.d, 2) + (vw_ctrl_.scale[f_idx_] + 1) * pow(f_d(2), 2);
    theta_f = atan(theta_f);
    R_f = Eigen::Matrix3f::Identity() + sin(theta_f) * u_x_f;
    R_f += (1 - cos(theta_f)) * (u_x_f * u_x_f);

    theta_b = vw_ctrl_.scale[f_idx_] * f_d(1) * see_.d;
    theta_b /= pow(see_.d, 2) + (vw_ctrl_.scale[f_idx_] + 1) * pow(f_d(1), 2);
    theta_b = atan(theta_b);
    R_b = Eigen::Matrix3f::Identity() + sin(theta_b) * u_x_b;
    R_b += (1 - cos(theta_b)) * (u_x_b * u_x_b);

    v_p -= f.getVector3fMap();
    v_p = R_b * (v_p + (vw_ctrl_.scale[f_idx_] + 1) * f_d(1) * f_f);
    v_p = R_f * (v_p + (vw_ctrl_.scale[f_idx_] + 1) * f_d(2) * f_b);
    v_p += f.getVector3fMap();

    vw_ctrl_.scale[f_idx_] *= 2;
    vw_ctrl_.distance[f_idx_] = f_d.norm();

    ROS_INFO_STREAM("Adjusting view");
    AdjustView(f_idx_, v_p);
  } else if (vw_ctrl_.switched[f_idx_]) {
    ROS_INFO_STREAM("Switching view");
    SwitchView(f_idx_);
  } else {
    ROS_INFO_STREAM("Freeing point");
    RemoveFrontierPoint(f_idx_);
    AddOutlierPoint(f_idx_);
  }
}

//! Switch view proposal to the view from which the point was first observed
/*!
  \param pt_idx Index of the point
*/
void SeeCore::SwitchView(int pt_idx) {
  vw_ctrl_.switched[pt_idx] = false;
  vw_ctrl_.distance[pt_idx] = FLT_MAX;
  vw_ctrl_.scale[pt_idx] = 1;

  AdjustView(pt_idx, see_pt_ptr_->points[pt_idx].getViewVector3fMap());
}

//! Adjust the view proposal associated with a given point
/*!
  \param pt_idx Index of the point
  \param new_view New view proposal for pt_idx
*/
void SeeCore::AdjustView(int pt_idx, Eigen::Vector3f new_view) {
  Eigen::Vector3f v_s, v_p, f_p;

  f_p = see_pt_ptr_->points[pt_idx].getVector3fMap();
  v_s = (f_p - new_view).normalized();
  v_p = f_p - (see_.d * v_s);

  see_vw_ptr_->points[pt_idx].getVector3fMap() = v_p;
  see_vw_ptr_->points[pt_idx].getViewVector3fMap() = v_s;
}

//! Update the NBV to account for external view constraints
/*!
  \param new_view The constrained NBV
*/
void SeeCore::UpdateNBV(SeeView new_view) {
  int delta;
  bool no_change = true;

  if ((new_view.getVector3fMap() - nbv_.getVector3fMap()).norm() > see_.ups) {
    no_change = false;
    see_vw_ptr_->points[f_idx_] = new_view;
  }
  if (!IsVisible(f_idx_, f_idx_, delta) || no_change) {
    RemoveFrontierPoint(f_idx_);
    AddOutlierPoint(f_idx_);
  }
  SelectNBV();
}

//! Update the view cloud octree
/*!
  Update the octree used to search the set of view proposals with new views
 */
void SeeCore::UpdateViewOctree() {
  see_vw_oct_ptr_ = boost::make_shared<SeeViewOctree>(see_.psi);
  see_vw_oct_ptr_->setInputCloud(see_vw_ptr_, F_);
  see_vw_oct_ptr_->addPointsFromInputCloud();
}

//! Update the frontier point octree
/*!
  Update the octree used to search the set of frontier points
 */
void SeeCore::UpdateFrontierOctree() {
  see_fpt_oct_ptr_ = boost::make_shared<SeePointOctree>(see_.r);
  see_fpt_oct_ptr_->setInputCloud(see_pt_ptr_, F_);
  see_fpt_oct_ptr_->addPointsFromInputCloud();
}

//! Update the outlier point octree
/*!
  Update the octree used to search the set of outlier points
 */
void SeeCore::UpdateOutlierOctree() {
  see_opt_oct_ptr_ = boost::make_shared<SeePointOctree>(see_.r);
  see_opt_oct_ptr_->setInputCloud(see_pt_ptr_, O_);
  see_opt_oct_ptr_->addPointsFromInputCloud();
}

//! Select a NBV
/*!
  Selected a next best view from the set of view proposals
 */
void SeeCore::SelectNBV() {
  if (F_->size()) {
    UpdateViewOctree();
    SelectNBVFromGraph();
    nbv_ = see_vw_ptr_->points[f_idx_];
  } else {
    done_ = true;
  }
}

//! Select a NBV from the frontier-view visibility graph
/*!
  Select a NBV using the frontier-view visibility graph
 */
void SeeCore::SelectNBVFromGraph() {
  SeeView s_view;
  lemon::ListDigraph::Node nbv, s_node;
  int nbv_arcs, min_arcs, s_arcs, v_idx;
  float v_dst, s_dist, ratio_curr, ratio_best;

  see_vw_oct_ptr_->approxNearestSearch(c_v_, v_idx, v_dst);
  nbv = see_vw_gph_ptr_->nodeFromId(G_idx_[v_idx]);
  nbv_arcs = lemon::countOutArcs(*see_vw_gph_ptr_, nbv);
  min_arcs = nbv_arcs;

  ratio_best = 0;
  lemon::ListDigraph::InArcIt a(*see_vw_gph_ptr_, nbv);
  for (; a != lemon::INVALID; ++a) {
    s_node = see_vw_gph_ptr_->source(a);
    s_arcs = lemon::countOutArcs(*see_vw_gph_ptr_, s_node);
    s_view = see_vw_ptr_->points[(*see_vw_map_ptr_)[s_node]];
    s_dist = (c_v_.getVector3fMap() - s_view.getVector3fMap()).norm() + 0.001;
    if (s_arcs > min_arcs) {
      ratio_curr = s_arcs / s_dist;
      if (ratio_curr > ratio_best) {
        nbv = s_node;
        nbv_arcs = s_arcs;
        ratio_best = ratio_curr;
      }
    }
  }

  if (nbv != lemon::INVALID) {
    f_idx_ = (*see_vw_map_ptr_)[nbv];
  }

  ROS_INFO_STREAM("Selected view can observe " << nbv_arcs + 1 << " frontiers");
}

//! Get the SEE algorithm parameters
/*!
  \return The parameters used by SEE
*/
const AlgParams &SeeCore::GetParams() { return see_; }

//! Get the NBV
/*!
  \return The NBV proposed by SEE
*/
const SeeView &SeeCore::GetNBV() { return nbv_; }

//! Get the latest view
/*!
  \return Get the sensor view associated with the latest observation
*/
const SeeView &SeeCore::GetCurrentView() { return c_v_; }

//! Get the Pointcloud
/*!
  \return Get the combined pointcloud from all previous observations
*/
SeePointCloudConstPtr SeeCore::GetPointCloud() { return see_pt_ptr_; }

//! Get the NBV Proposals
/*!
  \return Get the set of proposed NBVs from SEE
*/
SeeViewCloudConstPtr SeeCore::GetViewCloud() { return see_vw_ptr_; }

//! Get octree structure used for pointcloud search
/*!
  \return Octree used for pointcloud search and classification
*/
SeePointOctreeConstPtr SeeCore::GetOctree() { return see_apt_oct_ptr_; }

//! NBV Frontier Index
/*!
  \return Index of the frontier point associated with the chosen NBV
*/
int SeeCore::GetFrontierIndex() { return f_idx_; }

//! Get Number of Frontiers Observed
/*!
  \return Number of frontier points observed by the latest view
*/
int SeeCore::GetObservedFrontierCount() { return f_cnt_; }

//! Get View Number
/*!
  \return Get number of views processed by SEE
*/
int SeeCore::GetViewNum() { return v_num_; }

//! Get SEE completion status
/*!
  \return SEE completion state
*/
bool SeeCore::IsDone() { return done_; }

} // namespace core
} // namespace see
} // namespace ori
