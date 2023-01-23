// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef SEE_PCL_COMMON_H
#define SEE_PCL_COMMON_H

#define PCL_NO_PRECOMPILE

#include <Eigen/Core>

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/octree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>

#include <pcl/common/impl/centroid.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/impl/octree_search.hpp>
#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/surface/impl/gp3.hpp>

#include <pcl/geometry/mesh_conversion.h>
#include <pcl/geometry/mesh_indices.h>
#include <pcl/geometry/mesh_traits.h>

#include <pcl/geometry/get_boundary.h>
#include <pcl/geometry/polygon_mesh.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>

#include <pcl/filters/impl/crop_box.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/incremental_registration.h>

#include <pcl/registration/impl/correspondence_estimation.hpp>
#include <pcl/registration/impl/incremental_registration.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_color_handlers.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

#include <see_common/pcl_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> XYZPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr XYZPointCloudPtr;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> XYZPointOctree;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr
    XYZPointOctreePtr;
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr XYZPointCloudConstPtr;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::ConstPtr
    XYZPointOctreeConstPtr;

#endif // SEE_PCL_COMMON_H
