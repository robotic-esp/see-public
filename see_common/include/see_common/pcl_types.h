// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef SEE_PCL_TYPES_H
#define SEE_PCL_TYPES_H

namespace pcl {

typedef Eigen::Map<Eigen::Array3f> Array3fMap;
typedef const Eigen::Map<const Eigen::Array3f> Array3fMapConst;
typedef Eigen::Map<Eigen::Array4f, Eigen::Aligned> Array4fMap;
typedef const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> Array4fMapConst;
typedef Eigen::Map<Eigen::Vector3f> Vector3fMap;
typedef const Eigen::Map<const Eigen::Vector3f> Vector3fMapConst;
typedef Eigen::Map<Eigen::Vector4f, Eigen::Aligned> Vector4fMap;
typedef const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned>
    Vector4fMapConst;

typedef Eigen::Matrix<uint8_t, 3, 1> Vector3c;
typedef Eigen::Map<Vector3c> Vector3cMap;
typedef const Eigen::Map<const Vector3c> Vector3cMapConst;
typedef Eigen::Matrix<uint8_t, 4, 1> Vector4c;
typedef Eigen::Map<Vector4c, Eigen::Aligned> Vector4cMap;
typedef const Eigen::Map<const Vector4c, Eigen::Aligned> Vector4cMapConst;

#define PCL_ADD_UNION_FRONTIER4D \
                                 \
  union EIGEN_ALIGN16 {          \
    float data_f[4];             \
                                 \
    float frontier[3];           \
                                 \
    struct {                     \
      float frontier_x;          \
                                 \
      float frontier_y;          \
                                 \
      float frontier_z;          \
    };                           \
  };

#define PCL_ADD_EIGEN_MAPS_FRONTIER4D                           \
                                                                \
  inline pcl::Vector3fMap getFrontierVector3fMap() {            \
    return (pcl::Vector3fMap(data_f));                          \
  }                                                             \
                                                                \
  inline pcl::Vector3fMapConst getFrontierVector3fMap() const { \
    return (pcl::Vector3fMapConst(data_f));                     \
  }                                                             \
                                                                \
  inline pcl::Vector4fMap getFrontierVector4fMap() {            \
    return (pcl::Vector4fMap(data_f));                          \
  }                                                             \
                                                                \
  inline pcl::Vector4fMapConst getFrontierVector4fMap() const { \
    return (pcl::Vector4fMapConst(data_f));                     \
  }

#define PCL_ADD_FRONTIER4D \
                           \
  PCL_ADD_UNION_FRONTIER4D \
  PCL_ADD_EIGEN_MAPS_FRONTIER4D

#define PCL_ADD_UNION_BOUNDARY4D \
                                 \
  union EIGEN_ALIGN16 {          \
    float data_b[4];             \
                                 \
    float boundary[3];           \
                                 \
    struct {                     \
      float boundary_x;          \
                                 \
      float boundary_y;          \
                                 \
      float boundary_z;          \
    };                           \
  };

#define PCL_ADD_EIGEN_MAPS_BOUNDARY4D                           \
                                                                \
  inline pcl::Vector3fMap getBoundaryVector3fMap() {            \
    return (pcl::Vector3fMap(data_b));                          \
  }                                                             \
                                                                \
  inline pcl::Vector3fMapConst getBoundaryVector3fMap() const { \
    return (pcl::Vector3fMapConst(data_b));                     \
  }                                                             \
                                                                \
  inline pcl::Vector4fMap getBoundaryVector4fMap() {            \
    return (pcl::Vector4fMap(data_b));                          \
  }                                                             \
                                                                \
  inline pcl::Vector4fMapConst getBoundaryVector4fMap() const { \
    return (pcl::Vector4fMapConst(data_b));                     \
  }

#define PCL_ADD_BOUNDARY4D \
  PCL_ADD_UNION_BOUNDARY4D \
  PCL_ADD_EIGEN_MAPS_BOUNDARY4D

#define PCL_ADD_UNION_VIEW4D \
                             \
  union EIGEN_ALIGN16 {      \
    float data_v[4];         \
                             \
    float view[3];           \
                             \
    struct {                 \
      float view_x;          \
                             \
      float view_y;          \
                             \
      float view_z;          \
    };                       \
  };

#define PCL_ADD_EIGEN_MAPS_VIEW4D                           \
                                                            \
  inline pcl::Vector3fMap getViewVector3fMap() {            \
    return (pcl::Vector3fMap(data_v));                      \
  }                                                         \
                                                            \
  inline pcl::Vector3fMapConst getViewVector3fMap() const { \
    return (pcl::Vector3fMapConst(data_v));                 \
  }                                                         \
                                                            \
  inline pcl::Vector4fMap getViewVector4fMap() {            \
    return (pcl::Vector4fMap(data_v));                      \
  }                                                         \
                                                            \
  inline pcl::Vector4fMapConst getViewVector4fMap() const { \
    return (pcl::Vector4fMapConst(data_v));                 \
  }

#define PCL_ADD_VIEW4D \
                       \
  PCL_ADD_UNION_VIEW4D \
  PCL_ADD_EIGEN_MAPS_VIEW4D

#define PCL_ADD_UNION_POSE4D \
                             \
  union EIGEN_ALIGN16 {      \
    float data_p[4];         \
                             \
    float pose[3];           \
                             \
    struct {                 \
      float pose_x;          \
                             \
      float pose_y;          \
                             \
      float pose_z;          \
    };                       \
  };

#define PCL_ADD_EIGEN_MAPS_POSE4D                           \
                                                            \
  inline pcl::Vector3fMap getPoseVector3fMap() {            \
    return (pcl::Vector3fMap(data_p));                      \
  }                                                         \
                                                            \
  inline pcl::Vector3fMapConst getPoseVector3fMap() const { \
    return (pcl::Vector3fMapConst(data_p));                 \
  }                                                         \
                                                            \
  inline pcl::Vector4fMap getPoseVector4fMap() {            \
    return (pcl::Vector4fMap(data_p));                      \
  }                                                         \
                                                            \
  inline pcl::Vector4fMapConst getPoseVector4fMap() const { \
    return (pcl::Vector4fMapConst(data_p));                 \
  }

#define PCL_ADD_POSE4D \
                       \
  PCL_ADD_UNION_POSE4D \
  PCL_ADD_EIGEN_MAPS_POSE4D

#define PCL_ADD_UNION_META4D \
                             \
  union EIGEN_ALIGN16 {      \
    float data_m[4];         \
                             \
    struct {                 \
      PCL_ADD_UNION_RGB;     \
                             \
      float label;           \
                             \
      float intensity;       \
                             \
      float curvature;       \
    };                       \
  };

#define PCL_ADD_EIGEN_MAPS_META4D                           \
                                                            \
  inline pcl::Vector3fMap getMetaVector3fMap() {            \
    return (pcl::Vector3fMap(data_m));                      \
  }                                                         \
                                                            \
  inline pcl::Vector3fMapConst getMetaVector3fMap() const { \
    return (pcl::Vector3fMapConst(data_m));                 \
  }                                                         \
                                                            \
  inline pcl::Vector4fMap getMetaVector4fMap() {            \
    return (pcl::Vector4fMap(data_m));                      \
  }                                                         \
                                                            \
  inline pcl::Vector4fMapConst getMetaVector4fMap() const { \
    return (pcl::Vector4fMapConst(data_m));                 \
  }

#define PCL_ADD_META4D \
                       \
  PCL_ADD_UNION_META4D \
  PCL_ADD_EIGEN_MAPS_RGB \
  PCL_ADD_EIGEN_MAPS_META4D

}  // namespace pcl

struct SeePoint {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_FRONTIER4D;
  PCL_ADD_BOUNDARY4D;
  PCL_ADD_VIEW4D;
  PCL_ADD_POSE4D;
  PCL_ADD_META4D;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    SeePoint,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(
        float, frontier_x, frontier_x)(float, frontier_y, frontier_y)(
        float, frontier_z, frontier_z)(float, view_x, view_x)(
        float, view_y, view_y)(float, view_z, view_z)(
        float, pose_x, pose_x)(float, pose_y, pose_y)(
        float, pose_z, pose_z)(float, rgb, rgb)(float, label, label)(
        float, intensity, intensity)(float, curvature, curvature))

struct SeeView {
  PCL_ADD_POINT4D;
  PCL_ADD_VIEW4D;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    SeeView,
    (float, x, x)(float, y, y)(float, z, z)(
      float, view_x, view_x)(float, view_y, view_y)(float, view_z, view_z))

typedef pcl::PointCloud<SeePoint> SeePointCloud;
typedef pcl::PointCloud<SeePoint>::Ptr SeePointCloudPtr;
typedef pcl::octree::OctreePointCloudSearch<SeePoint> SeePointOctree;
typedef pcl::octree::OctreePointCloudSearch<SeePoint>::Ptr SeePointOctreePtr;
typedef pcl::PointCloud<SeePoint>::ConstPtr SeePointCloudConstPtr;
typedef pcl::octree::OctreePointCloudSearch<SeePoint>::ConstPtr
    SeePointOctreeConstPtr;

typedef pcl::PointCloud<SeeView> SeeViewCloud;
typedef pcl::PointCloud<SeeView>::Ptr SeeViewCloudPtr;
typedef pcl::octree::OctreePointCloudSearch<SeeView> SeeViewOctree;
typedef pcl::octree::OctreePointCloudSearch<SeeView>::Ptr SeeViewOctreePtr;
typedef pcl::PointCloud<SeeView>::ConstPtr SeeViewCloudConstPtr;
typedef pcl::octree::OctreePointCloudSearch<SeeView>::ConstPtr
    SeeViewOctreeConstPtr;

struct MeshTraits
{
  typedef pcl::PointXYZ         VertexData;
  typedef int                   HalfEdgeData;
  typedef int                   EdgeData;
  typedef pcl::Normal           FaceData;

  typedef boost::false_type IsManifold;
};

/** SEE Point Classifications
 *  Point classifications used by SEE
 */
enum PointClass
{
  null_pt,     /**< No classification */
  core_pt,     /**< Core point */
  frontier_pt, /**< Frontier point */
  outlier_pt   /**< Outlier point */
};

typedef pcl::geometry::PolygonMesh <MeshTraits> EdgeMesh;
typedef pcl::visualization::PCLVisualizer::Ptr SeeViewerPtr;
typedef pcl::visualization::PointCloudColorHandlerCustom<SeePoint>
    SeePointGraphColor;
typedef pcl::visualization::PointCloudColorHandlerGenericField<SeePoint>
    SeePointLabelColor;
typedef pcl::visualization::PointCloudColorHandlerRGBAField<SeePoint>
    SeePointRGBAColor;
typedef pcl::visualization::PointCloudColorHandlerRGBField<SeePoint>
    SeePointRGBColor;

#endif  // SEE_PCL_TYPES_H
