#ifndef LIDARDATA_H
#define LIDARDATA_H

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace pcl {
    struct PointXYZIRT {
    PCL_ADD_POINT4D;                 // quad-word XYZ
    float intensity;                 ///< laser intensity reading
    uint16_t ring;                   ///< laser ring number
    float time;                      ///< laser time reading
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
    } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRT, (float, x, x)  //
                                  (float, y, y)                             //
                                  (float, z, z)                             //
                                  (float, intensity, intensity)             //
                                  (uint16_t, ring, ring)                    //
                                  (float, time, time)                       //
)

typedef pcl::PointXYZIRT RTPoint;
typedef pcl::PointCloud<RTPoint> RTPointCloud;

namespace pcl {
    struct PointXYZIRAWT {
    PCL_ADD_POINT4D;                 // quad-word XYZ
    float intensity;                 ///< laser intensity reading
    float raw_x;
    float raw_y;
    float raw_z;
    float time;                      ///< laser time reading
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
    } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRAWT, (float, x, x)  //
                                  (float, y, y)                             //
                                  (float, z, z)                             //
                                  (float, intensity, intensity)             //
                                  (float, raw_x, raw_x)                    //
                                  (float, raw_y, raw_y)                    //
                                  (float, raw_z, raw_z)                    //
                                  (float, time, time)                       //
)


typedef pcl::PointXYZIRAWT RAWPoint;
typedef pcl::PointCloud<RAWPoint> RAWPointCloud;

#endif