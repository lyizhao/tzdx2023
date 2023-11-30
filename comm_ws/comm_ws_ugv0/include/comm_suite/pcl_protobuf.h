#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "protos/base_data.pb.h"

void pclToProtobuf(const pcl::PointCloud<pcl::PointXYZI> &pclCloud, tzdx::comm_proto::PointCloudXYZIBody &protobufCloud)
{
    for (const auto &point : pclCloud.points)
    {
        tzdx::comm_proto::PointXYZI *p = protobufCloud.add_points();
        p->set_x(point.x);
        p->set_y(point.y);
        p->set_z(point.z);
        p->set_intensity(point.intensity);
    }
}

void protobufToPCL(const tzdx::comm_proto::PointCloudXYZIBody& protobufCloud, pcl::PointCloud<pcl::PointXYZI>& pclCloud) {
  for (const auto& point : protobufCloud.points()) {
    pcl::PointXYZI pclPoint;
    pclPoint.x = point.x();
    pclPoint.y = point.y();
    pclPoint.z = point.z();
    pclPoint.intensity = point.intensity();
    pclCloud.push_back(pclPoint);
  }
}