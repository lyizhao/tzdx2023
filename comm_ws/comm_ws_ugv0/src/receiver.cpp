#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>

#include "comm_suite/pcl_protobuf.h"
#include "comm_suite/exp_protobuf.h"

class CommReveiver
{
public:
    CommReveiver(ros::NodeHandle nh)
    {
        n_=nh;

        sub_ = n_.subscribe("comm_data", 10, &CommReveiver::data_msg_cbk, this);
        pcl_pub_ = n_.advertise<sensor_msgs::PointCloud2>("pcl_out", 10);

        exp_pro.init(n_);
        

    }

    void data_msg_cbk(const std_msgs::UInt8MultiArray::ConstPtr &msg)
    {
        tzdx::comm_proto::BasePacket packet;

        packet.ParseFromArray(msg->data.data(), msg->data.size());
        switch (packet.type())
        {
        case tzdx::comm_proto::PacketType::PACKET_MAP:{
            pcl::PointCloud<pcl::PointXYZI> points;
            sensor_msgs::PointCloud2 pcl_msg;
            protobufToPCL(packet.pcl(), points);

            pcl::toROSMsg(points, pcl_msg);

            // fill msg header
            pcl_msg.header.frame_id = packet.frame();
            pcl_msg.header.seq = packet.seq();
            pcl_msg.header.stamp.sec = packet.second();
            pcl_msg.header.stamp.nsec = packet.nanosecond();
            pcl_pub_.publish(pcl_msg);
            break;
        }
        

        case tzdx::comm_proto::PacketType::PACKET_NAV:{
            exp_pro.comm_sub_callback(msg);
            break;
        }
        }

    }

private:
    ros::NodeHandle n_;

    ros::Subscriber sub_;
    ros::Publisher pcl_pub_;

    Exp_protobuf exp_pro;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    CommReveiver receiver(nh);
    Exp_protobuf exp_pro;
    exp_pro.init(nh);
    ros::spin();
    return 0;
}