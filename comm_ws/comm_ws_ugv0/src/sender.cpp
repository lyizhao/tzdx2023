#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include "comm_suite/pcl_protobuf.h"
#include "comm_suite/exp_protobuf.h"

class CommSender
{
public:
    CommSender(ros::NodeHandle nh)
    {
        n_=nh;

        pcl_recv_ = n_.subscribe("pcl_in", 10, &CommSender::pcl_in_cbk, this);
        chatter_pub_ = n_.advertise<std_msgs::UInt8MultiArray>("comm_data", 10);
    }

    void pcl_in_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        tzdx::comm_proto::BasePacket packet;
        tzdx::comm_proto::PointCloudXYZIBody* pcl_body = new tzdx::comm_proto::PointCloudXYZIBody();
        pcl::PointCloud<pcl::PointXYZI> points;
        std_msgs::UInt8MultiArray msg_send;

        pcl::fromROSMsg(*msg, points);
        pclToProtobuf(points, *pcl_body);
        packet.set_frame(msg->header.frame_id);
        packet.set_seq(msg->header.seq);
        packet.set_second(msg->header.stamp.sec);
        packet.set_nanosecond(msg->header.stamp.sec);
        packet.set_type(tzdx::comm_proto::PacketType::PACKET_MAP);

        packet.set_allocated_pcl(pcl_body);

        msg_send.data.resize(packet.ByteSizeLong());
        packet.SerializeToArray(msg_send.data.data(), packet.ByteSizeLong());

        chatter_pub_.publish(msg_send);
    }

    void exp_comm_pub_callback(Exp_protobuf exp_pro)
    {
        std_msgs::UInt8MultiArray msg_send;
        std::cout<<"flag020"<<std::endl;
        tzdx::comm_proto::BasePacket packet=exp_pro.exp2comm();
        if(exp_pro.msg_flag==0) return;

        std::cout<<"flag021 "<<std::endl;
        exp_pro.exp2comm();
        std::cout<<"flag022 "<<std::endl;
        packet=exp_pro.exp2comm();
        std::cout<<"flag023 "<<std::endl;
        std::cout<<packet.ByteSizeLong()<<std::endl;
        msg_send.data.resize(packet.ByteSizeLong());
        packet.SerializeToArray(msg_send.data.data(), packet.ByteSizeLong());
        std::cout<<"flag024"<<std::endl;
        chatter_pub_.publish(msg_send);
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber pcl_recv_;

    ros::Publisher chatter_pub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    CommSender sender_node(nh);
    Exp_protobuf exp_pro;
    exp_pro.init(nh);
    std::cout<<"flag0"<<std::endl;
    ros::Rate r(50);
    while (ros::ok())
    {
        std::cout<<"flag01"<<std::endl;
        ros::spinOnce();
        std::cout<<"flag02"<<std::endl;
        sender_node.exp_comm_pub_callback(exp_pro);
        std::cout<<"flag03"<<std::endl;
        r.sleep();
    }
    
}