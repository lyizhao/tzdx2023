#include "protos/base_data.pb.h"
#include <std_msgs/UInt8MultiArray.h>
#include "exploration_fame/plan_env_msgs/ChunkData.h"
#include "exploration_fame/plan_env_msgs/ChunkStamps.h"
#include "exploration_fame/plan_env_msgs/IdxList.h"
#include "exploration_fame/exploration_manager/DroneState.h"
#include "exploration_fame/exploration_manager/PairOpt.h"
#include "exploration_fame/exploration_manager/PairOptResponse.h"
#include "exploration_fame/bspline/Bspline.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

class Exp_protobuf{
    public:
        void init(ros::NodeHandle n_);
        tzdx::comm_proto::BasePacket exp2comm();
        void comm_sub_callback(const std_msgs::UInt8MultiArray::ConstPtr &msg_p);
        bool msg_flag = 0;
        bool chunk_data_flag=0, chunk_stamps_flag=0, bsp_flag=0, drone_state_flag, pairopt_flag=0, pairoptres_flag=0;

    private:
        std::string ego_name = "ugv0";

        plan_env_msgs::ChunkData chunk_data;
        plan_env_msgs::ChunkStamps chunk_stamps;
        bspline::Bspline bsp;
        exploration_manager::DroneState drone_state;
        exploration_manager::PairOpt pairopt;
        exploration_manager::PairOptResponse pairoptres;
        
        
        //the other robot
        plan_env_msgs::ChunkData chunk_data_oth;
        plan_env_msgs::ChunkStamps chunk_stamps_oth;
        bspline::Bspline bsp_oth;
        exploration_manager::DroneState drone_state_oth;
        exploration_manager::PairOpt pairopt_oth;
        exploration_manager::PairOptResponse pairoptres_oth;
        bool chunk_data_oth_flag=0, chunk_stamps_oth_flag=0, bsp_oth_flag=0, drone_state_oth_flag, pairopt_oth_flag=0, pairoptres_oth_flag=0;

        ros::Subscriber chunk_data_sub, chunk_stamps_sub, bsp_sub, drone_state_sub, pairopt_sub, pairoptres_sub;
        ros::Publisher chunk_data_pub, chunk_stamps_pub, bsp_pub, drone_state_pub, pairopt_pub, pairoptres_pub;

        void chunk_data_sub_callback(const plan_env_msgs::ChunkDataConstPtr &msg_p);
        void chunk_stamps_sub_callback(const plan_env_msgs::ChunkStampsConstPtr &msg_p);
        void bspline_sub_callback(const bspline::BsplineConstPtr &msg_p);
        void drone_state_callback(const exploration_manager::DroneStateConstPtr &msg_p);
        void pairopt_sub_callback(const exploration_manager::PairOptConstPtr &msg_p);
        void pairoptres_sub_callback(const exploration_manager::PairOptResponseConstPtr &msg_p);


};

