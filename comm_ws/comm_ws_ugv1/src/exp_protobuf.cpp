#include "comm_suite/exp_protobuf.h"

void Exp_protobuf::init(ros::NodeHandle n_)
{
    //for exploration--------
    // /multi_map_manager/chunk_data plan_env_msgs::ChunkData
    // expsub_cmd0=nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd_1",3,&Exp2Sim::expsub_cmd0_callback, this);
    chunk_data_sub = n_.subscribe<plan_env_msgs::ChunkData>("/multi_map_manager/chunk_data", 10, &Exp_protobuf::chunk_data_sub_callback, this);
    chunk_data_pub=n_.advertise<plan_env_msgs::ChunkData>("/multi_map_manager/chunk_data",10);

    // /multi_map_manager/chunk_stamps plan_env_msgs/ChunkStamps
    chunk_stamps_sub=n_.subscribe<plan_env_msgs::ChunkStamps>("/multi_map_manager/chunk_stamps", 10, &Exp_protobuf::chunk_stamps_sub_callback,this);
    chunk_stamps_pub=n_.advertise<plan_env_msgs::ChunkStamps>("/multi_map_manager/chunk_stamps",10);

    // /planning/swarm_traj bspline/Bspline
    bsp_sub=n_.subscribe<bspline::Bspline>("/planning/swarm_traj",10,&Exp_protobuf::bspline_sub_callback,this);
    bsp_pub=n_.advertise<bspline::Bspline>("/planning/swarm_traj",10);

    // /swarm_expl/drone_state exploration_manager/DroneState
    drone_state_sub=n_.subscribe<exploration_manager::DroneState>("/swarm_expl/drone_state",10,&Exp_protobuf::drone_state_callback,this);
    drone_state_pub=n_.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state",10);

    // /swarm_expl/pair_opt exploration_manager/PairOpt
    pairopt_sub=n_.subscribe<exploration_manager::PairOpt>("/swarm_expl/pair_opt",10,&Exp_protobuf::pairopt_sub_callback,this);
    pairopt_pub=n_.advertise<exploration_manager::PairOpt>("/swarm_expl/pair_opt",10);

    // /swarm_expl/pair_opt_res exploration_manager/PairOptResponse
    pairoptres_sub=n_.subscribe<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res",10,&Exp_protobuf::pairoptres_sub_callback,this);
    pairoptres_pub=n_.advertise<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res",10);
}

tzdx::comm_proto::BasePacket Exp_protobuf::exp2comm()
{
    tzdx::comm_proto::BasePacket packet;
    // tzdx::comm_proto::PointCloudXYZIBody* pcl_body = new tzdx::comm_proto::PointCloudXYZIBody();
    std_msgs::UInt8MultiArray msg_send;

    tzdx::comm_proto::ExplorationMsg* pexpmsg=new tzdx::comm_proto::ExplorationMsg();


    //chunk_data
    if(chunk_data_flag){
    tzdx::comm_proto::ChunkData *pcdmsg=new tzdx::comm_proto::ChunkData();
    pcdmsg->set_chunk_drone_id(chunk_data.chunk_drone_id);
    pcdmsg->set_from_drone_id(chunk_data.from_drone_id);
    pcdmsg->set_idx(chunk_data.idx);
    pcdmsg->set_latest_idx(chunk_data.latest_idx);
    pcdmsg->set_pos_x(chunk_data.pos_x);
    pcdmsg->set_pos_y(chunk_data.pos_y);
    pcdmsg->set_pos_z(chunk_data.pos_z);
    pcdmsg->set_to_drone_id(chunk_data.to_drone_id);
    for (const auto &v_adrs : chunk_data.voxel_adrs)
    {
        pcdmsg->add_voxel_adrs(v_adrs);
    }
    for (const auto &v_occ : chunk_data.voxel_occ_)
    {
        pcdmsg->add_voxel_occ_(v_occ);
    }
    pexpmsg->set_allocated_chunkdata(pcdmsg);

    msg_flag=1;
    }


    //chunk_stamps
    if(chunk_stamps_flag){
    tzdx::comm_proto::ChunkStamps *pcsmsg=new tzdx::comm_proto::ChunkStamps();
    pcsmsg->set_from_drone_id(chunk_stamps.from_drone_id);
    pcsmsg->set_time(chunk_stamps.time);
    for (const auto &idxl : chunk_stamps.idx_lists)
    {   
        tzdx::comm_proto::IdxList* pidxlmsg = pcsmsg->add_idxlists();;
        for(const auto &idx : (idxl.ids))
        {
            pidxlmsg->add_idx(idx);
        }
       
    }
    pexpmsg->set_allocated_chunkstamps(pcsmsg);

    msg_flag=1;
    }


    //bsp
    if(bsp_flag){
    tzdx::comm_proto::Bspline *pbspmsg=new tzdx::comm_proto::Bspline();
    pbspmsg->set_drone_id(bsp.drone_id);
    pbspmsg->set_order(bsp.order);
    pbspmsg->set_traj_id(bsp.traj_id);
    pbspmsg->set_yaw_dt(bsp.yaw_dt);
    for (const auto &kn : bsp.knots)
    {
        pbspmsg->add_knots(kn);
    }
    for (const auto &yp : bsp.yaw_pts)
    {
        pbspmsg->add_knots(yp);
    }
    tzdx::comm_proto::PointXYZ* ppp = pbspmsg->add_pos_pts();
    for (const auto &point : bsp.pos_pts)
    {   
        ppp->set_x(point.x);
        ppp->set_y(point.y);
        ppp->set_z(point.z);
       
    }
    pbspmsg->set_secs(bsp.start_time.sec);
    pbspmsg->set_nsecs(bsp.start_time.nsec);
    pexpmsg->set_allocated_bsp(pbspmsg);

    msg_flag=1;
    }


    //drone_state
    if(drone_state_flag){
    tzdx::comm_proto::DroneState* pdsmsg=new tzdx::comm_proto::DroneState();
    pdsmsg->set_drone_id(drone_state.drone_id);
    tzdx::comm_proto::PointXYZ* pp=new tzdx::comm_proto::PointXYZ();
    pp->set_x(drone_state.goal_posit.x);
    pp->set_y(drone_state.goal_posit.y);
    pp->set_z(drone_state.goal_posit.z);
    pdsmsg->set_allocated_goal_posit(pp);
    for (const auto &gids : drone_state.grid_ids)
    {
        pdsmsg->add_grid_ids(gids);
    }
    for (const auto &dsp : drone_state.pos)
    {
        pdsmsg->add_pos(dsp);
    }
    pdsmsg->set_recent_attempt_time(drone_state.recent_attempt_time);
    pdsmsg->set_role(drone_state.role);
    pdsmsg->set_stamp(drone_state.stamp);
    for(const auto &dsv : drone_state.vel)
    {
        pdsmsg->add_vel(dsv);
    }
    pdsmsg->set_yaw(drone_state.yaw);
    pexpmsg->set_allocated_dronestate(pdsmsg);

    msg_flag=1;
    }

    

    //pairopt
    if(pairopt_flag){
    tzdx::comm_proto::PairOpt* ppomsg=new tzdx::comm_proto::PairOpt;
    tzdx::comm_proto::PointXYZ* pp=new tzdx::comm_proto::PointXYZ();
    pp->set_x(pairopt.ego_posit.x);
    pp->set_y(pairopt.ego_posit.y);
    pp->set_z(pairopt.ego_posit.z);
    ppomsg->set_allocated_ego_posit(pp);
    ppomsg->set_from_drone_id(pairopt.from_drone_id);
    tzdx::comm_proto::PointXYZ* pp2=new tzdx::comm_proto::PointXYZ();
    pp2->set_x(pairopt.other_posit.x);
    pp2->set_y(pairopt.other_posit.y);
    pp2->set_z(pairopt.other_posit.z);
    ppomsg->set_allocated_other_posit(pp2);
    ppomsg->set_stamp(pairopt.stamp);
    ppomsg->set_to_drone_id(pairopt.to_drone_id);
    for(const auto &eids : pairopt.ego_ids)
    {
        ppomsg->add_ego_ids(eids);
    }
    for(const auto &oids : pairopt.other_ids)
    {
        ppomsg->add_other_ids(oids);
    }
    pexpmsg->set_allocated_pairopt(ppomsg);

    msg_flag=1;
    }


    //paireoptres
    if(pairoptres_flag){
    tzdx::comm_proto::PairOptRes* ppormsg=new tzdx::comm_proto::PairOptRes();
    ppormsg->set_from_drone_id(pairoptres.from_drone_id);
    ppormsg->set_stamp(pairoptres.stamp);
    ppormsg->set_status(pairoptres.status);
    ppormsg->set_to_drone_id(pairoptres.to_drone_id);
    pexpmsg->set_allocated_pairoptres(ppormsg);

    msg_flag=1;
    }

    pexpmsg->add_flag(chunk_data_flag);
    pexpmsg->add_flag(chunk_stamps_flag);
    pexpmsg->add_flag(bsp_flag);
    pexpmsg->add_flag(drone_state_flag);
    pexpmsg->add_flag(pairopt_flag);
    pexpmsg->add_flag(pairoptres_flag);

    // bool chunk_data_flag=0, chunk_stamps_flag=0, bsp_flag=0, drone_state_flag, pairopt_flag=0, pairoptres_flag=0;


    //packet
    packet.set_allocated_expmsg(pexpmsg);
    packet.set_type(tzdx::comm_proto::PacketType::PACKET_NAV);
    packet.set_frame("world");
    packet.set_second(ros::Time::now().sec);
    packet.set_nanosecond(ros::Time::now().nsec);
    packet.set_sender("ugv1");
    packet.set_seq (1);

    std::cout<<"flag0218 "<<std::endl;

    return packet;
}

void Exp_protobuf::chunk_data_sub_callback(const plan_env_msgs::ChunkDataConstPtr &msg_p)
{
    chunk_data_flag = 1;

    chunk_data.chunk_drone_id=msg_p->chunk_drone_id;
    chunk_data.from_drone_id=msg_p->from_drone_id;
    chunk_data.idx=msg_p->idx;
    chunk_data.latest_idx=msg_p->latest_idx;
    chunk_data.pos_x=msg_p->pos_x;
    chunk_data.pos_y=msg_p->pos_y;
    chunk_data.pos_z=msg_p->pos_z;
    chunk_data.to_drone_id=msg_p->to_drone_id;
    chunk_data.voxel_adrs=msg_p->voxel_adrs;
    chunk_data.voxel_occ_=msg_p->voxel_occ_;
}

void Exp_protobuf::chunk_stamps_sub_callback(const plan_env_msgs::ChunkStampsConstPtr &msg_p)
{
    chunk_stamps.from_drone_id=msg_p->from_drone_id;
    chunk_stamps.idx_lists=msg_p->idx_lists;
    chunk_stamps.time=msg_p->time;

    chunk_stamps_flag = 1;
}

void Exp_protobuf::bspline_sub_callback(const bspline::BsplineConstPtr &msg_p)
{
    bsp.drone_id=msg_p->drone_id;
    bsp.knots=msg_p->knots;
    bsp.order=msg_p->order;
    bsp.pos_pts=msg_p->pos_pts;
    bsp.start_time=msg_p->start_time;
    bsp.traj_id=msg_p->traj_id;
    bsp.yaw_dt=msg_p->yaw_dt;
    bsp.yaw_pts=msg_p->yaw_pts;

    bsp_flag = 1;
}

void Exp_protobuf::drone_state_callback(const exploration_manager::DroneStateConstPtr &msg_p)
{
    drone_state.drone_id=msg_p->drone_id;
    drone_state.goal_posit=msg_p->goal_posit;
    drone_state.grid_ids=msg_p->grid_ids;
    drone_state.pos=msg_p->pos;
    drone_state.recent_attempt_time=msg_p->recent_attempt_time;
    drone_state.role=msg_p->role;
    drone_state.stamp=msg_p->stamp;
    drone_state.vel=msg_p->vel;
    drone_state.yaw=msg_p->yaw;

    drone_state_flag = 1;
}

void Exp_protobuf::pairopt_sub_callback(const exploration_manager::PairOptConstPtr &msg_p)
{
    pairopt.ego_ids=msg_p->ego_ids;
    pairopt.ego_posit=msg_p->ego_posit;
    pairopt.from_drone_id=msg_p->from_drone_id;
    pairopt.other_ids=msg_p->other_ids;
    pairopt.other_posit=msg_p->other_posit;
    pairopt.stamp=msg_p->stamp;
    pairopt.to_drone_id=msg_p->to_drone_id;

    pairopt_flag = 1;
}

void Exp_protobuf::pairoptres_sub_callback(const exploration_manager::PairOptResponseConstPtr &msg_p)
{
    pairoptres.from_drone_id=msg_p->from_drone_id;
    pairoptres.stamp=msg_p->stamp;
    pairoptres.status=msg_p->status;
    pairoptres.to_drone_id=msg_p->to_drone_id;

    pairoptres_flag = 1;
}

void Exp_protobuf::comm_sub_callback(const std_msgs::UInt8MultiArray::ConstPtr &msg_p){

    
    tzdx::comm_proto::BasePacket packet;

    packet.ParseFromArray(msg_p->data.data(), msg_p->data.size());

    tzdx::comm_proto::ExplorationMsg expmsg=packet.expmsg();
    std::string sender_name = packet.sender();
    if(sender_name==ego_name) return;

    std::vector<int32_t> flags;
    for (const auto &flag : expmsg.flag())
    {
        flags.push_back(flag);
    }

    if(flags[0]){
    tzdx::comm_proto::ChunkData cdmsg=expmsg.chunkdata();
    chunk_data_oth.chunk_drone_id = cdmsg.chunk_drone_id();
    chunk_data_oth.from_drone_id = cdmsg.from_drone_id();
    chunk_data_oth.idx = cdmsg.idx();
    chunk_data_oth.latest_idx = cdmsg.latest_idx();
    chunk_data_oth.pos_x = cdmsg.pos_x();
    chunk_data_oth.pos_y = cdmsg.pos_y();
    chunk_data_oth.pos_z = cdmsg.pos_z();
    chunk_data_oth.to_drone_id = cdmsg.to_drone_id();
    chunk_data_oth.voxel_adrs.clear();
    for (const auto &v_adrs : cdmsg.voxel_adrs())
    {
        chunk_data_oth.voxel_adrs.push_back(v_adrs);
    }
    chunk_data_oth.voxel_occ_.clear();
    for (const auto &v_occ : cdmsg.voxel_occ_())
    {
        chunk_data_oth.voxel_occ_.push_back(v_occ);
    }
    chunk_data_pub.publish(chunk_data_oth);
    }


    if(flags[1]){
    tzdx::comm_proto::ChunkStamps csmsg = expmsg.chunkstamps();
    chunk_stamps_oth.from_drone_id = csmsg.from_drone_id();
    chunk_stamps_oth.idx_lists.clear();
    for (const auto &idxl : csmsg.idxlists())
    {   
        plan_env_msgs::IdxList idxlist;
        for(const auto &idx : idxl.idx())
        {
            idxlist.ids.push_back(idx);
            
        }
        chunk_stamps_oth.idx_lists.push_back(idxlist);
        
    }
    chunk_stamps_oth.time = csmsg.time();
    chunk_stamps_pub.publish(chunk_stamps_oth);
    }


    if(flags[2]){
    tzdx::comm_proto::Bspline bspmsg = expmsg.bsp();
    bsp_oth.drone_id = bspmsg.drone_id();
    bsp_oth.order = bspmsg.order();
    bsp_oth.traj_id = bspmsg.traj_id();
    bsp_oth.yaw_dt = bspmsg.yaw_dt();
    bsp_oth.start_time.sec = bspmsg.secs();
    bsp_oth.start_time.nsec = bspmsg.nsecs();
    bsp_oth.knots.clear();
    for (const auto &kn : bspmsg.knots())
    {
        bsp_oth.knots.push_back(kn);
    }
    bsp_oth.yaw_pts.clear();
    for (const auto &yp : bspmsg.yaw_pts())
    {
        bsp_oth.yaw_pts.push_back(yp);
    }
    bsp_oth.pos_pts.clear();
    for (const auto &pp : bspmsg.pos_pts())
    {
        geometry_msgs::Point p;
        p.x = pp.x();
        p.y = pp.y();
        p.z = pp.z();
        bsp_oth.pos_pts.push_back(p);
    }
    bsp_pub.publish(bsp_oth);
    }

    if(flags[3]){
    tzdx::comm_proto::DroneState dsmsg = expmsg.dronestate();
    drone_state_oth.drone_id = dsmsg.drone_id();
    geometry_msgs::Point p;
    p.x = (dsmsg.goal_posit()).x();
    p.y = (dsmsg.goal_posit()).y();
    p.z = (dsmsg.goal_posit()).z();
    drone_state_oth.goal_posit = p;
    drone_state_oth.grid_ids.clear();
    for (const auto &gids : dsmsg.grid_ids())
    {
        drone_state_oth.grid_ids.push_back(gids);
    }
    drone_state_oth.pos.clear();
    for (const auto &pos : dsmsg.pos())
    {
        drone_state_oth.pos.push_back(pos);
    }
    drone_state_oth.recent_attempt_time = dsmsg.recent_attempt_time();
    drone_state_oth.role = dsmsg.role();
    drone_state_oth.stamp = dsmsg.stamp();
    drone_state_oth.vel.clear();
    for (const auto &vel : dsmsg.vel())
    {
        drone_state_oth.vel.push_back(vel);
    }
    drone_state_oth.yaw = dsmsg.yaw();
    drone_state_pub.publish(drone_state_oth);
    }


    if(flags[4]){
    tzdx::comm_proto::PairOpt pomsg = expmsg.pairopt();
    geometry_msgs::Point p;
    p.x = (pomsg.ego_posit()).x();
    p.y = (pomsg.ego_posit()).y();
    p.z = (pomsg.ego_posit()).z();
    pairopt_oth.ego_posit = p;
    pairopt_oth.from_drone_id = pomsg.from_drone_id();
    p.x = (pomsg.other_posit()).x();
    p.y = (pomsg.other_posit()).y();
    p.z = (pomsg.other_posit()).z();
    pairopt_oth.other_posit = p;
    pairopt_oth.stamp = pomsg.stamp();
    pairopt_oth.to_drone_id = pomsg.to_drone_id();
    pairopt_oth.ego_ids.clear();
    for (const auto &eids : pomsg.ego_ids())
    {
        pairopt_oth.ego_ids.push_back(eids);
    }
    pairopt_oth.other_ids.clear();
    for (const auto &oids : pomsg.other_ids())
    {
        pairopt_oth.other_ids.push_back(oids);
    }
    pairopt_pub.publish(pairopt_oth);
    }


    if(flags[5]){
    tzdx::comm_proto::PairOptRes pormsg = expmsg.pairoptres();
    pairoptres_oth.from_drone_id = pormsg.from_drone_id();
    pairoptres_oth.stamp = pormsg.stamp();
    pairoptres_oth.status = pormsg.status();
    pairoptres_oth.to_drone_id = pormsg.to_drone_id();
    pairoptres_pub.publish(pairoptres_oth);
    }


}