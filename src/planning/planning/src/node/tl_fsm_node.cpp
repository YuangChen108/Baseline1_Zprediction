#include "node/tl_fsm_node.h"
using rot_util = rotation_util::RotUtil;

TLFSM::TLFSM(){
    #ifdef ROS
    para_ptr_ = std::make_shared<parameter_server::ParaeterSerer>(
        "/home/Quadrotor-Landing-with-Minco/src/planning/planning/config/config.yaml");
    #endif
    #ifdef SS_DBUS
    para_ptr_ = std::make_shared<parameter_server::ParaeterSerer>(
        "/blackbox/config/config.yaml");
    #endif
    dataManagerPtr_ = std::make_shared<ShareDataManager>(para_ptr_);
    
    planner_ = std::make_shared<Planner>(dataManagerPtr_, para_ptr_);
    data_callbacks_ = std::make_shared<DataCallBacks>(dataManagerPtr_, para_ptr_);
    traj_server_ = std::make_shared<TrajServer>(dataManagerPtr_, para_ptr_);

    para_ptr_->get_para("fsm_mode", fsm_mode_);
    para_ptr_->get_para("land_p_x", land_dp_.x());
    para_ptr_->get_para("land_p_y", land_dp_.y());
    para_ptr_->get_para("land_p_z", land_dp_.z());
}

void TLFSM::run(){
    while (!dataManagerPtr_->s_exit_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        switch (state_)
        {
        case IDLE:
            if (dataManagerPtr_->auto_mode_){
                INFO_MSG_YELLOW("[FSM] IDLE -> HOVER");
                planner_->set_mode(Planner::HOVER);
                state_ = HOVER;
            }
            break;
        case HOVER:
        {
            Odom odom_data;
            if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
                break;
            }
            Odom target_data;
            if (!dataManagerPtr_->get_car_odom(target_data)){
                break;
            }
            if (dataManagerPtr_->auto_mode_ && fsm_mode_ == 0 && dataManagerPtr_->plan_trigger_received_){
                bool is_switch_to_track = judge_to_track(odom_data.odom_p_, target_data.odom_p_);
                if (is_switch_to_track){
                    INFO_MSG_YELLOW("[FSM] HOVER -> TRACK");
                    planner_->set_mode(Planner::TRACK);
                    state_ = TRACK;
                }else{
                    INFO_MSG_YELLOW("[FSM] HOVER -> GOAL");
                    planner_->set_mode(Planner::GOAL);
                    state_ = GOAL;
                }
            }
            if (dataManagerPtr_->auto_mode_ && fsm_mode_ == 1 && dataManagerPtr_->plan_trigger_received_){
                INFO_MSG_YELLOW("[FSM] HOVER -> GOAL");
                planner_->set_mode(Planner::GOAL);
                state_ = GOAL;
            }
            break;
        }
        case TRACK:
            if (!dataManagerPtr_->auto_mode_){
                INFO_MSG_YELLOW("[FSM] TRACK -> HOVER");
                planner_->set_mode(Planner::HOVER);
                state_ = HOVER;
            }
            if (dataManagerPtr_->land_trigger_received_){
                INFO_MSG_YELLOW("[FSM] TRACK -> LAND");
                planner_->set_mode(Planner::LAND);
                state_ = LAND;
            }
            break;
        case LAND:
        {
            // Odom odom_data;
            // if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
            //     continue;
            // }
            // Odom target_data;
            // if (!dataManagerPtr_->get_car_odom(target_data)){
            //     continue;
            // }
            // Eigen::Vector3d land_pose;
            // double theta = rot_util::quaternion2yaw(target_data.odom_q_);
            // land_pose = target_data.odom_p_ + rot_util::yaw2quaternion(theta) * land_dp_;
            // std::vector<Eigen::Vector3d> land_pose_pc;
            // land_pose_pc.push_back(land_pose);
            // VIS(vis_ptr_->visualize_path(land_pose_pc, "/land_pose"));

            // auto format_double_value = [](double val, int fixed){
            //     auto str = std::to_string(val);
            //     return str.substr(0, str.find(".") + fixed + 1);
            // };
            // std::string oss = "Pose Error: " + format_double_value((odom_data.odom_p_ - land_pose).x(), 3) + ", " + format_double_value((odom_data.odom_p_ - land_pose).y(), 3);
            // APPLOG(AppDebugLog::instance(.log(oss.c_str())));
            // INFO_MSG(oss.c_str());

            if (!dataManagerPtr_->auto_mode_){
                INFO_MSG_YELLOW("[FSM] LAND -> HOVER");
                planner_->set_mode(Planner::HOVER);
                state_ = HOVER;
            }
            bool stopped = judge_to_stop();
            if (stopped){
                // INFO_MSG_YELLOW("[FSM] LAND -> STOP");
                // planner_->set_mode(Planner::IDLE);
                // state_ = STOP;
                INFO_MSG_YELLOW("[FSM] Landed! Force Switching to TRACK (Glued mode).");
                planner_->set_mode(Planner::IDLE); // 让规划器休息
                state_ = STOP;                     // 切入停止状态
            }
            break;
        }
        case STOP:
        {
            BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/state", STOP));
            Odom odom_data;
            if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
                break;
            }
            Odom target_data;
            if (!dataManagerPtr_->get_car_odom(target_data)){
                break;
            }
            Eigen::Vector3d land_pose;
            double theta = rot_util::quaternion2yaw(target_data.odom_q_);
            land_pose = target_data.odom_p_ + rot_util::yaw2quaternion(theta) * land_dp_;
            auto format_double_value = [](double val, int fixed){
                auto str = std::to_string(val);
                return str.substr(0, str.find(".") + fixed + 1);
            };
            std::string oss = "Pose Error: " + format_double_value((odom_data.odom_p_ - land_pose).x(), 3) + ", " + format_double_value((odom_data.odom_p_ - land_pose).y(), 3);
            APPLOG(AppDebugLog::instance().log(oss.c_str()));
            INFO_MSG(oss.c_str());
            break;
        }
            
        case GOAL:
        {
            Odom odom_data;
            if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
                break;
            }
            Odom target_data;
            if (!dataManagerPtr_->get_car_odom(target_data)){
                break;
            }
            bool is_switch_to_track = judge_to_track(odom_data.odom_p_, target_data.odom_p_);
            if (is_switch_to_track){
                INFO_MSG_YELLOW("[FSM] GOAL -> TRACK");
                planner_->set_mode(Planner::TRACK);
                state_ = TRACK;
            }
            break;
        }
            
        default:
            break;
        }
    }
    INFO_MSG_RED("[FSM] Thread Exit.");
}

bool TLFSM::judge_to_track(const Eigen::Vector3d& odom_p, const Eigen::Vector3d& target_p){
    Eigen::Vector3d tar_p = target_p;
    bool switch_to_track = false;
    GData<double> track_dis_data, track_h_data;
    dataManagerPtr_->get_data(dataManagerPtr_->tracking_dis_info_, track_dis_data);
    dataManagerPtr_->get_data(dataManagerPtr_->tracking_height_info_, track_h_data);
    tar_p.z() += track_h_data.data_;
    double dis2car = (odom_p - tar_p).norm();
    INFO_MSG("[FSM] dis2car: " << dis2car <<", track_dis: " << track_dis_data.data_);
    static std::deque<std::pair<double, TimePoint>> dis_vec;
    static double vec_len = 30;
    dis_vec.emplace_back(dis2car, TimeNow());
    bool stable_state = false;
    if (dis_vec.size() > vec_len){
        dis_vec.pop_front();
        double dv = abs(dis_vec.back().first - dis_vec.front().first) / (durationSecond(dis_vec.back().second, dis_vec.front().second));
        INFO_MSG("[plan node] dv: " << dv);
        if (dv < 0.5){
            stable_state = true;
        }
    }
    if ( dis2car <= track_dis_data.data_ + 9.0 && stable_state){
        switch_to_track = true;
    }
    return switch_to_track;
}


bool TLFSM::judge_to_stop(){
    TrajData traj_data;
    if (!dataManagerPtr_->get_traj(dataManagerPtr_->traj_info_, traj_data)){
        return false;
    }
    if (traj_data.getTrajState() != TrajData::D7){
        return false;
    }

    Odom odom_data, target_data;
    if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data) || 
        !dataManagerPtr_->get_car_odom(target_data)) {
        return false;
    }

    Eigen::Vector3d drone_p = odom_data.odom_p_;
    Eigen::Vector3d boat_p  = target_data.odom_p_;
    Eigen::Vector3d drone_v = odom_data.odom_v_;
    Eigen::Vector3d boat_v  = target_data.odom_v_;

    // ==================== 🛠️ 实际轨迹记录 (保持原样) ====================
    std::string real_path = "/home/Quadrotor-Landing-with-Minco/experiments_data/actual_traj.csv";
    std::ofstream real_file(real_path, std::ios::out | std::ios::app);
    if (real_file.is_open()) {
        real_file.seekp(0, std::ios::end);
        if (real_file.tellp() == 0) { 
            real_file << "SysTime,Real_UAV_X,Real_UAV_Y,Real_UAV_Z,Real_Boat_X,Real_Boat_Y,Real_Boat_Z,Real_UAV_VZ,Real_Boat_VZ\n";
        }
        real_file << ros::Time::now().toSec() << ","
                  << drone_p.x() << "," << drone_p.y() << "," << drone_p.z() << ","
                  << boat_p.x() << "," << boat_p.y() << "," << boat_p.z() << ","
                  << drone_v.z() << "," << boat_v.z() << "\n";
        real_file.close();
    }

    // 1. 直接用无人机和船体原点算距离（最安全、最真实）
    double dist_z  = abs(drone_p.z() - boat_p.z());
    double dist_xy = (drone_p.head(2) - boat_p.head(2)).norm();
    
    double rel_speed = (drone_v - boat_v).norm();
    double rel_vz = abs(drone_v.z() - boat_v.z());
    double wave_z = boat_p.z(); // 记录海浪状态用船体原点即可

    auto record_landing_state = [&](bool is_success, const std::string& reason) {
        std::string state_path = "/home/Quadrotor-Landing-with-Minco/experiments_data/landing_states.csv";
        std::ofstream state_file(state_path, std::ios::out | std::ios::app);
        if (state_file.is_open()) {
            state_file.seekp(0, std::ios::end);
            if (state_file.tellp() == 0) {
                state_file << "Time,UAV_X,UAV_Y,UAV_Z,Boat_X,Boat_Y,Boat_Z,Dist_XY,Dist_Z,Rel_Speed,Rel_VZ,Wave_Z,Success,Reason\n";
            }
            state_file << ros::Time::now().toSec() << ","
                       << drone_p.x() << "," << drone_p.y() << "," << drone_p.z() << ","
                       << boat_p.x() << "," << boat_p.y() << "," << boat_p.z() << ","
                       << dist_xy << "," << dist_z << "," << rel_speed << ","
                       << rel_vz << "," << wave_z << ","
                       << (is_success ? 1 : 0) << "," << reason << "\n";
            state_file.close();
        }
    };

// ==================== 判定逻辑 ====================
    // 首先获取真实的 Z 轴高度差 (带符号：正数在上方，负数在海里)
    double uav_z = odom_data.odom_p_.z();
    double boat_z = target_data.odom_p_.z();
    double z_diff = uav_z - boat_z; 

    // 1. 【高优先级】触板/穿透判定 (Terminal Strike)
    // - 水平容忍放宽到 0.45m (海上 45cm 完全在降落板捕获范围内)
    // - 高度差：允许在板面上方 15cm 到底部 60cm 之间 (完美捕捉“穿甲”瞬间)
    // - 删掉 rel_vz 限制，因为我们就是要暴力下砸！把总速度放宽到 3.0
    if (dist_xy < 0.45 && z_diff < 0.15 && z_diff > -0.60 && rel_speed < 3.0) {
        ROS_INFO_STREAM("\033[1;32m[FSM] 捕获物理接触(硬着陆)！Success=1. XY_err: " << dist_xy << " Z_diff: " << z_diff << "\033[0m");
        if (planner_) planner_->saveLandingData(odom_data, target_data, true); 
        record_landing_state(true, "Touchdown_Success");
        return true;
    }

    // 2. 【低优先级】超时判定
    TimePoint sample_time = TimeNow();
    double t_elapsed = durationSecond(sample_time, traj_data.start_time_);
    
    // 如果规划的时间已经跑完了，但还没触发上面的物理接触
    if (t_elapsed > traj_data.getTotalDuration()) {
        INFO_MSG_YELLOW("[FSM] 时间耗尽结算。XY_err: " << dist_xy << "m, Z_diff: " << z_diff << "m");
        
        // 超时后的最后宽限检查 (打补丁)
        // 只要水平误差在 0.6m 以内，且 Z 轴确实已经压下去了(哪怕穿模穿到了 -1.5m)，都算接受！
        if (dist_xy < 0.6 && z_diff < 0.3 && z_diff > -1.5) {
             record_landing_state(true, "Timeout_Acceptable");
             return true;
        } else {
             record_landing_state(false, "Timeout_Fail");
             return true; 
        }
    }

    return false;
}

bool TLFSM::set_thread_para(std::shared_ptr<std::thread>& thread, const int priority, const char* name){
    pthread_setname_np(thread->native_handle(), name);
    struct sched_param thread_param = {};
    thread_param.sched_priority = priority;
    bool succ = false;
    if (pthread_setschedparam(thread->native_handle(), SCHED_RR, &thread_param) == 0){
        INFO_MSG("Set thread priority "<<priority);
        succ = true;
    }else{
        INFO_MSG("Fail to set thread priority "<<priority);
        succ = false;
    }
    return succ;
}

  #ifdef ROS
  void TLFSM::init_ros(ros::NodeHandle& nh){    
    data_callbacks_->init_ros(nh);
    traj_server_->init_ros(nh);

    vis_ptr_ = std::make_shared<vis_interface::VisInterface>(nh);
    planner_->set_vis_ptr(vis_ptr_);
  }
  #endif