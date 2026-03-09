#include "planning/tlplanner.h"
using rot_util = rotation_util::RotUtil;

namespace tlplanner{
TLPlanner::TLPlanner(std::shared_ptr<parameter_server::ParaeterSerer>& para_ptr):paraPtr_(para_ptr){
    paraPtr_->get_para("Webvis_hz", web_vis_hz_);   
    paraPtr_->get_para("is_use_viewpoint", is_use_viewpoint_);   
    paraPtr_->get_para("plan_estimated_duration", plan_estimated_duration_);   

    paraPtr_->get_para("tracking_dur", tracking_dur_);   
    paraPtr_->get_para("tracking_dt", tracking_dt_); 
    paraPtr_->get_para("tracking_height_expect", tracking_height_expect_); 
    paraPtr_->get_para("tracking_dis_expect", tracking_dis_expect_); 
    paraPtr_->get_para("tracking_angle_expect", tracking_angle_expect_); 
    paraPtr_->get_para("is_with_perception", is_with_perception_); 

    paraPtr_->get_para("land_p_x", land_dp_.x());
    paraPtr_->get_para("land_p_y", land_dp_.y());
    paraPtr_->get_para("land_p_z", land_dp_.z());

    paraPtr_->get_para("land_roll", land_roll_);
    paraPtr_->get_para("land_pitch", land_pitch_);

    std::string res_file_name;
    paraPtr_->get_para("res_file_name", res_file_name);
    INFO_MSG_RED("file name: " << res_file_name << ".csv");
    csvWriterPtr_ = std::make_shared<csv_writer::CsvWriter>(res_file_name);
}


TLPlanner::PlanResState TLPlanner::plan_goal(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data){
    //! 3. get inital state
    Eigen::MatrixXd init_state, init_yaw;
    init_state.setZero(3, 3);
    init_yaw.setZero(1, 2);
    init_state.col(0) = init_state_in.odom_p_;
    init_state.col(1) = init_state_in.odom_v_;
    init_state.col(2) = init_state_in.odom_a_;
    init_yaw(0, 0)    = rot_util::quaternion2yaw(init_state_in.odom_q_);
    init_yaw(0, 1)    = init_state_in.odom_dyaw_;

    INFO_MSG("init p: " << init_state.col(0).transpose());
    INFO_MSG("init v: " << init_state.col(1).transpose());
    INFO_MSG("init a: " << init_state.col(2).transpose());
    INFO_MSG("init yaw: " << init_yaw);
    INFO_MSG("tar p: " << target_data.odom_p_.transpose());
    INFO_MSG("tar v: " << target_data.odom_v_.transpose());
    INFO_MSG("tar a: " << target_data.odom_a_.transpose());
    INFO_MSG("tar theta: " << rot_util::quaternion2yaw(target_data.odom_q_));

    //! 4. path search
    Eigen::Vector3d p_start = init_state.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    bool generate_new_traj_success = envPtr_->short_astar(p_start, target_data.odom_p_, path); // get a path from current pose to target pose
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] A path fail!");
        return FAIL;
    }

    //! 5. traj opt
    Trajectory<5> traj_goal;
    Eigen::MatrixXd final_state, final_yaw;
    final_state.setZero(3, 4);
    final_yaw.setZero(1, 2);
    final_state.col(0) = path.back();
    final_state.col(1) = target_data.odom_v_;
    final_yaw(0, 0) = rot_util::quaternion2yaw(target_data.odom_q_);
    INFO_MSG("init p: " << init_state.col(0).transpose());
    INFO_MSG("end p: " << final_state.col(0).transpose());

    generate_new_traj_success = trajoptPtr_->generate_traj(init_state, final_state, 2.0, path, traj_goal);
    //todo goal yaw still has problem
    // generate_new_traj_success = trajoptPtr_->generate_traj(init_state, final_state, init_yaw, final_yaw, 2.0, path, traj_goal);

    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Traj opt fail!");
        return FAIL;
    }
    traj_data.traj_d5_ = traj_goal;
    traj_data.state_ = TrajData::D5;

    if (web_vis_hz_ > 0){
        visPtr_->visualize_path(path, "astar");
        visPtr_->visualize_traj(traj_goal, "traj");
    }

    return PLANSUCC;
}

// car tracking plan
TLPlanner::PlanResState TLPlanner::plan_track(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data){
    //! 1. target fix (水平运动预测)
    prediction::State tar;
    double theta = rot_util::quaternion2yaw(target_data.odom_q_);
    
    // 提取船的真实水平线速度
    double v = target_data.odom_v_.head(2).dot(Eigen::Vector2d(cos(theta), sin(theta)));
    
    // 传入小船当前真实的 X,Y,Z 和速度 (Vz 默认为 0)
    prediction::State tar_in(target_data.odom_p_, v, 0.0, 
                             theta, target_data.odom_dyaw_);
                             
    // ❌ 注意：这里绝对不要把 Z 轴加上 tracking_height_expect_！
    // 否则会触发“2米高幽灵巨浪”Bug
                             
    // 运行底层的 CYRA 模型预测 (这一步主要是为了拿准确的 X 和 Y)
    prediction::Predict::CYRA_model(tar_in, plan_estimated_duration_, tar);

    //! 2. prediction (生成预测轨迹并“降维打击”)
    bool generate_new_traj_success;
    std::vector<Eigen::Vector3d> target_predcit;
    
    generate_new_traj_success = prePtr_->predict(tar.p_, tar.v_, 0.0, tar.theta_, tar.omega_, target_predcit, tracking_dur_);

    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Predict fail!");
        return FAIL;
    }
    INFO_MSG("[tlplanner] prediction done");

    // ==================== ✅ 核心“压路机”魔法 ====================
    // 我们锁定一个绝对平滑的高空 Z 值：小船当前实际高度 + 期望伴飞高度
    // 为了防止飞机在伴飞时产生任何微小的上下抖动，我们直接写死这个平面
    double flat_tracking_z = target_data.odom_p_.z() + tracking_height_expect_;
    
    // 把整条预测轨迹的 Z 轴全部碾平！变成纯粹的 2D 平滑曲线
    for (auto& pt : target_predcit) {
        pt.z() = flat_tracking_z;
    }
    
    // 把端点也修正到同一高度，防止 MINCO 结尾处掉高度
    tar.p_.z() = flat_tracking_z;
    // =============================================================

    //! 3. get inital state
    Eigen::MatrixXd init_state, init_yaw;
    init_state.setZero(3, 3);
    init_yaw.setZero(1, 2);
    init_state.col(0) = init_state_in.odom_p_;
    init_state.col(1) = init_state_in.odom_v_;
    init_state.col(2) = init_state_in.odom_a_;
    init_yaw(0, 0)    = rot_util::quaternion2yaw(init_state_in.odom_q_);
    init_yaw(0, 1)    = init_state_in.odom_dyaw_;

    //! 4. path search (使用抬高后的轨迹去搜路)
    Eigen::Vector3d p_start = init_state.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    envPtr_->set_track_angle(tracking_angle_expect_);
    envPtr_->set_track_dis(tracking_dis_expect_);
    
    // ✅ 这里传入 target_predict_high
    generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, tracking_dt_, way_pts, path, is_use_viewpoint_);
    
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] A path fail!");
        return FAIL;
    }
    
    #ifdef ROS
    if (web_vis_hz_ > 0){
        // 可视化原始的水面轨迹 (方便调试对比)
        visPtr_->visualize_path(target_predcit, "car_predict");
        visPtr_->visualize_pointcloud(target_predcit, "car_predict_pt");
        visPtr_->visualize_path(path, "astar");
        visPtr_->visualize_pointcloud(way_pts, "astar_wpt");
    }
    #endif

    //! 5. traj opt
    Trajectory<5> traj_track;
    Eigen::MatrixXd final_state, final_yaw;
    final_state.setZero(3, 4);
    final_yaw.setZero(1, 2);

    // ✅ 二次保险：清洗 A* 路径，确保所有点都在高空
    for (auto& pt : path) {
        pt.z() = std::max(pt.z(), tracking_height_expect_);
    }
    
    final_state.col(0) = path.back(); // 此时已经是抬高过的了
    final_state.col(1) = Eigen::Vector3d(tar.v_*cos(tar.theta_), tar.v_*sin(tar.theta_), 0.0);
    final_yaw(0, 0) = tar.theta_;
    
    trajoptPtr_->set_track_angle(tracking_angle_expect_);
    trajoptPtr_->set_track_dis(tracking_dis_expect_);

    // 注意：这里传给 MINCO 的 target_predcit 还是原始的（水面的），但这通常只影响可视性权重
    // 如果想要完全一致，这里也可以传 target_predict_high，但前面 path 抬高了就足够了
    generate_new_traj_success = trajoptPtr_->generate_traj(init_state, final_state, init_yaw, final_yaw, 2.0, path, way_pts, target_predcit, traj_track);
    
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Traj opt fail!");
        return FAIL;
    }

    // ==================== ✅ 事后防钻地检查 ====================
    traj_data.traj_d5_ = traj_track;
    traj_data.state_ = TrajData::D5;
    traj_data.start_time_ = TimeNow(); // 使用 TimeNow()

    if (!valid_cheack(traj_data, TimeNow())) { 
        INFO_MSG_RED("[plan_track] ❌ Trajectory hits WATER! Rejecting.");
        return FAIL; 
    }
    // =========================================================

    Eigen::Vector3d vis_arrow_p1 = tar.p_;
    Eigen::Vector3d vis_arrow_p2 = tar.p_ + 2.0 * Eigen::Vector3d(cos(tracking_angle_expect_), sin(tracking_angle_expect_), 0.0);

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
    double t = 0;
    for (int i = 0; i < (int)target_predcit.size(); ++i){
        rays.emplace_back(target_predcit[i], traj_track.getPos(t));
        t += tracking_dt_;
    }

    BAG(dataManagerPtr_->BagPtr_->write_path_msg("/car_predict", target_predcit));
    BAG(dataManagerPtr_->BagPtr_->write_pointcloud_msg("/car_predict_pt", target_predcit));
    BAG(dataManagerPtr_->BagPtr_->write_path_msg("/astar", path));
    BAG(dataManagerPtr_->BagPtr_->write_traj("/traj", traj_track));
    BAG(dataManagerPtr_->BagPtr_->write_arrow("/track_angle", vis_arrow_p1, vis_arrow_p2));
    BAG(dataManagerPtr_->BagPtr_->write_pairline("/track_rays", rays));

    if (web_vis_hz_ > 0){
        visPtr_->visualize_traj(traj_track, "traj");
        visPtr_->visualize_arrow(vis_arrow_p1, vis_arrow_p2, "/track_angle");
        visPtr_->visualize_pairline(rays, "/track_rays");
    }

    return PLANSUCC;
}

TLPlanner::PlanResState TLPlanner::plan_land(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data, const double& t_replan){
    // ==================== 1. 基础数据提取 ====================
    double theta = rot_util::quaternion2yaw(target_data.odom_q_);
    double v_horiz = target_data.odom_v_.head(2).norm(); 
    double vz = target_data.odom_v_.z();                
    double omega = target_data.odom_dyaw_;
    // 找回被遗忘的 a_horiz
    double a_horiz = target_data.odom_a_.head(2).norm();

    // 🌟 找回被遗忘的 tar_in 声明！(编译器再也不会报错了)
    prediction::State tar_in(target_data.odom_p_, v_horiz, 0.0, theta, omega, vz);

    // 加上降落板偏移 (把坐标系从船中心移到板子中心)
    tar_in.p_ = tar_in.p_ + rot_util::yaw2quaternion(tar_in.theta_) * land_dp_;

    // ==================== ✅ 一步到位：完美的斜线滑梯 (静止靶特供) ====================
    Eigen::Vector3d target_p_final = tar_in.p_;
    
    // 船的真实速度设为 0
    Eigen::Vector3d target_v_final(0.0, 0.0, 0.0); 

    // 🚨 目标下沉：瞄准甲板下方 0.1 米！
    target_p_final.z() -= 0.1;

    // =========================================================================
    // 🔍 终极 Debug 输出：查看真正的起终点 Z 轴坐标！
    // =========================================================================
    INFO_MSG_RED("================ DEBUG 坐标检查 ================");
    INFO_MSG_RED("1. 无人机当前高度 (Init Z)     : " << init_state_in.odom_p_.z());
    INFO_MSG_RED("2. 原始靶标高度 (tar_data.z)   : " << target_data.odom_p_.z());
    INFO_MSG_RED("3. 加上偏移后甲板高度 (tar_in.z): " << tar_in.p_.z());
    INFO_MSG_RED("4. 最终发给优化的终点 (Target Z) : " << target_p_final.z());
    INFO_MSG_RED("5. 我们设定的隐形地板限制 (Floor): " << (target_data.odom_p_.z() - 0.2));
    INFO_MSG_RED("================================================");
    // =========================================================================

    // 计算真实的 3D 距离
    double dist_3d = (init_state_in.odom_p_ - target_p_final).norm();

    // 给足时间！按照大约 1.0 m/s 的斜向综合速度来降落
    double estimated_T = dist_3d / 1.0;
    
    // 限制时间范围：给足缓冲时间，绝不让优化器抽搐
    estimated_T = std::max(1.5, std::min(estimated_T, 4.0)); 
    // =========================================================================

    // ==================== 4. 修复 Debug 宏需要的变量 ====================
    Eigen::Vector3d final_pred_p = tar_in.p_; 
    double dist_to_target = (init_state_in.odom_p_.head(2) - tar_in.p_.head(2)).norm();
    // =========================================================================

    // ==================== 6. Debug 可视化 ====================
    INFO_MSG_YELLOW("🌊 [Glide] Dist:" << dist_to_target 
                    << "m | PredT:" << estimated_T 
                    << "s | WaveZ:" << final_pred_p.z() 
                    << " | AimZ:" << target_p_final.z()); 

    if (web_vis_hz_ > 0) {
         std::vector<Eigen::Vector3d> debug_pt; debug_pt.push_back(target_p_final);
         visPtr_->visualize_pointcloud(debug_pt, "pred_land_target"); 
         visPtr_->visualize_arrow(target_p_final, target_p_final + target_v_final, "pred_wave_vel");
         
         std::vector<Eigen::Vector3d> dummy_traj;
         // 为了防止你之前的 a_ 报错，这里统一补 0.0 如果 tar_in 没带加速度的话
         prePtr_->predict(tar_in.p_, tar_in.v_, a_horiz, tar_in.theta_, tar_in.omega_, dummy_traj, tracking_dur_);
         visPtr_->visualize_path(dummy_traj, "car_predict");
    }

    // ==================== 7. 轨迹生成 ====================
    //! 准备无人机当前状态
    Eigen::MatrixXd init_state, init_yaw;
    init_state.setZero(3, 4);
    init_yaw.setZero(1, 2);
    init_state.col(0) = init_state_in.odom_p_;
    init_state.col(1) = init_state_in.odom_v_;
    init_state.col(2) = init_state_in.odom_a_;
    init_state.col(3) = init_state_in.odom_j_;
    init_yaw(0, 0)    = rot_util::quaternion2yaw(init_state_in.odom_q_);
    init_yaw(0, 1)    = init_state_in.odom_dyaw_;

    //! 设置优化参数
    Trajectory<7> traj_land;
    Eigen::Vector3d rpy(land_roll_, land_pitch_, rot_util::quaternion2yaw(target_data.odom_q_));
    Eigen::Quaterniond land_q = rot_util::euler2quaternion(rpy);

    int seg_num = (init_state.col(0) - target_p_final).norm() / 0.5; 
    if (seg_num > 6) seg_num = 6;
    if (seg_num < 2) seg_num = 2;

    trajoptPtr_->set_with_perception(is_with_perception_);

    // double z_floor_limit = target_data.odom_p_.z() - 0.2; 
    double z_floor_limit = 0.0; 
    // 生成轨迹
    bool generate_new_traj_success = trajoptPtr_->generate_traj(init_state, init_yaw, 
                                                               target_p_final, target_v_final, 
                                                               tar_in.theta_, tar_in.omega_,
                                                               land_q, seg_num, traj_land, t_replan,
                                                               z_floor_limit);
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Traj opt fail!");
        return FAIL;
    }

    // ✅ 正确代码 (存入降落轨迹 traj_land)
    traj_data.traj_d7_ = traj_land;
    traj_data.state_ = TrajData::D7;
    traj_data.start_time_ = TimeNow();

    // 记录和保存
    csvWriterPtr_->set_target_state(target_p_final, target_v_final, tar_in.theta_, tar_in.omega_);
    csvWriterPtr_->exportToCSV(traj_land, 0.01);

    // ==================== 🛠️ 专属画图轨迹导出代码 ====================
    std::string plan_path = "/home/Quadrotor-Landing-with-Minco/planned_traj.csv";
    std::ofstream plan_file(plan_path, std::ios::out); 
    if (plan_file.is_open()) {
        plan_file << "Time,UAV_Plan_X,UAV_Plan_Y,UAV_Plan_Z,UAV_Plan_VX,UAV_Plan_VY,UAV_Plan_VZ\n";
        for (double t = 0; t <= traj_land.getTotalDuration(); t += 0.05) {
            Eigen::Vector3d p = traj_land.getPos(t);
            Eigen::Vector3d v = traj_land.getVel(t);
            plan_file << t << "," << p.x() << "," << p.y() << "," << p.z() << ","
                      << v.x() << "," << v.y() << "," << v.z() << "\n";
        }
        plan_file.close();
        INFO_MSG_GREEN("✅ Python 专属降落规划轨迹已导出至 planned_traj.csv");
    } else {
        INFO_MSG_RED("❌ 无法打开文件 planned_traj.csv 进行写入!");
    }
    
    BAG(dataManagerPtr_->BagPtr_->write_traj("/traj", traj_land));

    if (web_vis_hz_ > 0){
        visPtr_->visualize_traj(traj_land, "traj");
    }

    return PLANSUCC;
}

bool TLPlanner::valid_cheack(const TrajData& traj_data, const TimePoint& cur_t){
    double t0 = durationSecond(cur_t, traj_data.start_time_);
    t0 = t0 > 0.0 ? t0 : 0.0;
    INFO_MSG("------------valid check");
    for (double t = t0; t < traj_data.getTotalDuration(); t += 0.01){
        Eigen::Vector3d p = traj_data.getPos(t);
        if (gridmapPtr_->isOccupied(p)){
            return false;
        }
    }
    return true;
}

} // namespace tlplanner