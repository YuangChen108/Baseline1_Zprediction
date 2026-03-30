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
    double flat_tracking_z =  tracking_height_expect_;
    
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

TLPlanner::PlanResState TLPlanner::plan_land(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data, const double& t_replan) {
    // ==================== 1. 基础数据提取 (保留原样) ====================
    double theta = rot_util::quaternion2yaw(target_data.odom_q_);
    double v_horiz = target_data.odom_v_.head(2).norm(); 
    double vz = target_data.odom_v_.z();                
    double omega = target_data.odom_dyaw_;
    double a_horiz = target_data.odom_a_.head(2).norm();

    // 🚀 开关：测试时建议开启 Predict，我们的熔断逻辑会自动处理末端
    bool is_use_predict = true; 

    // --- 变量提前声明 ---
    Eigen::Vector3d target_p_final;
    Eigen::Vector3d target_v_final;
    double target_theta_final;
    double target_omega_final;
    double estimated_T = 0.0; 
    double log_T = 0.0; 
    bool generate_new_traj_success = false;

    // 初始输入状态（加上降落板偏移）
    prediction::State tar_now(target_data.odom_p_, v_horiz, 0.0, theta, omega, vz);
    tar_now.p_ = tar_now.p_ + rot_util::yaw2quaternion(tar_now.theta_) * land_dp_;
    
    double dist_xy = (init_state_in.odom_p_.head(2) - tar_now.p_.head(2)).norm();
    double dist_3d = (init_state_in.odom_p_ - tar_now.p_).norm();

    // ==================== 🚀 2. 融合源代码：自适应分段与 TTG 分配 ====================
    // 💡 学习源代码：根据空间距离动态确定分段数，确保轨迹自由度与距离匹配
    int seg_num = dist_3d / 0.6; // 参考源代码 0.5m/段，这里微调至 0.6m
    if (a_horiz > 1.5) seg_num += 1; // 目标加速度大时增加自由度以防无法收敛
    seg_num = std::max(2, std::min(seg_num, 6)); // 限制在合理区间

    if (is_use_predict && dist_3d > 1.0) { // 🚨 略微扩大熔断范围至 1.0m 以配合自适应分段
        double current_uav_speed = init_state_in.odom_v_.norm();
        // 💡 学习源代码：更激进的逼近速度期望
        double dynamic_approach_speed = std::max(2.5, current_uav_speed); 
        double raw_T = dist_3d / dynamic_approach_speed;

        static double last_estimated_T = 1.0;
        // 💡 学习源代码：利用平滑的 TTG 作为 MINCO 优化的种子时间
        estimated_T = 0.5 * last_estimated_T + 0.5 * raw_T; 
        
        estimated_T = std::max(0.6, std::min(estimated_T, 2.5)); 
        last_estimated_T = estimated_T;

        prediction::State tar_pred;
        prediction::Predict::CYRA_model(tar_now, estimated_T, tar_pred);

        target_p_final = tar_pred.p_;
        target_v_final = Eigen::Vector3d(tar_pred.v_ * cos(tar_pred.theta_), 
                                         tar_pred.v_ * sin(tar_pred.theta_), 
                                         tar_pred.vz_);
        target_theta_final = tar_pred.theta_;
        target_omega_final = tar_pred.omega_;
        log_T = estimated_T; 
    } else {
        // 🚨 【不带预测 / 熔断模式】：锁定实时观测
        target_p_final.x() = tar_now.p_.x();
        target_p_final.y() = tar_now.p_.y();
        target_p_final.z() = tar_now.p_.z(); 

        target_v_final.x() = tar_now.v_ * cos(tar_now.theta_);
        target_v_final.y() = tar_now.v_ * sin(tar_now.theta_);
        target_v_final.z() = 0.0; 

        target_theta_final = tar_now.theta_;
        target_omega_final = tar_now.omega_;
        
        // 💡 学习源代码：末端不再给死 0.15s，而是给一个极短的种子时间引导 MINCO 快速下砸
        estimated_T = (dist_3d < 0.6) ? 0.2 : 0.4; 
        log_T = 0.0;
        
        // 🚨 强制末端分段：在熔断区内将 seg_num 降到最低（2-3段），消除多余回环
        seg_num = (dist_3d < 0.8) ? 2 : 3;
    }

    // ==================== ✅ 3. 暴力下压补偿策略 (保持原样) ====================
    double z_offset = -0.25;
    if (dist_3d < 0.8) {
        // 目标垂直速度 = 船速 - 1.2m/s，确保产生极强的撞击冲量
        target_v_final.z() = std::min(-1.5, target_v_final.z() - 1.0); 
        z_offset = -0.55; // 压得更深，瞄准板下 55cm
    }

    target_v_final.z() = std::min(0.0, target_v_final.z());
    target_p_final.z() += z_offset;

    // ==================== 5. 轨迹生成 (MINCO) (保留原样) ====================
    Eigen::MatrixXd init_state, init_yaw;
    init_state.setZero(3, 4); init_yaw.setZero(1, 2);
    init_state.col(0) = init_state_in.odom_p_;
    init_state.col(1) = init_state_in.odom_v_;
    init_state.col(2) = init_state_in.odom_a_;
    init_state.col(3) = init_state_in.odom_j_;
    init_yaw(0, 0)    = rot_util::quaternion2yaw(init_state_in.odom_q_);
    init_yaw(0, 1)    = init_state_in.odom_dyaw_;

    Trajectory<7> traj_land;
    Eigen::Vector3d rpy(land_roll_, land_pitch_, target_theta_final); 
    Eigen::Quaterniond land_q = rot_util::euler2quaternion(rpy);

    trajoptPtr_->set_with_perception(is_with_perception_);
    double z_floor_limit = std::min(init_state.col(0).z(), target_p_final.z()) - 1.2; 

    // 调用 MINCO，使用自适应计算出的 seg_num
    generate_new_traj_success = trajoptPtr_->generate_traj(
        init_state, init_yaw, target_p_final, target_v_final, 
        target_theta_final, target_omega_final, land_q, seg_num, 
        traj_land, t_replan, z_floor_limit);

    if (!generate_new_traj_success) {
        return FAIL;
    }

    // 更新状态
    traj_data.traj_d7_ = traj_land;
    traj_data.state_ = TrajData::D7;
    traj_data.start_time_ = TimeNow();

    // ==================== 📊 全维度降落深度分析 (保留原样) ====================
    auto now_chrono = TimeNow();
    auto dur_since_epoch = now_chrono.time_since_epoch();
    double time_sec = std::chrono::duration_cast<std::chrono::microseconds>(dur_since_epoch).count() / 1e6;

    static std::ofstream detail_file;
    if (!detail_file.is_open()) {
        detail_file.open("/home/Quadrotor-Landing-with-Minco/experiments_data/landing_deep_analysis.csv", std::ios::out);
        detail_file << "Time,Is_Predict,Dist_3D,Dist_XY,"
                    << "UAV_X,UAV_Y,UAV_Z,UAV_VX,UAV_VY,UAV_VZ," 
                    << "Boat_X,Boat_Y,Boat_Z,Boat_VX,Boat_VY,Boat_VZ,"
                    << "Target_PX,Target_PY,Target_PZ,Target_VZ,"
                    << "Estimated_T,Seg_Num,Z_Offset,"
                    << "Traj_Duration,Total_Cost" 
                    << std::endl;
    }
    Eigen::Vector3d uav_v = init_state_in.odom_v_;
    Eigen::Vector3d uav_p = init_state_in.odom_p_;
    double current_traj_cost = 0.0; 

    detail_file << std::fixed << std::setprecision(6)
                << time_sec << ","
                << (is_use_predict ? 1 : 0) << ","
                << dist_3d << "," << dist_xy << ","
                << uav_p.x() << "," << uav_p.y() << "," << uav_p.z() << ","
                << uav_v.x() << "," << uav_v.y() << "," << uav_v.z() << ","
                << tar_now.p_.x() << "," << tar_now.p_.y() << "," << tar_now.p_.z() << ","
                << target_data.odom_v_.x() << "," << target_data.odom_v_.y() << "," << target_data.odom_v_.z() << ","
                << target_p_final.x() << "," << target_p_final.y() << "," << target_p_final.z() << ","
                << target_v_final.z() << "," 
                << estimated_T << "," << seg_num << "," << z_offset << ","
                << traj_land.getTotalDuration() << "," << current_traj_cost << std::endl;
    detail_file.flush();

    // ==================== 7. 数据导出与可视化 (保留原样) ====================
    csvWriterPtr_->set_target_state(target_p_final, target_v_final, target_theta_final, target_omega_final);
    csvWriterPtr_->exportToCSV(traj_land, 0.01);

    std::string plan_path = "/home/Quadrotor-Landing-with-Minco/experiments_data/planned_traj.csv";
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
    }

    BAG(dataManagerPtr_->BagPtr_->write_traj("/traj", traj_land));
    if (web_vis_hz_ > 0) visPtr_->visualize_traj(traj_land, "traj");

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