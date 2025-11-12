#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Odometry.h> 
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

//定义结构体，储存参数
struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    // std::string odomTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    //构造函数，从参数服务器中读取参数，这个服务器位置由nh_priv中的节点名称决定，储存在相同节点名称命名的yaml文件中
    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        // nh_priv.getParam("OdomTopic",odomTopic);       //新增里程计话题参数
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
    }
};

//定义全局规划类
//该类用于处理全局规划的逻辑
class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    //订阅地图和目标点的消息
    //mapSub用于订阅地图点云数据，targetSub用于订阅目标点位置
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;
    // ros::Subscriber odomSub;      //新增里程计订阅者

    //定义bool类型变量，表示地图是否初始化完成
    bool mapInitialized;

    // // 定义bool类型变量，表示里程计是否接收到第一个数据
    // bool odomInitialized;
    //定义体素地图对象
    voxel_map::VoxelMap voxelMap;
    //定义可视化对象
    Visualizer visualizer;

    //定义起始点和终点向量
    std::vector<Eigen::Vector3d> startGoal;

    //定义轨迹对象
    Trajectory<5> traj;
    //定义轨迹时间数
    double trajStamp;

public:
    //构造函数，初始化成员变量
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        //这里决定了ros::spinonce()调用的回调函数
        mapSub = nh.subscribe(config.mapTopic, 1, &GlobalPlanner::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
        // odomSub = nh.subscribe(config.odomTopic, 1, &GlobalPlanner::odomCallBack, this,
        //                        ros::TransportHints().tcpNoDelay());
        ROS_INFO("GlobalPlanner initialized. Waiting for map data on %s...", config.mapTopic.c_str());
        // ROS_INFO("Waiting for odometry data on %s...", config.odomTopic.c_str());
    }

    //地图回调函数，用于处理接收到的点云数据，将数据存储到体素地图中
    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // ADDED: 确认地图回调函数被调用
        // ROS_INFO("--- MapCallBack triggered! Message frame_id: %s. ---", msg->header.frame_id.c_str());
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));

            mapInitialized = true;
            ROS_INFO("Map initialized and dilated successfully.");
        }
    }

    // //里程计回调函数，用于处理接收到的里程计数据，并将当前位置作为起始点
    // inline void odomCallBack(const nav_msgs::Odometry::ConstPtr &msg)
    // {
    //     currentPos = Eigen::Vector3d(msg->pose.pose.position.x,
    //                                  msg->pose.pose.position.y,
    //                                  msg->pose.pose.position.z);
    //     if (!odomInitialized)
    //     {
    //         odomInitialized = true;
    //         ROS_INFO("Odometry received. Current position stored.");
    //     }
    // }

    //目标点回调函数，用于处理接收到的目标点位置，然后调用plan函数进行路径规划
    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // ADDED: 确认目标点回调函数被调用，并检查地图初始化状态
        ROS_INFO("--- TargetCallBack triggered! MapInitialized: %s ---", mapInitialized ? "True" : "False");
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const double zGoal = config.mapBound[4] + config.dilateRadius +
                                 fabs(msg->pose.orientation.z) *
                                     (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
            if (voxelMap.query(goal) == 0)
            {
                visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
                startGoal.emplace_back(goal);
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            plan();
        }
        return;
    }

    //规划函数，用于进行全局路径规划
    inline void plan()
    {
        if (startGoal.size() == 2)
        {   
            //定义三维向量route存储路径点
            std::vector<Eigen::Vector3d> route;
            //这个函数将route填充了由informed RRT*算法生成的路径点
            sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
                                                   startGoal[1],
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   route);
            //定义多面体向量hpolys存储安全飞行走廊
            std::vector<Eigen::MatrixX4d> hPolys;
            //定义点云向量pc存储地图表面点
            std::vector<Eigen::Vector3d> pc;

            voxelMap.getSurf(pc);

            sfc_gen::convexCover(route,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 3.0,
                                 hPolys);
            //得到了简化后的凸多面体hpolys
            sfc_gen::shortCut(hPolys);

            if (route.size() > 1)
            {   
                //将凸多面体安全飞行走廊可视化
                visualizer.visualizePolytope(hPolys);
                //定义初始状态和终止状态矩阵
                Eigen::Matrix3d iniState;
                Eigen::Matrix3d finState;
                //将路径的起点和终点作为初始状态和终止状态，并将速度和加速度设为0
                iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                
                gcopter::GCOPTER_PolytopeSFC gcopter;

                // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
                // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
                // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
                //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
                // initialize some constraint parameters
                
                //定义eigen向量存储约束参数
                Eigen::VectorXd magnitudeBounds(5);
                Eigen::VectorXd penaltyWeights(5);
                Eigen::VectorXd physicalParams(6);
                //将配置参数传入约束参数向量中
                magnitudeBounds(0) = config.maxVelMag;
                magnitudeBounds(1) = config.maxBdrMag;
                magnitudeBounds(2) = config.maxTiltAngle;
                magnitudeBounds(3) = config.minThrust;
                magnitudeBounds(4) = config.maxThrust;
                penaltyWeights(0) = (config.chiVec)[0];
                penaltyWeights(1) = (config.chiVec)[1];
                penaltyWeights(2) = (config.chiVec)[2];
                penaltyWeights(3) = (config.chiVec)[3];
                penaltyWeights(4) = (config.chiVec)[4];
                physicalParams(0) = config.vehicleMass;
                physicalParams(1) = config.gravAcc;
                physicalParams(2) = config.horizDrag;
                physicalParams(3) = config.vertDrag;
                physicalParams(4) = config.parasDrag;
                physicalParams(5) = config.speedEps;
                const int quadratureRes = config.integralIntervs;

                traj.clear();

                //设置优化器，若setup返回false则表示优化失败，直接返回
                if (!gcopter.setup(config.weightT,
                                   iniState, finState,
                                   hPolys, INFINITY,
                                   config.smoothingEps,
                                   quadratureRes,
                                   magnitudeBounds,
                                   penaltyWeights,
                                   physicalParams))
                {
                    return;
                }
                
                //如果优化器返回的代价为无穷大，那么也是优化失败，直接返回
                if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
                {
                    return;
                }

                //如果轨迹段数大于0，说明轨迹存在
                if (traj.getPieceNum() > 0)
                {
                    //记录轨迹起始时间
                    trajStamp = ros::Time::now().toSec();
                    //将轨迹和路径可视化
                    visualizer.visualize(traj, route);
                }
            }
        }
    }


    inline void process()
    {
        //用eigen向量存储物理参数，包括质量，重力加速度，水平阻力系数，垂直阻力系数，寄生阻力系数，速度平滑因子，然后传入微分平坦映射对象中
        Eigen::VectorXd physicalParams(6);
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;

        //定义微分平坦的映射对象
        flatness::FlatnessMap flatmap;
        //传入物理参数，进行微分平坦初始化
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
                      physicalParams(3), physicalParams(4), physicalParams(5));
        
        //如果轨迹段数大于0，说明轨迹存在
        if (traj.getPieceNum() > 0)
        {   
            //计算当前时间和轨迹起始时间的差
            const double delta = ros::Time::now().toSec() - trajStamp;
            //如果差值存在且在轨迹总时长范围内
            if (delta > 0.0 && delta < traj.getTotalDuration())
            {
                //定义thr存储推力，quat存储四元数，omg存储角速度
                double thr;
                Eigen::Vector4d quat;
                Eigen::Vector3d omg;
                //通过微分平坦映射计算推力，四元数，角速度
                flatmap.forward(traj.getVel(delta),
                                traj.getAcc(delta),
                                traj.getJer(delta),
                                0.0, 0.0,
                                thr, quat, omg);
                //计算速度，机体角速度，倾斜角
                double speed = traj.getVel(delta).norm();
                double bodyratemag = omg.norm();
                double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
                std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
                speedMsg.data = speed;
                thrMsg.data = thr;
                tiltMsg.data = tiltangle;
                bdrMsg.data = bodyratemag;
                visualizer.speedPub.publish(speedMsg);
                visualizer.thrPub.publish(thrMsg);
                visualizer.tiltPub.publish(tiltMsg);
                visualizer.bdrPub.publish(bdrMsg);

                visualizer.visualizeSphere(traj.getPos(delta),
                                           config.dilateRadius);
            }
        }
    }
};

int main(int argc, char **argv)
{   
    //定义节点名称，初始化节点
    ros::init(argc, argv, "cpstart_node");
    //创建节点句柄
    ros::NodeHandle nh_;
    //创建全局规划器对象，ros::NodeHandle("~")是私有节点句柄
    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);

    //设置循环频率
    ros::Rate lr(1000);
    while (ros::ok())
    {   
        //调用globalplanner类中的process函数
        global_planner.process();
        //用于调用回调函数，这里启动后会调用mapCallBack和targetCallBack函数，也负责管理起始点和终点是否由手动点击选择
        ros::spinOnce();
        //按照循环频率延时
        lr.sleep();
    }

    return 0;
}
