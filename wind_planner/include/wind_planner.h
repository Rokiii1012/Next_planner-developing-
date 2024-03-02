#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include "planner.h"

using namespace std::placeholders;

class WindPlanner : public rclcpp::Node
{   
    public:
        WindPlanner() : Node("wind_planner_node"){}
        void Init();

    private:
        enum class FsmState
        {
            INIT,
            WAIT_TARGET,
            GEN_TRAJ,
            EXEC_TRAJ,
            REPLAN
        };
        FsmState current_fsm_state_;
        bool has_odom_;
        bool has_target_;
        bool has_map_;
        // 暂存地图信息
        nav_msgs::msg::OccupancyGrid::SharedPtr map_info_;
        nav_msgs::msg::Odometry::SharedPtr odom_info_;
        geometry_msgs::msg::PoseStamped::SharedPtr target_pose_info_;
        rclcpp::Time plan_finished_time_;
        std::unique_ptr<Planner> planner_;
    

        void FsmStateChange(const FsmState& new_state,const std::string& trigger);
        // 可以直接访问当前状态，可以不传入参数
        void PrintState();
        bool Plan();

        // bool has_trigger_;

       // 回调函数
       void ExecFsmCallback();
       void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
       void TargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);      
       void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

       // 执行器，时刻执行有限状态机
       rclcpp::TimerBase::SharedPtr actuator_timer_;
       rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
       rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
       rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
       rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;


};