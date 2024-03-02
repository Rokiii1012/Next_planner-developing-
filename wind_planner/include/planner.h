#pragma once

#include <math.h>
#include <memory>
#include <iostream>
#include <Eigen/Eigen>
#include <osqp.h>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <map> 
#include <vector>
#include <algorithm>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "point.h"

using PointPtr = Point*;
class Planner
{
    public:

        Planner() = default;
        ~Planner() = default;

        void CreateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_info);
        bool WhetherOnMap(double index_x,double index_y);
        double GetgCost(const PointPtr& current_point,const PointPtr& neighbor_point);
        double GethCost(const PointPtr& current_point,const PointPtr& goal_point);

        void PathFinder(const Point& init_point,const Point& goal_point,const double& resultion);
        void GetNeighborPoint(PointPtr& current_point,const double& reslution);     
        void TrajectoryOptimization(const int& piece_num,
                                    const Eigen::Vector2d& init_pos,
                                    const Eigen::Vector2d& init_vel,
                                    const Eigen::Vector2d& init_acc,
                                    const Eigen::Vector2d& end_pos,
                                    const Eigen::Vector2d& end_vel,
                                    const Eigen::Vector2d& end_acc,
                                    const Eigen::Matrix2Xd& intermediate_point,
                                    const Eigen::VectorXd& time_allocation,
                                    Eigen::Matrix2Xd& coefficient_matrix);
        void DeleteMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_info);

 
        
        std::vector<PointPtr> GetPath()
        {
            return path_;
        }
    private:

       
        std::vector<PointPtr> path_;
        std::multimap<double,PointPtr> openlist_;
        // 二维数组
        // 后续比较Point*** map_2d_; std::multimap<double,Point*> openlist_;按说指针更快，因为指针只有（64位系统）八个字节，在容器中插入删除更快
        //map_2d_是一个数组，数组存储的数据类型是Point[]
        PointPtr** map_2d_;
        // 地图尺寸
        int x_size_;
        int y_size_;
        // std::shared_ptr<std::shared_ptr<Point>> map_2d_;
        PointPtr init_point_;
        PointPtr goal_point_;
        PointPtr neighbor_point_;



};