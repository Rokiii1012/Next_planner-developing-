#include "planner.h"

void Planner::CreateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_info)
{
    // x方向是宽
    int x_size = map_info->info.width;
    int y_size = map_info->info.height;

    x_size_ = x_size;
    y_size_ = y_size;
    // map_2d_ 为**
    map_2d_ = new PointPtr*[x_size];
    for (int i = 0; i < x_size; i++)
    {   
        //map_2d_[i]为*
        map_2d_[i] = new PointPtr[y_size];
        
        for (int j = 0; j < y_size; j++)
        {
        // map_2d_为PointPtr;
            map_2d_[i][j] = new Point(i,j,map_info->info.resolution);
            // RCLCPP_INFO(rclcpp::get_logger("map"), "Signed int8 value: %d", map_info->data[x_size * i + j]);  
            // index = (x-ox)/resolution+(y-oy)/resolution*width
            // 空间复杂度问题
           if (map_info->data[x_size * j + i] != 0)
           {
             
              map_2d_[i][j]->state_= Point::STATE::OBSTACLE;
            //   RCLCPP_INFO(this->get_logger(),"");
           }
           
                
        }
         
    }
    
}

void Planner::DeleteMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map_info)
{
    int x_size = map_info->info.width;
    int y_size = map_info->info.height;
    for (int i = 0; i < x_size; i++)
    {  
    // 首先删除每行的PointPtr对象  
        for (int j = 0; j < y_size; j++)  
        {   

            delete map_2d_[i][j]; // 假设PointPtr是指向Point对象的指针  

        }  

    // 然后删除每行的指针数组  

        delete[] map_2d_[i];  

    }  

    // 最后删除整个二维数组的指针数组  
    delete[] map_2d_;  

    // 现在map_2d_指向的内存已经被释放，你可以将其设置为nullptr，以避免悬挂指针  

    map_2d_ = nullptr;
}

bool Planner::WhetherOnMap(double index_x,double index_y)
{
    
    if (index_x >= 0 && index_x < x_size_ && index_y >= 0 && index_y < y_size_)
    {
        // std::cout << "在地图中"<< std::endl;
        return true;
    }
    return false;
    // std::cout << "不在地图中"<< std::endl;

}

double Planner::GetgCost(const PointPtr& current_point,const PointPtr& parent_point)
{   
    double x_distance = std::abs(current_point->coord_x_ - parent_point->coord_x_);
    double y_distance = std::abs(current_point->coord_y_ - parent_point->coord_y_);
    
    return parent_point->g_cost_ + std::sqrt(std::pow(x_distance,2)+std::pow(y_distance,2));


}


double Planner::GethCost(const PointPtr& current_point,const PointPtr& goal_point)
{
    // 欧式几何距离
    double x_distance = std::abs(current_point->coord_x_ - goal_point->coord_x_);
    double y_distance = std::abs(current_point->coord_y_ - goal_point->coord_y_);

    return std::sqrt(std::pow(x_distance,2)+std::pow(y_distance,2));

}

void Planner::GetNeighborPoint(PointPtr& current_point,const double& reslution)
{


    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {


            if (WhetherOnMap(current_point->index_x_ + i,current_point->index_y_ + j))
            {   
                // std::cout << "获取neighbor point"<< std::endl;
                neighbor_point_ = map_2d_[current_point->index_x_ + i][current_point->index_y_ + j];
                // std::cout << static_cast<int>(neighbor_point_->state_)<< std::endl;

                if (neighbor_point_->state_ != Point::STATE::OBSTACLE)
                {   
                    // std::cout << "如果不是障碍物"<< std::endl;
                    // neighbor_list.push_back(neighbor_point_);
                    /*在C++中，当你将一个局部变量插入到一个容器中（如std::vector、std::list、std::map等），
                    这个局部变量的值会被复制或移动到容器中。这意味着容器中存储的是该局部变量的副本，而不是局部变量本身。
                    因此，当局部变量离开其作用域并被销毁时，容器中的元素（即局部变量的副本）并不会受到影响。
                    容器中的元素会继续存在，并且它们的值仍然是插入时局部变量的值。*/
                    if (neighbor_point_->state_ == Point::STATE::NOTVISITED)
                    {   
                        // std::cout << "没有被访问过"<< std::endl;
                        // 勿忘g是累加值
                        neighbor_point_->g_cost_ = neighbor_point_->g_cost_ + GetgCost(neighbor_point_,current_point);
                        neighbor_point_->h_cost_ = GethCost(neighbor_point_,goal_point_);
                        neighbor_point_->f_cost_ = neighbor_point_->g_cost_ + neighbor_point_->h_cost_;
                        neighbor_point_->parent_ = current_point;
                        neighbor_point_->state_ = Point::STATE::OPENLIST;
                        openlist_.insert(std::make_pair( neighbor_point_->f_cost_,
                                                         neighbor_point_));

                    }
                    else if (neighbor_point_->state_ == Point::STATE::OPENLIST)
                    {   
                        // std::cout << "在openlist中"<< std::endl;
                        if (GetgCost(neighbor_point_,current_point) + GethCost(neighbor_point_,goal_point_) < neighbor_point_->f_cost_)
                        {   

                            openlist_.erase(neighbor_point_->f_cost_);
                            neighbor_point_->g_cost_ = neighbor_point_->g_cost_ + GetgCost(neighbor_point_,current_point);
                            neighbor_point_->h_cost_ = GethCost(neighbor_point_,goal_point_);
                            neighbor_point_->f_cost_ = neighbor_point_->g_cost_ + neighbor_point_->h_cost_;                    
                            neighbor_point_->parent_ = current_point;
                            openlist_.insert(std::make_pair( neighbor_point_->f_cost_,
                                                             neighbor_point_));

                        }
                        
                    }                  

                }
  
            }
            
        }
        
    }
    
    
    
}




void Planner::PathFinder(const Point& init_point,const Point& goal_point,const double& resultion)
{   
   
    init_point_ = new Point(init_point.index_x_,init_point.index_y_,resultion);
    goal_point_ = new Point(goal_point.index_x_,goal_point.index_y_,resultion);
    // *(init_point_) = init_point;
    // *(goal_point_) = goal_point;
    // std::cout << "init和goal初始化成功"<< std::endl;
    // openlist初始化，将起点加入openlist
    openlist_.clear();
    openlist_.insert(std::make_pair(init_point_->f_cost_,init_point_));
    init_point_->state_ = Point::STATE::OPENLIST;
    // std::cout << "将起点放入openlist成功"<< std::endl;
    static int num = 0;
    // 检查Astar逻辑
    while (!openlist_.empty())
    {   
        num ++;
        PointPtr tmp_point = openlist_.begin()->second;
        // 忘了这步会进入死循环
        tmp_point->state_ = Point::STATE::CLOSELIST; 
        openlist_.erase(openlist_.begin());
        // 判断是否到达终点
        if (tmp_point->index_x_ == goal_point_->index_x_&&tmp_point->index_y_ == goal_point_->index_y_)
        {   
            goal_point_ = tmp_point;
            // std::cout << "已到达终点，结束循环" << std::endl;
            RCLCPP_INFO(rclcpp::get_logger("Astar"),"已到达终点，结束循环");
            break;
        }
        
        GetNeighborPoint(tmp_point,resultion);
        
    }
    // 开始回溯路径
    path_.clear();
    PointPtr tmp_point = new Point;
    tmp_point = goal_point_;
    path_.push_back(tmp_point);
    // 如果不创建新地图,为什么goal_point的parent是空指针
    // 逻辑判断
    while (tmp_point && (init_point_->index_x_ != tmp_point->index_x_ || init_point_->index_y_ != tmp_point->index_y_))
    {
        tmp_point = tmp_point->parent_;
        path_.push_back(tmp_point);
        
    }

    path_.push_back(tmp_point);
    std::reverse(path_.begin(),path_.end());
    // std::cout << "回溯成功"<< std::endl;
    // delete tmp_point;

}


void Planner::TrajectoryOptimization(const int& piece_num,
                                    const Eigen::Vector2d& init_pos,
                                    const Eigen::Vector2d& init_vel,
                                    const Eigen::Vector2d& init_acc,
                                    const Eigen::Vector2d& end_pos,
                                    const Eigen::Vector2d& end_vel,
                                    const Eigen::Vector2d& end_acc,
                                    const Eigen::Matrix2Xd& intermediate_point,
                                    const Eigen::VectorXd& time_allocation,
                                    Eigen::Matrix2Xd& coefficient_matrix)
{
    // pT Q p，Q是一个n*n的矩阵
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6*piece_num,6*piece_num);

    for (int i = 0; i < piece_num; i++)
    {
        Q.block<3,3>(6*i,6*i) <<  720 * pow(time_allocation[i],5),    360 * pow(time_allocation[i],4),    120 * pow(time_allocation[i],3),
                                360 * pow(time_allocation[i],4),    192 * pow(time_allocation[i],3),    72 * pow(time_allocation[i],2),
                                120 * pow(time_allocation[i],3),    72 * pow(time_allocation[i],2),     36 * time_allocation[i];

    }
    // std::cout << Q << std::endl;



    
    // 有n段轨迹，约束总数 = 起止点约束（6） + 中间点约束 （n-1） + p，v,a,j连续性约束(4 * (n - 1)) = 5 * n + 1;
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6 + 5 * (piece_num - 1),6 * piece_num);
    // 起点和终点的约束
    M.block<3,3>(0,0) << 1, 0,  0,
                         0, 1,  0,
                         0, 0,  2; 
    M.block<3,6>(3 + 5 * (piece_num - 1),6 * (piece_num - 1)) << 
        
        1,  time_allocation[piece_num-1],    pow(time_allocation[piece_num-1],2),    pow(time_allocation[piece_num-1],3),    pow(time_allocation[piece_num-1],4),    pow(time_allocation[piece_num-1],5),
        0,  1                           ,    2 * time_allocation[piece_num-1],       3 * pow(time_allocation[piece_num-1],2),   4 * pow(time_allocation[piece_num-1],3),    5 * pow(time_allocation[piece_num-1],4),
        0,  0,                               2,                                      6 * time_allocation[piece_num-1],       12 * pow(time_allocation[piece_num-1],2),  20 *   pow(time_allocation[piece_num-1],3);  

    // 连续性约束
    for (int i = 0; i < piece_num-1; i++)
    {
        M.block<5,12>(3 + 5 * i,6 * i) << 

         1,  time_allocation[i],    pow(time_allocation[i],2),    pow(time_allocation[i],3),    pow(time_allocation[i],4),    pow(time_allocation[i],5), 0, 0, 0, 0, 0, 0,
         1,  time_allocation[i],    pow(time_allocation[i],2),    pow(time_allocation[i],3),    pow(time_allocation[i],4),    pow(time_allocation[i],5),-1, 0, 0, 0, 0, 0,
         0,  1,                     2 * time_allocation[i],       3 * pow(time_allocation[i],2),    4 *  pow(time_allocation[i],3),     5 * pow(time_allocation[i],4), 0, -1, 0, 0, 0, 0,
         0,  0,                     2,                            6 * time_allocation[i],       12 * pow(time_allocation[i],2),     20 * pow(time_allocation[i],3),    0, 0, -2, 0, 0, 0, 
         0,  0,                     0,                            6,                        24 * time_allocation[i],            60 * pow(time_allocation[i],2),        0, 0,  0,-6, 0, 0;

    }
    //  std::cout << M << std::endl;
  
    // x坐标的b矩阵
    Eigen::VectorXd b_x =  Eigen::VectorXd::Zero(6 + 5 * (piece_num - 1));
    // 起点和终点信息
    b_x.block<3,1>(0,0) << init_pos(0),init_vel(0),init_acc(0);
    b_x.block<3,1>(3 + 5 * (piece_num-1),0) << end_pos(0),end_vel(0),end_acc(0);
    // 中间点信息
    for (int i = 0; i < piece_num-1; i++)
    {
        b_x[3 + 5 * i] = intermediate_point(0,i);
    }
    
     // y坐标的b矩阵
    Eigen::VectorXd b_y =  Eigen::VectorXd::Zero(6 + 5 * (piece_num - 1));
    // 起点和终点信息
    b_y.block<3,1>(0,0) << init_pos(1),init_vel(1),init_acc(1);
    b_y.block<3,1>(3 + 5 * (piece_num-1),0) << end_pos(1),end_vel(1),end_acc(1);
    // 中间点信息
    for (int i = 0; i < piece_num-1; i++)
    {
        b_y[3 + 5 * i] = intermediate_point(1,i);
    } 
    

    Eigen::JacobiSVD<Eigen::MatrixXd> svd1(M);
    std::cout << "约束矩阵的秩"<<svd1.rank() <<std::endl;

    // 创建增广矩阵 x
    Eigen::MatrixXd augmented_matrix_x(M.rows(),M.cols()+1);
    augmented_matrix_x<< M,b_x;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd2(augmented_matrix_x);
    std::cout << "与b_x增广矩阵的秩"<<svd2.rank() <<std::endl;

    augmented_matrix_x<< M,b_y;   
    Eigen::JacobiSVD<Eigen::MatrixXd> svd3(augmented_matrix_x);
    std::cout << "与b_y增广矩阵的秩"<<svd3.rank() <<std::endl;

    //   std::cout << b_x << std::endl;
    //   std::cout << b_y << std::endl;

    //  转稀疏矩阵，只有稀疏矩阵 Q变H，M变L
      Eigen::SparseMatrix<double> H(Q.rows(),Q.cols());
      Eigen::SparseMatrix<double> L(M.rows(),M.cols());

      Eigen::VectorXd lower_bound_x = b_x;
      Eigen::VectorXd upper_bound_x = b_x;


      Eigen::VectorXd lower_bound_y = b_y;
      Eigen::VectorXd upper_bound_y = b_y;

      for (int r = 0; r < Q.rows(); r++)
      {
        for (int c = 0; c < Q.cols(); c++)
        {
            H.insert(r,c) = Q(r,c);
        }
        
      }

       for (int r = 0; r < M.rows(); r++)
      {
        for (int c = 0; c < M.cols(); c++)
        {
            L.insert(r,c) = M(r,c);
        }
        
      }   


          
    // 标准解法
    // 第一步 创建实例
     OsqpEigen::Solver solver;
    // 第二步 设置Setting参数
     solver.settings()->setWarmStart(true);
    // 第三步 设置data参数   
     solver.data()->setNumberOfVariables(M.cols());
     solver.data()->setNumberOfConstraints(M.rows());

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(Q.rows(),1);
  
     solver.data()->setHessianMatrix(H);
     solver.data()->setGradient(gradient);
     solver.data()->setLinearConstraintsMatrix(L);
     solver.data()->setLowerBound(lower_bound_x);
     solver.data()->setUpperBound(upper_bound_x);
    // 第四步 初始化Slover
     solver.initSolver();
     Eigen::VectorXd qp_solution_x;
    // 第五步 求解优化问题
     solver.solveProblem();
    // 第六步 获取最优解
     qp_solution_x = solver.getSolution();

    //  std::cout << qp_solution_x << std::endl;

       // 标准解法
    // 第一步 创建实例
     OsqpEigen::Solver solver2;
    // 第二步 设置Setting参数
     solver2.settings()->setWarmStart(true);
    // 第三步 设置data参数   
     solver2.data()->setNumberOfVariables(M.cols());
     solver2.data()->setNumberOfConstraints(M.rows());

    // Eigen::VectorXd gradient = Eigen::VectorXd::Zero(Q.rows(),1);
  
     solver2.data()->setHessianMatrix(H);
     solver2.data()->setGradient(gradient);
     solver2.data()->setLinearConstraintsMatrix(L);
     solver2.data()->setLowerBound(lower_bound_y);
     solver2.data()->setUpperBound(upper_bound_y);
    // 第四步 初始化Slover
     solver2.initSolver();
     Eigen::VectorXd qp_solution_y;
    // 第五步 求解优化问题
     solver2.solveProblem();
    // 第六步 获取最优解
     qp_solution_y = solver2.getSolution();

    //  std::cout << qp_solution_y << std::endl;

    
}