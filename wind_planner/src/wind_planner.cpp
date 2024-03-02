#include "wind_planner.h"

void WindPlanner::Init()
{
  has_odom_ = true;
  has_target_ = false;
  current_fsm_state_ = FsmState::INIT;

  planner_ = std::make_unique<Planner>();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map",10,std::bind(&WindPlanner::MapCallback,this,_1));
  // 执行有限状态机，进行规划状态转移
  actuator_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&WindPlanner::ExecFsmCallback,this));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",1,std::bind(&WindPlanner::OdomCallback,this,_1));
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",1,
                                                                                 std::bind(&WindPlanner::TargetPoseCallback,this,_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("astar_path",10);
}

bool WindPlanner::Plan()
{
  planner_->CreateMap(map_info_);
  // 别忘了删除地图，以免内存消耗
  RCLCPP_INFO(this->get_logger(),"地图创建成功");
  // Point init_pose{odom_info_->pose.pose.position.x,odom_info_->pose.pose.position.y,map_info_->info.resolution};
  Point init_pose{5,5,map_info_->info.resolution};
  // 缺乏障碍物判断，待补充
  if (!planner_->WhetherOnMap(target_pose_info_->pose.position.x,target_pose_info_->pose.position.y))
  { 
    has_target_ = false;
    RCLCPP_INFO(this->get_logger(),"目标点设置不合适，请重新设置");
  
    return false;

  }
  Point target_pose{target_pose_info_->pose.position.x,target_pose_info_->pose.position.y,map_info_->info.resolution};
  std::cout << target_pose.index_x_ <<std::endl;
  std::cout << target_pose.index_y_ <<std::endl;

  planner_->PathFinder(init_pose,target_pose,map_info_->info.resolution);
  // path_pub_->publish();
  // 发布的path必须带上时间，否则会段报错
  std::vector<PointPtr> path = planner_->GetPath();
  nav_msgs::msg::Path p ;
  p.header.frame_id = "map";
  p.header.stamp = this->now();
  std::cout << "创建临时变量" <<std::endl;
  
  for (int i = 0; i <path.size() ; i++)
  {
       geometry_msgs::msg::PoseStamped tmp_p;
        tmp_p.header.frame_id = "map";
        tmp_p.header.stamp = this->now();
        tmp_p.pose.position.x = path[i]->coord_x_;
        tmp_p.pose.position.y = path[i]->coord_y_;
        p.poses.push_back(tmp_p);
  }
  std::cout << "创建临时变量3" <<std::endl;
  path_pub_->publish(p);
  planner_->DeleteMap(map_info_);
  return true;
}

void WindPlanner::PrintState()
{
  static std::string state_str[6] = {"INIT","WAIT_TARGET","GEN_TRAJ","EXEC_TRAJ","REPLAN"};
  int current = static_cast<int>(current_fsm_state_);
  // RCLCPP_INFO需要的是c风格的字符串
  RCLCPP_INFO(this->get_logger(),"current FSM state is %s",state_str[current].c_str());

}
void WindPlanner::FsmStateChange(const FsmState& new_state,const std::string& trigger)
{
  int past = static_cast<int>(current_fsm_state_);
  current_fsm_state_ = new_state;
  int current = static_cast<int>(current_fsm_state_);
  static std::string state_name[6] = {"INIT","WAIT_TARGET","GEN_TRAJ","EXEC_TRAJ","REPLAN"};

  RCLCPP_INFO(this->get_logger(),"FSM state from %s to %s,trigger:%s",state_name[past].c_str(),state_name[current].c_str(),trigger.c_str());


}


void WindPlanner::ExecFsmCallback()
{ 
  // 只初始化一次,每一秒打印一次相关信息
  static int counter = 0;
  counter ++;
  if (counter == 100)
  {
     PrintState();
     if (!has_odom_)
     {
        RCLCPP_INFO(this->get_logger(),"no odom!");
     }
     if (!has_map_)
     {
        RCLCPP_INFO(this->get_logger(),"no map info!");
     }
     if (!has_target_)
     {  
        RCLCPP_INFO(this->get_logger(),"no target!");
     }
     
     counter = 0;  
  }

  switch (current_fsm_state_)
  {
    case WindPlanner::FsmState::INIT:
    {
      if (!has_odom_)
      {
          return;
      }
      if (!has_map_)
      {
          return;
      }
      FsmStateChange(WindPlanner::FsmState::WAIT_TARGET,"FSM");       
      break;
    }

    case WindPlanner::FsmState::WAIT_TARGET:
    {
      if (!has_target_)
      {
          return;
      }
      FsmStateChange(WindPlanner::FsmState::GEN_TRAJ,"FSM");
      break;     
    }

    case WindPlanner::FsmState::GEN_TRAJ:
    {
      
      bool success = Plan();
      if (success)
      {
        FsmStateChange(WindPlanner::FsmState::EXEC_TRAJ,"FSM");
      }
      else if(!success && has_target_)
      {
        FsmStateChange(WindPlanner::FsmState::GEN_TRAJ,"FSM");
      }
      else
      {
        FsmStateChange(WindPlanner::FsmState::WAIT_TARGET,"target error");
      }
      
      break;
    }

    case WindPlanner::FsmState::EXEC_TRAJ:
    {
      // rclcpp::Time current_time = this->now();
      // // 
      // double time_interval = (plan_finished_time_-current_time).seconds();
      // if ()
      // {
      //   /* code */
      // 什么时候进行重规划，咱们进行的重规划是还是全局的，当
      // }
      
      break;
    }

    case WindPlanner::FsmState::REPLAN:
    { 
      // 从当前位置进行重规划，
      bool success = Plan();
      if (success)
      {
        FsmStateChange(WindPlanner::FsmState::EXEC_TRAJ,"FSM");
      }
      else if(!success && has_target_)
      {
        FsmStateChange(WindPlanner::FsmState::REPLAN,"FSM");
      }
      else
      {
        FsmStateChange(WindPlanner::FsmState::WAIT_TARGET,"target error");
      }
      break;
    }

  }
  

}

void WindPlanner::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{

  odom_info_ = msg;

}

void WindPlanner::TargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  target_pose_info_ = msg;
  has_target_ = true;
  // msg->pose.position.x = 
  if (current_fsm_state_ == WindPlanner::FsmState::WAIT_TARGET)
  {
    FsmStateChange(WindPlanner::FsmState::GEN_TRAJ,"targetcallback");
  }
  else if (current_fsm_state_ == WindPlanner::FsmState::EXEC_TRAJ)
  {
     FsmStateChange(WindPlanner::FsmState::REPLAN,"targetcallback");
  }
  
  
}

void WindPlanner::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

  map_info_ = msg; 
  RCLCPP_INFO(this->get_logger(),"获取到map_server发送的地图信息");
  
  has_map_ = true;
  


}