#include "wind_planner.h"
#include <backward.hpp>

namespace backward {
backward::SignalHandling sh;
}

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);

    auto wind_planner_node = std::make_shared<WindPlanner>();
    wind_planner_node->Init();
    
    rclcpp::spin(wind_planner_node);
    rclcpp::shutdown();

}