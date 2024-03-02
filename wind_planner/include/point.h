#pragma once
#include <rclcpp/rclcpp.hpp>

#define inf 1>>20
class Point
{
    public:
        Point() = default;
        Point(int x,int y,double resultion) : index_x_(x),index_y_(y),resultion_(resultion),parent_(nullptr),
                                   state_(Point::STATE::NOTVISITED),g_cost_(inf),h_cost_(inf),f_cost_(inf)
        {
            // CoordToIndex(coord_x_,coord_y_,resultion_);
            IndexToCoord(index_x_,index_y_,resultion);
        }

        double coord_x_;
        double coord_y_;
        int index_x_;
        int index_y_;
        double resultion_;
        // 指针体积小
        Point* parent_;
        // 0为未访问，1为访问过且在openset中，-1为访问过且在closeset中
        // int visited_or_not_;

        enum class STATE{NOTVISITED = 0,OPENLIST = 1,CLOSELIST = 2,OBSTACLE = 3};
        STATE state_;
        double g_cost_;
        double h_cost_;
        double f_cost_;

        // 类内定义，自动转换为内敛函数
        void CoordToIndex(double coord_x,double coord_y,double resultion)
        {   
            // 向下取整
            index_x_ =(int)(coord_x / resultion);
            index_y_ =(int)(coord_y / resultion);
            
        }
        void IndexToCoord(int index_x,int index_y,double resultion)
        {
            coord_x_ = (double)((0.5 + index_x) * resultion);
            coord_y_ = (double)((0.5 + index_y) * resultion);

        }

    private:

};