#ifndef PATH_H
#define PATH_H

#include "geometry_msgs/Point.h"
#include <vector>
#include "ros/ros.h"
#include <string>
#include <math.h>
#include "geometry_msgs/PoseArray.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

class path
{
private:
    struct node{
        double x;
        double y;

        int id;

        std::vector<int> connected_nodes;

//        double heuristic;

        double g_score;
        double f_score;

        bool open;
        bool closed;

        int pos;

        int came_from;

    };
private:
    path();

    void setup_node_list();
//    bool setup_h();

    int find_closest_node(geometry_msgs::Point given_point);
    int find_pos(int id);

    bool sort_list(std::vector<node> &list);

    bool sort_list_biggest(std::vector<node> &list);

    int update_current_point(geometry_msgs::Point start_point);
    int update_end_point(geometry_msgs::Point end_point);

    bool display_list(std::vector<node> &list, cv::Mat &img, char name[], int b, int g, int r, bool lines);

    void reset_nodes();

    void publish_path(std::vector<node> &list);

    void start_point_callback(const geometry_msgs::PointConstPtr &start_msg);
    void end_point_callback(const geometry_msgs::PointConstPtr &end_msg);
    void print_h(std::vector<node> &node_list);
    void shutdown(const std_msgs::BoolConstPtr &msg);

    double distance_between_nodes(node &node1, node &node2);

    bool update_list_info(std::vector<node> &list, int id, double g_score, double f_score, int came_from);

    void display_node_data(node &node1, char node_name[]);

    node smallest_node(std::vector<node> &list);

    geometry_msgs::PoseArray reverse_path(geometry_msgs::PoseArray &path);

private:
    geometry_msgs::Point _start_point;
    geometry_msgs::Point _end_point;

    std::vector<int> _path_order;
    std::vector<node> _node_list;

    ros::NodeHandle _n;

    int _start_id;
    int _end_id;

    int _start_pos;
    int _end_pos;

    ros::Publisher _path_pub;
    ros::Publisher _error_pub;
    ros::Subscriber _start_sub;
    ros::Subscriber _end_sub;
    ros::Subscriber _shutdown_sub;

    cv::Mat img_open;

    geometry_msgs::Point _given_start_point;
    geometry_msgs::Point _given_end_point;

    cv::Mat img_for_path;



public:
    path(ros::NodeHandle nh);

    int find_path();
};

#endif // PATH_H
