#ifndef MAIN_PATH_H
#define MAIN_PATH_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <string>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/Float32.h"

#define safeRange 0.01

class main_path
{
private:// any structs belonging to this class
    struct landmark{
        double x;
        double y;

        std::string name;

        bool goal_line_of_sight;
        bool start_line_of_sight;
        bool already_passed;

        double dist_to_start;
        double dist_to_goal;

        bool closed;
        bool open;
        double cost;

    };


private://methods
    main_path();

    //Returns VOID
    void a_star_path_callback(const geometry_msgs::PoseArrayConstPtr &msg);
    void setup_landmarks();
    void set_line_of_sights(geometry_msgs::Point start, geometry_msgs::Point goal);
    void set_distances(geometry_msgs::Point start, geometry_msgs::Point goal);
    void publish_pts(std::vector<cv::Point2f> &list);
    void shutdown(const std_msgs::BoolConstPtr &msg);
    void set_closed(landmark &land);
    void sort(std::vector<landmark> &list);
    void orientation_callback(const std_msgs::Float32ConstPtr &msg);


    //Returns BOOL
    bool check_intersection(cv::Point2f &pt1, cv::Point2f &pt2);
    bool can_i_see_a_landmark(cv::Point2f &pt1, std::string &closest_landmark);
    bool landmark_can_see_goal(std::string &landmark_name);
    bool landmark_can_see_start(std::string &landmark_name);
    bool closest_to_goal(std::string &closest_name);
    bool closest_to_start(std::string &closest_name);
    bool check_linked_landmarks(std::vector<std::string> &linked_list);
    bool line_of_sight(main_path::landmark &land1, main_path::landmark &land2);
    bool is_landmark(cv::Point2f pt, std::string &name);


    //Returns Double
    double max(double x, double y);
    double min(double x, double y);
    double distance_between_two_points(cv::Point2f &pt1, cv::Point2f &pt2);
    double distance_between_two_landmarks(landmark &land1, landmark &land2);


    //pt returns

    cv::Point2f landmark_position(std::string &landmark_name);

    landmark return_landmark(std::string &land);

    std::string landmark_can_see_both();
    std::string natural_lang_gen(std::vector<cv::Point2f> &list);


private://members

    //ROS STUFF
    ros::NodeHandle _n;

    ros::Publisher _error_pub;
    ros::Publisher _path_pub;
    ros::Publisher _path_points_pub;

    ros::Subscriber _path_sub;
    ros::Subscriber _shutdown_sub;
    ros::Subscriber _orientation_sub;

    geometry_msgs::PoseArray _a_star_path;

    //c++ standard libraries
    std::vector<landmark> _landmark_list;
    std::vector<cv::Point2f> _path_pts;

    //OPENCV STUFF
    cv::Mat _map;

    double _orientation;

public:
    main_path(ros::NodeHandle &n);
    void find_steps();
};

#endif // MAIN_PATH_H
