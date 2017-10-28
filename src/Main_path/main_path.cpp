#include "a_start/main_path.h"


main_path::main_path(ros::NodeHandle &n):
    _n(n)
{
    std::string error_name;
    std::string raw_path;
    std::string calculated_paths;
    std::string path_points;
    std::string map_location;
    std::string shutdown_name;
    std::string orientation;

    _n.getParam("/main_path/errors/main_path", error_name);
    _n.getParam("/main_path/paths/a_star", raw_path);
    _n.getParam("/main_path/paths/main_path", calculated_paths);
    _n.getParam("/main_path/paths/main_path_points", path_points);
    _n.getParam("/main_path/image", map_location);
    _n.getParam("/main_path/shutdown", shutdown_name);
    _n.getParam("/main_path/paths/orientation", orientation);

    _error_pub = _n.advertise<std_msgs::String>(error_name.c_str(), 1);
    _path_pub = _n.advertise<std_msgs::String>(calculated_paths.c_str(), 1);
    _path_points_pub = _n.advertise<geometry_msgs::PoseArray>(path_points.c_str(), 1);

    _path_sub = _n.subscribe(raw_path.c_str(), 1, &main_path::a_star_path_callback, this);
    _shutdown_sub = _n.subscribe(shutdown_name.c_str(), 1, &main_path::shutdown, this);
    _orientation_sub = _n.subscribe(orientation.c_str(),1, &main_path::orientation_callback, this);

    _map = cv::imread(map_location.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

    setup_landmarks();
    _orientation = 0;
}

void main_path::a_star_path_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    _a_star_path = *msg;
    _path_pts.clear();

    for(int i = 0; i < _a_star_path.poses.size(); i++)
    {
        geometry_msgs::Pose pose = _a_star_path.poses.at(i);
        cv::Point2f pt;

        pt.x = pose.position.x;
        pt.y = pose.position.y;

        _path_pts.push_back(pt);
    }
}

void main_path::setup_landmarks()
{
    int count;

    _n.getParam("main_path/count", count);

    for(int i = 0; i < count; i++)
    {
        std::string index = std::to_string(i);
        std::string baseline = "main_path/landmark";
        baseline.append(index.c_str());
        std::string nameGet = baseline;

        std::vector<int> xy_coordinates;

        baseline.append("/location");

        _n.getParam(baseline, xy_coordinates);

        landmark temp;
        temp.x = xy_coordinates.front();
        temp.y = xy_coordinates.back();

        std::string name;

        nameGet.append("/name");


        _n.getParam(nameGet.c_str(), name);

        temp.name = name;

        _landmark_list.push_back(temp);


    }
}

void main_path::set_line_of_sights(geometry_msgs::Point start, geometry_msgs::Point goal)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);


        cv::Point2f pt_landmark;
        cv::Point2f pt_start;
        cv::Point2f pt_goal;

        pt_landmark.x = temp.x;
        pt_landmark.y = temp.y;

        pt_start.x = start.x;
        pt_start.y = start.y;

        pt_goal.x = goal.x;
        pt_goal.y = goal.y;

        temp.start_line_of_sight = !check_intersection(pt_landmark, pt_start);
        temp.goal_line_of_sight = !check_intersection(pt_landmark, pt_goal);

        _landmark_list.at(i) = temp;
    }
}

void main_path::set_distances(geometry_msgs::Point start, geometry_msgs::Point goal)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        cv::Point2f pt_landmark;
        cv::Point2f pt_start;
        cv::Point2f pt_goal;

        pt_landmark.x = temp.x;
        pt_landmark.y = temp.y;

        pt_start.x = start.x;
        pt_start.y = start.y;

        pt_goal.x = goal.x;
        pt_goal.y = goal.y;

        temp.dist_to_start = distance_between_two_points(pt_landmark, pt_start);
        temp.dist_to_goal = distance_between_two_points(pt_landmark, pt_goal);

        temp.already_passed = false;
        temp.closed = false;
        temp.open = false;

        _landmark_list.at(i) = temp;
    }
}

void main_path::publish_pts(std::vector<cv::Point2f> &list)
{
    geometry_msgs::PoseArray path;

    for(int i = 0; i < list.size(); i++)
    {
        cv::Point2f pt = list.at(i);

        geometry_msgs::Pose pose;

        pose.position.x = pt.x;
        pose.position.y = pt.y;

        path.poses.push_back(pose);
    }

    _path_points_pub.publish(path);
}

void main_path::shutdown(const std_msgs::BoolConstPtr &msg)
{
    std_msgs::String err_str;
    err_str.data = "WENT INTO SHUTDOWN";
    _error_pub.publish(err_str);


    ros::shutdown();
}

void main_path::set_closed(main_path::landmark &land)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        if(land.name.compare(_landmark_list.at(i).name))
        {
            _landmark_list.at(i).closed = true;
        }
    }
}

void main_path::sort(std::vector<main_path::landmark> &list)
{
    for(int i = 0; i < list.size() - 1; i++)
    {
        for(int x = 0; x < list.size() - i - 1; x++)
        {
            if(list.at(x).cost < list.at(x + 1).cost)
            {
                landmark temp = list.at(x);
                list.at(x) = list.at(x + 1);
                list.at(x + 1) = temp;
            }
        }
    }
}

void main_path::orientation_callback(const std_msgs::Float32ConstPtr &msg)
{
    _orientation = msg->data;
}

double main_path::distance_between_two_points(cv::Point2f &pt1, cv::Point2f &pt2)
{
    double dist;

    dist = sqrt(pow(pt1.x - pt2.x , 2) + pow(pt1.y - pt2.y , 2));

    return dist;
}

double main_path::distance_between_two_landmarks(main_path::landmark &land1, main_path::landmark &land2)
{
    cv::Point2f pt1;
    cv::Point2f pt2;


    pt1.x = land1.x;
    pt1.y = land1.y;

    pt2.x = land2.x;
    pt2.y = land2.y;


    return distance_between_two_points(pt2,pt1);
}

cv::Point2f main_path::landmark_position(std::string &landmark_name)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(!temp.name.compare(landmark_name))
        {
            cv::Point2f pt;

            pt.x = temp.x;
            pt.y = temp.y;

            return pt;
        }
    }

    std_msgs::String str_err;

    str_err.data = "The landmark was not found within the list of landmarks. Chosen landmark must be wrong (Landmark Position) Name:";
    str_err.data.append(landmark_name);

    _error_pub.publish(str_err);
}

main_path::landmark main_path::return_landmark(std::string &land)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        if(!land.compare(_landmark_list.at(i).name))
            return _landmark_list.at(i);
    }

    std_msgs::String str_err;

    str_err.data = "The landmark was not found within the list of landmarks. Chosen landmark must be wrong (return landmark) Name:";
    str_err.data.append(land);

    _error_pub.publish(str_err);
}

std::string main_path::landmark_can_see_both()
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        if(_landmark_list.at(i).goal_line_of_sight && _landmark_list.at(i).start_line_of_sight)
            return _landmark_list.at(i).name;
    }

    std::string empty;

    return empty;
}

std::string main_path::natural_lang_gen(std::vector<cv::Point2f> &list)
{
    double current_orientation = _orientation;

    std::string toSend = "Directions\n";

    for(int i = 0; i < list.size() - 1 ; i++)
    {
        cv::Point2f currentPt = list.at(i);
        cv::Point2f nextPt = list.at(i + 1);


        double next_orientation;

        double dx = nextPt.x - currentPt.x;
        double dy = nextPt.y - currentPt.y;

        bool calculate = true;

        if(dy < 0.1 && dy > -0.1)
        {
            calculate = false;

            if(dx > 0)
                next_orientation = M_PI/2;
            else
                next_orientation = -M_PI/2;
        }

        if(dx < 0.1 && dx > -0.1)
        {
            calculate = false;

            if(dy > 0)
                next_orientation = M_PI;
            else
                next_orientation = 0.0;
        }

        if(calculate)
        {
            next_orientation = atan2(dy,dx) + M_PI/2;
        }

        int distance_to_next_point = round(sqrt(pow(dy,2) + pow(dx,2)));

        if(next_orientation - current_orientation < M_PI && next_orientation - current_orientation > 0)
        {
            toSend.append("Turn Right");

        }
        else if(next_orientation - current_orientation > M_PI || next_orientation - current_orientation < 0)
        {
            toSend.append("Turn left");
        }

        if(i + 1 == list.size() - 1)
        {
            toSend.append(" and walk about ");
            toSend.append(std::to_string(distance_to_next_point));
            toSend.append(" meters, at this point you will reach your destination");
            break;
        }

        std::string possible_name;

        if(is_landmark(nextPt, possible_name))
        {
            toSend.append(" and walk about ");
            toSend.append(std::to_string(distance_to_next_point));
            toSend.append(" meters until you reach ");
            toSend.append(possible_name);
            toSend.append(" ,from there\n");
        }
        else if(i + 2 < list.size())
        {
            std::string possible_name_2;
            cv::Point2f aheadpt = list.at(i+2);
            if(is_landmark(aheadpt, possible_name_2))
            {
                toSend.append(" and walk about ");
                toSend.append(std::to_string(distance_to_next_point));
                toSend.append(" meters until you can see ");
                toSend.append(possible_name_2);
                toSend.append(" ,now you can\n");
            }
            else
            {
                toSend.append(" and walk about ");
                toSend.append(std::to_string(distance_to_next_point));
                toSend.append(" meters and then \n");
            }

        }
        else
        {
            toSend.append(" and walk about ");
            toSend.append(std::to_string(distance_to_next_point));
            toSend.append(" meters and then \n");
        }

        current_orientation = next_orientation;
    }

    return toSend;
}

bool main_path::check_intersection(cv::Point2f &pt1, cv::Point2f &pt2)
{
    if(pt1.x - pt2.x < safeRange && pt1.x - pt2.x > -safeRange)
    {
        for(int i = min(pt1.x, pt2.x); i < max(pt1.x, pt2.x); i++)
        {
            if(_map.at<uchar>(pt1.y,i) < 250)
            {
                return true;
            }
        }
        return false;
    }
    double gradient = (pt1.y - pt2.y)/(pt1.x - pt2.x);

    for(int y = min(pt1.y,pt2.y); y <= max(pt1.y,pt2.y); y++)
    {
        for(int x = min(pt1.x, pt2.x); x <= max(pt1.x, pt2.x); x++)
        {
            double check = gradient*(x - pt1.x) + pt1.y;

            if(y <= check + 1 || y >= check + 1)
            {
                if(_map.at<uchar>(y,x) < 250)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

bool main_path::can_i_see_a_landmark(cv::Point2f &pt1, std::string &closest_landmark)
{
    bool yes = false;
    double dist = 10000.0;
    int pos = 0;

    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        cv::Point2f pt_landmark;
        pt_landmark.x = temp.x;
        pt_landmark.y = temp.y;

        if(!check_intersection(pt1, pt_landmark))
        {
            if(temp.dist_to_goal < dist && !temp.already_passed)
            {
                yes = true;
                dist = temp.dist_to_goal;
                closest_landmark = temp.name;
                pos = i;
            }
        }

    }

    if(yes)
    {
        _landmark_list.at(pos).already_passed = true;
    }

    return yes;
}

bool main_path::landmark_can_see_goal(std::string &landmark_name)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(!temp.name.compare(landmark_name))
        {
           return temp.goal_line_of_sight;
        }
    }

    std_msgs::String str_err;

    str_err.data = "The landmark was not found within the list of landmarks. Chosen landmark must be wrong (landmark_can_see_goal) Name:";
    str_err.data.append(landmark_name);

    _error_pub.publish(str_err);
}

bool main_path::landmark_can_see_start(std::string &landmark_name)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(!temp.name.compare(landmark_name))
        {
           return temp.start_line_of_sight;
        }
    }

    std_msgs::String str_err;

    str_err.data = "The landmark was not found within the list of landmarks. Chosen landmark must be wrong(landmark_can_see_start) Name:";
    str_err.data.append(landmark_name);

    _error_pub.publish(str_err);
}

bool main_path::closest_to_goal(std::string &closest_name)
{
    double smallest = _landmark_list.front().dist_to_goal;
    std::string name = _landmark_list.front().name;

    bool closest_can_see_goal = _landmark_list.front().goal_line_of_sight;

    for(int i = 1; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(!temp.goal_line_of_sight)
            continue;

        if(temp.dist_to_goal < smallest)
        {
            smallest = temp.dist_to_goal;
            name = temp.name;
            closest_can_see_goal = temp.goal_line_of_sight;
        }
    }

    closest_name = name;

    return closest_can_see_goal;
}

bool main_path::closest_to_start(std::string &closest_name)
{
    double smallest = _landmark_list.front().dist_to_start;
    std::string name = _landmark_list.front().name;

    bool closest_can_see_start = _landmark_list.front().start_line_of_sight;

    for(int i = 1; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(!temp.start_line_of_sight)
            continue;

        if(temp.dist_to_start < smallest)
        {
            smallest = temp.dist_to_start;
            name = temp.name;
            closest_can_see_start = temp.start_line_of_sight;
        }
    }

    closest_name = name;

    return closest_can_see_start;
}

bool main_path::check_linked_landmarks(std::vector<std::string> &linked_list)
{
    bool linked = false;

    std::string before_goal;
    std::string before_start;

    bool can_see_goal = closest_to_goal(before_goal);
    bool can_see_start = closest_to_start(before_start);

    if(can_see_goal && can_see_start)
    {
        landmark land1 = return_landmark(before_goal);
        landmark land2 = return_landmark(before_start);

        bool landmarks_see_each_other = line_of_sight(land1,land2);

        if(landmarks_see_each_other)
        {
            linked_list.push_back(before_start);
            linked_list.push_back(before_goal);
            return true;
        }
    }

    return linked;
}

bool main_path::line_of_sight(main_path::landmark &land1, main_path::landmark &land2)
{
    cv::Point2f pt1;
    cv::Point2f pt2;


    pt1.x = land1.x;
    pt1.y = land1.y;

    pt2.x = land2.x;
    pt2.y = land2.y;


    if(check_intersection(pt1,pt2))
        return false;

    return true;
}

bool main_path::is_landmark(cv::Point2f pt, std::string &name)
{
    for(int i = 0; i < _landmark_list.size(); i++)
    {
        landmark temp = _landmark_list.at(i);

        if(std::abs(temp.x - pt.x) < 1 && std::abs(temp.y - pt.y) < 1)
        {
            name = temp.name;
            return true;
        }
    }

    return false;
}

double main_path::max(double x, double y)
{
    if(x > y)
    {
        return x;
    }

    return y;
}

double main_path::min(double x, double y)
{
    if(x < y)
    {
        return x;
    }

    return y;
}

void main_path::find_steps()
{
    if(!ros::ok())
    {
        return;
    }
    if(_a_star_path.poses.empty())
    {
        std_msgs::String err_str;
        err_str.data = "NO PATH TO USE";
        _error_pub.publish(err_str);
        return;
    }

    geometry_msgs::Point start;
    geometry_msgs::Point goal;

    start = _a_star_path.poses.front().position;
    goal =_a_star_path.poses.back().position;

    set_line_of_sights(start, goal);
    set_distances(start, goal);


    bool found = false;
    int upto = 0;

    std::string directions = "Path Directions: ";

    std::vector<cv::Point2f> point_list;

    point_list.push_back(_path_pts.front());

    std::string landmark_sees_both = landmark_can_see_both();


    //----------------------------------------------------------------------------
    //                              New Path Here
    //----------------------------------------------------------------------------

    bool goal_found = false;

    std::string closest_landmark;
    bool landmark_can_see = closest_to_goal(closest_landmark);
    std::vector<std::string> list;

    if(!check_intersection(_path_pts.front(), _path_pts.back()))
    {
        directions.append("\nYou can see the goal from here");
        goal_found = true;
        directions.append("\nUsing rule1");
    }
    else if(!landmark_sees_both.empty())
    {
        directions.append("\nGo past ");
        directions.append(landmark_sees_both);
        directions.append(" You can see the goal from there");

        point_list.push_back(landmark_position(landmark_sees_both));
        goal_found = true;
        directions.append("\nUsing rule2");

    }
    else if(check_linked_landmarks(list))
    {
        for(int i = 0; i < list.size(); i++)
        {
            directions.append("\nGo past ");
            directions.append(list.at(i));

            point_list.push_back(landmark_position(list.at(i)));
        }

        directions.append(" You can see the goal from there");
        goal_found = true;
        directions.append("\nUsing rule3");
    }

    if(!goal_found)
    {
        std::vector<std::string> landmarks_used;
        std::string before_goal;
        bool can_see_goal = closest_to_goal(before_goal);
        point_list.clear();

        for(int i = 0; i < _path_pts.size() - 1; i++)
        {
            if(!check_intersection(_path_pts.at(i), _path_pts.back()) || !check_intersection(_path_pts.at(i + 1), _path_pts.back()))
            {
                directions.append("\nYou can see the goal from this point");
                point_list.push_back(_path_pts.at(i));
                directions.append("\nUsing rule 4.1");

                break;
            }

            if(can_see_goal)
            {
                cv::Point2f pt = landmark_position(before_goal);

                if(!check_intersection(_path_pts.at(i), pt))
                {
                    directions.append("\nYou can see landmark ");
                    directions.append(before_goal);
                    directions.append(" from this point.");

                    directions.append("Go past ");
                    directions.append(before_goal);
                    directions.append(" you will be able to see the goal from there");
                    point_list.push_back(_path_pts.at(i));
                    point_list.push_back(pt);

                    directions.append("\nUsing rule 4.2");

                    break;
                }
            }
            std::string visible_landmark;
            bool done = false;
            while(can_i_see_a_landmark(_path_pts.at(i), visible_landmark) && !done)
            {
                if(landmarks_used.empty())
                {
                    landmarks_used.push_back(visible_landmark);

                    directions.append("\nGo past this point (point to direction), you can see");
                    directions.append(visible_landmark);
                    directions.append(" from there");

                    point_list.push_back(_path_pts.at(i));
                    directions.append("Go past ");
                    directions.append(visible_landmark);

                    point_list.push_back(landmark_position(visible_landmark));

                    done = true;
                }

                if(done)
                    continue;


                bool toContinue = false;


                for(int x = 0; x < landmarks_used.size(); x++)
                {
                    cv::Point2f current_landmark = landmark_position(visible_landmark);
                    cv::Point2f already_used = landmark_position(landmarks_used.at(x));
                    if(distance_between_two_points(_path_pts.back(), current_landmark) > distance_between_two_points(_path_pts.back(),already_used))
                        toContinue = true;

                }

                if(toContinue)
                    continue;

                landmarks_used.push_back(visible_landmark);

                directions.append("\nGo past this point (point to direction), you can see");
                directions.append(visible_landmark);
                directions.append(" from there");

                point_list.push_back(_path_pts.at(i));

                directions.append("Go past ");
                directions.append(visible_landmark);

                point_list.push_back(landmark_position(visible_landmark));
            }
        }
    }

    point_list.push_back(_path_pts.back());

    directions = natural_lang_gen(point_list);


    //----------------------------------------------------------------------------
    //                              End New Path Here
    //----------------------------------------------------------------------------


    std_msgs::String msg;

    msg.data = directions.c_str();

    _path_pub.publish(msg);

    publish_pts(point_list);

}
