#include "a_start/path.h"

path::path(ros::NodeHandle nh):
    _n(nh)
{
    ROS_INFO("STARTING NODE SETUP");
    setup_node_list();
    ROS_INFO("NODE SETUP DONE");

    std::string error_out;
    std::string path_output;
    std::string shutdown_name;

    _n.getParam("/a_star/errors/a_star", error_out);
    _n.getParam("/a_star/paths/a_star", path_output);
    _n.getParam("/a_star/shutdown", shutdown_name);

    _start_id = -1;
    _end_id = -1;

    _path_pub = _n.advertise<geometry_msgs::PoseArray>(path_output.c_str(), 1);
    _error_pub = _n.advertise<std_msgs::String>(error_out.c_str(), 100);


    _start_sub = _n.subscribe("/capstone/path/start", 1, &path::start_point_callback, this);
    _end_sub = _n.subscribe("/capstone/path/end", 1, &path::end_point_callback, this);
    _shutdown_sub = _n.subscribe(shutdown_name.c_str(), 1, &path::shutdown, this);

    img_open = cv::imread("/home/denis/catkin_ws/src/a_start/data/map4.png", CV_LOAD_IMAGE_COLOR);
    img_for_path = cv::imread("/home/denis/catkin_ws/src/a_start/data/map4.png", CV_LOAD_IMAGE_GRAYSCALE);


    _given_start_point.x = 1.1;
    _given_start_point.y = 3.0;

    _given_end_point.x = 40;
    _given_end_point.y = 10;



}

int path::find_path()
{
    if(!ros::ok())
    {
        return 0;
    }
    std_msgs::String error_str;

//    ROS_INFO("Resetting nodes");
    reset_nodes();

    cv::Point2f pt_s;
    cv::Point2f pt_e;


//    ROS_INFO("UPRATE POINTS");
    update_current_point(_given_start_point);
    update_end_point(_given_end_point);


//    ROS_INFO("UPRATE POINTS DONE");


    if(_given_start_point.x == _given_end_point.x && _given_start_point.y == _given_end_point.y)
    {
        ROS_ERROR("START POINT THE SAME AS END POINT");
        error_str.data = "START POINT THE SAME AS END POINT";
        _error_pub.publish(error_str);
        return -1;
    }

    if(_given_start_point.x < 0 || _given_start_point.x >= img_for_path.cols ||  _given_start_point.y < 0 || _given_start_point.y >= img_for_path.rows)
    {
        ROS_ERROR("GIVEN START POINT IS OUT OF RANGE");
        error_str.data = "GIVEN START POINT IS OUT OF RANGE";
        _error_pub.publish(error_str);
        return -1;
    }

    if(_given_end_point.x < 0 || _given_end_point.x >= img_for_path.cols ||  _given_end_point.y < 0 || _given_end_point.y >= img_for_path.rows)
    {
        ROS_ERROR("GIVEN END POINT IS OUT OF RANGE");
        error_str.data = "GIVEN END POINT IS OUT OF RANGE";
        _error_pub.publish(error_str);
        return -1;
    }

    if(img_for_path.at<uchar>(_given_start_point.y,_given_start_point.x) < 250)
    {
        ROS_ERROR("START POINT is in a wall");
        error_str.data = "START POINT is in a wall";
        _error_pub.publish(error_str);
        return -1;
    }

    if(img_for_path.at<uchar>(_given_end_point.y,_given_end_point.x) < 250)
    {
        ROS_ERROR("END POINT is in a wall");
        error_str.data = "END POINT is in a wall";
        _error_pub.publish(error_str);
        return -1;
    }


    std::vector<node> open_list;
    std::vector<node> closed_list;

    node start = _node_list.at(_start_pos);
    node end = _node_list.at(_end_pos);

    start.g_score = 0;
    start.f_score = distance_between_nodes(start, end);

    open_list.push_back(start);

    _node_list.at(_start_pos).open = true;

//    ROS_INFO("STARTING SEARCH");

    while(!open_list.empty())
    {
        sort_list(open_list);

        node current = open_list.back();

        int pos = find_pos(current.id);


        if(current.id == end.id)
        {
            //donet
            closed_list.push_back(current);
//            std::cout << "FOUND" << std::endl;
            break;
        }

        open_list.pop_back();
        closed_list.push_back(current);


        _node_list.at(pos).closed = true;

        for(int i = 0; i < current.connected_nodes.size(); i++)
        {
            int neighbour_pos = find_pos(current.connected_nodes.at(i));

            node neighbour = _node_list.at(neighbour_pos);


            if(_node_list.at(neighbour_pos).closed)
            {
                continue;
            }

            if(!_node_list.at(neighbour_pos).open)
            {
                open_list.push_back(neighbour);
                _node_list.at(neighbour_pos).open = true;

            }


            neighbour.g_score = /*temp_g_score;*/distance_between_nodes(current, neighbour);
            neighbour.f_score = neighbour.g_score + distance_between_nodes(neighbour, end);


            update_list_info(open_list, neighbour.id, neighbour.g_score, neighbour.f_score, current.id);

        }
    }

    if(open_list.empty())
    {
        ROS_ERROR("NO PATH FOUND");
        error_str.data = "NO PATH WAS FOUND";
        _error_pub.publish(error_str);
        std::vector<node> empty_list;
        publish_path(empty_list);
    }
    else
    {
        publish_path(closed_list);
    }

}

void path::setup_node_list()
{
    cv::Mat img = cv::imread("/home/denis/catkin_ws/src/a_start/data/map4.png", CV_LOAD_IMAGE_GRAYSCALE);
    for(int y = 0; y < img.rows; y++)
    {
        for(int x = 0; x < img.cols; x++)
        {
            if(img.at<uchar>(y,x) < 250)
            {
                continue;
            }

            node temp;

            temp.x = x;
            temp.y = y;

            temp.id = y * img.cols + x;

            temp.open = false;
            temp.closed = false;

            for(int y_temp = y - 1; y_temp < y + 2; y_temp++)
            {
                for(int x_temp = x - 1; x_temp < x + 2; x_temp++)
                {
                    if(y_temp < 0 || y_temp > img.rows || x_temp < 0 || x_temp > img.cols)
                    {
                        continue;
                    }

                    if(img.at<uchar>(y_temp,x_temp) < 250)
                    {
                        continue;
                    }

                    if(y_temp == y && x_temp == x)
                    {
                        continue;
                    }


                    temp.connected_nodes.push_back(y_temp * img.cols + x_temp);

                }

            }


            _node_list.push_back(temp);
        }
    }
}

int path::find_closest_node(geometry_msgs::Point given_point)
{
    if(_node_list.empty())
    {
        std_msgs::String error_str;
        error_str.data = "NODE LIST IS EMPTY";
        _error_pub.publish(error_str);
        return -1;
    }

    double min = sqrt(pow((given_point.x) - _node_list.front().x, 2) + pow((given_point.y) - _node_list.front().y, 2));
    int found_id = 0;

    for (int i = 1; i < _node_list.size(); i++)
    {
        double temp = sqrt(pow((given_point.x) - _node_list.at(i).x, 2) + pow((given_point.y) - _node_list.at(i).y, 2));

        if (temp < min)
        {
            min = temp;
            found_id = _node_list.at(i).id;
        }
    }

    return found_id;
}

int path::find_pos(int id)
{
    if(id < 0)
    {
        std_msgs::String error_str;
        error_str.data = "FIND_POS/NODE ID IS LESS THAN 0";
        _error_pub.publish(error_str);
        return -1;
    }

    for(int i = 0; i < _node_list.size(); i++)
    {
        if(_node_list.at(i).id == id)
        {
            return i;
        }
    }

    std_msgs::String error_str;
    error_str.data = "FIND_POS/UNABLE TO FIND NODE";
    _error_pub.publish(error_str);
    return -2;
}

bool path::sort_list(std::vector<node> &list)
{
    if(list.empty())
    {
        ROS_ERROR("LIST IS EMPTY");
        std_msgs::String error_str;
        error_str.data = "SORT_LIST/LIST IS EMPTY";
        _error_pub.publish(error_str);
        return false;
    }

    for(int i = 0; i < list.size() - 1; i++)
    {
        for(int x = 0; x < list.size() - i - 1; x++)
        {
            if(list.at(x).f_score < list.at(x + 1).f_score)
            {
                node temp = list.at(x);
                list.at(x) = list.at(x + 1);
                list.at(x + 1) = temp;
            }
        }
    }

    return true;
}

bool path::sort_list_biggest(std::vector<path::node> &list)
{
    if(list.empty())
    {
        ROS_ERROR("LIST IS EMPTY");
        return false;
    }

    for(int i = 0; i < list.size() - 1; i++)
    {
        for(int x = 0; x < list.size() - i - 1; x++)
        {
            if(list.at(x).f_score > list.at(x + 1).f_score)
            {
                node temp = list.at(x);
                list.at(x) = list.at(x + 1);
                list.at(x + 1) = temp;
            }
        }
    }

    return true;
}

int path::update_current_point(geometry_msgs::Point start_point)
{
    _start_point = start_point;
    _start_id = find_closest_node(_start_point);
    _start_pos = find_pos(_start_id);

    return _start_id;

}

int path::update_end_point(geometry_msgs::Point end_point)
{
    _end_point = end_point;
    _end_id = find_closest_node(_end_point);
    _end_pos = find_pos(_end_id);

    return _end_id;
}

bool path::display_list(std::vector<path::node> &list, cv::Mat &img, char name[], int b, int g, int r, bool lines = false)
{
    cv::Mat made_img;


 // MAKE STUFF HERE
    if(lines)
    {
        made_img = img;
        node goal = _node_list.at(_end_pos);
        cv::Point2f pt_end;

        pt_end.x = goal.x;
        pt_end.y = goal.y;

        cv::Point2f pt_front;
        node front = list.front();

        pt_front.x = front.x;
        pt_front.y = front.y;

        cv::line(made_img, pt_end, pt_front, cv::Scalar(b,g,r), 1);

        for(int i = 0; i < list.size() - 1; i++)
        {
            node temp = list.at(i);
            node temp2 = list.at(i + 1);

            cv::Point2f pt;
            cv::Point2f pt2;

            pt.x = temp.x + img.cols/2;
            pt.y = img.rows/2 - temp.y;

            pt2.x = temp2.x + img.cols/2;
            pt2.y = img.rows/2 - temp2.y;

            cv::line(made_img, pt, pt2, cv::Scalar(b,g,r), 1);

        }


    }
    else
    {
        cv::namedWindow(name, CV_WINDOW_NORMAL);
        made_img = img.clone();

        cv::Point2f pt;

        pt.x = _start_point.x;
        pt.y = _start_point.y;

        cv::circle(made_img, pt, 1, cv::Scalar(0,255,0));

        pt.x = _end_point.x ;
        pt.y = _end_point.y;

        cv::circle(made_img, pt, 1, cv::Scalar(0,0,255));

        for(int i = 0; i < list.size(); i++)
        {
            node temp = list.at(i);

            cv::Point2f pt;
            pt.x = temp.x;
            pt.y = temp.y;

            if(pt.x > made_img.cols || pt.x < 0 || pt.y > made_img.rows || pt.y < 0)
            {
                ROS_ERROR("ACCESSING BEYOND IMAGE");
                return false;
            }

            cv::Vec3b color;
            color[0] = b;
            color[1] = g;
            color[2] = r;
            made_img.at<cv::Vec3b>(pt) = color;

        }
        cv::imshow(name, made_img);
        cv::waitKey(3);
    }


}

void path::reset_nodes()
{
    for(int i = 0; i < _node_list.size(); i++)
    {
        _node_list.at(i).open = false;
        _node_list.at(i).closed = false;
    }
}

void path::publish_path(std::vector<path::node> &list)
{

//    sort_list_biggest(list);
    geometry_msgs::PoseArray path;


    while(!list.empty())
    {
        node temp = list.back();

        geometry_msgs::Point pt;
        geometry_msgs::Pose pose;


        pt.x = temp.x;
        pt.y = temp.y;

        pose.position = pt;

        if (temp.id == _start_id)
        {
            path.poses.push_back(pose);
            break;
        }

        path.poses.push_back(pose);

        list.pop_back();

        for(int i = 0; i < list.size(); i++)
        {
            node temp2 = list.at(i);

            if(temp2.id == temp.came_from)
            {
                node holder = list.at(i);
                list.at(i) = list.at(list.size() - 1);

                list.at(list.size() - 1) = holder;
                break;
            }
        }
    }

//    ROS_INFO("PUBLISHED");
    path = reverse_path(path);
    _path_pub.publish(path);
}

void path::start_point_callback(const geometry_msgs::PointConstPtr &start_msg)
{
    _given_start_point = *start_msg;
}

void path::end_point_callback(const geometry_msgs::PointConstPtr &end_msg)
{
    _given_end_point = *end_msg;
}

void path::print_h(std::vector<node> &node_list)
{
    cv::namedWindow("H", cv::WINDOW_NORMAL);
    cv::Mat img = img_open.clone();
    cv::cvtColor(img,img, CV_BGR2GRAY);

    for(int i = 0; i < node_list.size(); i++)
    {
        node temp = node_list.at(i);

        int x = temp.x + img_open.cols/2;
        int y = img_open.rows/2 - temp.y;

        img.at<uchar>(y,x) = 0;

        img.at<uchar>(y,x) = temp.f_score;
    }

    cv::imshow("H", img);
    cv::waitKey(3);
}

void path::shutdown(const std_msgs::BoolConstPtr &msg)
{
    std_msgs::String error_str;
    error_str.data = "GOING FOR SHUTDOWN";
    _error_pub.publish(error_str);

    ros::shutdown();
}

double path::distance_between_nodes(path::node &node1, path::node &node2)
{
    double dist = 0.0;

    dist = sqrt(pow((node2.x - node1.x),2) + pow((node2.y - node1.y),2));

//    dist = std::abs(node2.x - node1.x) + std::abs(node2.y - node1.y);

    return dist;
}

bool path::update_list_info(std::vector<path::node> &list, int id , double g_score, double f_score, int came_from)
{
    for(int i = 0; i < list.size(); i++)
    {
        if(list.at(i).id == id)
        {
            list.at(i).g_score = g_score;
            list.at(i).f_score = f_score;
            list.at(i).came_from = came_from;
            break;
        }
    }

    return true;
}

void path::display_node_data(path::node &node1, char node_name[])
{
    std::cout << "*****DISPLAYING NODE INFO*****" << std::endl;
    std::cout << "NODE NAME -> " << node_name << std::endl;
    std::cout << "ID -> " << node1.id << std::endl;
    std::cout << "X -> " << node1.x << " Y -> " << node1.y << std::endl;
    std::cout << "OPEN? -> " << node1.open << " CLODED? ->" << node1.closed << std::endl;
    std::cout << "GSCORE -> " << node1.g_score << " FSCORE -> " << node1.f_score << std::endl;
    std::cout << "*****FINISHED DISPLAYING NODE INFO*****" << std::endl;
}

path::node path::smallest_node(std::vector<path::node> &list)
{
    if(list.empty())
    {
        ROS_INFO("EMPTY");
    }

    node smallest = list.front();

    for(int i = 0; i < list.size(); i++)
    {
        node temp = list.at(i);

        if(temp.f_score < smallest.f_score)
        {
            smallest = temp;
        }
    }

    return smallest;
}

geometry_msgs::PoseArray path::reverse_path(geometry_msgs::PoseArray &path)
{
    geometry_msgs::PoseArray path_reversed;

    for(int i = path.poses.size() - 1; i >=0; i--)
    {
        path_reversed.poses.push_back(path.poses.at(i));
    }

    return path_reversed;
}

