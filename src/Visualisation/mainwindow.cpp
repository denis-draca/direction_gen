#include "a_start/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(ros::NodeHandle &n, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow), _n(n)
{
    ui->setupUi(this);

    multiplier = 8;

    std::string error_out_a_star;
    std::string error_out_main;
    std::string raw_path;
    std::string directions;
    std::string direction_pts;
    std::string shutdown_name;
    std::string orientation;

    _n.getParam("/visualiser/errors/a_star", error_out_a_star);
    _n.getParam("/visualiser/errors/main_path", error_out_main);
    _n.getParam("/visualiser/paths/a_star", raw_path);
    _n.getParam("/visualiser/paths/main_path", directions);
    _n.getParam("/visualiser/paths/main_path_points", direction_pts);
    _n.getParam("/visualiser/shutdown", shutdown_name);
    _n.getParam("/visualiser/paths/orientation", orientation);

    _path_sub = _n.subscribe(raw_path.c_str(), 1, &MainWindow::path_callback,this);
    _error_a_start_sub = _n.subscribe(error_out_a_star.c_str(),1, &MainWindow::a_start_error_callback, this);
    _error_main_sub = _n.subscribe(error_out_main.c_str(), 1, &MainWindow::main_path_error_callback, this);
    _path_directions_sub = _n.subscribe(directions.c_str(), 1 , &MainWindow::directions_callback, this);
    _path_points_sub = _n.subscribe(direction_pts.c_str(), 1, &MainWindow::direction_pts_callback, this);

    _start_point_pub = _n.advertise<geometry_msgs::Point>("/capstone/path/start", 1);
    _end_point_pub = _n.advertise<geometry_msgs::Point>("/capstone/path/end", 1);
    _shutdown_pub = _n.advertise<std_msgs::Bool>(shutdown_name.c_str(), 1);
    _orientation_pub = _n.advertise<std_msgs::Float32>(orientation.c_str(),1);


    QString fileName = "/home/denis/catkin_ws/src/a_start/data/map4.png"; /*QFileDialog::getOpenFileName(this,"Open Image File",QDir::currentPath())*/

    if(!fileName.isEmpty())
    {
        QImage image(fileName);

        if(image.isNull())
        {
            std::cout << "ERROR" << std::endl;
            QMessageBox::information(this,"Image Viewer","Error Displaying image");
            return;
        }

        path_img = cv::imread("/home/denis/catkin_ws/src/a_start/data/map4.png", CV_LOAD_IMAGE_COLOR);

        cv::Mat temp = path_img.clone();

        cv::Size size(path_img.cols * multiplier, path_img.rows * multiplier);
        cv::resize(path_img, temp, size);

        ui->img_label->setPixmap(QPixmap::fromImage(Mat2QImage(temp)));

        ui->in_start_x->setValidator(new QIntValidator(0, path_img.cols, this));
        ui->in_start_y->setValidator(new QIntValidator(0, path_img.rows, this));
        ui->in_end_x->setValidator(new QIntValidator(0, path_img.cols, this));
        ui->in_end_y->setValidator(new QIntValidator(0, path_img.rows, this));
//        ui->in_orientation->setValidator(new QDoubleValidator(0, 6.283, 3, this));
        QDoubleValidator *test = new QDoubleValidator;
        test->setDecimals(3);
        test->setBottom(0.0);
        test->setTop(6.283);

        test->setNotation(QDoubleValidator::StandardNotation);
        ui->in_orientation->setValidator(test);
    }


    setup_landmarks();
    display_landmarks();

    _timer = new QTimer(this);
     connect(_timer, SIGNAL(timeout()), this, SLOT(check_callbacks()));
    _timer->start(100);

    _timer_2 = new QTimer(this);
     connect(_timer_2, SIGNAL(timeout()), this, SLOT(check_mouse_pos()));
    _timer_2->start(17);

    _error_show = true;
    _set_slider = true;
    _landmark_update_done = false;

    //Directory member setup
    _astar_screenshot_dir = "/home/denis/catkin_ws/src/a_start/for Report";
    _directions_screenshot_dir = "/home/denis/catkin_ws/src/a_start/for Report";
    _landmarks_screenshot_dir = "/home/denis/catkin_ws/src/a_start/for Report";

    int x = ui->img_uInput->pos().x();
    int y = ui->img_uInput->pos().y();

    ui->img_uInput->setGeometry(x, y, 800, 400);
    setMouseTracking(true);


    _button_flag = -1;

    _arrow_length = 20;
    _orientation = 0;
    _thickness = 2;
    _arrow_gain = 0.2;
    _preset = -1;

    ui->slide_arrow_length->setSliderPosition(_arrow_length);
    ui->slide_arrow_thickness->setSliderPosition(((double)_thickness/15.0)*100);

}

cv::Mat MainWindow::QImage2Mat(const QImage &src)
{
//    cv::Mat tmp(src.height(), src.width(), CV_8UC, (uchar*)src.bits(), src.bytesPerLine());
//    cv::Mat result;
//    cvtColor(tmp, result, CV_BGR2RGB);

    //    return result;
}

cv::Mat MainWindow::resize_to_multipler(cv::Mat &small_img)
{
    int scale_factor = multiplier;
    cv::Mat larger_img(small_img.rows * scale_factor, small_img.cols*scale_factor, CV_8UC3, cv::Scalar(0,0,0));

    for(int y = 0; y < small_img.rows; y++)
    {
        for(int x = 0; x < small_img.cols; x++)
        {
            cv::Vec3b cell = small_img.at<cv::Vec3b>(y,x);

            for(int z = (y*scale_factor); z < (y*scale_factor) + (scale_factor); z++)
            {
                for(int t = (x*scale_factor) ; t < (x*scale_factor) + (scale_factor); t++)
                {
                    if(z >= 0 && z < larger_img.rows && t >=0 && t < larger_img.cols)
                    {
                        larger_img.at<cv::Vec3b>(z,t) = cell;
                    }
                }
            }
        }
    }

    return larger_img;



}

void MainWindow::path_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{

    geometry_msgs::PoseArray derivedMsg = *msg;

    cv::Mat display_path = path_to_img(derivedMsg);

    _resized_a_star = resize_to_multipler(display_path);

    ui->img_label->setPixmap(QPixmap::fromImage(Mat2QImage(_resized_a_star)));
}

cv::Mat MainWindow::path_to_img(geometry_msgs::PoseArray &path)
{
    cv::Mat img = path_img.clone();

    for(int i = 1; i < path.poses.size(); i++)
    {
        cv::Point2f pt1;
        cv::Point2f pt2;

        geometry_msgs::Pose pose1 = path.poses.at(i-1);
        geometry_msgs::Pose pose2 = path.poses.at(i);

        pt1.x = pose1.position.x;
        pt1.y = pose1.position.y;

        pt2.x = pose2.position.x;
        pt2.y = pose2.position.y;

        cv::line(img, pt1, pt2, cv::Scalar(255,0,0));

    }

    cv::Point2f pt1;
    cv::Point2f pt2;

    pt1.x = _start_x;
    pt1.y = _start_y;

    pt2.x = _end_x;
    pt2.y = _end_y;

    cv::circle(img, pt1, 1, cv::Scalar(0,255,0));
    cv::circle(img, pt2, 1, cv::Scalar(0,0,255));


    return img;
}

void MainWindow::update_inputs()
{
    std::string start_x = ui->in_start_x->text().toUtf8().constData();
    std::string start_y = ui->in_start_y->text().toUtf8().constData();
    std::string end_x = ui->in_end_x->text().toUtf8().constData();
    std::string end_y = ui->in_end_y->text().toUtf8().constData();


    if(start_x.empty())
        _start_x = -1;
    else
        _start_x = std::stoi(start_x);

    if(start_y.empty())
        _start_y = -1;
    else
        _start_y = std::stoi(start_y);

    if(end_x.empty())
        _end_x = -1;
    else
        _end_x = std::stoi(end_x);

    if(end_y.empty())
        _end_y = -1;
    else
        _end_y = std::stoi(end_y);

    if(start_x.empty() && start_y.empty() && end_x.empty() && end_y.empty())
    {
        return;
    }

}

void MainWindow::setup_landmarks()
{
    int count;

    _n.getParam("visualiser/count", count);

    for(int i = 0; i < count; i++)
    {
        std::string index = std::to_string(i);
        std::string baseline = "visualiser/landmark";

        baseline.append(index.c_str());
        std::string nameGet = baseline;
        baseline.append("/location");

        std::vector<int> xy_coordinates;

        _n.getParam(baseline, xy_coordinates);

        std::pair<int,int> temp;
        temp.first = xy_coordinates.front();
        temp.second = xy_coordinates.back();

        _landmarks.push_back(temp);

        nameGet.append("/name");

        std::string tempName;

        _n.getParam(nameGet.c_str(), tempName);

        _landmark_names.push_back(tempName);
    }


}

void MainWindow::display_landmarks()
{
    cv::Mat landmark_disp = path_img.clone();

    for(int i = 0; i < _landmarks.size(); i++)
    {
        std::pair<int,int> temp = _landmarks.at(i);

        cv::Point2f pt;

        pt.x = temp.first;
        pt.y = temp.second;

        cv::circle(landmark_disp, pt, 1, cv::Scalar(50, 100, 200));

        if(i != _landmarks.size() - 1)
        {
            cv::Point2f pt2;

            pt2.x = _landmarks.at(i + 1).first;
            pt2.y = _landmarks.at(i + 1).second;

            cv::line(landmark_disp, pt, pt2, cv::Scalar(20,255,255));
        }
    }

    cv::Point2f pt1;

    pt1.x = _landmarks.front().first;
    pt1.y = _landmarks.front().second;

    cv::Point2f pt2;

    pt2.x = _landmarks.back().first;
    pt2.y = _landmarks.back().second;

    cv::line(landmark_disp, pt1, pt2, cv::Scalar(20,255,255));


    _landmark_map = resize_to_multipler(landmark_disp);

    name_landmarks(_landmark_map);

    ui->img_landmarks->setPixmap(QPixmap::fromImage(Mat2QImage(_landmark_map)));

}

void MainWindow::a_start_error_callback(const std_msgs::String &msg)
{
    _a_start_error.append("\n");
    _a_start_error.append(msg.data);

    if(_error_show)
    ui->out_ASTART_error->setText(_a_start_error.c_str());
}

void MainWindow::main_path_error_callback(const std_msgs::String &msg)
{
    _main_path_error.append("\n");
    _main_path_error.append(msg.data);

    if(_error_show)
    ui->out_MAIN_error->setText(_main_path_error.c_str());
}

void MainWindow::directions_callback(const std_msgs::String &msg)
{
    _direction_list = msg.data;
    if(ui->ch_directions->isChecked())
        ui->out_directions->setText(_direction_list.c_str());
}

void MainWindow::direction_pts_callback(const geometry_msgs::PoseArrayConstPtr &msg)
{
    geometry_msgs::PoseArray pt_list = *msg;

    if(pt_list.poses.empty())
    {
        _main_path_error.append("\n");
        _main_path_error.append("empty path transmitted");
        ui->out_MAIN_error->setText(_main_path_error.c_str());
        return;
    }

    _point_generator_selections.clear();
    cv::Mat img = path_img.clone();

    for(int i = 0; i < pt_list.poses.size(); i++)
    {
        cv::Point2f pt;

        pt.x = pt_list.poses.at(i).position.x;
        pt.y = pt_list.poses.at(i).position.y;

        _point_generator_selections.push_back(pt);

        if(i != pt_list.poses.size() - 1)
        {
            cv::Point2f pt2;
            pt2.x = pt_list.poses.at(i + 1).position.x;
            pt2.y = pt_list.poses.at(i + 1).position.y;
            cv::line(img, pt, pt2, cv::Scalar(0,255,255));
        }

        if(i == 0)
            cv::circle(img, pt, 1, cv::Scalar(0,255,0));
        else if(i == pt_list.poses.size() - 1)
            cv::circle(img, pt, 1, cv::Scalar(0,0,255));
        else
            cv::circle(img, pt, 1, cv::Scalar(255,0,0));
    }

    cv::Point2f start_pt;
    start_pt.x = pt_list.poses.front().position.x*multiplier + multiplier/2;
    start_pt.y = pt_list.poses.front().position.y*multiplier + multiplier/2;

    _directions_map = resize_to_multipler(img);

    name_landmarks(_directions_map);

    arrow(_directions_map, start_pt, _orientation, cv::Scalar(0,255,0), _arrow_length);

    ui->img_directions->setPixmap(QPixmap::fromImage(Mat2QImage(_directions_map)));

}

void MainWindow::name_landmarks(cv::Mat &img)
{
    for(int i = 0; i < _landmarks.size(); i++)
    {
        std::pair<int,int> temp = _landmarks.at(i);

        cv::Point2f pt;

        pt.x = temp.first * multiplier;
        pt.y = temp.second * multiplier + 40;

        cv::putText(img, _landmark_names.at(i), pt, 1, 1, cv::Scalar(50,150,255));
    }
}

void MainWindow::general_display(int x, int y, bool draw)
{
    cv::Mat landmark_disp = path_img.clone();

    if(!_landmark_update_done)
    {
        for(int i = 0; i < _landmarks.size(); i++)
        {
            std::pair<int,int> temp = _landmarks.at(i);

            cv::Point2f pt;

            pt.x = temp.first;
            pt.y = temp.second;

            cv::circle(landmark_disp, pt, 1, cv::Scalar(50, 100, 200));
        }

//        _landmark_update_done = true;
    }

    geometry_msgs::Point start_pt;
    geometry_msgs::Point end_pt;

    bool unset = false;
    if(_start_x == -1)
        unset = true;
    else
        start_pt.x = _start_x;

    if(_start_y == -1)
        unset = true;
    else
        start_pt.y = _start_y;

    if(_end_x == -1)
        unset = true;
    else
       end_pt.x = _end_x;

    if(_end_y == -1)
        unset = true;
    else
        end_pt.y = _end_y;

    if(!unset)
    {
        cv::Point2f pt1;
        cv::Point2f pt2;

        pt1.x = _start_x;
        pt1.y = _start_y;

        pt2.x = _end_x;
        pt2.y = _end_y;

        cv::circle(landmark_disp, pt1, 1, cv::Scalar(0,255,0));
        cv::circle(landmark_disp, pt2, 1, cv::Scalar(0,0,255));
    }

    if(draw)
    {
        if(ui->rb_circle->isChecked())
        {
            cv::Point2f pt_draw;
            pt_draw.x = x;
            pt_draw.y = y;

            cv::circle(landmark_disp, pt_draw, 1, cv::Scalar(255,0,0));
        }
        else if(ui->rb_crosshair->isChecked())
        {
            cv::Point2f pt_top;
            cv::Point2f pt_bottom;
            cv::Point2f pt_left;
            cv::Point2f pt_right;

            pt_top.x = x;
            pt_top.y = 0;
            pt_bottom.x = x;
            pt_bottom.y = landmark_disp.rows;

            pt_left.x = 0;
            pt_left.y = y;
            pt_right.x = landmark_disp.cols;
            pt_right.y = y;

            cv::line(landmark_disp, pt_top, pt_bottom,cv::Scalar(255,0,0));
            cv::line(landmark_disp, pt_left, pt_right,cv::Scalar(255,0,0));

        }
    }


    if(!_user_selections.empty())
    {
        cv::Point2f pt1;
        cv::Point2f pt2;

        pt1.x = _start_x;
        pt1.y = _start_y;

        pt2.x = _end_x;
        pt2.y = _end_y;

        cv::line(landmark_disp, pt1, _user_selections.front(), cv::Scalar(0,255,255));
        cv::line(landmark_disp, pt2, _user_selections.back(), cv::Scalar(0,255,255));
    }

    for(int i = 0; i < _user_selections.size(); i++)
    {
        cv::circle(landmark_disp,_user_selections.at(i),1, cv::Scalar(255,0,0));

        if(i > 0)
        {
            cv::line(landmark_disp, _user_selections.at(i - 1), _user_selections.at(i), cv::Scalar(0,255,255));
        }
    }

    _input_map = resize_to_multipler(landmark_disp);

    cv::Point2f start;
    start.x = _start_x * multiplier + multiplier/2;
    start.y = _start_y * multiplier + multiplier/2;

    arrow(_input_map, start, _orientation, cv::Scalar(0,255,0), _arrow_length, _thickness);
    name_landmarks(_input_map);

    ui->img_uInput->setPixmap(QPixmap::fromImage(Mat2QImage(_input_map)));

}

void MainWindow::arrow(cv::Mat &img, cv::Point2f pt, double orientation, cv::Scalar color, int length, int thickness)
{
    cv::Point2f endPoint;

    endPoint.x = (length * sin(orientation)) + pt.x;
    endPoint.y = pt.y - (length * cos(orientation));

    if(endPoint.x >= img.cols)
    {
        endPoint.x = img.cols;
        endPoint.y = (endPoint.x - pt.x)/tan(orientation);
        endPoint.y = pt.y - endPoint.y;
    }

    if(endPoint.x < 0)
    {
        endPoint.x = 0;
        endPoint.y = (endPoint.x - pt.x)/tan(orientation);
        endPoint.y = pt.y - endPoint.y;
    }

    if(endPoint.y >= img.rows)
    {
        endPoint.y = img.rows;
        endPoint.x = (-endPoint.y + pt.y)*tan(orientation);
        endPoint.x = endPoint.x + pt.x;
    }

    if(endPoint.y < 0)
    {
        endPoint.y = 0;
        endPoint.x = (-endPoint.y + pt.y)*tan(orientation);
        endPoint.x = endPoint.x + pt.x;
    }

    cv::line(img, pt, endPoint, color, _thickness);

    cv::Point2f pt_head1;
    cv::Point2f pt_head2;

    double headLength = sqrt(pow(pt.x - endPoint.x,2) + pow(pt.y - endPoint.y,2)) * _arrow_gain;

    double dy = -endPoint.y + pt.y;
    double dx = -endPoint.x + pt.x;

    double phi = -atan2(dy,dx);

    double beta = phi - M_PI/4;
    double zeta = M_PI - phi;
    double alpha = zeta - M_PI/4;


    double x_shift1 = headLength * cos(beta);
    double y_shift1 = headLength * sin(beta);

    double x_shift2 = headLength * cos(alpha);
    double y_shift2 = headLength * sin(alpha);


    pt_head1.x = endPoint.x + x_shift1;
    pt_head1.y = endPoint.y - y_shift1;


    pt_head2.x = endPoint.x - x_shift2;
    pt_head2.y = endPoint.y - y_shift2;

    cv::line(img, endPoint, pt_head1, color, _thickness);
    cv::line(img, endPoint, pt_head2, color, _thickness);

}

MainWindow::~MainWindow()
{
    delete ui;
}

QImage MainWindow::Mat2QImage(cv::Mat const& src)
{
    cv::Mat temp;
    cv::cvtColor(src, temp, CV_BGR2RGB);
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits();

    return dest;
}

void MainWindow::on__bu_find_path_clicked()
{
    update_inputs();

    geometry_msgs::Point start_pt;
    geometry_msgs::Point end_pt;

    bool unset = false;
    if(_start_x == -1)
        unset = true;
    else
        start_pt.x = _start_x;

    if(_start_y == -1)
        unset = true;
    else
        start_pt.y = _start_y;

    if(_end_x == -1)
        unset = true;
    else
       end_pt.x = _end_x;

    if(_end_y == -1)
        unset = true;
    else
        end_pt.y = _end_y;

    if(unset)
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No START and/or END point given, please make sure to provide both"));
        return;
    }

    cv::Mat temp;

    cv::Mat display_path = path_img.clone();

    cv::Point2f pt1;
    cv::Point2f pt2;

    pt1.x = _start_x;
    pt1.y = _start_y;

    pt2.x = _end_x;
    pt2.y = _end_y;

    if(_set_slider)
    {
        double x_pos = ((double)_end_x/(double)path_img.cols)*100.0;
        double y_pos = ((double)_end_y/(double)path_img.rows)*100.0;

        double x_pos_start = ((double)_start_x/(double)path_img.cols)*100.0;
        double y_pos_start = ((double)_start_y/(double)path_img.rows)*100.0;

        ui->slide_shifter->setSliderPosition(x_pos);
        ui->slide_shifter_y->setSliderPosition(y_pos);

        ui->slide_start_x->setSliderPosition(x_pos_start);
        ui->slide_start_y->setSliderPosition(y_pos_start);
    }

    cv::circle(display_path, pt1, 1, cv::Scalar(0,255,0));
    cv::circle(display_path, pt2, 1, cv::Scalar(0,0,255));

    cv::Size size(display_path.cols * multiplier, display_path.rows * multiplier);
    cv::resize(display_path, temp, size);

    ui->img_label->setPixmap(QPixmap::fromImage(Mat2QImage(temp)));

    _start_point_pub.publish(start_pt);
    _end_point_pub.publish(end_pt);

}


void MainWindow::on_in_start_x_editingFinished()
{
//    update_inputs();

    cv::Mat show_img = path_img.clone();

    cv::Point2f start_pt;
    cv::Point2f end_pt;

    if(!_start_x)
        start_pt.x = 0;
    else
        start_pt.x = _start_x;

    if(!_start_y)
        start_pt.y = 0;
    else
        start_pt.y = _start_y;

    if(!_end_x)
        end_pt.x = 0;
    else
       end_pt.x = _end_x;

    if(!_end_y)
        end_pt.y = 0;
    else
        end_pt.y = _end_y;



}

void MainWindow::check_callbacks()
{
    ros::spinOnce();
}

void MainWindow::check_mouse_pos()
{
    if(ui->directions->currentIndex() == 5)
    {
        bool status = false;

        QPoint mouse_point = QCursor::pos();

        mouse_point = mapFromGlobal(mouse_point);

        QPoint widget_point = ui->img_uInput->pos();


        int offset_x = widget_point.x() + 13;
        int offset_y = widget_point.y() + 42;

        int size_x = offset_x + ui->img_uInput->width();
        int size_y = offset_y + ui->img_uInput->height();


        if(mouse_point.x() > offset_x && mouse_point.y() > offset_y && mouse_point.x() < size_x && mouse_point.y() < size_y)
            status = true;
        else
            _button_flag = -1;


        if(_button_flag == 1 && status)
        {
            cv::Point2f pt;

            pt.x = (mouse_point.x() - offset_x)/multiplier;
            pt.y = (mouse_point.y() - offset_y)/multiplier;

            _user_selections.push_back(pt);
            _button_flag = -1;
        }

        general_display((mouse_point.x() - offset_x)/multiplier, (mouse_point.y() - offset_y)/multiplier, status);

        if(status)
        {
            std::string cursor_display = "(";
            cursor_display.append(std::to_string((mouse_point.x() - offset_x)/multiplier));
            cursor_display.append(",");
            cursor_display.append(std::to_string((mouse_point.y() - offset_y)/multiplier));
            cursor_display.append(")");

            ui->label_cursor->setText(cursor_display.c_str());

        }

        std::string total_selections;

        total_selections.append(std::to_string(_user_selections.size()));

        ui->label_selections->setText(total_selections.c_str());

    }

//    _mouse_pressed = false;
}

void MainWindow::on_bu_clear_clicked()
{
    ui->out_ASTART_error->setText(" ");
    ui->out_MAIN_error->setText(" ");

    _a_start_error.clear();
    _main_path_error.clear();
}

void MainWindow::on_checkBox_clicked(bool checked)
{
    if(checked)
        _display_err = true;
    else
        _display_err = false;
}
void MainWindow::on_ch_disp_error_clicked(bool checked)
{
    if(checked)
    {
        _error_show = true;
    }
    else
    {
        _error_show = false;
    }
}

void MainWindow::on_bu_shutdown_clicked()
{
    std_msgs::Bool shutdown;
    shutdown.data = true;

    _shutdown_pub.publish(shutdown);

    MainWindow::close();
}

void MainWindow::on_bu_set_dir_clicked()
{
    _astar_screenshot_dir = QFileDialog::getExistingDirectory(this, "set A* screenshot path");
}

void MainWindow::on_bu_screenshot_aStar_clicked()
{
    std::string file_name = ui->in_screenshot->text().toUtf8().constData();
    std::string dir = _astar_screenshot_dir.toUtf8().constData();

    if(file_name.empty())
        ui->out_ASTART_error->setText("No name given for screenshot");

    if(_resized_a_star.empty())
        ui->out_ASTART_error->setText("No image to take a screenshot of");

    dir.append("/");
    dir.append(file_name.c_str());
    dir.append(".jpg");

    cv::imwrite(dir.c_str(), _resized_a_star);

}

void MainWindow::on_bu_set_dir_directions_clicked()
{
    _directions_screenshot_dir = QFileDialog::getExistingDirectory(this, "Set direction screenshot path");
}

void MainWindow::on_bu_screenshot_directions_clicked()
{
    std::string file_name = ui->in_screenshot_directions->text().toUtf8().constData();
    std::string dir = _directions_screenshot_dir.toUtf8().constData();

    if(file_name.empty())
        ui->out_MAIN_error->setText("No name given for screenshot");

    if(_resized_a_star.empty())
        ui->out_MAIN_error->setText("No image to take a screenshot of");

    dir.append("/");
    dir.append(file_name.c_str());

    std::string directions_img_dir = dir;
    std::string directions_path_dir = dir;
    std::string raw_path = dir;

    directions_img_dir.append(".jpg");
    directions_path_dir.append("(string).txt");
    raw_path.append("(raw_path).jpg");


    cv::imwrite(directions_img_dir.c_str(), _directions_map);
    cv::imwrite(raw_path.c_str(), _resized_a_star);

    std::ofstream file;
    file.open(directions_path_dir.c_str());

    file << _direction_list;

    file.close();


}

void MainWindow::on_bu_user_submit_clicked()
{
    if(_preset < 0)
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("Use a preset please"));
        return;
    }


    std::string name = ui->in_user_name->text().toUtf8().constData();
    std::string directions = ui->in_user_direction->toPlainText().toUtf8().constData();
    std::string feedback = ui->in_user_direction_feedback->toPlainText().toUtf8().constData();

    if(name.empty())
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("I would like to know your name"));
        return;
    }

    if(directions.empty() || directions == "Please input some text")
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No User Directions Given! Please provide some"));
        return;
    }

    if(feedback.empty() || feedback == "Please input some text")
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No Feedback Given! Please provide some."));
        return;
    }


    if(_user_selections.empty())
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No Points Given! Please provide some, the input in on the next tab"));
        return;
    }

    std::string location = QFileDialog::getExistingDirectory(this, "Save user input").toUtf8().constData();

    if(location.empty())
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No save location given!!"));
        return;
    }

    if(_point_generator_selections.empty())
    {
        QMessageBox::information(this, tr("Direction Generator"), tr("No Directions were generated by the software, investigate this"));
        return;
    }



    std::ofstream file;

    location.append("/");
    location.append(name);

    if(!QDir().exists(location.c_str()))
        QDir().mkdir(location.c_str());

    std::string untainted_location = "Data has been saved to: \n";
    untainted_location.append(location);

    location.append("/");
    location.append(std::to_string(_preset));

    if(!QDir().exists(location.c_str()))
        QDir().mkdir(location.c_str());


    location.append("/");
    std::string presetText = location;
    std::string presetPath = location;
    location.append(name);

    std::string raw_path = location;
    std::string direction_path = location;
    std::string direction_list = location;
    std::string landmarks = location;
    std::string userpath = location;
    std::string metadata = location;
    std::string userfeedback = location;


    raw_path.append("(raw_path).jpg");
    direction_path.append("(direction_path).jpg");
    direction_list.append("(listed_paths).txt");
    landmarks.append("(landmarks).jpg");
    userpath.append("(user_path).jpg");
    metadata.append("(metadata).txt");
    userfeedback.append("(userfeedback).txt");
    presetPath.append(std::to_string(_preset));
    presetPath.append(".jpg");
    presetText.append(std::to_string(_preset));
    presetText.append(".txt");

    cv::imwrite(raw_path.c_str(), _resized_a_star);
    cv::imwrite(direction_path.c_str(), _directions_map);
    cv::imwrite(landmarks.c_str(), _landmark_map);
    cv::imwrite(userpath.c_str(), _input_map);
    cv::imwrite(presetPath.c_str(), _input_map);

    file.open(presetText);
    file << directions;

    file.close();


    //Write Metadata

    file.open(metadata);
    file << "Name: " << name << std::endl;

    file << "start (x,y): " << "(" << _start_x << "," << _start_y << ")" << std::endl;
    file << "end (x,y): " << "(" << _end_x << "," << _end_y << ")" << std::endl;

    file.close();


    //Write User feedback
    file.open(userfeedback);
    file << feedback;
    file.close();


    //Write user directions
    location.append(".txt");

    file.open(location);

    file << directions << std::endl << "USER POINTS\n";
    file << _start_x << "," << _start_y << std::endl;

    for (int i = 0; i < _user_selections.size(); i++)
    {
        file << std::to_string(_user_selections.at(i).x);
        file << ",";
        file << std::to_string(_user_selections.at(i).y) << std::endl;
    }
    file << _end_x << "," << _end_y << std::endl;

    file.close();


    //Write My Directions
    file.open(direction_list);

    file << _direction_list << "\nDirection Points\n";

    for (int i = 0; i < _point_generator_selections.size(); i++)
    {
        file << std::to_string(_point_generator_selections.at(i).x);
        file << ",";
        file << std::to_string(_point_generator_selections.at(i).y) << std::endl;
    }

    file.close();

    QMessageBox::information(this, tr("Direction Generator"), tr(untainted_location.c_str()));

}

void MainWindow::on_slide_shifter_sliderMoved(int position)
{

    double percentage = ((double)position)/100.0;

    int value = path_img.cols * percentage;

    ui->in_end_x->setText(std::to_string(value).c_str());

    _set_slider = false;
    on__bu_find_path_clicked();
    _set_slider = true;

}


void MainWindow::on_slide_shifter_y_sliderMoved(int position)
{
    double percentage = ((double)position)/100.0;

    int value = path_img.rows * percentage;

    ui->in_end_y->setText(std::to_string(value).c_str());

    _set_slider = false;
    on__bu_find_path_clicked();
    _set_slider = true;
}

void MainWindow::on_slide_start_x_sliderMoved(int position)
{
    double percentage = ((double)position)/100.0;

    int value = path_img.cols * percentage;

    ui->in_start_x->setText(std::to_string(value).c_str());

    _set_slider = false;
    on__bu_find_path_clicked();
    _set_slider = true;
}

void MainWindow::on_slide_start_y_sliderMoved(int position)
{
    double percentage = ((double)position)/100.0;

    int value = path_img.rows * percentage;

    ui->in_start_y->setText(std::to_string(value).c_str());

    _set_slider = false;
    on__bu_find_path_clicked();
    _set_slider = true;
}

void MainWindow::mouseReleaseEvent ( QMouseEvent * event )
{
    _button_flag = event->button();
}


void MainWindow::on_bu_clear_uinputs_clicked()
{
    _user_selections.clear();
}

void MainWindow::on_bu_undo_uinput_clicked()
{
    if(_user_selections.empty())
    {
        QMessageBox::information(this,tr("User Input"),tr("Nothing to undo"));
        return;
    }

    _user_selections.pop_back();
}

void MainWindow::on_slide_orientation_sliderMoved(int position)
{
    double percentage = (double)position/100.0;

    double range = 2*M_PI;

    _orientation = range * percentage;

    ui->in_orientation->setText(std::to_string(_orientation).c_str());

    std_msgs::Float32 msg;
    msg.data = _orientation;

    _orientation_pub.publish(msg);

//    std::cout << "orientation***: " << _orientation << " pos: " << position << std::endl;
}

void MainWindow::on_slide_arrow_length_sliderMoved(int position)
{
    double percentage = (double)position/100.0;

    double range = 100;

    _arrow_length = range * percentage + 1;

//    std::cout << "length***: " << _arrow_length << std::endl;
}

void MainWindow::on_slide_arrow_thickness_sliderMoved(int position)
{
    double percentage = (double)position/100.0;

    double range = 15;

    _thickness = range * percentage + 1;
}

void MainWindow::on_in_orientation_returnPressed()
{
    std::string orientation = ui->in_orientation->text().toUtf8().constData();

    _orientation = std::stod(orientation);

    std_msgs::Float32 msg;
    msg.data = _orientation;

    ui->slide_orientation->setSliderPosition((_orientation/6.23)*100);

    _orientation_pub.publish(msg);
}

void MainWindow::on_buP1_clicked()
{
    _preset = 1;
   int startX = 5;
   int startY = 5;

   int endX = 35;
   int endY = 10;

   _orientation = 0;

   ui->in_start_x->setText(std::to_string(startX).c_str());
   ui->in_start_y->setText(std::to_string(startY).c_str());
   ui->in_end_x->setText(std::to_string(endX).c_str());
   ui->in_end_y->setText(std::to_string(endY).c_str());
   ui->in_orientation->setText(std::to_string(_orientation).c_str());

   _set_slider = true;
   on__bu_find_path_clicked();
   _set_slider = true;

   on_in_orientation_returnPressed();
}

void MainWindow::on_buP2_clicked()
{
    _preset = 2;
    int startX = 5;
    int startY = 5;

    int endX = 51;
    int endY = 43;

    _orientation = 0;

    ui->in_start_x->setText(std::to_string(startX).c_str());
    ui->in_start_y->setText(std::to_string(startY).c_str());
    ui->in_end_x->setText(std::to_string(endX).c_str());
    ui->in_end_y->setText(std::to_string(endY).c_str());
    ui->in_orientation->setText(std::to_string(_orientation).c_str());

    _set_slider = true;
    on__bu_find_path_clicked();
    _set_slider = true;

    on_in_orientation_returnPressed();
}

void MainWindow::on_buP3_clicked()
{
    _preset = 3;
    int startX = 34;
    int startY = 5;

    int endX = 51;
    int endY = 43;

    _orientation = 3.58;

    ui->in_start_x->setText(std::to_string(startX).c_str());
    ui->in_start_y->setText(std::to_string(startY).c_str());
    ui->in_end_x->setText(std::to_string(endX).c_str());
    ui->in_end_y->setText(std::to_string(endY).c_str());
    ui->in_orientation->setText(std::to_string(_orientation).c_str());

    _set_slider = true;
    on__bu_find_path_clicked();
    _set_slider = true;

    on_in_orientation_returnPressed();
}

void MainWindow::on_buP4_clicked()
{
    _preset = 4;
    int startX = 34;
    int startY = 46;

    int endX = 92;
    int endY = 43;

    _orientation = 1.88;

    ui->in_start_x->setText(std::to_string(startX).c_str());
    ui->in_start_y->setText(std::to_string(startY).c_str());
    ui->in_end_x->setText(std::to_string(endX).c_str());
    ui->in_end_y->setText(std::to_string(endY).c_str());
    ui->in_orientation->setText(std::to_string(_orientation).c_str());

    _set_slider = true;
    on__bu_find_path_clicked();
    _set_slider = true;

    on_in_orientation_returnPressed();
}

void MainWindow::on_buP5_clicked()
{
    _preset = 5;
    int startX = 5;
    int startY = 5;

    int endX = 90;
    int endY = 43;

    _orientation = 3.76;

    ui->in_start_x->setText(std::to_string(startX).c_str());
    ui->in_start_y->setText(std::to_string(startY).c_str());
    ui->in_end_x->setText(std::to_string(endX).c_str());
    ui->in_end_y->setText(std::to_string(endY).c_str());
    ui->in_orientation->setText(std::to_string(_orientation).c_str());

    _set_slider = true;
    on__bu_find_path_clicked();
    _set_slider = true;

    on_in_orientation_returnPressed();
}
