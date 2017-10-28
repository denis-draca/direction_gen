#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QGraphicsPixmapItem>
#include <iostream>
#include <QGraphicsScene>
#include <QWheelEvent>
#include <geometry_msgs/Point.h>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include <QValidator>
#include <QTimer>
#include "geometry_msgs/PoseArray.h"
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <fstream>
#include <QMessageBox>
#include <QCursor>
#include <QPoint>
#include "std_msgs/Float32.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private: //methods
    explicit MainWindow(QWidget *parent = 0);

    QImage Mat2QImage(cv::Mat const& src);
    cv::Mat QImage2Mat(QImage const& src);
    cv::Mat resize_to_multipler(cv::Mat &small_img);

    void path_callback(const geometry_msgs::PoseArrayConstPtr &msg);
    cv::Mat path_to_img(geometry_msgs::PoseArray &path);

    void update_inputs();

    void setup_landmarks();

    void display_landmarks();

    void a_start_error_callback(const std_msgs::String &msg);
    void main_path_error_callback(const std_msgs::String &msg);
    void directions_callback(const std_msgs::String &msg);
    void direction_pts_callback(const geometry_msgs::PoseArrayConstPtr &msg);
    void name_landmarks(cv::Mat &img);

    void general_display(int x, int y, bool draw);
    void arrow(cv::Mat &img, cv::Point2f pt, double orientation, cv::Scalar color, int length = 1, int thickness = 1);


public:
    explicit MainWindow(ros::NodeHandle &n, QWidget *parent = 0);
    ~MainWindow();


private slots:
    void on__bu_find_path_clicked();
    void on_checkBox_clicked(bool checked);

    void on_in_start_x_editingFinished();

    void check_callbacks();
    void check_mouse_pos();
    void mouseReleaseEvent(QMouseEvent * event);

    void on_bu_clear_clicked();

private slots:
    void on_ch_disp_error_clicked(bool checked);

    void on_bu_shutdown_clicked();

    void on_bu_set_dir_clicked();

    void on_bu_screenshot_aStar_clicked();

    void on_bu_set_dir_directions_clicked();

    void on_bu_screenshot_directions_clicked();

    void on_bu_user_submit_clicked();

    void on_slide_shifter_sliderMoved(int position);


    void on_slide_shifter_y_sliderMoved(int position);

    void on_slide_start_x_sliderMoved(int position);

    void on_slide_start_y_sliderMoved(int position);

    void on_bu_clear_uinputs_clicked();

    void on_bu_undo_uinput_clicked();

    void on_slide_orientation_sliderMoved(int position);

    void on_slide_arrow_length_sliderMoved(int position);

    void on_slide_arrow_thickness_sliderMoved(int position);

    void on_in_orientation_returnPressed();

    void on_buP1_clicked();

    void on_buP2_clicked();

    void on_buP3_clicked();

    void on_buP4_clicked();

    void on_buP5_clicked();

private: // members
    Ui::MainWindow *ui;

    ros::NodeHandle _n;

    cv::Mat path_img;
    cv::Mat _resized_a_star;
    cv::Mat _landmark_map;
    cv::Mat _directions_map;
    cv::Mat _input_map;

    int multiplier;

    int _start_x;
    int _start_y;
    int _end_x;
    int _end_y;
    int _button_flag;
    int _arrow_length;
    int _thickness;
    int _preset;

    double _orientation;
    double _arrow_gain;


    QTimer *_timer;
    QTimer *_timer_2;

    ros::Subscriber _path_sub;
    ros::Subscriber _error_a_start_sub;
    ros::Subscriber _error_main_sub;
    ros::Subscriber _path_directions_sub;
    ros::Subscriber _path_points_sub;

    ros::Publisher _start_point_pub;
    ros::Publisher _end_point_pub;
    ros::Publisher _shutdown_pub;
    ros::Publisher _orientation_pub;

    std::vector<std::pair<int,int>> _landmarks;
    std::vector<cv::Point2f> _user_selections;
    std::vector<cv::Point2f> _point_generator_selections;
    std::vector<std::string> _landmark_names;

    std::string _a_start_error;
    std::string _main_path_error;
    std::string _direction_list;

    QString _astar_screenshot_dir;
    QString _directions_screenshot_dir;
    QString _landmarks_screenshot_dir;

    QMessageBox _error_box;

    bool _error_show;
    bool _display_err;
    bool _set_slider;
    bool _landmark_update_done;
    bool _mouse_pressed;

};

#endif // MAINWINDOW_H
