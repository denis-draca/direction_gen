#include "a_start/mainwindow.h"
#include "QApplication"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualiser");
    ros::NodeHandle n;


    QApplication app(argc, argv);

    MainWindow main(n);


    main.show();


    app.exec();
}
