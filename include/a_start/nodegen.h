#ifndef NODEGEN_H
#define NODEGEN_H

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

class NodeGen
{

private:
    cv::Mat img;

public:
    NodeGen();

    void generate_nodes(char location[], char file_name[]);
};

#endif // NODEGEN_H
