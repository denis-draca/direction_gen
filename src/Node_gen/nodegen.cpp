#include "a_start/nodegen.h"

NodeGen::NodeGen()
{
}

void NodeGen::generate_nodes(char location[], char file_name[])
{
    img = cv::imread(location, CV_LOAD_IMAGE_GRAYSCALE);

    std::ofstream file;

    file.open(file_name, std::ios::trunc);

    std::vector<int> node_list;

    for(int y = 0; y < img.rows; y++)
    {
        for(int x = 0; x < img.cols; x++)
        {
            uchar value = img.at<uchar>(y,x);

            if(value <= 100)
            {
                continue;
            }

            unsigned int node_id = (y*img.cols) + x;
            node_list.push_back(node_id);

            file << "node_" << node_id << ":" << std::endl;
            file << " x: " << x << std::endl;
            file << " y: " << y << std::endl;

            std::vector<int> connected_nodes;

            cv::Mat img_temp;

            img_temp = img.clone();
            cv::cvtColor(img_temp, img_temp, cv::COLOR_GRAY2BGR);

            cv::Vec3b centre;
            centre[0] = 0;
            centre[1] = 0;
            centre[2] = 255;

            img_temp.at<cv::Vec3b>(y,x) = centre;

            for(int y_temp = y - 1; y_temp < y + 2; y_temp++)
            {
                for(int x_temp = x - 1; x_temp < x + 2; x_temp++)
                {
                   if(x_temp < 0 || x_temp > img.cols || y_temp < 0 || y_temp > img.rows)
                   {
                       continue;
                   }

                   if((y_temp * img.cols) + x_temp == node_id)
                   {
                       continue;
                   }

                   uchar connected_value = img.at<uchar>(y_temp, x_temp);

                   if(connected_value > 100)
                   {
                       int connected_id = (y_temp * img.cols) + x_temp;

                       connected_nodes.push_back(connected_id);

                       cv::Vec3b color;
                       color[0] = 0;
                       color[1] = 255;
                       color[2] = 0;

                       img_temp.at<cv::Vec3b>(y_temp,x_temp) = color;
                   }

                }
            }

            cv::namedWindow("hi", cv::WINDOW_NORMAL);
            cv::imshow("hi", img_temp);
            cv::waitKey(3);

            if(connected_nodes.size() == 1)
            {
                file << " connected_nodes: [" << connected_nodes.front() << "]" << std::endl;
            }
            else if(connected_nodes.size() > 1)
            {
                file << " connected_nodes: [" << connected_nodes.front();

                for(int pos = 1; pos < connected_nodes.size(); pos++)
                {
                    file << "," << connected_nodes.at(pos);
                }

                file << "]" << std::endl;
            }
            else
            {
                file << " connected_nodes: []" << std::endl;
            }
        }
    }

    file << "node_list: [" << node_list.front();

    for(int pos = 1; pos < node_list.size(); pos++)
    {
        file << "," << node_list.at(pos);
    }

    file << "]" << std::endl;

    file.close();


}
