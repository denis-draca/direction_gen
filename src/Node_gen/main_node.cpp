#include "a_start/nodegen.h"


int main(int argc, char** argv)
{
    NodeGen node_gen;

    char img_name[] = "/home/denis/catkin_ws/src/a_start/data/map4.png";
    char file_name[] = "/home/denis/catkin_ws/src/a_start/data/nodes.yaml";

    node_gen.generate_nodes(img_name, file_name);
}
