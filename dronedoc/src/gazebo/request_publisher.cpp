#include <iostream>
#include <math.h>
#include <deque>
#include <sdf/sdf.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

#include "map_request.pb.h"
#include "vector2d.pb.h"

#include <ros/ros.h>
#include <boost/filesystem/path.hpp>

using namespace std;

bool createVector(const char * vectorString,
                       gazebo::msgs::Vector2d* origin)
{
    string cornersStr = vectorString;
    size_t opening=0;
    size_t closing=0;

    opening = cornersStr.find('(', closing);
    closing = cornersStr.find(')', opening);
    if (opening == string::npos || closing == string::npos)
    {
        ROS_ERROR("Poorly formed string: %s", cornersStr.c_str());
        ROS_ERROR("( found at: %ld ) found at: %ld", opening, closing);
        return false;
    }
    string oneCornerStr = cornersStr.substr(opening + 1, closing - opening - 1);
    size_t commaLoc = oneCornerStr.find(",");
    string x = oneCornerStr.substr(0,commaLoc);
    string y = oneCornerStr.substr(commaLoc + 1, oneCornerStr.length() - commaLoc);
    origin->set_x(atof(x.c_str()));
    origin->set_y(atof(y.c_str()));

    return true;
}

int main(int argc, char * argv[])
{
    if (argc > 6)
    {
        dronedoc::msgs::MapRequest request;
        gazebo::msgs::Vector2d* origin = request.mutable_origin();

        if (!createVector(argv[3], origin))
        {
            ROS_ERROR("Failed to create rasterize region.");
            return -1;
        }


        request.set_height(atof(argv[1]));
        request.set_resolution(atof(argv[2]));
        request.set_map_width(atof(argv[4]));
        request.set_map_height(atof(argv[5]));

        std::string filename;
        if(argc < 7){
            filename = "map.png";
        } else {
            filename = argv[6];
        }
        boost::filesystem::path fname(filename);
        fname = boost::filesystem::absolute(fname);
        request.set_filename(fname.string());

        if (argc == 8)
        {
            request.set_threshold(atoi(argv[7]));
        }

        gazebo::transport::init();
        gazebo::transport::run();
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init("default");


        ROS_INFO("\nRequest: \n"
                 " origin.x: %f origin.y: %f\n"
                 " map_width: %f map_height: %f\n"
                 " Height: %f\n Resolution: %f\n Filename: %s\n Threshold: %d",
                 request.origin().x(),
                 request.origin().y(),
                 request.map_width(),
                 request.map_height(),
                 request.height(),
                 request.resolution(),
                 request.filename().c_str(),
                 request.threshold());

        gazebo::transport::PublisherPtr imagePub =
                node->Advertise<dronedoc::msgs::MapRequest>("~/collision_map/command");
        imagePub->WaitForConnection();
        imagePub->Publish(request);

        gazebo::transport::fini();
        return 0;
    }

    std::cout << "Usage: " <<
                 "rosrun dronedoc request_publisher a1 a2 a3 a4 a5 [a6 a7]\n" <<
                 "\ta1: height\n" <<
                 "\ta2: resolution\n" <<
                 "\ta3: \"(origin.x, origin.y)\"\n" <<
                 "\t  Origin is the point on lower left corner of map image\n"
                 "\ta4: map_width\n" <<
                 "\ta5: map_height\n" <<
                 "\ta6: filename\t[default=\"map\"]\n" <<
                 "\ta7: threshold\t[default=255]\n" << std::endl;

    return -1;
}