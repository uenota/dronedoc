#include <iostream>
#include <math.h>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "map_request.pb.h"

#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <fstream>
#include <boost/filesystem.hpp>

namespace gazebo
{
typedef const boost::shared_ptr<const dronedoc::msgs::MapRequest> MapRequestPtr;

class MapCreator : public WorldPlugin
{
  transport::NodePtr node;
  transport::PublisherPtr imagePub;
  transport::SubscriberPtr commandSubscriber;
  physics::WorldPtr world;

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    node = transport::NodePtr(new transport::Node());
    world = _parent;
    // Initialize the node with the world name
    node->Init(world->GetName());
    ROS_INFO("Subscribing to: ~/collision_map/command");
    commandSubscriber = node->Subscribe("~/collision_map/command",
      &MapCreator::create, this);
    imagePub = node->Advertise<msgs::Image>("~/collision_map/image");
  }

  public: void create(MapRequestPtr &msg)
  {
    ROS_INFO("Received message");

    ROS_INFO("Creating collision map with origin at (%f, %f), width %f and height %f "
              "with collision projected from z = %f\n"
              "Resolution = %f m\n"
              "Occupied spaces will be filled with: %d",
             msg->origin().x(),
             msg->origin().y(),
             msg->map_width(),
             msg->map_height(),
             msg->height(),
             msg->resolution(),
             msg->threshold());

    int count_vertical = msg->map_height() / msg->resolution();
    int count_horizontal = msg->map_width() / msg->resolution();

    if (count_vertical == 0 || count_horizontal == 0)
    {
      ROS_ERROR("Image has a zero dimensions, check coordinates");
      return;
    }
    double x,y;

    boost::gil::gray8_pixel_t fill(255-msg->threshold());
    boost::gil::gray8_pixel_t blank(255);
    boost::gil::gray8_image_t image(count_horizontal, count_vertical);

    double dist;
    std::string entityName;
    math::Vector3 start, end;
    start.z = msg->height();
    end.z = 0.001;

    gazebo::physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    ROS_INFO("Rasterizing model and checking collisions");
    boost::gil::fill_pixels(image._view, blank);

    int progress = 0;
    for (int i = 0; i < count_vertical; ++i)
    {
      if(progress != std::round(i * 100.0 / count_vertical)){
        ROS_INFO("%d%% complete", progress);
      }
      progress = std::round(i * 100.0 / count_vertical);
      x = msg->origin().x();
      y = i * msg->resolution() + msg->origin().y();
      for (int j = 0; j < count_horizontal; ++j)
      {
        x += msg->resolution();

        start.x = end.x = x;
        start.y = end.y = y;
        ray->SetPoints(start, end);
        ray->GetIntersection(dist, entityName);
        if (!entityName.empty())
        {
          image._view(j,(count_vertical-1)-i) = fill;
        }
      }
    }

    boost::filesystem::path fname(msg->filename());
    fname = boost::filesystem::absolute(fname);

    ROS_INFO("Completed calculations, writing to image");
    if (!msg->filename().empty())
    {
      boost::gil::gray8_view_t view = image._view;
      boost::gil::png_write_view(fname.string(), view);
    }

    ROS_INFO("Writing map info");

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "image";
    out << YAML::Value << fname.filename().string();
    out << YAML::Key << "resolution";
    out << YAML::Value << msg->resolution();
    out << YAML::Key << "negate";
    out << YAML::Value << 0;
    out << YAML::Key << "free_thresh";
    out << YAML::Value << 0.196;
    out << YAML::Key << "occupied_thresh";
    out << YAML::Value << 0.65;
    out << YAML::Key << "origin";
    out << YAML::Value;
    out << YAML::Flow;
    out << YAML::BeginSeq << msg->origin().x() << msg->origin().y() << 0.00 << YAML::EndSeq;

    std::ofstream yaml_out(fname.replace_extension(".yaml").string());
    yaml_out << out.c_str();
    yaml_out.close();

    ROS_INFO("Map built successfully");
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(MapCreator)
}