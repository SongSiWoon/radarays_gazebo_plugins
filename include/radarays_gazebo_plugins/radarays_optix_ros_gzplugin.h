#ifndef GAZEBO_RADARAYS_OPTIX_ROS_PLUGIN_H
#define GAZEBO_RADARAYS_OPTIX_ROS_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <radarays_gazebo_plugins/radarays_optix_gzplugin.h>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>

#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>

#include <memory>
#include <unordered_map>

namespace rm = rmagine;

namespace gazebo
{

class RadaRaysOptixROS : public SensorPlugin
{
public:
    using Base = SensorPlugin;

    RadaRaysOptixROS();

    virtual ~RadaRaysOptixROS();

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:

    void parseOutputs(sdf::ElementPtr outputs);

    virtual void OnUpdate();

    sensors::RadaRaysOptixPtr m_radarays_sensor;

    event::ConnectionPtr m_update_conn;

    gazebo_ros::Node::SharedPtr m_nh;

    image_transport::Publisher m_pub_img;

    std::string m_frame_id;
    std::string m_topic;

    sdf::ElementPtr m_sdf;
};

} // namespace gazebo

#endif // GAZEBO_RADARAYS_OPTIX_ROS_PLUGIN_H
