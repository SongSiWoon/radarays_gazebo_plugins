#ifndef GAZEBO_RADARAYS_EMBREE_ROS_PLUGIN_H
#define GAZEBO_RADARAYS_EMBREE_ROS_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <radarays_gazebo_plugins/radarays_embree_gzplugin.h>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <image_transport/image_transport.hpp>

#include <memory>
#include <unordered_map>

namespace rm = rmagine;

namespace gazebo
{

class RadaRaysEmbreeROS : public SensorPlugin
{
public:
    using Base = SensorPlugin;

    RadaRaysEmbreeROS();

    virtual ~RadaRaysEmbreeROS();

    virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:

    void parseOutputs(sdf::ElementPtr outputs);

    virtual void OnUpdate();

    sensors::RadaRaysEmbreePtr m_radarays_sensor;

    event::ConnectionPtr m_update_conn;

    gazebo_ros::Node::SharedPtr m_nh;

    image_transport::Publisher m_pub_img;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub_pc;

    std::string m_frame_id;
    std::string m_topic;
    std::string m_pc_topic;

    float m_pc_fov_min = -M_PI;
    float m_pc_fov_max = M_PI;

    sdf::ElementPtr m_sdf;
};

} // namespace gazebo

#endif // GAZEBO_RADARAYS_EMBREE_ROS_PLUGIN_H
