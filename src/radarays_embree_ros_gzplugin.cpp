#include <radarays_gazebo_plugins/radarays_embree_ros_gzplugin.h>

#include <iostream>
#include <cv_bridge/cv_bridge.h>


namespace gazebo
{

RadaRaysEmbreeROS::RadaRaysEmbreeROS()
:Base()
{
    // RCLCPP_INFO(..., "[RadaRaysEmbreeROS] Construct");
}

RadaRaysEmbreeROS::~RadaRaysEmbreeROS()
{
    // std::cout << "[RadaRaysEmbreeROS] Destroy" << std::endl;
}

void RadaRaysEmbreeROS::parseOutputs(sdf::ElementPtr outputs)
{
}

void RadaRaysEmbreeROS::Load(
    sensors::SensorPtr _sensor,
    sdf::ElementPtr _sdf)
{
    m_sdf = _sdf;

    m_radarays_sensor =
        std::dynamic_pointer_cast<sensors::RadaRaysEmbree>(_sensor);

    if (!m_radarays_sensor)
    {
        gzerr << "RadaRaysEmbreeROS requires a RadaRaysEmbree sensor.\n";
        return;
    }

    if (!rclcpp::ok())
    {
        RCLCPP_FATAL(rclcpp::get_logger("radarays"),
            "A ROS2 node for Gazebo has not been initialized, unable to load plugin. "
            "Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package.");
        return;
    }

    m_nh = gazebo_ros::Node::Get(_sdf);

    m_frame_id = m_sdf->Get<std::string>("frame");
    m_topic = m_sdf->Get<std::string>("topic");

    m_pub_img = image_transport::create_publisher(m_nh.get(), m_topic);

    // Connect to the sensor update event.
    m_update_conn = m_radarays_sensor->ConnectUpdated(
        std::bind(&RadaRaysEmbreeROS::OnUpdate, this));

    RCLCPP_INFO(m_nh->get_logger(), "[RadaRaysEmbreeROS] Loaded.");
}

void RadaRaysEmbreeROS::OnUpdate()
{
    common::Time stamp_gz = m_radarays_sensor->stamp();

    // Publish polar image
    auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "mono8",
                m_radarays_sensor->m_polar_image).toImageMsg();

    msg->header.stamp.sec = stamp_gz.sec;
    msg->header.stamp.nanosec = stamp_gz.nsec;
    msg->header.frame_id = m_frame_id;

    m_pub_img.publish(*msg);
}

GZ_REGISTER_SENSOR_PLUGIN(RadaRaysEmbreeROS)

} // namespace gazebo
