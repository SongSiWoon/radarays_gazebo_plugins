#include <radarays_gazebo_plugins/radarays_optix_ros_gzplugin.h>

#include <iostream>
#include <cv_bridge/cv_bridge.h>


namespace gazebo
{

RadaRaysOptixROS::RadaRaysOptixROS()
:Base()
{
    // RCLCPP_INFO(..., "[RadaRaysOptixROS] Construct");
}

RadaRaysOptixROS::~RadaRaysOptixROS()
{
    // std::cout << "[RadaRaysOptixROS] Destroy" << std::endl;
}

void RadaRaysOptixROS::parseOutputs(sdf::ElementPtr outputs)
{
}

void RadaRaysOptixROS::Load(
    sensors::SensorPtr _sensor,
    sdf::ElementPtr _sdf)
{
    m_sdf = _sdf;

    m_radarays_sensor =
        std::dynamic_pointer_cast<sensors::RadaRaysOptix>(_sensor);

    if (!m_radarays_sensor)
    {
        gzerr << "RadaRaysOptixROS requires a RadaRaysOptix sensor.\n";
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
        std::bind(&RadaRaysOptixROS::OnUpdate, this));

    RCLCPP_INFO(m_nh->get_logger(), "[RadaRaysOptixROS] Loaded.");
}

void RadaRaysOptixROS::OnUpdate()
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

GZ_REGISTER_SENSOR_PLUGIN(RadaRaysOptixROS)

} // namespace gazebo
