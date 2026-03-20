#include <radarays_gazebo_plugins/radarays_embree_ros_gzplugin.h>

#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>


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

    // PointCloud2 publisher (optional SDF element)
    if(m_sdf->HasElement("pointcloud_topic"))
    {
        m_pc_topic = m_sdf->Get<std::string>("pointcloud_topic");
    } else {
        m_pc_topic = "radar/points";
    }
    m_pub_pc = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>(m_pc_topic, 10);

    // PointCloud2 FOV filter (optional)
    if(m_sdf->HasElement("pc_fov_min"))
        m_pc_fov_min = m_sdf->Get<float>("pc_fov_min");
    if(m_sdf->HasElement("pc_fov_max"))
        m_pc_fov_max = m_sdf->Get<float>("pc_fov_max");

    RCLCPP_INFO(m_nh->get_logger(), "[RadaRaysEmbreeROS] PointCloud2 topic: %s, FOV: [%.2f, %.2f] rad",
        m_pc_topic.c_str(), m_pc_fov_min, m_pc_fov_max);

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

    // Publish PointCloud2 with FOV filtering
    auto& pts = m_radarays_sensor->m_radar_points;
    if(!pts.empty() && m_pub_pc->get_subscription_count() > 0)
    {
        // Filter points by FOV (azimuth angle from x,y)
        std::vector<radarays_gazebo_plugins::RadarPoint> filtered;
        filtered.reserve(pts.size());
        for(auto& pt : pts)
        {
            float azimuth = atan2(pt.y, pt.x);
            if(azimuth >= m_pc_fov_min && azimuth <= m_pc_fov_max)
                filtered.push_back(pt);
        }

        if(!filtered.empty())
        {
            sensor_msgs::msg::PointCloud2 pc_msg;
            pc_msg.header.stamp.sec = stamp_gz.sec;
            pc_msg.header.stamp.nanosec = stamp_gz.nsec;
            pc_msg.header.frame_id = m_frame_id;
            pc_msg.height = 1;
            pc_msg.width = filtered.size();
            pc_msg.is_dense = true;
            pc_msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modifier(pc_msg);
            modifier.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
            modifier.resize(filtered.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_i(pc_msg, "intensity");

            for(size_t i = 0; i < filtered.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
            {
                *iter_x = filtered[i].x;
                *iter_y = filtered[i].y;
                *iter_z = filtered[i].z;
                *iter_i = filtered[i].intensity;
            }

            m_pub_pc->publish(pc_msg);
        }
    }
}

GZ_REGISTER_SENSOR_PLUGIN(RadaRaysEmbreeROS)

} // namespace gazebo
