#include <radarays_gazebo_plugins/radarays_embree_ros_gzplugin.h>

#include <iostream>
#include <cstring>
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

    // name of the Doppler field, "v_doppler" matches ti_mmwave_ros2_pkg
    if(m_sdf->HasElement("doppler_field"))
        m_doppler_field = m_sdf->Get<std::string>("doppler_field");

    // Heat maps: <pointcloud_topic>/../heatmap_range_doppler and heatmap_range_azimuth
    {
        std::string base = m_pc_topic;
        const size_t slash = base.rfind('/');
        base = (slash == std::string::npos) ? std::string() : base.substr(0, slash + 1);
        std::string rd_topic = base + "heatmap_range_doppler";
        std::string ra_topic = base + "heatmap_range_azimuth";
        if(m_sdf->HasElement("heatmap_range_doppler_topic"))
            rd_topic = m_sdf->Get<std::string>("heatmap_range_doppler_topic");
        if(m_sdf->HasElement("heatmap_range_azimuth_topic"))
            ra_topic = m_sdf->Get<std::string>("heatmap_range_azimuth_topic");
        m_pub_heat_rd = m_nh->create_publisher<sensor_msgs::msg::Image>(rd_topic, 10);
        m_pub_heat_ra = m_nh->create_publisher<sensor_msgs::msg::Image>(ra_topic, 10);
    }

    // GT PointCloud2 publisher
    m_pc_gt_topic = m_pc_topic + "_gt";
    m_pub_pc_gt = m_nh->create_publisher<sensor_msgs::msg::PointCloud2>(m_pc_gt_topic, 10);

    RCLCPP_INFO(m_nh->get_logger(), "[RadaRaysEmbreeROS] PointCloud2 topic: %s, GT: %s, FOV: [%.2f, %.2f] rad",
        m_pc_topic.c_str(), m_pc_gt_topic.c_str(), m_pc_fov_min, m_pc_fov_max);

    // Connect to the sensor update event.
    m_update_conn = m_radarays_sensor->ConnectUpdated(
        std::bind(&RadaRaysEmbreeROS::OnUpdate, this));

    RCLCPP_INFO(m_nh->get_logger(), "[RadaRaysEmbreeROS] Loaded.");
}

void RadaRaysEmbreeROS::publishHeatmap(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub,
    const radarays_gazebo_plugins::RadarDetector::Heatmap& map,
    const std_msgs::msg::Header& header)
{
    if(map.rows <= 0 || map.cols <= 0
        || map.data.size() != static_cast<size_t>(map.rows) * map.cols)
    {
        return;
    }
    sensor_msgs::msg::Image img;
    img.header = header;
    img.height = map.rows;
    img.width = map.cols;
    img.encoding = "32FC1";
    img.is_bigendian = false;
    img.step = map.cols * sizeof(float);
    img.data.resize(map.data.size() * sizeof(float));
    std::memcpy(img.data.data(), map.data.data(), img.data.size());
    pub->publish(img);
}

void RadaRaysEmbreeROS::OnUpdate()
{
    common::Time stamp_gz = m_radarays_sensor->stamp();

    // Heat maps: publish what the last frame computed, then tell the
    // detector whether the next frame needs them at all
    {
        auto& det = m_radarays_sensor->detector();
        std_msgs::msg::Header header;
        header.stamp.sec = stamp_gz.sec;
        header.stamp.nanosec = stamp_gz.nsec;
        header.frame_id = m_frame_id;
        const bool want_rd = m_pub_heat_rd->get_subscription_count() > 0;
        const bool want_ra = m_pub_heat_ra->get_subscription_count() > 0;
        if(want_rd && det.want_range_doppler) publishHeatmap(m_pub_heat_rd, det.range_doppler, header);
        if(want_ra && det.want_range_azimuth) publishHeatmap(m_pub_heat_ra, det.range_azimuth, header);
        det.want_range_doppler = want_rd;
        det.want_range_azimuth = want_ra;
    }

    // Publish polar image, the sensor only renders it while somebody listens
    const bool image_wanted = m_pub_img.getNumSubscribers() > 0;
    if(image_wanted && m_radarays_sensor->m_image_requested
        && !m_radarays_sensor->m_polar_image.empty())
    {
        auto msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(),
                    "mono8",
                    m_radarays_sensor->m_polar_image).toImageMsg();

        msg->header.stamp.sec = stamp_gz.sec;
        msg->header.stamp.nanosec = stamp_gz.nsec;
        msg->header.frame_id = m_frame_id;

        m_pub_img.publish(*msg);
    }
    m_radarays_sensor->m_image_requested = image_wanted;

    // Publish PointCloud2 with FOV filtering
    auto& pts = m_radarays_sensor->m_radar_points;
    // a real radar also reports frames without detections
    if(m_pub_pc->get_subscription_count() > 0)
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
            modifier.setPointCloud2Fields(5,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
                m_doppler_field.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32);
            modifier.resize(filtered.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x(pc_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(pc_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(pc_msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_i(pc_msg, "intensity");
            sensor_msgs::PointCloud2Iterator<float> iter_v(pc_msg, m_doppler_field);

            for(size_t i = 0; i < filtered.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i, ++iter_v)
            {
                *iter_x = filtered[i].x;
                *iter_y = filtered[i].y;
                *iter_z = filtered[i].z;
                *iter_i = filtered[i].intensity;
                *iter_v = filtered[i].velocity;
            }

            m_pub_pc->publish(pc_msg);
        }
    }

    // Publish GT PointCloud2 with same FOV filtering
    auto& pts_gt = m_radarays_sensor->m_radar_points_gt;
    if(!pts_gt.empty())
    {
        std::vector<radarays_gazebo_plugins::RadarPoint> filtered_gt;
        filtered_gt.reserve(pts_gt.size());
        for(auto& pt : pts_gt)
        {
            float azimuth = atan2(pt.y, pt.x);
            if(azimuth >= m_pc_fov_min && azimuth <= m_pc_fov_max)
                filtered_gt.push_back(pt);
        }

        if(!filtered_gt.empty())
        {
            sensor_msgs::msg::PointCloud2 gt_msg;
            gt_msg.header.stamp.sec = stamp_gz.sec;
            gt_msg.header.stamp.nanosec = stamp_gz.nsec;
            gt_msg.header.frame_id = m_frame_id;
            gt_msg.height = 1;
            gt_msg.width = filtered_gt.size();
            gt_msg.is_dense = true;
            gt_msg.is_bigendian = false;

            sensor_msgs::PointCloud2Modifier modifier_gt(gt_msg);
            modifier_gt.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
            modifier_gt.resize(filtered_gt.size());

            sensor_msgs::PointCloud2Iterator<float> gx(gt_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> gy(gt_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> gz(gt_msg, "z");
            sensor_msgs::PointCloud2Iterator<float> gi(gt_msg, "intensity");

            for(size_t i = 0; i < filtered_gt.size(); ++i, ++gx, ++gy, ++gz, ++gi)
            {
                *gx = filtered_gt[i].x;
                *gy = filtered_gt[i].y;
                *gz = filtered_gt[i].z;
                *gi = filtered_gt[i].intensity;
            }

            m_pub_pc_gt->publish(gt_msg);
        }
    }
}

GZ_REGISTER_SENSOR_PLUGIN(RadaRaysEmbreeROS)

} // namespace gazebo
