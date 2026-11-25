#ifndef GAZEBO_RADARAYS_EMBREE_PLUGIN_H
#define GAZEBO_RADARAYS_EMBREE_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>

#include <rmagine/noise/Noise.hpp>

#include <mutex>
#include <shared_mutex>
#include <memory>
#include <optional>
#include <unordered_map>


#include <rmagine_gazebo_plugins/rmagine_embree_spherical_gzplugin.h>

#include <radarays_ros/Radar.hpp>
#include <radarays_ros/msg/radar_materials.hpp>
#include <radarays_ros/RadarModelConfig.hpp>
#include <radarays_ros/msg/radar_params.hpp>
#include <radarays_ros/radar_types.h>

#include <radarays_gazebo_plugins/radar_point.h>
#include <radarays_gazebo_plugins/radar_detector.h>
#include <mutex>

namespace rm = rmagine;

namespace gazebo
{

namespace sensors
{
class RadaRaysEmbree : public RmagineEmbreeSpherical
{
public:
    using Base = RmagineEmbreeSpherical;

    RadaRaysEmbree();
    virtual ~RadaRaysEmbree();

    virtual void Init() override;
    virtual void Load(const std::string& world_name) override;

    // simulation buffer
    cv::Mat m_polar_image;

    // 3D radar points for PointCloud2 output
    std::vector<radarays_gazebo_plugins::RadarPoint> m_radar_points;

    // Ground truth points from direct ray casting hits
    std::vector<radarays_gazebo_plugins::RadarPoint> m_radar_points_gt;

    // Set by the ROS plugin: build the polar image only if somebody listens
    bool m_image_requested = true;

    // detection stage, public for the ROS plugin: heat map requests and results
    radarays_gazebo_plugins::RadarDetector& detector() { return m_detector; }

protected:
    virtual bool UpdateImpl(const bool _force) override;

    void loadDetectorSDF();
    void loadParamsSDF();
    void loadParams();

    void searchForMaterials();

    void simulate(rmagine::Transform Tsm);

// For Radar
    void updateDynCfg(radarays_ros::RadarModelConfig &config, uint32_t level);

    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& params);

    // ROS NODE
    rclcpp::Node::SharedPtr m_nh_p;

    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback;

    // one ray tracer, all beams of a pass go through it in a single call
    rm::OnDnSimulatorEmbreePtr m_sim;

     // Params
    radarays_ros::msg::RadarParams m_params;
    rm::SphericalModel m_radar_model;
    radarays_ros::RadarModelConfig m_cfg;

    // materials
    int m_material_id_air = 0;
    std::vector<int> m_object_materials;
    bool m_materials_searched = false;

    float m_wave_energy_threshold = 0.001;
    std::vector<radarays_ros::DirectedWave> m_waves_start;
    bool m_resample = true;
    bool m_resample_every_frame = false;

    // guards the configuration: ROS thread writes, sensor thread reads
    std::mutex m_cfg_mutex;

    // FMCW detection stage (rays -> sparse detections)
    radarays_gazebo_plugins::RadarDetector m_detector;
    std::vector<radarays_gazebo_plugins::RadarReturn> m_returns;

    // <ray_tracing> from the sensor SDF: beam width [deg], rays per beam, bounces
    double m_sdf_beam_width = 1.5;
    int m_sdf_n_samples = 2;
    int m_sdf_n_reflections = 2;

    // <ray_tracing><self_clearance> [m]: rays start this far from the sensor,
    // so the vehicle carrying the radar does not block its own view
    float m_self_clearance = 0.0;

    // material of map objects without <radarays_material>
    radarays_ros::msg::RadarMaterial m_default_material;

    // Sensor velocity in the sensor frame, for Doppler calculation
    struct { float x; float y; float z; } m_sensor_velocity = {0, 0, 0};
    float m_max_signal = 120.0;
};

using RadaRaysEmbreePtr = std::shared_ptr<RadaRaysEmbree>;

// will by generated in cpp
void RegisterRadaRaysEmbree();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RADARAYS_EMBREE_PLUGIN_H
