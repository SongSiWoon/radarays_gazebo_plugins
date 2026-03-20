#ifndef GAZEBO_RADARAYS_OPTIX_PLUGIN_H
#define GAZEBO_RADARAYS_OPTIX_PLUGIN_H

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
#include <rmagine/simulation/SphereSimulatorOptix.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>

#include <rmagine/noise/Noise.hpp>

#include <mutex>
#include <shared_mutex>
#include <memory>
#include <optional>
#include <unordered_map>


#include <rmagine_gazebo_plugins/rmagine_optix_spherical_gzplugin.h>

#include <radarays_ros/Radar.hpp>
#include <radarays_ros/msg/radar_materials.hpp>
#include <radarays_ros/RadarModelConfig.hpp>
#include <radarays_ros/msg/radar_params.hpp>
#include <radarays_ros/radar_types.h>

#include <radarays_gazebo_plugins/radar_point.h>

namespace rm = rmagine;

namespace gazebo
{

namespace sensors
{
class RadaRaysOptix : public RmagineOptixSpherical
{
public:
    using Base = RmagineOptixSpherical;

    using ResT = rm::Bundle<
                rm::Hits<rm::VRAM_CUDA>,
                rm::Ranges<rm::VRAM_CUDA>,
                rm::Normals<rm::VRAM_CUDA>,
                rm::ObjectIds<rm::VRAM_CUDA>
            >;

    RadaRaysOptix();
    virtual ~RadaRaysOptix();

    virtual void Init() override;
    virtual void Load(const std::string& world_name) override;

    // simulation buffer
    cv::Mat m_polar_image;

    // 3D radar points for PointCloud2 output
    std::vector<radarays_gazebo_plugins::RadarPoint> m_radar_points;

protected:
    virtual bool UpdateImpl(const bool _force) override;

    void uploadBuffers();

    void initWorkMemory();

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

     // Params
    radarays_ros::msg::RadarParams m_params;
    rm::SphericalModel m_radar_model;
    radarays_ros::RadarModelConfig m_cfg;

    // materials
    int m_material_id_air = 0;
    std::vector<int> m_object_materials;
    bool m_materials_searched = false;

    // simulator
    rm::OnDnSimulatorOptixPtr m_sim;

    // global GPU buffers
    rm::Memory<int, rm::VRAM_CUDA> m_object_materials_gpu;
    rm::Memory<radarays_ros::msg::RadarMaterial, rm::VRAM_CUDA> m_materials_gpu;

    // preallocated memory for faster mem handling
    std::vector<rm::OnDnModel_<rm::VRAM_CUDA> > m_waves_gpu;
    std::vector<rm::Memory<radarays_ros::DirectedWaveAttributes, rm::VRAM_CUDA> > m_wave_attributes_gpu;
    std::vector<rm::Memory<radarays_ros::Signal, rm::VRAM_CUDA> > m_signals;
    std::vector<rm::Memory<uint8_t, rm::VRAM_CUDA> > m_signal_mask;
    std::vector<ResT> m_results;


    float m_wave_energy_threshold = 0.001;
    std::vector<radarays_ros::DirectedWave> m_waves_start;
    bool m_resample = true;
    float m_max_signal = 120.0;
};

using RadaRaysOptixPtr = std::shared_ptr<RadaRaysOptix>;

// will by generated in cpp
void RegisterRadaRaysOptix();

} // namespace sensors

} // namespace gazebo

#endif // GAZEBO_RADARAYS_OPTIX_PLUGIN_H
