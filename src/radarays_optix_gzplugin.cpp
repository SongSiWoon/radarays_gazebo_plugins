#include <radarays_gazebo_plugins/radarays_optix_gzplugin.h>

#include <gazebo/sensors/SensorFactory.hh>

#include <iostream>

#include <rmagine/noise/GaussianNoise.hpp>
#include <rmagine/noise/UniformDustNoise.hpp>
#include <rmagine/noise/RelGaussianNoise.hpp>

#include <gazebo/common/Console.hh>

#include <radarays_ros/RadarGPU.hpp>

#include <radarays_ros/ros_helper.h>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
#include <radarays_ros/radar_algorithms.cuh>
#include <radarays_ros/image_algorithms.cuh>

#include <cv_bridge/cv_bridge.h>
#include <random>

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>

using namespace radarays_ros;
namespace rm = rmagine;


namespace gazebo
{

namespace sensors
{


static rm::Transform to_rm(const ignition::math::Pose3d& pose)
{
    rmagine::Transform T;
    T.R.x = pose.Rot().X();
    T.R.y = pose.Rot().Y();
    T.R.z = pose.Rot().Z();
    T.R.w = pose.Rot().W();
    T.t.x = pose.Pos().X();
    T.t.y = pose.Pos().Y();
    T.t.z = pose.Pos().Z();
    return T;
}


// names of the radar model parameters (everything except "det.*")
static const std::vector<std::string> RADAR_PARAM_NAMES = {
    "n_cells", "resolution", "scroll_image", "range_min", "range_max",
    "beam_width", "n_samples", "n_reflections", "beam_sample_dist",
    "beam_sample_dist_normal_p_in_cone", "signal_denoising",
    "signal_denoising_triangular_width", "signal_denoising_triangular_mode",
    "signal_denoising_gaussian_width", "signal_denoising_gaussian_mode",
    "signal_denoising_mb_width", "signal_denoising_mb_mode", "signal_max",
    "ambient_noise", "ambient_noise_energy_max", "ambient_noise_energy_min",
    "ambient_noise_energy_loss", "ambient_noise_at_signal_0",
    "ambient_noise_at_signal_1", "ambient_noise_perlin_scale_low",
    "ambient_noise_perlin_scale_high", "ambient_noise_perlin_p_low",
    "record_multi_reflection", "record_multi_path", "multipath_threshold",
    "include_motion", "resample_every_frame"
};

RadaRaysOptix::RadaRaysOptix()
:Base() // if sensor is base class: Base(sensors::RAY)
{
    gzdbg << "[RadaRaysOptix] Constructed." << std::endl;
}

RadaRaysOptix::~RadaRaysOptix()
{
    gzdbg << "[RadaRaysOptix] Destroyed." << std::endl;
}

void RadaRaysOptix::loadDetectorSDF()
{
    // default reflection of surfaces without <radarays_material>:
    // weak angle independent scattering plus a lobe around the surface normal
    m_default_material.velocity = 0.0;  // opaque
    m_default_material.ambient  = 0.1;
    m_default_material.diffuse  = 1.0;
    m_default_material.specular = 20.0;

    if(this->sdf->HasElement("default_material"))
    {
        sdf::ElementPtr mat = this->sdf->GetElement("default_material");
        if(mat->HasElement("velocity")) m_default_material.velocity = mat->Get<float>("velocity");
        if(mat->HasElement("ambient"))  m_default_material.ambient = mat->Get<float>("ambient");
        if(mat->HasElement("diffuse"))  m_default_material.diffuse = mat->Get<float>("diffuse");
        if(mat->HasElement("specular")) m_default_material.specular = mat->Get<float>("specular");
    }

    // ray tracing effort, defaults of the ROS parameters with the same names
    if(this->sdf->HasElement("ray_tracing"))
    {
        sdf::ElementPtr rt = this->sdf->GetElement("ray_tracing");
        if(rt->HasElement("n_samples"))     m_sdf_n_samples = rt->Get<int>("n_samples");
        if(rt->HasElement("n_reflections")) m_sdf_n_reflections = rt->Get<int>("n_reflections");
        if(rt->HasElement("beam_width"))    m_sdf_beam_width = rt->Get<double>("beam_width");
        if(rt->HasElement("self_clearance")) m_self_clearance = rt->Get<double>("self_clearance");
    }

    if(this->sdf->HasElement("radar_detector"))
    {
        sdf::ElementPtr det = this->sdf->GetElement("radar_detector");
        for(auto& elem : m_detector.params.fields())
        {
            if(det->HasElement(elem.first))
            {
                *elem.second = det->Get<double>(elem.first);
            }
        }
    }
}

void RadaRaysOptix::loadParamsSDF()
{
    loadDetectorSDF();

    if (!this->sdf->HasElement("ray")) {
        return;
    }

    sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
    if (rayElem->HasElement("scan")) {
        sdf::ElementPtr scanElem = rayElem->GetElement("scan");
        if (scanElem->HasElement("horizontal")) {
            auto h = scanElem->GetElement("horizontal");
            m_radar_model.theta.size = h->Get<int>("samples");
            m_radar_model.theta.min = h->Get<float>("min_angle");
            m_radar_model.theta.inc = h->Get<float>("increment");
        }
        if (scanElem->HasElement("vertical")) {
            auto v = scanElem->GetElement("vertical");
            m_radar_model.phi.min = v->Get<float>("min_angle");
            m_radar_model.phi.inc = v->Get<float>("increment");
            m_radar_model.phi.size = v->Get<int>("samples");
            gzdbg << "[RadaRaysOptix] Loaded vertical scan: phi.min="
                << m_radar_model.phi.min << " phi.inc=" << m_radar_model.phi.inc
                << " phi.size=" << m_radar_model.phi.size << std::endl;
        }
    }
    if (rayElem->HasElement("range")) {
        auto r = rayElem->GetElement("range");
        m_radar_model.range.min = r->Get<float>("min");
        m_radar_model.range.max = r->Get<float>("max");
    }

    m_polar_image = cv::Mat_<unsigned char>(0, m_radar_model.theta.size);
}

void RadaRaysOptix::loadParams()
{
    m_params = default_params();
    m_material_id_air = 0;
    m_wave_energy_threshold = 0.001;
    m_resample = true;

    m_radar_model.theta.inc = -(2 * M_PI) / 400;
    m_radar_model.theta.min = 0.0;
    m_radar_model.theta.size = 400;
    m_radar_model.phi.inc = 1.0;
    m_radar_model.phi.min = 0.0;
    m_radar_model.phi.size = 1;


    // 2 default materials: air, wall stone
    m_params.materials.data.resize(2);
    {
        radarays_ros::msg::RadarMaterial material_air;
        material_air.velocity = 0.3; // m/ns
        material_air.ambient  = 1.0;
        material_air.diffuse  = 0.0;
        material_air.specular = 1.0;
        m_params.materials.data[0] = material_air;
        m_params.materials.data[1] = m_default_material;
    }

    // placeholder until the map is available: searchForMaterials() sizes the
    // table to the map and is called again whenever geometries come or go
    size_t n_default_objects = 100;

    m_object_materials.resize(n_default_objects);
    for(size_t i=0; i<n_default_objects; i++)
    {
        m_object_materials[i] = 1; // wall stone
    }

    // find real materials
    m_polar_image = cv::Mat_<unsigned char>(0, m_radar_model.theta.size);
    
    // sensor geometry, ray tracing settings and detection model from the SDF
    loadParamsSDF();
    

}

void RadaRaysOptix::uploadBuffers()
{
    rm::Memory<int, rm::RAM_CUDA> object_materials2(m_object_materials.size());
    for(size_t i=0; i<m_object_materials.size(); i++)
    {
        object_materials2[i] = m_object_materials[i];
    }
    m_object_materials_gpu = object_materials2;

    rm::Memory<RadarMaterial, rm::RAM_CUDA> materials2(m_params.materials.data.size());
    for(size_t i=0; i<m_params.materials.data.size(); i++)
    {
        materials2[i] = m_params.materials.data[i];
    }
    m_materials_gpu = materials2;
}

void RadaRaysOptix::searchForMaterials()
{
    // Called with the map locked, whenever the set of map geometries changed.
    //
    // material 0: air
    // material 1: default for every geometry without <radarays_material>
    m_params.materials.data.resize(2);
    {
        radarays_ros::msg::RadarMaterial material_air;
        material_air.velocity = 0.3; // m/ns
        material_air.ambient  = 1.0;
        material_air.diffuse  = 0.0;
        material_air.specular = 1.0;
        m_params.materials.data[0] = material_air;
        m_params.materials.data[1] = m_default_material;
    }

    // 1. materials given in the world SDF, by "model::link::visual"
    std::unordered_map<std::string, radarays_ros::msg::RadarMaterial> sdf_materials;

    sdf::ElementPtr world_sdf = this->world->SDF();
    if(world_sdf && world_sdf->HasElement("model"))
    {
        for(sdf::ElementPtr model_sdf = world_sdf->GetElement("model");
            model_sdf;
            model_sdf = model_sdf->GetNextElement("model"))
        {
            if(!model_sdf->HasElement("link"))
            {
                continue;
            }
            const std::string model_name = model_sdf->GetAttribute("name")->GetAsString();

            for(sdf::ElementPtr link_sdf = model_sdf->GetElement("link");
                link_sdf;
                link_sdf = link_sdf->GetNextElement("link"))
            {
                if(!link_sdf->HasElement("visual"))
                {
                    continue;
                }
                const std::string link_name = link_sdf->GetAttribute("name")->GetAsString();

                for(sdf::ElementPtr vis_sdf = link_sdf->GetElement("visual");
                    vis_sdf;
                    vis_sdf = vis_sdf->GetNextElement("visual"))
                {
                    if(!vis_sdf->HasElement("radarays_material"))
                    {
                        continue;
                    }
                    const std::string vis_name = vis_sdf->GetAttribute("name")->GetAsString();
                    sdf::ElementPtr mat_sdf = vis_sdf->GetElement("radarays_material");

                    radarays_ros::msg::RadarMaterial material = m_default_material;
                    if(mat_sdf->HasElement("velocity")) material.velocity = mat_sdf->Get<float>("velocity");
                    if(mat_sdf->HasElement("ambient"))  material.ambient = mat_sdf->Get<float>("ambient");
                    if(mat_sdf->HasElement("diffuse"))  material.diffuse = mat_sdf->Get<float>("diffuse");
                    if(mat_sdf->HasElement("specular")) material.specular = mat_sdf->Get<float>("specular");

                    sdf_materials[model_name + "::" + link_name + "::" + vis_name] = material;
                }
            }
        }
    }

    // 2. every geometry of the map gets a material. Geometries of models
    // that are not part of the world file (spawned later) use the default,
    // so the GPU never indexes outside of the table.
    m_object_materials.assign(1, 1);
    m_object_links.clear();
    size_t n_custom = 0;

    if(m_map)
    {
        for(const auto& elem : m_map->scene()->geometries())
        {
            const unsigned int obj_id = elem.first;
            // links of non static models move: remembered for the Doppler of their returns
            {
                const std::string& geom_name = elem.second->name;
                const size_t last_sep = geom_name.rfind("::");
                if(last_sep != std::string::npos)
                {
                    physics::LinkPtr link = boost::dynamic_pointer_cast<physics::Link>(
                        this->world->EntityByName(geom_name.substr(0, last_sep)));
                    if(link && !link->IsStatic())
                    {
                        if(m_object_links.size() <= obj_id)
                        {
                            m_object_links.resize(obj_id + 1);
                        }
                        m_object_links[obj_id] = link;
                    }
                }
            }

            if(m_object_materials.size() <= obj_id)
            {
                m_object_materials.resize(obj_id + 1, 1);
            }

            auto mat_it = sdf_materials.find(elem.second->name);
            if(mat_it != sdf_materials.end())
            {
                m_params.materials.data.push_back(mat_it->second);
                m_object_materials[obj_id] = m_params.materials.data.size() - 1;
                n_custom++;
            }
        }
    }

    gzmsg << "[RadaRaysOptix] Materials updated: " << m_object_materials.size()
          << " map objects, " << n_custom << " with <radarays_material>" << std::endl;

    uploadBuffers();
}

void RadaRaysOptix::Init()
{
    Base::Init();

    // The parameter node lives in the namespace of the robot (the ROS plugin
    // of this sensor carries it), so several drones do not share "/radar".
    sdf::ElementPtr node_sdf = this->sdf;
    if(this->sdf->HasElement("plugin"))
    {
        sdf::ElementPtr plugin_sdf = this->sdf->GetElement("plugin");
        if(plugin_sdf->HasElement("ros"))
        {
            node_sdf = plugin_sdf;
        }
    }
    m_nh_p = gazebo_ros::Node::Get(node_sdf, "radar");

    loadParams();

    // Declare ROS2 parameters (radar model config).
    // Sensor geometry defaults come from the SDF, so the SDF stays the
    // single place that describes the device.
    const double default_resolution = m_detector.params.range_resolution;
    m_nh_p->declare_parameter("n_cells",
        static_cast<int>(std::ceil(m_radar_model.range.max / default_resolution)));
    m_nh_p->declare_parameter("resolution", default_resolution);
    m_nh_p->declare_parameter("scroll_image", 0);
    m_nh_p->declare_parameter("range_min", static_cast<double>(m_radar_model.range.min));
    m_nh_p->declare_parameter("range_max", static_cast<double>(m_radar_model.range.max));
    m_nh_p->declare_parameter("beam_width", m_sdf_beam_width);
    m_nh_p->declare_parameter("n_samples", m_sdf_n_samples);
    m_nh_p->declare_parameter("n_reflections", m_sdf_n_reflections);
    m_nh_p->declare_parameter("beam_sample_dist", 0);
    m_nh_p->declare_parameter("beam_sample_dist_normal_p_in_cone", 1.0);
    m_nh_p->declare_parameter("signal_denoising", 0);
    m_nh_p->declare_parameter("signal_denoising_triangular_width", 3);
    m_nh_p->declare_parameter("signal_denoising_triangular_mode", 0);
    m_nh_p->declare_parameter("signal_denoising_gaussian_width", 3);
    m_nh_p->declare_parameter("signal_denoising_gaussian_mode", 0);
    m_nh_p->declare_parameter("signal_denoising_mb_width", 3);
    m_nh_p->declare_parameter("signal_denoising_mb_mode", 0);
    m_nh_p->declare_parameter("signal_max", 120.0);
    m_nh_p->declare_parameter("ambient_noise", 0);
    m_nh_p->declare_parameter("ambient_noise_energy_max", 0.1);
    m_nh_p->declare_parameter("ambient_noise_energy_min", 0.0);
    m_nh_p->declare_parameter("ambient_noise_energy_loss", 0.1);
    m_nh_p->declare_parameter("ambient_noise_at_signal_0", 0.5);
    m_nh_p->declare_parameter("ambient_noise_at_signal_1", 0.0);
    m_nh_p->declare_parameter("ambient_noise_perlin_scale_low", 0.01);
    m_nh_p->declare_parameter("ambient_noise_perlin_scale_high", 0.1);
    m_nh_p->declare_parameter("ambient_noise_perlin_p_low", 0.5);
    m_nh_p->declare_parameter("record_multi_reflection", true);
    m_nh_p->declare_parameter("record_multi_path", false);
    m_nh_p->declare_parameter("multipath_threshold", 0.9);
    m_nh_p->declare_parameter("include_motion", false);
    m_nh_p->declare_parameter("resample_every_frame", false);

    // Detection model: SDF values are the defaults, tunable at runtime
    for(auto& elem : m_detector.params.fields())
    {
        m_nh_p->declare_parameter("det." + elem.first, *elem.second);
    }

    // Apply every declared parameter, not only a subset: what
    // "ros2 param get" reports is what the simulation uses.
    {
        std::lock_guard<std::mutex> lock(m_cfg_mutex);

        radarays_ros::RadarModelConfig initial_cfg;
        for(const std::string& name : RADAR_PARAM_NAMES)
        {
            applyParam(m_nh_p->get_parameter(name), initial_cfg);
        }
        for(auto& elem : m_detector.params.fields())
        {
            applyParam(m_nh_p->get_parameter("det." + elem.first), initial_cfg);
        }
        updateDynCfg(initial_cfg, 0);
    }

    // Register parameter change callback
    m_param_callback = m_nh_p->add_on_set_parameters_callback(
        std::bind(&RadaRaysOptix::onParameterChange, this, std::placeholders::_1));

    if(!m_map)
    {
        m_waiting_for_map = true;
    }
}

void RadaRaysOptix::applyParam(
    const rclcpp::Parameter& param,
    radarays_ros::RadarModelConfig& cfg)
{
    const std::string& name = param.get_name();

    if(name.rfind("det.", 0) == 0)
    {
        const std::string field = name.substr(4);
        for(auto& elem : m_detector.params.fields())
        {
            if(elem.first == field)
            {
                *elem.second = param.as_double();
            }
        }
        return;
    }

    if(name == "n_cells")                              cfg.n_cells = param.as_int();
    else if(name == "resolution")                     cfg.resolution = param.as_double();
    else if(name == "scroll_image")                   cfg.scroll_image = param.as_int();
    else if(name == "range_min")                      cfg.range_min = param.as_double();
    else if(name == "range_max")                      cfg.range_max = param.as_double();
    else if(name == "beam_width")                     cfg.beam_width = param.as_double();
    else if(name == "n_samples")                      cfg.n_samples = param.as_int();
    else if(name == "n_reflections")                  cfg.n_reflections = param.as_int();
    else if(name == "beam_sample_dist")               cfg.beam_sample_dist = param.as_int();
    else if(name == "beam_sample_dist_normal_p_in_cone") cfg.beam_sample_dist_normal_p_in_cone = param.as_double();
    else if(name == "signal_denoising")               cfg.signal_denoising = param.as_int();
    else if(name == "signal_denoising_triangular_width") cfg.signal_denoising_triangular_width = param.as_int();
    else if(name == "signal_denoising_triangular_mode")  cfg.signal_denoising_triangular_mode = param.as_int();
    else if(name == "signal_denoising_gaussian_width")   cfg.signal_denoising_gaussian_width = param.as_int();
    else if(name == "signal_denoising_gaussian_mode")    cfg.signal_denoising_gaussian_mode = param.as_int();
    else if(name == "signal_denoising_mb_width")         cfg.signal_denoising_mb_width = param.as_int();
    else if(name == "signal_denoising_mb_mode")          cfg.signal_denoising_mb_mode = param.as_int();
    else if(name == "signal_max")                     cfg.signal_max = param.as_double();
    else if(name == "ambient_noise")                  cfg.ambient_noise = param.as_int();
    else if(name == "ambient_noise_energy_max")       cfg.ambient_noise_energy_max = param.as_double();
    else if(name == "ambient_noise_energy_min")       cfg.ambient_noise_energy_min = param.as_double();
    else if(name == "ambient_noise_energy_loss")      cfg.ambient_noise_energy_loss = param.as_double();
    else if(name == "ambient_noise_at_signal_0")      cfg.ambient_noise_at_signal_0 = param.as_double();
    else if(name == "ambient_noise_at_signal_1")      cfg.ambient_noise_at_signal_1 = param.as_double();
    else if(name == "ambient_noise_perlin_scale_low") cfg.ambient_noise_perlin_scale_low = param.as_double();
    else if(name == "ambient_noise_perlin_scale_high") cfg.ambient_noise_perlin_scale_high = param.as_double();
    else if(name == "ambient_noise_perlin_p_low")     cfg.ambient_noise_perlin_p_low = param.as_double();
    else if(name == "record_multi_reflection")        cfg.record_multi_reflection = param.as_bool();
    else if(name == "record_multi_path")              cfg.record_multi_path = param.as_bool();
    else if(name == "multipath_threshold")            cfg.multipath_threshold = param.as_double();
    else if(name == "include_motion")                 cfg.include_motion = param.as_bool();
    else if(name == "resample_every_frame")           m_resample_every_frame = param.as_bool();
}

rcl_interfaces::msg::SetParametersResult RadaRaysOptix::onParameterChange(
    const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for(const auto& param : params)
    {
        const std::string& name = param.get_name();
        if((name == "n_samples" || name == "n_reflections") && param.as_int() < 1)
        {
            result.successful = false;
            result.reason = name + " must be >= 1";
            return result;
        }
        if(name.rfind("det.", 0) == 0)
        {
            const std::string reason = radarays_gazebo_plugins::checkDetectorParam(
                name.substr(4), param.as_double());
            if(!reason.empty())
            {
                result.successful = false;
                result.reason = reason;
                return result;
            }
        }
    }

    // the sensor thread reads the configuration while simulating
    std::lock_guard<std::mutex> lock(m_cfg_mutex);

    radarays_ros::RadarModelConfig cfg = m_cfg;
    for(const auto& param : params)
    {
        applyParam(param, cfg);
    }
    updateDynCfg(cfg, 0);

    return result;
}

void RadaRaysOptix::Load(const std::string& world_name)
{
    Base::Load(world_name);
    gzdbg << "LOAD - RadaRaysOptix!" << std::endl;
}

void RadaRaysOptix::simulate(rm::Transform Tsm)
{
    // 
    if(m_polar_image.rows != m_cfg.n_cells)
    {
        gzdbg << "[RadaRaysOptix] Resize canvas to " << m_cfg.n_cells << std::endl;
        m_polar_image.resize(m_cfg.n_cells);
        gzdbg << "[RadaRaysOptix] Resizing canvas - done." << std::endl;
    }

    m_polar_image.setTo(cv::Scalar(0));

    std::vector<float> denoising_weights;
    int denoising_mode = 0;

    if(m_cfg.signal_denoising > 0)
    {
        if(m_cfg.signal_denoising == 1)
        {
            denoising_mode = m_cfg.signal_denoising_triangular_mode * m_cfg.signal_denoising_triangular_width;
            denoising_weights = make_denoiser_triangular(
                m_cfg.signal_denoising_triangular_width,
                denoising_mode
            );
            
        } else if(m_cfg.signal_denoising == 2) {
            denoising_mode = m_cfg.signal_denoising_gaussian_mode * m_cfg.signal_denoising_gaussian_width;
            denoising_weights = make_denoiser_gaussian(
                m_cfg.signal_denoising_gaussian_width,
                denoising_mode
            );

        } else if(m_cfg.signal_denoising == 3) {
            denoising_mode = m_cfg.signal_denoising_mb_mode * m_cfg.signal_denoising_mb_width;
            denoising_weights = make_denoiser_maxwell_boltzmann(
                m_cfg.signal_denoising_mb_width,
                denoising_mode
            );
        }

        // scale so that mode has weight 1
        if(denoising_weights.size() > 0)
        {
            double denoising_mode_val = denoising_weights[denoising_mode];

            for(size_t i=0; i<denoising_weights.size(); i++)
            {
                denoising_weights[i] /= denoising_mode_val;
            }
        }
        
    }

    // upload buffers
    rm::Memory<float, rm::VRAM_CUDA> denoising_weights_gpu;
    if(denoising_weights.size())
    {
        rm::Memory<float, rm::RAM_CUDA> denoising_weights2(denoising_weights.size());
        for(size_t i=0; i<denoising_weights.size(); i++)
        {
            denoising_weights2[i] = denoising_weights[i];
        }
        denoising_weights_gpu = denoising_weights2;
    }


    DirectedWaveAttributes wave_att_ex;
    wave_att_ex.energy       =  1.0;
    wave_att_ex.polarization =  0.5;
    wave_att_ex.frequency    = 76.5; // GHz
    wave_att_ex.velocity     =  0.3; // m / ns - speed in air
    wave_att_ex.material_id  =  0;   // air
    wave_att_ex.time         =  0.0; // ns

    const size_t n_cells = static_cast<size_t>(m_polar_image.rows);
    const size_t n_angles = static_cast<size_t>(m_polar_image.cols);
    // number of reflection passes, 0 when disabled
    const size_t n_passes = static_cast<size_t>(std::max(m_cfg.n_reflections, 0));

    // prepare radar model

    const size_t n_phi = m_radar_model.phi.size;
    size_t n_rays = n_angles * n_phi * m_cfg.n_samples;

    rm::OnDnModel waves;
    waves.range = m_radar_model.range;
    waves.dirs.resize(n_rays);
    waves.origs.resize(n_rays);
    waves.width = n_angles * n_phi;
    waves.height = m_params.model.n_samples;
    rm::Memory<DirectedWaveAttributes> wave_attributes(n_rays);

    rm::Vector front = {1.0, 0.0, 0.0};
    if(m_resample || m_resample_every_frame)
    {
        m_ray_dirs_local_cache = sample_cone(
            front,
            m_params.model.beam_width,
            m_params.model.n_samples,
            m_cfg.beam_sample_dist,
            m_cfg.beam_sample_dist_normal_p_in_cone
        );
        m_resample = false;
    }
    rm::Memory<rm::Vector>& ray_dirs_local = m_ray_dirs_local_cache;

    for(size_t angle_id=0; angle_id<n_angles; angle_id++)
    {
        for(size_t phi_id=0; phi_id<n_phi; phi_id++)
        {
            float phi_angle = m_radar_model.phi.min + phi_id * m_radar_model.phi.inc;
            size_t col_id = angle_id * n_phi + phi_id;

            rm::Transform Tas;
            Tas.R = rm::EulerAngles{0.0, phi_angle, m_radar_model.getTheta(angle_id)};
            Tas.t = m_radar_model.getOrigin(0, angle_id);

            for(size_t sample_id = 0; sample_id < m_params.model.n_samples; sample_id++)
            {
                size_t buf_id = sample_id * waves.width + col_id;
                waves.dirs[buf_id] = Tas.R * ray_dirs_local[sample_id];
                // start outside of the own vehicle, the skipped distance still counts
                waves.origs[buf_id] = waves.dirs[buf_id] * m_self_clearance;
                wave_attributes[buf_id] = wave_att_ex;
                wave_attributes[buf_id].time = m_self_clearance / wave_att_ex.velocity;
            }
        }
    }


    // GPU ray buffers, rebuilt whenever the number of rays changed
    if(m_work_memory_dirty || m_waves_gpu.size() != n_passes)
    {
        initWorkMemory();
        m_work_memory_dirty = false;
    }


    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tsms_;
    { // upload
        rm::Memory<rm::Transform> Tsms(1);
        Tsms[0] = Tsm;
        Tsms_ = Tsms;
    }


    // 1. Generate Signals
    {

        if(m_cfg.n_reflections > 0)
        {
             // upload first bunch of rays
            m_waves_gpu[0].origs = waves.origs;
            m_waves_gpu[0].dirs = waves.dirs;

            if(wave_attributes.size() != m_wave_attributes_gpu[0].size())
            {
                gzdbg << "PREALLOCATED MEMORY WRONG!" << std::endl;
                return;
            }
            m_wave_attributes_gpu[0] = wave_attributes;


            // m_results.size() == Number of reflections
            // For each reflection
            // - sim
            // - reflection shader
            // - if 
            for(size_t i=0; i<n_passes; i++)
            {
                m_sim->setModel(m_waves_gpu[i]);
                
                
                m_sim->simulate(Tsms_, m_results[i]);


                move_waves(
                    m_waves_gpu[i].origs,
                    m_waves_gpu[i].dirs,
                    m_wave_attributes_gpu[i],
                    m_results[i].ranges, 
                    m_results[i].hits);

                if(m_materials_gpu.size() == 0 || m_object_materials.size() == 0)
                {
                    gzdbg << "[RadaRaysOptix] WARNING. materials or object->materials buffers seam to have wrong sizes" << std::endl;
                    gzdbg << "- Materials: " << m_materials_gpu.size() << std::endl;
                    gzdbg << "- Objects with materials: " << m_object_materials.size() << std::endl;
                }
               
                signal_shader(
                    m_materials_gpu,
                    m_object_materials_gpu,
                    m_material_id_air,

                    m_waves_gpu[i].dirs,
                    m_wave_attributes_gpu[i],
                    m_results[i].hits,
                    m_results[i].normals,
                    m_results[i].object_ids,
                    
                    m_signals[i]
                );

                if(i + 1 < n_passes)
                {
                    // i -> i+1
                    auto lo = m_waves_gpu[i+1].origs(0, m_waves_gpu[i].origs.size());
                    auto ld = m_waves_gpu[i+1].dirs(0, m_waves_gpu[i].dirs.size());
                    auto la = m_wave_attributes_gpu[i+1](0, m_wave_attributes_gpu[i].size());
                    
                    auto ro = m_waves_gpu[i+1].origs(m_waves_gpu[i].origs.size(), m_waves_gpu[i].origs.size() * 2);
                    auto rd = m_waves_gpu[i+1].dirs(m_waves_gpu[i].dirs.size(), m_waves_gpu[i].dirs.size() * 2);
                    auto ra = m_wave_attributes_gpu[i+1](m_wave_attributes_gpu[i].size(), m_wave_attributes_gpu[i].size() * 2);

                    // FRESNEL SPLIT
                    fresnel_split(
                        m_materials_gpu,
                        m_object_materials_gpu,
                        m_material_id_air,
                        // INCIDENCE
                        m_waves_gpu[i].origs,
                        m_waves_gpu[i].dirs,
                        m_wave_attributes_gpu[i],
                        m_results[i].hits,
                        m_results[i].normals,
                        m_results[i].object_ids,
                        // SPLIT
                        lo, ld, la,
                        ro, rd, ra
                    );
                } // fresnel split
            }
        }  
    }
    

    // Generate GT points from first pass ray casting hits (before signal processing)
    m_radar_points_gt.clear();
    if(m_gt_requested && m_cfg.n_reflections > 0)
    {
        rm::Memory<uint8_t> hits_gt = m_results[0].hits;
        rm::Memory<float> ranges_gt = m_results[0].ranges;
        cudaDeviceSynchronize();

        size_t wave_width_gt = n_angles * n_phi;
        for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
        {
            for(size_t phi_id = 0; phi_id < n_phi; phi_id++)
            {
                size_t col_id = angle_id * n_phi + phi_id;
                // Use first sample only for GT (sample_id=0)
                size_t hit_id = 0 * wave_width_gt + col_id;
                if(hit_id < hits_gt.size() && hits_gt[hit_id])
                {
                    float range = ranges_gt[hit_id] + m_self_clearance;
                    if(range >= m_radar_model.range.min && range <= m_radar_model.range.max)
                    {
                        float theta = m_radar_model.getTheta(angle_id);
                        float phi_angle = m_radar_model.phi.min + phi_id * m_radar_model.phi.inc;

                        radarays_gazebo_plugins::RadarPoint pt;
                        pt.x = range * cos(phi_angle) * cos(theta);
                        pt.y = range * cos(phi_angle) * sin(theta);
                        pt.z = -range * sin(phi_angle);
                        pt.intensity = 1.0;
                        pt.velocity = 0.0f;
                        m_radar_points_gt.push_back(pt);
                    }
                }
            }
        }
    }

    // download the signals of all passes
    std::vector<rm::Memory<Signal> > signals_cpu(n_passes);
    std::vector<rm::Memory<uint8_t> > hits_cpu(n_passes);

    for(size_t i=0; i<n_passes; i++)
    {
        signals_cpu[i] = m_signals[i];
        hits_cpu[i] = m_results[i].hits;
    }

    // velocity of moving map objects in the sensor frame. Only the directly
    // hit object (first pass) is considered, multipath keeps the static Doppler.
    std::vector<rm::Vector> object_velocity(m_object_links.size(), rm::Vector{0.0f, 0.0f, 0.0f});
    std::vector<uint8_t> object_moves(m_object_links.size(), 0);
    bool any_moving = false;
    for(size_t i = 0; i < m_object_links.size(); i++)
    {
        if(m_object_links[i])
        {
            const auto v = m_object_links[i]->WorldLinearVel();
            if(v.SquaredLength() > 1e-6)
            {
                object_velocity[i] = Tsm.R.inv() * rm::Vector{(float)v.X(), (float)v.Y(), (float)v.Z()};
                object_moves[i] = 1;
                any_moving = true;
            }
        }
    }
    rm::Memory<unsigned int> object_ids_cpu;
    if(any_moving && m_cfg.n_reflections > 0)
    {
        object_ids_cpu = m_results[0].object_ids;
    }

    cudaDeviceSynchronize();

    // the polar image is only built when somebody listens to it
    const bool make_image = m_image_requested;

    rm::Memory<float, rm::UNIFIED_CUDA> img;
    rm::Memory<float, rm::UNIFIED_CUDA> max_vals;
    rm::Memory<float, rm::UNIFIED_CUDA> random_begins;
    if(make_image)
    {
        img.resize(n_cells * n_angles);
        max_vals.resize(n_angles);
        random_begins.resize(n_angles);
        cudaMemset(img.raw(), 0, n_cells * n_angles * sizeof(float) );
        cudaMemset(max_vals.raw(), 0, n_angles * sizeof(float) );
    }

    // width of the wave model is n_angles * n_phi
    const size_t wave_width = n_angles * n_phi;
    const size_t n_samples = m_cfg.n_samples;

    // solid angle one beam stands for (before splitting into samples)
    const float d_theta = fabs(m_radar_model.theta.inc);
    const float d_phi = (n_phi > 1) ? fabs(m_radar_model.phi.inc) : d_theta;

    // 2. collect the returns of every ray
    // Single threaded on purpose: these are a few thousand signals, and a
    // parallel region here keeps all cores of the host busy at sensor rate.
    m_returns.clear();

    for(size_t angle_id_ = 0; angle_id_ < n_angles; angle_id_++)
    {
        const size_t angle_id = (m_cfg.scroll_image + angle_id_) % n_angles;
        const unsigned int img_offset = angle_id * n_cells;
        const float theta = m_radar_model.getTheta(angle_id);
        const float cos_theta = cos(theta);
        const float sin_theta = sin(theta);

        float max_val = 0.0;

        for(size_t phi_id=0; phi_id<n_phi; phi_id++)
        {
            const size_t col_id = angle_id * n_phi + phi_id;
            const float phi_angle = m_radar_model.phi.min + phi_id * m_radar_model.phi.inc;
            const float cos_phi = cos(phi_angle);
            const float sin_phi = sin(phi_angle);

            // beam direction in the sensor frame (positive phi pitches down)
            const float ux = cos_phi * cos_theta;
            const float uy = cos_phi * sin_theta;
            const float uz = -sin_phi;

            // Doppler: radial velocity of a static world seen from the moving sensor
            const float radial_velocity = -(m_sensor_velocity.x * ux
                + m_sensor_velocity.y * uy + m_sensor_velocity.z * uz);
            const float solid_angle = d_theta * d_phi * fabs(cos_phi) / static_cast<float>(n_samples);

            for(size_t pass_id=0; pass_id < n_passes; pass_id++)
            {
                if(pass_id > 0 && !m_cfg.record_multi_reflection)
                {
                    break;
                }

                const size_t size_factor = static_cast<size_t>(1) << pass_id;

                for(size_t sample_id=0; sample_id < n_samples * size_factor; sample_id++)
                {
                    const size_t signal_id = sample_id * wave_width + col_id;

                    if(signal_id >= hits_cpu[pass_id].size() || !hits_cpu[pass_id][signal_id])
                    {
                        continue;
                    }

                    const Signal signal = signals_cpu[pass_id][signal_id];
                    // the wave travels 0.3 m/ns, there and back
                    const float signal_dist = 0.3 * signal.time / 2.0;
                    const float strength = signal.strength;

                    if(!std::isfinite(signal_dist) || !std::isfinite(strength))
                    {
                        continue;
                    }

                    if(make_image)
                    {
                        const int cell = static_cast<int>(signal_dist / m_cfg.resolution);
                        if(cell >= 0 && static_cast<size_t>(cell) < n_cells)
                        {
                            if(m_cfg.signal_denoising > 0)
                            {
                                for(int vid = 0; vid < static_cast<int>(denoising_weights.size()); vid++)
                                {
                                    const int glob_id = vid + cell - denoising_mode;
                                    if(glob_id > 0 && static_cast<size_t>(glob_id) < n_cells)
                                    {
                                        const float new_val = img[img_offset + glob_id]
                                            + strength * denoising_weights[vid];
                                        img[img_offset + glob_id] = new_val;
                                        max_val = std::max(max_val, new_val);
                                    }
                                }
                            } else {
                                const float new_val = std::max(img[img_offset + cell], strength);
                                img[img_offset + cell] = new_val;
                                max_val = std::max(max_val, new_val);
                            }
                        }
                    }

                    if(signal_dist >= m_radar_model.range.min
                        && signal_dist <= m_radar_model.range.max
                        && strength > m_wave_energy_threshold)
                    {
                        radarays_gazebo_plugins::RadarReturn ret;
                        ret.range = signal_dist;
                        ret.dx = ux;
                        ret.dy = uy;
                        ret.dz = uz;
                        ret.power = strength;
                        ret.radial_velocity = radial_velocity;
                        if(any_moving && pass_id == 0 && signal_id < object_ids_cpu.size())
                        {
                            const unsigned int obj = object_ids_cpu[signal_id];
                            if(obj < object_moves.size() && object_moves[obj])
                            {
                                // range rate = (v_object - v_sensor) . beam direction
                                ret.radial_velocity += object_velocity[obj].x * ux
                                    + object_velocity[obj].y * uy + object_velocity[obj].z * uz;
                            }
                        }
                        ret.solid_angle = solid_angle;
                        m_returns.push_back(ret);
                    }
                }
            }
        }

        if(make_image)
        {
            max_vals[angle_id] = max_val;
        }
    }

    // 3. detection: sparse point cloud like the real device
    if(m_detector.params.enabled > 0.5)
    {
        radarays_gazebo_plugins::RadarSensorPose sensor_pose;
        sensor_pose.x = Tsm.t.x;
        sensor_pose.y = Tsm.t.y;
        sensor_pose.z = Tsm.t.z;
        sensor_pose.qx = Tsm.R.x;
        sensor_pose.qy = Tsm.R.y;
        sensor_pose.qz = Tsm.R.z;
        sensor_pose.qw = Tsm.R.w;
        m_radar_points = m_detector.process(m_returns, sensor_pose);
    } else {
        // raw ray hits, for debugging the ray tracing
        m_radar_points.clear();
        m_radar_points.reserve(m_returns.size());
        for(const auto& ret : m_returns)
        {
            radarays_gazebo_plugins::RadarPoint pt;
            pt.x = ret.range * ret.dx;
            pt.y = ret.range * ret.dy;
            pt.z = ret.range * ret.dz;
            pt.intensity = ret.power;
            pt.velocity = ret.radial_velocity;
            m_radar_points.push_back(pt);
        }
    }

    // 4. polar image
    if(make_image)
    {
        if(m_cfg.ambient_noise)
        {
            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);

            for(size_t i=0; i<n_angles; i++)
            {
                max_vals[i] = std::max(
                    (float)m_cfg.ambient_noise_energy_max,
                    max_vals[i]);

                random_begins[i] = dist_uni(gen) * 1000.0;
            }

            AmbientNoiseParams noise_params;
            noise_params.noise_at_signal_0 = m_cfg.ambient_noise_at_signal_0;
            noise_params.noise_at_signal_1 = m_cfg.ambient_noise_at_signal_1;
            noise_params.noise_energy_loss = m_cfg.ambient_noise_energy_loss;
            noise_params.noise_energy_min = m_cfg.ambient_noise_energy_min;
            noise_params.noise_energy_max = m_cfg.ambient_noise_energy_max;
            noise_params.resolution = m_cfg.resolution;

            double scale_low = m_cfg.ambient_noise_perlin_scale_low;
            double scale_high = m_cfg.ambient_noise_perlin_scale_high;
            double p_low = m_cfg.ambient_noise_perlin_p_low;

            fill_perlin_noise_hilo(
                img, max_vals,
                n_angles, n_cells,
                random_begins, random_begins,
                scale_low, scale_high,
                p_low,
                noise_params
            );
            cudaDeviceSynchronize();
        }

        float max_signal = m_cfg.signal_max;
        cv::Mat_<float> polar_img_f(static_cast<int>(n_cells), static_cast<int>(n_angles));

        for(size_t x=0; x<n_angles; x++)
        {
            // columns without any return stay black instead of dividing by zero
            const float max_val = max_vals[x];
            const float scale = (max_val > 0.0f) ? max_signal / max_val : 0.0f;

            for(size_t y=0; y<n_cells; y++)
            {
                polar_img_f.at<float>(y, x) = img[x * n_cells + y] * scale;
            }
        }
        polar_img_f.convertTo(m_polar_image, CV_8UC1);
    }
    
}

bool RadaRaysOptix::UpdateImpl(const bool _force)
{
    if(!m_map)
    {
        if(!m_waiting_for_map)
        {
            gzdbg << "[RadaRaysOptix] Start waiting for RmagineOptixMap..." << std::endl;
            m_waiting_for_map = true;
        }
        return false;
    } else {
        if(m_waiting_for_map) {
            gzdbg << "[RadaRaysOptix] Map received!" << std::endl;
            m_waiting_for_map = false;
        }

        if(!m_sim)
        {
            m_sim = std::make_shared<rm::OnDnSimulatorOptix>(m_map);
            gzdbg << "[RadaRaysOptix] OnDnSimulator constructed." << std::endl;
            
            m_sim->setTsb(rm::Transform::Identity());
            gzdbg << "[RadaRaysOptix] OnDnSimulator map set." << std::endl;
            
            m_sim->preBuildProgram<ResT>();
        }

    }

    auto pose = this->parentEntity->WorldPose();
    auto linear_vel = this->parentEntity->WorldLinearVel();

    rm::Transform Tbm = to_rm(pose);

    rm::Transform Tsm = Tbm * m_Tsb;

    // Doppler needs the sensor velocity expressed in the sensor frame
    const rm::Vector vel_world = {(float)linear_vel.X(), (float)linear_vel.Y(), (float)linear_vel.Z()};
    const rm::Vector vel_sensor = Tsm.R.inv() * vel_world;
    m_sensor_velocity = {vel_sensor.x, vel_sensor.y, vel_sensor.z};

    if(m_map_mutex)
    {
        m_map_mutex->lock_shared();
    }

    {
        // parameters may change from the ROS thread
        std::lock_guard<std::mutex> cfg_lock(m_cfg_mutex);

        // models can be spawned or removed at any time: keep the
        // object -> material table in sync with the map
        const auto geoms = m_map->scene()->geometries();
        const size_t geom_signature = geoms.size() * 100003
            + (geoms.empty() ? 0 : geoms.rbegin()->first);
        if(!m_materials_searched || geom_signature != m_geom_signature)
        {
            searchForMaterials();
            m_materials_searched = true;
            m_geom_signature = geom_signature;
        }

        // fill global simulation buffer
        simulate(Tsm);
    }

    if(m_map_mutex)
    {
        m_map_mutex->unlock_shared();
    }

    return Base::UpdateImpl(_force);
}

void RadaRaysOptix::initWorkMemory()
{
    gzdbg << "Resizing working memory for " << m_cfg.n_reflections << " reflections passes" << std::endl;
    if(m_cfg.n_reflections == 0)
    {
        gzdbg << "Cannot init working memory for 0 reflections!" << std::endl;
        return;
    }
    // update preallocated memory buffers
    m_waves_gpu.resize(m_cfg.n_reflections);
    m_wave_attributes_gpu.resize(m_cfg.n_reflections);
    m_results.resize(m_cfg.n_reflections);
    m_signals.resize(m_cfg.n_reflections);
    m_signal_mask.resize(m_cfg.n_reflections);

    const size_t n_passes = static_cast<size_t>(m_cfg.n_reflections);
    const size_t n_angles = m_radar_model.theta.size;
    const size_t n_phi = m_radar_model.phi.size;
    size_t n_rays = n_angles * n_phi * m_cfg.n_samples;

    if(m_cfg.n_reflections > 0)
    {
        gzdbg << "Alloc for Pass 1" << std::endl;
        // pass - i = 0
        {
            m_waves_gpu[0].width = n_angles * n_phi;
            m_waves_gpu[0].height = m_params.model.n_samples;
            m_waves_gpu[0].range = m_radar_model.range;
            m_waves_gpu[0].origs.resize(n_rays);
            m_waves_gpu[0].dirs.resize(n_rays);
        }

        m_wave_attributes_gpu[0].resize(n_rays);
        
        rm::resize_memory_bundle<rm::VRAM_CUDA>(
            m_results[0], m_waves_gpu[0].width, m_waves_gpu[0].height, 1);
        m_signals[0].resize(m_waves_gpu[0].size());
        m_signal_mask[0].resize(m_waves_gpu[0].size());

        // pass - i > 0
        for(size_t i=1; i<n_passes; i++)
        {
            gzdbg << "Alloc for Pass " << i+1 << std::endl;
            // pass i

            // m_waves_gpu
            {
                m_waves_gpu[i].width = m_waves_gpu[i-1].width;
                m_waves_gpu[i].height = m_waves_gpu[i-1].height * 2;
                m_waves_gpu[i].range = m_waves_gpu[i-1].range;
                m_waves_gpu[i].origs.resize(m_waves_gpu[i-1].origs.size() * 2);
                m_waves_gpu[i].dirs.resize(m_waves_gpu[i-1].dirs.size() * 2);
            }

            m_wave_attributes_gpu[i].resize(m_wave_attributes_gpu[i-1].size() * 2);

            rm::resize_memory_bundle<rm::VRAM_CUDA>(
                m_results[i], m_waves_gpu[i].width, m_waves_gpu[i].height, 1);
            
            m_signals[i].resize(m_waves_gpu[i].size());
            m_signal_mask[i].resize(m_waves_gpu[i].size());
        }
    }
}

void RadaRaysOptix::updateDynCfg(
    radarays_ros::RadarModelConfig &config,
    uint32_t level)
{
    RCLCPP_INFO(m_nh_p->get_logger(), "RadaRaysOptix - Changing Model");
    
    if(   config.beam_sample_dist != m_cfg.beam_sample_dist 
        || abs(config.beam_width - m_cfg.beam_width ) > 0.001
        || config.n_samples != m_cfg.n_samples
        || abs(config.beam_sample_dist_normal_p_in_cone - m_cfg.beam_sample_dist_normal_p_in_cone) > 0.001
    )
    {
        m_resample = true;
    }

    // update radar model
    m_radar_model.range.min = config.range_min;
    m_radar_model.range.max = config.range_max;

    // update params model
    // ray buffers depend on both values: rebuild them in the sensor thread
    if(config.n_reflections != static_cast<int>(m_params.model.n_reflections)
        || config.n_samples != static_cast<int>(m_params.model.n_samples))
    {
        m_work_memory_dirty = true;
    }

    m_params.model.beam_width = config.beam_width * M_PI / 180.0;
    m_params.model.n_samples = config.n_samples;
    m_params.model.n_reflections = config.n_reflections;

    m_cfg = config;
}


GZ_REGISTER_STATIC_SENSOR("radarays_optix", RadaRaysOptix)

} // namespace sensors

} // namespace gazebo