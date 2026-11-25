#include <radarays_gazebo_plugins/radarays_embree_gzplugin.h>

#include <gazebo/sensors/SensorFactory.hh>

#include <iostream>

#include <rmagine/noise/GaussianNoise.hpp>
#include <rmagine/noise/UniformDustNoise.hpp>
#include <rmagine/noise/RelGaussianNoise.hpp>

#include <gazebo/common/Console.hh>

#include <radarays_ros/RadarCPU.hpp>

#include <radarays_ros/ros_helper.h>
#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
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


RadaRaysEmbree::RadaRaysEmbree()
:Base() // if sensor is base class: Base(sensors::RAY)
{
    gzdbg << "[RadaRaysEmbree] Constructed." << std::endl;
}

RadaRaysEmbree::~RadaRaysEmbree()
{
    gzdbg << "[RadaRaysEmbree] Destroyed." << std::endl;
}

void RadaRaysEmbree::loadDetectorSDF()
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

void RadaRaysEmbree::loadParamsSDF()
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
            gzdbg << "[RadaRaysEmbree] Loaded vertical scan: phi.min="
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

void RadaRaysEmbree::loadParams()
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
        radarays_ros::msg::RadarMaterial material_wall_stone;
        material_wall_stone = m_default_material;
        m_params.materials.data[1] = material_wall_stone;
    }

    size_t n_default_objects = 100;

    m_object_materials.resize(n_default_objects);
    for(size_t i=0; i<n_default_objects; i++)
    {
        m_object_materials[i] = 1; // wall stone
    }

    // find real materials


    // this->sdf->GetElement("ray");


    m_polar_image = cv::Mat_<unsigned char>(0, m_radar_model.theta.size);


    loadParamsSDF();


    // material properties
    
    


}

void RadaRaysEmbree::searchForMaterials()
{
    m_object_materials.resize(0);
    // 0: air, 1: default for objects without <radarays_material>
    m_params.materials.data.resize(2);
    radarays_ros::msg::RadarMaterial material_air;
    material_air.velocity = 0.3; // m/ns
    material_air.ambient  = 1.0;
    material_air.diffuse  = 0.0;
    material_air.specular = 1.0;
    m_params.materials.data[0] = material_air;
    m_params.materials.data[1] = m_default_material;


    sdf::ElementPtr world_sdf = this->world->SDF();
    sdf::ElementPtr model_sdf = world_sdf->GetElement("model");

    for(sdf::ElementPtr model_sdf = world_sdf->GetElement("model");
        model_sdf;
        model_sdf = model_sdf->GetNextElement("model"))
    {
        std::string model_name = model_sdf->GetAttribute("name")->GetAsString();

        for(sdf::ElementPtr link_sdf = model_sdf->GetElement("link");
            link_sdf;
            link_sdf = link_sdf->GetNextElement("link"))
        {
            std::string link_name = link_sdf->GetAttribute("name")->GetAsString();
            
        
            for(sdf::ElementPtr vis_sdf = link_sdf->GetElement("visual");
                vis_sdf;
                vis_sdf = vis_sdf->GetNextElement("visual"))
            {
                std::string vis_name = vis_sdf->GetAttribute("name")->GetAsString();
                
                sdf::ElementPtr mat_sdf;
                if(vis_sdf->HasElement("radarays_material"))
                {
                    mat_sdf = vis_sdf->GetElement("radarays_material");
                }

                radarays_ros::msg::RadarMaterial material;
                
                if(mat_sdf)
                {
                    gzdbg << "Found RadaRays material in: " << model_name << "/" << link_name << "/" << vis_name << std::endl;
                    material.velocity = mat_sdf->Get<float>("velocity");
                    material.ambient = mat_sdf->Get<float>("ambient");
                    material.diffuse = mat_sdf->Get<float>("diffuse");
                    material.specular = mat_sdf->Get<float>("specular");
                } else {
                    // wall stone? defaults
                    material = m_default_material;
                }
                    

                // figure out id in map

                std::string name = model_name + "::" + link_name + "::" + vis_name;

                // find model in embree_map
                if(m_map_mutex)
                {
                    m_map_mutex->lock_shared();
                }

                int obj_id = -1;
                if(m_map)
                {
                    for(auto elem : m_map->scene->geometries())
                    {
                        if(elem.second->name == name)
                        {
                            obj_id = elem.first;
                            break;
                        }
                    }
                } else {
                    gzdbg << "ERROR: no embree map" << std::endl;
                    if(m_map_mutex)
                    {
                        m_map_mutex->unlock_shared();
                    }
                    continue;
                }

                if(m_map_mutex)
                {
                    m_map_mutex->unlock_shared();
                }

                if(obj_id >= 0)
                {

                    m_params.materials.data.push_back(material);
                    size_t mat_id = m_params.materials.data.size() - 1;

                    // objects without a material entry use the default
                    while(m_object_materials.size() <= static_cast<size_t>(obj_id))
                    {
                        m_object_materials.push_back(1);
                    }
                    // object -> material
                    m_object_materials[obj_id] = mat_id;
                }
            }
        }
    }

}

void RadaRaysEmbree::Init()
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

    // Declare ROS2 parameters (radar model config)
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

    // Pointcloud noise parameters
    // Detection model: SDF values are the defaults, tunable at runtime
    for(auto& elem : m_detector.params.fields())
    {
        m_nh_p->declare_parameter("det." + elem.first, *elem.second);
    }

    // Apply initial config from parameters
    radarays_ros::RadarModelConfig initial_cfg;
    initial_cfg.beam_width = m_nh_p->get_parameter("beam_width").as_double();
    initial_cfg.n_samples = m_nh_p->get_parameter("n_samples").as_int();
    initial_cfg.n_reflections = m_nh_p->get_parameter("n_reflections").as_int();
    initial_cfg.range_min = m_nh_p->get_parameter("range_min").as_double();
    initial_cfg.range_max = m_nh_p->get_parameter("range_max").as_double();
    initial_cfg.resolution = m_nh_p->get_parameter("resolution").as_double();
    initial_cfg.n_cells = m_nh_p->get_parameter("n_cells").as_int();
    initial_cfg.signal_max = m_nh_p->get_parameter("signal_max").as_double();
    // every declared value is applied: "ros2 param get" shows what is simulated
    initial_cfg.scroll_image = m_nh_p->get_parameter("scroll_image").as_int();
    initial_cfg.beam_sample_dist = m_nh_p->get_parameter("beam_sample_dist").as_int();
    initial_cfg.beam_sample_dist_normal_p_in_cone = m_nh_p->get_parameter("beam_sample_dist_normal_p_in_cone").as_double();
    initial_cfg.signal_denoising = m_nh_p->get_parameter("signal_denoising").as_int();
    initial_cfg.signal_denoising_triangular_width = m_nh_p->get_parameter("signal_denoising_triangular_width").as_int();
    initial_cfg.signal_denoising_triangular_mode = m_nh_p->get_parameter("signal_denoising_triangular_mode").as_int();
    initial_cfg.signal_denoising_gaussian_width = m_nh_p->get_parameter("signal_denoising_gaussian_width").as_int();
    initial_cfg.signal_denoising_gaussian_mode = m_nh_p->get_parameter("signal_denoising_gaussian_mode").as_int();
    initial_cfg.signal_denoising_mb_width = m_nh_p->get_parameter("signal_denoising_mb_width").as_int();
    initial_cfg.signal_denoising_mb_mode = m_nh_p->get_parameter("signal_denoising_mb_mode").as_int();
    initial_cfg.ambient_noise = m_nh_p->get_parameter("ambient_noise").as_int();
    initial_cfg.ambient_noise_energy_max = m_nh_p->get_parameter("ambient_noise_energy_max").as_double();
    initial_cfg.ambient_noise_energy_min = m_nh_p->get_parameter("ambient_noise_energy_min").as_double();
    initial_cfg.ambient_noise_energy_loss = m_nh_p->get_parameter("ambient_noise_energy_loss").as_double();
    initial_cfg.ambient_noise_at_signal_0 = m_nh_p->get_parameter("ambient_noise_at_signal_0").as_double();
    initial_cfg.ambient_noise_at_signal_1 = m_nh_p->get_parameter("ambient_noise_at_signal_1").as_double();
    initial_cfg.ambient_noise_perlin_scale_low = m_nh_p->get_parameter("ambient_noise_perlin_scale_low").as_double();
    initial_cfg.ambient_noise_perlin_scale_high = m_nh_p->get_parameter("ambient_noise_perlin_scale_high").as_double();
    initial_cfg.ambient_noise_perlin_p_low = m_nh_p->get_parameter("ambient_noise_perlin_p_low").as_double();
    initial_cfg.record_multi_reflection = m_nh_p->get_parameter("record_multi_reflection").as_bool();
    initial_cfg.record_multi_path = m_nh_p->get_parameter("record_multi_path").as_bool();
    initial_cfg.multipath_threshold = m_nh_p->get_parameter("multipath_threshold").as_double();
    initial_cfg.include_motion = m_nh_p->get_parameter("include_motion").as_bool();
    m_resample_every_frame = m_nh_p->get_parameter("resample_every_frame").as_bool();
    {
        std::lock_guard<std::mutex> lock(m_cfg_mutex);
        updateDynCfg(initial_cfg, 0);
    }

    // Register parameter change callback
    m_param_callback = m_nh_p->add_on_set_parameters_callback(
        std::bind(&RadaRaysEmbree::onParameterChange, this, std::placeholders::_1));

    if(!m_map)
    {
        m_waiting_for_map = true;
    }
}

rcl_interfaces::msg::SetParametersResult RadaRaysEmbree::onParameterChange(
    const std::vector<rclcpp::Parameter>& params)
{
    for(const auto& param : params)
    {
        const std::string& name = param.get_name();
        std::string reason;
        if((name == "n_samples" || name == "n_reflections") && param.as_int() < 1)
        {
            reason = name + " must be >= 1";
        }
        else if(name.rfind("det.", 0) == 0)
        {
            reason = radarays_gazebo_plugins::checkDetectorParam(name.substr(4), param.as_double());
        }
        if(!reason.empty())
        {
            rcl_interfaces::msg::SetParametersResult rejected;
            rejected.successful = false;
            rejected.reason = reason;
            return rejected;
        }
    }

    // the sensor thread reads the configuration while simulating
    std::lock_guard<std::mutex> lock(m_cfg_mutex);

    radarays_ros::RadarModelConfig cfg = m_cfg;

    for(const auto& param : params)
    {
        const std::string& name = param.get_name();
        if(name.rfind("det.", 0) == 0)
        {
            for(auto& elem : m_detector.params.fields())
            {
                if(elem.first == name.substr(4))
                {
                    *elem.second = param.as_double();
                }
            }
        }
        else if(name == "n_cells")                              cfg.n_cells = param.as_int();
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
        else if(name == "resample_every_frame")          m_resample_every_frame = param.as_bool();
    }

    updateDynCfg(cfg, 0);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void RadaRaysEmbree::Load(const std::string& world_name)
{
    Base::Load(world_name);
    gzdbg << "LOAD - RadaRaysEmbree!" << std::endl;
}

void RadaRaysEmbree::simulate(rm::Transform Tsm)
{
    // 
    if(m_polar_image.rows != m_cfg.n_cells)
    {
        gzdbg << "[RadaRaysEmbree] Resize canvas to " << m_cfg.n_cells << std::endl;
        m_polar_image.resize(m_cfg.n_cells);
        gzdbg << "[RadaRaysEmbree] Resizing canvas - done." << std::endl;
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


    DirectedWave wave;
    wave.energy       =  1.0;
    wave.polarization =  0.5;
    wave.frequency    = 76.5; // GHz
    wave.velocity     =  0.3; // m / ns - speed in air
    wave.material_id  =  0;   // air
    wave.time         =  0.0; // ns
    wave.ray.orig = {0.0, 0.0, 0.0};
    wave.ray.dir = {1.0, 0.0, 0.0};


    const int n_angles = m_polar_image.cols;


    if(m_resample || m_resample_every_frame)
    {
        m_waves_start = sample_cone_local(
            wave,
            m_params.model.beam_width,
            m_params.model.n_samples,
            m_cfg.beam_sample_dist,
            m_cfg.beam_sample_dist_normal_p_in_cone);
        m_resample = false;
    }


    int n_phi = m_radar_model.phi.size;

    m_radar_points_gt.clear();

    // All beams of one pass are traced with a single call. Tracing every beam
    // on its own (one model, one result buffer and one call per beam) cost a
    // full CPU core for work that takes a few milliseconds.
    if(!m_sim)
    {
        m_sim = std::make_shared<rm::OnDnSimulatorEmbree>(m_map);
        m_sim->setTsb(rm::Transform::Identity());
    }

    // the polar image is only built when somebody listens to it
    const bool make_image = m_image_requested;

    struct Beam
    {
        float ux, uy, uz;       // direction in the sensor frame
        float radial_velocity;  // Doppler of a static world
        float solid_angle;      // per ray
    };

    const size_t n_beams = static_cast<size_t>(n_angles) * n_phi;
    const size_t n_start = std::max<size_t>(m_waves_start.size(), 1);
    const float d_theta = fabs(m_radar_model.theta.inc);
    const float d_phi = (n_phi > 1) ? fabs(m_radar_model.phi.inc) : d_theta;

    std::vector<Beam> beams(n_beams);
    std::vector<DirectedWave> waves;
    std::vector<uint32_t> wave_beam;
    waves.reserve(n_beams * n_start);
    wave_beam.reserve(n_beams * n_start);

    for(size_t angle_id = 0; angle_id < (size_t)n_angles; angle_id++)
    {
        const float theta = m_radar_model.getTheta(angle_id);
        for(size_t phi_id = 0; phi_id < (size_t)n_phi; phi_id++)
        {
            const float phi_angle = m_radar_model.phi.min + phi_id * m_radar_model.phi.inc;
            const uint32_t beam_id = angle_id * n_phi + phi_id;

            Beam& beam = beams[beam_id];
            // positive phi pitches down
            beam.ux = cos(phi_angle) * cos(theta);
            beam.uy = cos(phi_angle) * sin(theta);
            beam.uz = -sin(phi_angle);
            beam.radial_velocity = -(m_sensor_velocity.x * beam.ux
                + m_sensor_velocity.y * beam.uy + m_sensor_velocity.z * beam.uz);
            beam.solid_angle = d_theta * d_phi * fabs(cos(phi_angle)) / static_cast<float>(n_start);

            const rm::Quaternion R = rm::EulerAngles{0.0, phi_angle, theta};
            for(const DirectedWave& w_start : m_waves_start)
            {
                DirectedWave w = w_start;
                w.ray.dir = R * w_start.ray.dir;
                // start outside of the own vehicle, the skipped distance still counts
                w.ray.orig = w.ray.dir * m_self_clearance;
                w.time = m_self_clearance / w.velocity;
                waves.push_back(w);
                wave_beam.push_back(beam_id);
            }
        }
    }

    rm::Memory<rm::Transform> Tsms(1);
    Tsms[0] = Tsm;

    m_returns.clear();
    std::vector<std::vector<Signal> > signals_per_angle;
    if(make_image)
    {
        signals_per_angle.resize(n_angles);
    }

    auto add_signal = [&](uint32_t beam_id, double time, double strength)
    {
        if(make_image)
        {
            signals_per_angle[beam_id / n_phi].push_back({time, strength});
        }

        // the wave travels 0.3 m/ns, there and back
        const float signal_dist = 0.3 * time / 2.0;
        if(std::isfinite(signal_dist) && std::isfinite(strength)
            && signal_dist >= m_radar_model.range.min
            && signal_dist <= m_radar_model.range.max
            && strength > m_wave_energy_threshold)
        {
            const Beam& beam = beams[beam_id];
            radarays_gazebo_plugins::RadarReturn ret;
            ret.range = signal_dist;
            ret.dx = beam.ux;
            ret.dy = beam.uy;
            ret.dz = beam.uz;
            ret.power = strength;
            ret.radial_velocity = beam.radial_velocity;
            ret.solid_angle = beam.solid_angle;
            m_returns.push_back(ret);
        }
    };

    using ResT = rm::Bundle<
        rm::Hits<rm::RAM>,
        rm::Ranges<rm::RAM>,
        rm::Normals<rm::RAM>,
        rm::ObjectIds<rm::RAM>
    >;

    for(size_t pass_id = 0; pass_id < (size_t)m_params.model.n_reflections && !waves.empty(); pass_id++)
    {
        rm::OnDnModel model = make_model(waves);
        m_sim->setModel(model);

        ResT results;
        results.hits.resize(model.size());
        results.ranges.resize(model.size());
        results.normals.resize(model.size());
        results.object_ids.resize(model.size());

        m_sim->simulate(Tsms, results);

        if(pass_id == 0)
        {
            // ground truth: first ray of every beam
            for(size_t i = 0; i < waves.size(); i += n_start)
            {
                if(!results.hits[i])
                {
                    continue;
                }
                const float range = results.ranges[i] + m_self_clearance;
                if(range >= m_radar_model.range.min && range <= m_radar_model.range.max)
                {
                    const Beam& beam = beams[wave_beam[i]];
                    radarays_gazebo_plugins::RadarPoint pt;
                    pt.x = range * beam.ux;
                    pt.y = range * beam.uy;
                    pt.z = range * beam.uz;
                    pt.intensity = 1.0;
                    pt.velocity = 0.0f;
                    m_radar_points_gt.push_back(pt);
                }
            }
        }

        std::vector<DirectedWave> waves_new;
        std::vector<uint32_t> wave_beam_new;
        waves_new.reserve(waves.size() * 2);
        wave_beam_new.reserve(waves.size() * 2);

        for(size_t i=0; i < waves.size(); i++)
        {
            if(!results.hits[i])
            {
                continue;
            }

            const DirectedWave& wave = waves[i];
            const uint32_t beam_id = wave_beam[i];
            const float wave_range = results.ranges[i];
            const rmagine::Vector surface_normal = results.normals[i].normalize();
            const unsigned int obj_id = results.object_ids[i];

            if(obj_id > 10000)
            {
                continue;
            }

            const DirectedWave incidence = wave.move(wave_range);

            DirectedWave reflection = incidence;
            DirectedWave refraction = incidence;

            if(static_cast<int>(incidence.material_id) == m_material_id_air)
            {
                // models spawned after the material search are not in the table
                refraction.material_id = (obj_id < m_object_materials.size())
                    ? m_object_materials[obj_id] : 1;
            } else {
                refraction.material_id = m_material_id_air;
            }

            float v_refraction = 1.0;
            if(incidence.material_id != refraction.material_id)
            {
                v_refraction = m_params.materials.data[refraction.material_id].velocity;
            } else {
                v_refraction = incidence.velocity;
            }

            auto res = fresnel(surface_normal, incidence, v_refraction);

            reflection.ray.dir = res.first.ray.dir;
            reflection.energy = res.first.energy;

            if(reflection.energy > m_wave_energy_threshold)
            {
                waves_new.push_back(reflection);
                wave_beam_new.push_back(beam_id);

                if(static_cast<int>(reflection.material_id) == m_material_id_air)
                {
                    auto material = m_params.materials.data[refraction.material_id];

                    double incidence_angle = get_incidence_angle(surface_normal, incidence);
                    double return_energy_path = back_reflection_shader(
                        incidence_angle,
                        reflection.energy,
                        material.ambient,
                        material.diffuse,
                        material.specular
                    );

                    // signal travelling back along the path it came
                    if(pass_id == 0 || m_cfg.record_multi_reflection)
                    {
                        add_signal(beam_id, incidence.time * 2.0, return_energy_path);
                    }

                    // signal travelling straight back to the sensor
                    if(pass_id > 0 && m_cfg.record_multi_path)
                    {
                        rm::Vector dir_sensor_to_hit = reflection.ray.orig;
                        const double distance_between_sensor_and_hit = dir_sensor_to_hit.l2norm();
                        dir_sensor_to_hit.normalizeInplace();
                        const double time_to_sensor = distance_between_sensor_and_hit / reflection.velocity;

                        double sensor_view_scalar = wave.ray.dir.dot(dir_sensor_to_hit);

                        double angle_between_reflection_and_sensor_dir
                            = angle_between(-reflection.ray.dir, dir_sensor_to_hit);

                        if(sensor_view_scalar > m_cfg.multipath_threshold)
                        {
                            double return_energy_air = back_reflection_shader(
                                angle_between_reflection_and_sensor_dir,
                                reflection.energy,
                                material.ambient,
                                material.diffuse,
                                material.specular
                            );
                            add_signal(beam_id, incidence.time + time_to_sensor, return_energy_air);
                        }
                    }
                }
            }

            refraction.ray.dir = res.second.ray.dir;
            refraction.energy = res.second.energy;

            if(refraction.energy > m_wave_energy_threshold)
            {
                waves_new.push_back(refraction);
                wave_beam_new.push_back(beam_id);
            }
        }

        // step off the surface so the next pass does not hit it again
        const float skip_dist = 0.001;
        for(size_t i=0; i<waves_new.size(); i++)
        {
            waves_new[i].moveInplace(skip_dist);
        }

        waves.swap(waves_new);
        wave_beam.swap(wave_beam_new);
    }

    // Signals -> polar image, one column per azimuth
    for(size_t angle_id = 0; make_image && angle_id < (size_t)n_angles; angle_id++)
    {
        const std::vector<Signal>& signals_merged = signals_per_angle[angle_id];

        cv::Mat_<float> slice(m_polar_image.rows, 1, 0.0);

        float max_val = 0.0;
        for(size_t i=0; i<signals_merged.size(); i++)
        {
            auto signal = signals_merged[i];
            float half_time = signal.time / 2.0;
            float signal_dist = 0.3 * half_time;

            int cell = static_cast<int>(signal_dist / m_cfg.resolution);
            if(cell < slice.rows)
            {
                if(m_cfg.signal_denoising > 0)
                {
                    for(int vid = 0; vid < static_cast<int>(denoising_weights.size()); vid++)
                    {
                        int glob_id = vid + cell - denoising_mode;
                        if(glob_id > 0 && glob_id < slice.rows)
                        {
                            slice.at<float>(glob_id, 0) += signal.strength * denoising_weights[vid];

                            if(slice.at<float>(glob_id, 0) > max_val)
                            {
                                max_val = slice.at<float>(glob_id, 0);
                            }
                        }
                    }
                } else {
                    slice.at<float>(cell, 0) = std::max(
                        slice.at<float>(cell, 0),
                        (float)signal.strength
                    );

                    if(slice.at<float>(cell, 0) > max_val)
                    {
                        max_val = slice.at<float>(cell, 0);
                    }
                }
            }
        }

        int col = (m_cfg.scroll_image + angle_id) % m_polar_image.cols;

        if(m_cfg.ambient_noise)
        {
            max_val = std::max((float)m_cfg.ambient_noise_energy_max, max_val);

            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);
            double random_begin = dist_uni(gen) * 1000.0;

            double scale_low = m_cfg.ambient_noise_perlin_scale_low;
            double scale_high = m_cfg.ambient_noise_perlin_scale_high;
            double p_low = m_cfg.ambient_noise_perlin_p_low;

            for(int i=0; i<slice.rows; i++)
            {
                float signal = slice.at<float>(i);

                // 1 = uniform, 2 = Perlin. Unknown values fall back to uniform
                // instead of reading an uninitialized value.
                double p = dist_uni(gen);
                if(m_cfg.ambient_noise == 2)
                {
                    p = perlin_noise_hilo(random_begin, random_begin,
                            static_cast<double>(col), static_cast<double>(i),
                            scale_low, scale_high,
                            p_low);
                }

                float signal_min = 0;
                float signal_max = max_val;
                float signal_amp = signal_max - signal_min;

                float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

                float noise_at_0 = signal_amp * m_cfg.ambient_noise_at_signal_0;
                float noise_at_1 = signal_amp * m_cfg.ambient_noise_at_signal_1;

                float signal__ = std::pow(signal_, 4.0);

                float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

                float noise_energy_max = signal_max * m_cfg.ambient_noise_energy_max;
                float noise_energy_min = signal_max * m_cfg.ambient_noise_energy_min;
                float energy_loss = m_cfg.ambient_noise_energy_loss;

                float y_noise = noise_amp * p;

                float x = (static_cast<float>(i) + 0.5) * m_cfg.resolution;

                y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * x) + noise_energy_min;
                y_noise = abs(y_noise);

                slice.at<float>(i) = signal + y_noise;
            }
        }

        // columns without any return stay black instead of dividing by zero
        slice *= (max_val > 0.0f) ? m_cfg.signal_max / max_val : 0.0f;
        slice.convertTo(m_polar_image.col(col), CV_8UC1);
    }

    // Detection: sparse point cloud like the real device
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


}

bool RadaRaysEmbree::UpdateImpl(const bool _force)
{
    if(!m_map)
    {
        if(!m_waiting_for_map)
        {
            gzdbg << "[RadaRaysEmbree] Start waiting for RmagineEmbreeMap..." << std::endl;
            m_waiting_for_map = true;
        }
        return false;
    } else {
        if(m_waiting_for_map) {
            gzdbg << "[RadaRaysEmbree] Map received!" << std::endl;
            m_waiting_for_map = false;
        }

        if(!m_materials_searched)
        {
            gzdbg << "Search for materials!!" << std::endl;
            searchForMaterials();
            m_materials_searched = true;
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

    // fill global simulation buffer
    {
        // parameters may change from the ROS thread
        std::lock_guard<std::mutex> cfg_lock(m_cfg_mutex);
        simulate(Tsm);
    }

    if(m_map_mutex)
    {
        m_map_mutex->unlock_shared();
    }

    return Base::UpdateImpl(_force);
}


void RadaRaysEmbree::updateDynCfg(
    radarays_ros::RadarModelConfig &config,
    uint32_t level)
{
    RCLCPP_INFO(m_nh_p->get_logger(), "RadaRaysEmbree - Changing Model");
    
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
    m_params.model.beam_width = config.beam_width * M_PI / 180.0;
    m_params.model.n_samples = config.n_samples;
    m_params.model.n_reflections = config.n_reflections;

    m_cfg = config;
}


GZ_REGISTER_STATIC_SENSOR("radarays_embree", RadaRaysEmbree)

} // namespace sensors

} // namespace gazebo