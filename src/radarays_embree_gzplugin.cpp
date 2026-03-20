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
#include <omp.h>

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

void RadaRaysEmbree::loadParamsSDF()
{
    if (!this->sdf->HasElement("ray")) {
        return;
    }

    sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
    if (rayElem->HasElement("scan")) {
        sdf::ElementPtr scanElem = rayElem->GetElement("scan");
        if (scanElem->HasElement("horizontal")) {
            auto h = scanElem->GetElement("horizontal");
            m_radar_model.theta.size = h->Get<int>("samples");
            // theta.inc and theta.min define the full-circle radar sweep
            // keep the default inc = -(2*PI)/size, don't use SDF increment
            m_radar_model.theta.inc = -(2.0 * M_PI) / m_radar_model.theta.size;
        }
        if (scanElem->HasElement("vertical")) {
            auto v = scanElem->GetElement("vertical");
            m_radar_model.phi.min = v->Get<float>("min_angle");
            m_radar_model.phi.inc = v->Get<float>("increment");
            m_radar_model.phi.size = v->Get<int>("samples");
            std::cout << "[RadaRaysEmbree] Loaded vertical scan: phi.min="
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
        material_wall_stone.velocity = 0.0;
        material_wall_stone.ambient  = 1.0;
        material_wall_stone.diffuse  = 0.0;
        material_wall_stone.specular = 3000.0;
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
    
    
    // TODO:
    // this is how it was in a static environment:
    // 
    // m_params.materials = loadRadarMaterialsFromParameterServer(m_nh_p);
    // m_nh_p->getParam("object_materials", m_object_materials);
    // m_nh_p->getParam("material_id_air", m_material_id_air);
    // 
    // But Gazebo environments are changing. Maybe we can fetch custom 
    // materials from Gazebo? 
    // - A first quick solution would be to pass a list of materials
    //   as before and does not allow to change the world (at own risk)
    // 
    // Or maybe: For each object spawn something is added to 
    //   dynamic reconfigure?





    // TODO: load additional params from gazebo - xml (instead of ros paremeters)
}

void RadaRaysEmbree::searchForMaterials()
{
    m_object_materials.resize(0);
    m_params.materials.data.resize(1);
    radarays_ros::msg::RadarMaterial material_air;
    material_air.velocity = 0.3; // m/ns
    material_air.ambient  = 1.0;
    material_air.diffuse  = 0.0;
    material_air.specular = 1.0;
    m_params.materials.data[0] = material_air;



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
                
                sdf::ElementPtr mat_sdf = vis_sdf->GetElement("radarays_material");

                radarays_ros::msg::RadarMaterial material;
                
                if(mat_sdf)
                {
                    std::cout << "Found RadaRays material in: " << model_name << "/" << link_name << "/" << vis_name << std::endl;
                    material.velocity = mat_sdf->Get<float>("velocity");
                    material.ambient = mat_sdf->Get<float>("ambient");
                    material.diffuse = mat_sdf->Get<float>("diffuse");
                    material.specular = mat_sdf->Get<float>("specular");
                } else {
                    // wall stone? defaults
                    std::cout << "-- Setting RadaRays materials to defaults" << std::endl;
                    material.velocity = 0.0;
                    material.ambient  = 1.0;
                    material.diffuse  = 0.0;
                    material.specular = 3000.0;
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
                    std::cout << "SEARCH FOR: " << name << std::endl;
                    for(auto elem : m_map->scene->geometries())
                    {
                        std::cout << elem.first << ".: " << elem.second->name << std::endl;
                        if(elem.second->name == name)
                        {
                            std::cout << "FOUND OBJ ID: " << elem.first << std::endl;
                            obj_id = elem.first;
                            break;
                        }
                    }
                } else {
                    std::cout << "ERROR: no embree map" << std::endl;
                    continue;
                }

                if(m_map_mutex)
                {
                    m_map_mutex->unlock_shared();
                }

                std::cout << "FOUND MODEL: " << name << std::endl;
                if(obj_id >= 0)
                {
                    std::cout << " -> embree map geometry id: " << obj_id << std::endl;

                    m_params.materials.data.push_back(material);
                    size_t mat_id = m_params.materials.data.size() - 1;

                    // fill with unknown with air materials
                    while(m_object_materials.size() <= obj_id)
                    {
                        m_object_materials.push_back(0);
                    }
                    // object -> material
                    m_object_materials[obj_id] = mat_id;
                }
            }
        }
        
        // TODO:
        // full_name = model/link/...
        // put full name to embree map elements and compare instead
        // thus, we can apply materials to model parts
        
    }

}

void RadaRaysEmbree::Init()
{
    Base::Init();

    m_nh_p = gazebo_ros::Node::Get(this->sdf);

    loadParams();

    // Declare ROS2 parameters (radar model config)
    m_nh_p->declare_parameter("n_cells", 200);
    m_nh_p->declare_parameter("resolution", 0.1);
    m_nh_p->declare_parameter("scroll_image", 0);
    m_nh_p->declare_parameter("range_min", 0.2);
    m_nh_p->declare_parameter("range_max", 100.0);
    m_nh_p->declare_parameter("beam_width", 3.0);
    m_nh_p->declare_parameter("n_samples", 10);
    m_nh_p->declare_parameter("n_reflections", 1);
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
    m_nh_p->declare_parameter("record_multi_reflection", false);
    m_nh_p->declare_parameter("record_multi_path", false);
    m_nh_p->declare_parameter("multipath_threshold", 0.9);
    m_nh_p->declare_parameter("include_motion", false);

    // Apply initial config from parameters
    radarays_ros::RadarModelConfig initial_cfg;
    updateDynCfg(initial_cfg, 0);

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
    radarays_ros::RadarModelConfig cfg = m_cfg;

    for(const auto& param : params)
    {
        const std::string& name = param.get_name();
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
    }

    updateDynCfg(cfg, 0);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void RadaRaysEmbree::Load(const std::string& world_name)
{
    Base::Load(world_name);
    std::cout << "LOAD - RadaRaysEmbree!" << std::endl;
}

void RadaRaysEmbree::simulate(rm::Transform Tsm)
{
    // 
    if(m_polar_image.rows != m_cfg.n_cells)
    {
        // m_polar_image = cv::Mat_<unsigned char>(n_cells, radar_model.theta.size);
        std::cout << "[RadaRaysEmbree] Resize canvas to " << m_cfg.n_cells << std::endl;
        m_polar_image.resize(m_cfg.n_cells);
        std::cout << "[RadaRaysEmbree] Resizing canvas - done." << std::endl;
    }

    // std::cout << "Fill canvas: " << m_polar_image.cols << "x" << m_polar_image.rows << std::endl;
    m_polar_image.setTo(cv::Scalar(0));

    std::vector<float> denoising_weights;
    int denoising_mode = 0;

    if(m_cfg.signal_denoising > 0)
    {
        // std::cout << "Signal Denoising: ";
        if(m_cfg.signal_denoising == 1)
        {
            // std::cout << "Triangular";
            denoising_mode = m_cfg.signal_denoising_triangular_mode * m_cfg.signal_denoising_triangular_width;
            denoising_weights = make_denoiser_triangular(
                m_cfg.signal_denoising_triangular_width,
                denoising_mode
            );
            
        } else if(m_cfg.signal_denoising == 2) {
            // std::cout << "Gaussian";
            denoising_mode = m_cfg.signal_denoising_gaussian_mode * m_cfg.signal_denoising_gaussian_width;
            denoising_weights = make_denoiser_gaussian(
                m_cfg.signal_denoising_gaussian_width,
                denoising_mode
            );

        } else if(m_cfg.signal_denoising == 3) {
            // std::cout << "Maxwell Boltzmann";
            denoising_mode = m_cfg.signal_denoising_mb_mode * m_cfg.signal_denoising_mb_width;
            denoising_weights = make_denoiser_maxwell_boltzmann(
                m_cfg.signal_denoising_mb_width,
                denoising_mode
            );
        }
        // std::cout << std::endl;

        // scale so that mode has weight 1
        // if(false)
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

    // el = sw();

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    // std::random_device                      rand_dev;
    // std::mt19937                            gen(rand_dev());
    // std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);


    // without motion: update Tsm only once
    // if(!m_cfg.include_motion)
    // {
    //     // if(!updateTsm())
    //     // {
    //     //     std::cout << "Couldn't get Transform between sensor and map. Skipping..." << std::endl;
    //     //     return msg;
    //     // }
    // }

    if(m_resample)
    {
        m_waves_start = sample_cone_local(
            wave,
            m_params.model.beam_width,
            m_params.model.n_samples,
            m_cfg.beam_sample_dist,
            m_cfg.beam_sample_dist_normal_p_in_cone);
        m_resample = false;
    }

    rm::StopWatch sw_radar_sim;
    sw_radar_sim();

    int n_phi = m_radar_model.phi.size;

    // Thread-local storage for radar points
    std::vector<std::vector<radarays_gazebo_plugins::RadarPoint>> thread_radar_points;
    {
        int max_threads = omp_get_max_threads();
        thread_radar_points.resize(max_threads);
    }

    #pragma omp parallel for
    for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
    {

        int tid = 0;
        {
            tid = omp_get_thread_num();
        }

        auto sims_it = m_sims.find(tid);
        if(m_sims.find(tid) == m_sims.end())
        {
            m_sims[tid] = std::make_shared<rm::OnDnSimulatorEmbree>(m_map);
            sims_it = m_sims.find(tid);
            auto Tsb = rm::Transform::Identity();
            sims_it->second->setTsb(Tsb);

            #pragma omp critical
            std::cout << "Created new simulator for thread " << tid << std::endl;
        }
        auto sim = sims_it->second;

        if(!sim)
        {
            std::cout << "ERROR!! Sim shared ptr empty" << std::endl;
        }

        // Collect all signals across phi for polar image (max-merge)
        std::vector<Signal> signals_merged;

        for(size_t phi_id = 0; phi_id < n_phi; phi_id++)
        {
            float phi_angle = m_radar_model.phi.min + phi_id * m_radar_model.phi.inc;

            std::vector<DirectedWave> waves = m_waves_start;

            rm::OnDnModel model = make_model(waves);
            sim->setModel(model);

            rm::Transform Tas;
            Tas.R = rm::EulerAngles{0.0, phi_angle, m_radar_model.getTheta(angle_id)};
            Tas.t = m_radar_model.getOrigin(0, angle_id);

            rm::Transform Tam = Tsm * Tas;

            rm::Memory<rm::Transform> Tams(1);
            Tams[0] = Tam;

            std::vector<Signal> signals;
            ///////
            /// 1. Signal generation
            for(size_t pass_id = 0; pass_id < m_params.model.n_reflections; pass_id++)
            {
                using ResT = rm::Bundle<
                    rm::Hits<rm::RAM>,
                    rm::Ranges<rm::RAM>,
                    rm::Normals<rm::RAM>,
                    rm::ObjectIds<rm::RAM>
                >;

                ResT results;

                results.hits.resize(model.size());
                results.ranges.resize(model.size());
                results.normals.resize(model.size());
                results.object_ids.resize(model.size());

                sim->simulate(Tams, results);

                std::vector<DirectedWave> waves_new;

                for(size_t i=0; i < waves.size(); i++)
                {
                    DirectedWave wave = waves[i];
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

                    if(incidence.material_id == m_material_id_air)
                    {
                        refraction.material_id = m_object_materials[obj_id];
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

                        if(reflection.material_id == m_material_id_air)
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

                            if(pass_id == 0 || m_cfg.record_multi_reflection)
                            {
                                float time_back = incidence.time * 2.0;
                                signals.push_back({time_back, return_energy_path});
                            }

                            if(pass_id > 0 && m_cfg.record_multi_path)
                            {
                                Signal signal_air;

                                rm::Vector dir_sensor_to_hit = reflection.ray.orig;
                                const double distance_between_sensor_and_hit = dir_sensor_to_hit.l2norm();
                                dir_sensor_to_hit.normalizeInplace();
                                const double time_to_sensor = distance_between_sensor_and_hit / reflection.velocity;

                                double sensor_view_scalar = wave.ray.dir.dot(dir_sensor_to_hit);

                                double angle_between_reflection_and_sensor_dir
                                    = angle_between(-reflection.ray.dir, dir_sensor_to_hit);

                                if(m_cfg.record_multi_path
                                    && sensor_view_scalar > m_cfg.multipath_threshold)
                                {
                                    double return_energy_air = back_reflection_shader(
                                        angle_between_reflection_and_sensor_dir,
                                        reflection.energy,
                                        material.ambient,
                                        material.diffuse,
                                        material.specular
                                    );

                                    signal_air.time = incidence.time + time_to_sensor;
                                    signal_air.strength = return_energy_air;

                                    signals.push_back(signal_air);
                                }
                            }
                        }
                    }

                    refraction.ray.dir = res.second.ray.dir;
                    refraction.energy = res.second.energy;

                    if(refraction.energy > m_wave_energy_threshold)
                    {
                        waves_new.push_back(refraction);
                    }
                }

                float skip_dist = 0.001;
                for(size_t i=0; i<waves_new.size(); i++)
                {
                    waves_new[i].moveInplace(skip_dist);
                }

                waves = waves_new;

                if(pass_id < m_params.model.n_reflections - 1)
                {
                    model = make_model(waves);
                    sim->setModel(model);
                }
            }

            // Generate 3D radar points from signals for this (theta, phi) combination
            float theta = m_radar_model.getTheta(angle_id);
            for(size_t i=0; i<signals.size(); i++)
            {
                auto signal = signals[i];
                float half_time = signal.time / 2.0;
                float signal_dist = 0.3 * half_time;

                if(signal_dist >= m_radar_model.range.min && signal_dist <= m_radar_model.range.max
                   && signal.strength > m_wave_energy_threshold)
                {
                    float cos_phi = cos(phi_angle);
                    float sin_phi = sin(phi_angle);
                    float cos_theta = cos(theta);
                    float sin_theta = sin(theta);

                    radarays_gazebo_plugins::RadarPoint pt;
                    pt.x = signal_dist * cos_phi * cos_theta;
                    pt.y = signal_dist * cos_phi * sin_theta;
                    pt.z = -signal_dist * sin_phi;
                    pt.intensity = static_cast<float>(signal.strength);
                    thread_radar_points[tid].push_back(pt);
                }
            }

            // Merge signals into polar image signals (max-merge across all phi)
            for(auto& sig : signals)
            {
                signals_merged.push_back(sig);
            }

        } // end phi loop

        //////////////////
        /// 2. Signals -> Canvas (max-merge all phi signals for this theta)
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
                    for(int vid = 0; vid < denoising_weights.size(); vid++)
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

            for(size_t i=0; i<slice.rows; i++)
            {
                float signal = slice.at<float>(i);

                double p;

                if(m_cfg.ambient_noise == 1)
                {
                    p = dist_uni(gen);
                } else if(m_cfg.ambient_noise == 2)
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

        slice *= m_cfg.signal_max / max_val;
        slice.convertTo(m_polar_image.col(col), CV_8UC1);
    }

    // Merge thread-local radar points
    m_radar_points.clear();
    for(auto& pts : thread_radar_points)
    {
        m_radar_points.insert(m_radar_points.end(), pts.begin(), pts.end());
    }

    double el_radar_sim = sw_radar_sim();

    // std::cout << "SIM in " << el_radar_sim << "s" << std::endl;
    // std::cout << std::fixed << std::setprecision(8) << el_radar_sim << std::endl;
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
            std::cout << "Search for materials!!" << std::endl;
            searchForMaterials();
            m_materials_searched = true;
        }
    }

    // std::cout << "RADARAYS UPDATE" << std::endl;
    auto pose = this->parentEntity->WorldPose();
    rm::Transform Tbm = to_rm(pose);
    rm::Transform Tsm = Tbm * m_Tsb;

    // std::cout << "World Pose of sensor: " << Tsm << std::endl;
    if(m_map_mutex)
    {
        m_map_mutex->lock_shared();
    }

    // fill global simulation buffer
    simulate(Tsm);

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
    
    // z_offset
    // auto T = rm::Transform::Identity();
    // T.t.z = config.z_offset;
    // sim->setTsb(T);

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