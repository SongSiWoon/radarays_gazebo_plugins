#ifndef RADARAYS_RADAR_DETECTOR_H
#define RADARAYS_RADAR_DETECTOR_H

/**
 * FMCW point-cloud detection stage for RadaRays.
 *
 * Turns per-ray returns into the sparse detection list a real mmWave radar
 * (e.g. TI IWR6843) outputs:
 *   rays -> range/Doppler cells -> CA-CFAR + peak grouping -> one (or two)
 *   angle estimates per cell -> strongest N points.
 *
 * Every number is a parameter (SDF <radar_detector> element, also exposed as
 * ROS parameters "det.<name>"), so a specific device can be calibrated.
 * Defaults follow measurements of an IWR6843 (range bin 0.0996 m,
 * Doppler bin 0.218 m/s, <= 36 points per frame, intensity in dB).
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <radarays_gazebo_plugins/radar_point.h>

namespace radarays_gazebo_plugins
{

/// One ray-traced return, sensor frame (x forward, y left, z up)
struct RadarReturn
{
    float range;            ///< half of the travelled path [m]
    float dx, dy, dz;       ///< unit direction of the transmitted beam
    float power;            ///< reflected energy from the material shader
    float radial_velocity;  ///< [m/s], negative = approaching
    float solid_angle;      ///< solid angle this ray stands for [sr]
};

struct RadarDetectorParams
{
    // 1 = FMCW detection model, 0 = raw ray hits (debugging)
    double enabled = 1.0;

    // range axis
    double range_resolution = 0.0996;  // [m] range bin width
    double range_min = 0.5;            // [m] first reported range
    double range_max = 25.0;           // [m] last reported range
    double range_bias = 0.5;           // [m] added to the true range

    // Doppler axis
    double doppler_resolution = 0.218; // [m/s] Doppler bin width
    double doppler_bins = 32.0;        // number of bins, wraps beyond +-bins/2

    // angle of arrival field of view [deg]
    double fov_azimuth_min = -60.0;
    double fov_azimuth_max = 60.0;
    double fov_elevation_min = -30.0;
    double fov_elevation_max = 30.0;

    // link budget
    double gain_db = 88.0;                   // maps cell power to intensity [dB]
    double range_loss_exponent = 1.0;        // per-ray power ~ range^-exponent
    double near_range_rolloff = 2.0;         // [m] below this range the receiver attenuates
    double near_range_attenuation_db = 20.0; // attenuation at range 0 [dB]
    double antenna_azimuth_exponent = 1.0;   // two-way pattern cos(az)^(2*exponent)
    double antenna_elevation_exponent = 2.0; // two-way pattern cos(el)^(2*exponent)

    // detection
    double noise_floor_db = 34.0;      // noise level in intensity units [dB]
    double cfar_threshold_db = 8.0;    // required level above the local noise estimate
    double cfar_guard_cells = 2.0;
    double cfar_window_cells = 8.0;
    double cfar_mode = 0.0;            // noise estimate from the two training windows: 0 = average (CA),
                                       // 1 = smaller one (CASO, less masking next to strong targets), 2 = greater one (CAGO)
    double peak_grouping = 1.0;        // 1 = keep only local maxima in range/Doppler
    double detection_probability = 1.0;
    double max_points = 36.0;          // per frame, strongest first

    // angle estimation
    double angular_separation_deg = 30.0; // targets in one cell closer than this merge
    double multi_object_ratio = 0.5;      // 2nd target needs this fraction of the 1st
    double max_points_per_cell = 2.0;

    // fluctuation
    double speckle = 0.5;                  // 0 = none, 1 = full Rayleigh fading
    double speckle_position_cell = 0.02;   // [m] sensor motion that decorrelates fading
    double speckle_rotation_cell = 0.01;   // quaternion units (~1 deg)
    double glint = 0.0;                    // 0 = a cell is located at its power centroid. > 0 = the reflectors of a
                                           // cell fade individually, the estimated direction wanders between them
                                           // whenever the fading pattern changes (weights ~ Exp(1)^glint)
    double intensity_noise_db = 0.3;       // per frame
    double azimuth_noise_deg = 0.02;       // per frame
    double elevation_noise_deg = 0.2;      // per frame
    double false_alarm_rate = 0.0;         // mean number of noise points per frame

    // angle output of the device
    double azimuth_fft_size = 64.0;        // azimuth is the peak of an N point FFT: y/r is a multiple of 2/N (0 = off)
    double elevation_bias_deg = 0.0;       // constant offset of the elevation estimate (uncalibrated rx phases)

    // weak copies of detections far out in Doppler, caused by airframe vibration (motors running)
    double sideband_rate = 0.0;            // mean number per frame
    double sideband_velocity = 3.1;        // [m/s] offset from the true Doppler, either sign
    double sideband_velocity_spread = 0.2; // [m/s]
    double sideband_margin_db = 4.0;       // mean level above the detection threshold

    // near-field point(s) at a fixed range: antenna leakage, or the airframe when mounted on a drone.
    // Two lobes, each present with its own probability.
    double leakage_enabled = 1.0;
    double leakage_range = 0.498;
    double leakage_azimuth_deg = -51.5;
    double leakage_elevation_deg = 3.3;
    double leakage_intensity_db = 44.8;
    double leakage_presence = 1.0;             // probability per frame
    double leakage_azimuth_jitter_deg = 0.0;   // per frame, std dev
    double leakage_elevation_jitter_deg = 0.0;
    double leakage_intensity_jitter_db = 0.0;  // on top of intensity_noise_db
    double leakage2_presence = 0.0;            // second lobe, same range / elevation / intensity
    double leakage2_azimuth_deg = 35.0;

    // heat maps (only computed while requested, see RadarDetector::want_*)
    double heatmap_azimuth_bins = 64.0;           // columns of the range-azimuth map, spread over the azimuth FoV
    double heatmap_azimuth_resolution_deg = 15.0; // angular resolution of the array: blur across azimuth

    std::vector<std::pair<std::string, double*> > fields()
    {
        return {
            {"enabled", &enabled},
            {"range_resolution", &range_resolution},
            {"range_min", &range_min},
            {"range_max", &range_max},
            {"range_bias", &range_bias},
            {"doppler_resolution", &doppler_resolution},
            {"doppler_bins", &doppler_bins},
            {"fov_azimuth_min", &fov_azimuth_min},
            {"fov_azimuth_max", &fov_azimuth_max},
            {"fov_elevation_min", &fov_elevation_min},
            {"fov_elevation_max", &fov_elevation_max},
            {"gain_db", &gain_db},
            {"range_loss_exponent", &range_loss_exponent},
            {"near_range_rolloff", &near_range_rolloff},
            {"near_range_attenuation_db", &near_range_attenuation_db},
            {"antenna_azimuth_exponent", &antenna_azimuth_exponent},
            {"antenna_elevation_exponent", &antenna_elevation_exponent},
            {"noise_floor_db", &noise_floor_db},
            {"cfar_threshold_db", &cfar_threshold_db},
            {"cfar_guard_cells", &cfar_guard_cells},
            {"cfar_window_cells", &cfar_window_cells},
            {"cfar_mode", &cfar_mode},
            {"peak_grouping", &peak_grouping},
            {"detection_probability", &detection_probability},
            {"max_points", &max_points},
            {"angular_separation_deg", &angular_separation_deg},
            {"multi_object_ratio", &multi_object_ratio},
            {"max_points_per_cell", &max_points_per_cell},
            {"speckle", &speckle},
            {"speckle_position_cell", &speckle_position_cell},
            {"speckle_rotation_cell", &speckle_rotation_cell},
            {"glint", &glint},
            {"intensity_noise_db", &intensity_noise_db},
            {"azimuth_noise_deg", &azimuth_noise_deg},
            {"elevation_noise_deg", &elevation_noise_deg},
            {"false_alarm_rate", &false_alarm_rate},
            {"azimuth_fft_size", &azimuth_fft_size},
            {"elevation_bias_deg", &elevation_bias_deg},
            {"sideband_rate", &sideband_rate},
            {"sideband_velocity", &sideband_velocity},
            {"sideband_velocity_spread", &sideband_velocity_spread},
            {"sideband_margin_db", &sideband_margin_db},
            {"leakage_enabled", &leakage_enabled},
            {"leakage_range", &leakage_range},
            {"leakage_azimuth_deg", &leakage_azimuth_deg},
            {"leakage_elevation_deg", &leakage_elevation_deg},
            {"leakage_intensity_db", &leakage_intensity_db},
            {"leakage_presence", &leakage_presence},
            {"leakage_azimuth_jitter_deg", &leakage_azimuth_jitter_deg},
            {"leakage_elevation_jitter_deg", &leakage_elevation_jitter_deg},
            {"leakage_intensity_jitter_db", &leakage_intensity_jitter_db},
            {"leakage2_presence", &leakage2_presence},
            {"leakage2_azimuth_deg", &leakage2_azimuth_deg},
            {"heatmap_azimuth_bins", &heatmap_azimuth_bins},
            {"heatmap_azimuth_resolution_deg", &heatmap_azimuth_resolution_deg}
        };
    }
};

/// Sensor pose in the world, only used to key the fading pattern
struct RadarSensorPose
{
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
};

class RadarDetector
{
public:
    RadarDetectorParams params;

    /**
     * Received level per cell in dB, the data a radar looks at before it
     * decides what is a detection. Same scale as the point intensity, cells
     * without a reflection show the noise floor.
     * data[row * cols + col], row = range bin (row 0 is range 0).
     */
    struct Heatmap
    {
        int rows = 0;
        int cols = 0;
        std::vector<float> data;
    };

    // set these to have process() fill the maps below
    bool want_range_doppler = false;
    bool want_range_azimuth = false;

    // columns: Doppler bins, centre column = 0 m/s, left = approaching
    Heatmap range_doppler;
    // columns: azimuth from fov_azimuth_min to fov_azimuth_max
    Heatmap range_azimuth;

    RadarDetector() : m_gen(std::random_device{}()) {}

    std::vector<RadarPoint> process(
        const std::vector<RadarReturn>& returns,
        const RadarSensorPose& pose)
    {
        const RadarDetectorParams& p = params;
        std::vector<RadarPoint> out;

        const double res = std::max(p.range_resolution, 1e-3);
        const int n_r = static_cast<int>(std::ceil(p.range_max / res)) + 2;
        const int n_d = std::max(1, static_cast<int>(p.doppler_bins));
        const int d_half = n_d / 2;
        const double dv = std::max(p.doppler_resolution, 1e-4);
        const double deg = M_PI / 180.0;

        m_power.assign(static_cast<size_t>(n_r) * n_d, 0.0);
        m_members.clear();

        const int n_az = std::max(1, static_cast<int>(p.heatmap_azimuth_bins));
        const double az_lo = p.fov_azimuth_min * deg;
        const double az_span = std::max((p.fov_azimuth_max - p.fov_azimuth_min) * deg, 1e-6);
        if(want_range_azimuth)
        {
            m_range_azimuth.assign(static_cast<size_t>(n_r) * n_az, 0.0);
        }
        m_members.reserve(returns.size());

        // 1. rays -> range/Doppler cells
        for(size_t i = 0; i < returns.size(); i++)
        {
            const RadarReturn& r = returns[i];
            if(!(r.power > 0.0f) || !(r.range > 1e-3f))
            {
                continue;
            }

            const double az = std::atan2(r.dy, r.dx);
            const double el = std::asin(std::max(-1.0f, std::min(1.0f, r.dz)));
            if(az < p.fov_azimuth_min * deg || az > p.fov_azimuth_max * deg
                || el < p.fov_elevation_min * deg || el > p.fov_elevation_max * deg)
            {
                continue;
            }

            const double r_meas = r.range + p.range_bias;
            const int rb = static_cast<int>(std::lround(r_meas / res));
            if(rb < 0 || rb >= n_r || rb * res < p.range_min - 1e-6
                || rb * res > p.range_max + 1e-6)
            {
                continue;
            }

            int db = static_cast<int>(std::lround(r.radial_velocity / dv));
            db = ((db + d_half) % n_d + n_d) % n_d; // velocity ambiguity

            double gain = std::pow(std::max(std::cos(az), 0.0), 2.0 * p.antenna_azimuth_exponent)
                        * std::pow(std::max(std::cos(el), 0.0), 2.0 * p.antenna_elevation_exponent);
            gain *= std::pow(static_cast<double>(r.range), -p.range_loss_exponent);
            if(r.range < p.near_range_rolloff && p.near_range_rolloff > 0.0)
            {
                const double att_db = p.near_range_attenuation_db
                    * (p.near_range_rolloff - r.range) / p.near_range_rolloff;
                gain *= std::pow(10.0, -att_db / 10.0);
            }

            const double pw = r.power * r.solid_angle * gain;
            if(!(pw > 0.0))
            {
                continue;
            }

            const uint32_t cell = static_cast<uint32_t>(db) * n_r + rb;
            m_power[cell] += pw;
            m_members.push_back({cell, static_cast<uint32_t>(i), pw, pw});
        }

        // 2. fading + receiver noise, convert to the linear intensity scale
        const double gain_lin = std::pow(10.0, p.gain_db / 10.0);
        const double noise_lin = std::pow(10.0, p.noise_floor_db / 10.0);
        std::normal_distribution<double> gauss(0.0, 1.0);
        std::uniform_real_distribution<double> uni(0.0, 1.0);

        const uint64_t pose_key = poseKey(pose, p);
        if(p.glint > 0.0)
        {
            for(Member& mem : m_members)
            {
                const double u = hashUniform(pose_key ^ (0xC2B2AE3D27D4EB4Full * (mem.ray + 1)));
                mem.weight = mem.power * std::pow(-std::log(u), p.glint);
            }
        }
        for(size_t c = 0; c < m_power.size(); c++)
        {
            if(m_power[c] <= 0.0)
            {
                continue;
            }
            double v = m_power[c] * gain_lin;
            if(p.speckle > 0.0)
            {
                // exponentially distributed power, fixed while the sensor is still
                const double u = hashUniform(pose_key ^ (0x9E3779B97F4A7C15ull * (c + 1)));
                v *= std::pow(-std::log(u), p.speckle);
            }
            if(p.intensity_noise_db > 0.0)
            {
                v *= std::pow(10.0, p.intensity_noise_db * gauss(m_gen) / 10.0);
            }
            m_power[c] = v;
        }

        // heat maps: the cells as the radar sees them, before detection
        if(want_range_doppler)
        {
            range_doppler.rows = n_r;
            range_doppler.cols = n_d;
            range_doppler.data.resize(static_cast<size_t>(n_r) * n_d);
            for(int rb = 0; rb < n_r; rb++)
            {
                for(int d = 0; d < n_d; d++)
                {
                    // receiver noise fluctuates from frame to frame
                    const double noise = noise_lin * (0.75 + 0.5 * uni(m_gen));
                    range_doppler.data[static_cast<size_t>(rb) * n_d + d] = static_cast<float>(
                        10.0 * std::log10(m_power[static_cast<size_t>(d) * n_r + rb] + noise));
                }
            }
        }
        if(want_range_azimuth)
        {
            // Built from the very values the angle estimate uses (cell level
            // after fading, ray weights after glint), so a detected point lies
            // on the ridge of this map and not next to it.
            m_cell_weight.assign(m_power.size(), 0.0);
            for(const Member& mem : m_members)
            {
                m_cell_weight[mem.cell] += mem.weight;
            }
            for(const Member& mem : m_members)
            {
                if(!(m_cell_weight[mem.cell] > 0.0))
                {
                    continue;
                }
                const RadarReturn& ray = returns[mem.ray];
                const double ray_az = std::atan2(ray.dy, ray.dx);
                const int ab = std::min(n_az - 1, std::max(0,
                    static_cast<int>((ray_az - az_lo) / az_span * n_az)));
                const int rb = mem.cell % n_r;
                m_range_azimuth[static_cast<size_t>(rb) * n_az + ab]
                    += m_power[mem.cell] * mem.weight / m_cell_weight[mem.cell];
            }

            // the array cannot resolve finer than its angular resolution:
            // every reflector is spread over the neighbouring azimuth bins
            const double bin_deg = az_span / deg / n_az;
            const double sigma = std::max(p.heatmap_azimuth_resolution_deg / 2.355 / bin_deg, 1e-3);
            const int half = std::max(0, std::min(n_az, static_cast<int>(std::ceil(3.0 * sigma))));
            std::vector<double> kernel(2 * half + 1);
            // peak of the kernel is 1: a single reflector shows its full level
            // at its own azimuth, like the output of a beamformer
            for(int k = -half; k <= half; k++)
            {
                kernel[k + half] = std::exp(-0.5 * k * k / (sigma * sigma));
            }

            range_azimuth.rows = n_r;
            range_azimuth.cols = n_az;
            range_azimuth.data.resize(static_cast<size_t>(n_r) * n_az);
            for(int rb = 0; rb < n_r; rb++)
            {
                const double* row = &m_range_azimuth[static_cast<size_t>(rb) * n_az];
                for(int a = 0; a < n_az; a++)
                {
                    double v = 0.0;
                    for(int k = -half; k <= half; k++)
                    {
                        const int src = a + k;
                        if(src >= 0 && src < n_az)
                        {
                            v += row[src] * kernel[k + half];
                        }
                    }
                    const double noise = noise_lin * (0.75 + 0.5 * uni(m_gen));
                    range_azimuth.data[static_cast<size_t>(rb) * n_az + a] = static_cast<float>(
                        10.0 * std::log10(v + noise));
                }
            }
        }

        // 3. CA-CFAR along range + peak grouping
        const int guard = std::max(0, static_cast<int>(p.cfar_guard_cells));
        const int window = std::max(1, static_cast<int>(p.cfar_window_cells));
        const double thresh = std::pow(10.0, p.cfar_threshold_db / 10.0);
        const bool grouping = p.peak_grouping > 0.5;
        const int cfar_mode = static_cast<int>(std::lround(p.cfar_mode));

        std::sort(m_members.begin(), m_members.end(),
            [](const Member& a, const Member& b){ return a.cell < b.cell; });

        std::vector<RadarPoint> cand;

        size_t m = 0;
        while(m < m_members.size())
        {
            const uint32_t cell = m_members[m].cell;
            size_t m_end = m;
            while(m_end < m_members.size() && m_members[m_end].cell == cell)
            {
                m_end++;
            }

            const int d = cell / n_r;
            const int rb = cell % n_r;
            const double v = m_power[cell];

            // local noise estimate from the training cells
            double side_sum[2] = {0.0, 0.0};
            int side_n[2] = {0, 0};
            for(int k = guard + 1; k <= guard + window; k++)
            {
                if(rb - k >= 0)  { side_sum[0] += m_power[cell - k] + noise_lin; side_n[0]++; }
                if(rb + k < n_r) { side_sum[1] += m_power[cell + k] + noise_lin; side_n[1]++; }
            }
            double noise_est = noise_lin;
            if(cfar_mode != 0 && side_n[0] > 0 && side_n[1] > 0)
            {
                const double lo = side_sum[0] / side_n[0];
                const double hi = side_sum[1] / side_n[1];
                noise_est = (cfar_mode == 1) ? std::min(lo, hi) : std::max(lo, hi);
            }
            else if(side_n[0] + side_n[1] > 0)
            {
                noise_est = (side_sum[0] + side_sum[1]) / (side_n[0] + side_n[1]);
            }

            bool detected = (v + noise_lin) > thresh * noise_est;

            if(detected && grouping)
            {
                if(rb > 0 && m_power[cell - 1] > v) detected = false;
                if(rb + 1 < n_r && m_power[cell + 1] >= v) detected = false;
                const int d_lo = (d + n_d - 1) % n_d;
                const int d_hi = (d + 1) % n_d;
                if(n_d > 2)
                {
                    if(m_power[static_cast<size_t>(d_lo) * n_r + rb] > v) detected = false;
                    if(m_power[static_cast<size_t>(d_hi) * n_r + rb] >= v) detected = false;
                }
            }

            if(detected && p.detection_probability < 1.0 && uni(m_gen) > p.detection_probability)
            {
                detected = false;
            }

            if(detected)
            {
                const double velocity = (d - d_half) * dv;
                estimateAngles(returns, m, m_end, rb * res, velocity, v, cand);
            }

            m = m_end;
        }

        // 4. measurement noise on the angle estimate
        for(RadarPoint& pt : cand)
        {
            const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if(r < 1e-6) continue;
            double az = std::atan2(pt.y, pt.x) + p.azimuth_noise_deg * deg * gauss(m_gen);
            double el = std::asin(pt.z / r) + (p.elevation_bias_deg + p.elevation_noise_deg * gauss(m_gen)) * deg;
            el = std::max(-0.5 * M_PI, std::min(0.5 * M_PI, el));
            pt.x = r * std::cos(el) * std::cos(az);
            pt.y = r * std::cos(el) * std::sin(az);
            pt.z = r * std::sin(el);
        }

        // 5. false alarms
        if(p.false_alarm_rate > 0.0)
        {
            std::poisson_distribution<int> n_fa(p.false_alarm_rate);
            const int n = n_fa(m_gen);
            for(int i = 0; i < n; i++)
            {
                const int rb = static_cast<int>(std::lround(
                    (p.range_min + uni(m_gen) * (p.range_max - p.range_min)) / res));
                const double az = (p.fov_azimuth_min + uni(m_gen) * (p.fov_azimuth_max - p.fov_azimuth_min)) * deg;
                const double el = (p.fov_elevation_min + uni(m_gen) * (p.fov_elevation_max - p.fov_elevation_min)) * deg;
                const int db = static_cast<int>(uni(m_gen) * n_d) - d_half;
                cand.push_back(makePoint(rb * res, az, el,
                    p.noise_floor_db + p.cfar_threshold_db - 3.0 * std::log10(uni(m_gen) + 1e-6),
                    db * dv));
            }
        }

        // vibration sidebands: a detection reappears far out in Doppler, barely above the threshold
        if(p.sideband_rate > 0.0 && !cand.empty())
        {
            std::poisson_distribution<int> n_sb(p.sideband_rate);
            const int n = n_sb(m_gen);
            const size_t n_src = cand.size();
            for(int i = 0; i < n; i++)
            {
                RadarPoint pt = cand[std::min(n_src - 1, static_cast<size_t>(uni(m_gen) * n_src))];
                const double sign = uni(m_gen) < 0.5 ? -1.0 : 1.0;
                const double v = pt.velocity + sign * (p.sideband_velocity + p.sideband_velocity_spread * gauss(m_gen));
                int db = static_cast<int>(std::lround(v / dv));
                db = ((db + d_half) % n_d + n_d) % n_d - d_half; // velocity ambiguity
                pt.velocity = db * dv;
                pt.intensity = p.noise_floor_db + p.cfar_threshold_db
                    - p.sideband_margin_db * std::log(uni(m_gen) + 1e-6);
                cand.push_back(pt);
            }
        }

        if(p.leakage_enabled > 0.5)
        {
            const double lobe_az[2] = {p.leakage_azimuth_deg, p.leakage2_azimuth_deg};
            const double lobe_presence[2] = {p.leakage_presence, p.leakage2_presence};
            for(int lobe = 0; lobe < 2; lobe++)
            {
                if(!(uni(m_gen) < lobe_presence[lobe]))
                {
                    continue;
                }
                const double az = (lobe_az[lobe] + p.leakage_azimuth_jitter_deg * gauss(m_gen)) * deg;
                double el = (p.leakage_elevation_deg + p.leakage_elevation_jitter_deg * gauss(m_gen)) * deg;
                el = std::max(p.fov_elevation_min * deg, std::min(p.fov_elevation_max * deg, el));
                const double intensity = p.leakage_intensity_db + p.intensity_noise_db * gauss(m_gen)
                    + p.leakage_intensity_jitter_db * gauss(m_gen);
                cand.push_back(makePoint(p.leakage_range, az, el, intensity, 0.0));
            }
        }

        // the azimuth comes from the peak bin of an FFT over the antenna row: the direction
        // cosine y/r only takes multiples of 2/N. Range and elevation (z) are kept.
        if(p.azimuth_fft_size >= 2.0)
        {
            const double half = 0.5 * std::floor(p.azimuth_fft_size);
            for(RadarPoint& pt : cand)
            {
                const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                if(r < 1e-6) continue;
                const double wz = pt.z / r;
                const double wy_max = std::sqrt(std::max(0.0, 1.0 - wz * wz));
                double k = std::round(pt.y / r * half);
                while(std::fabs(k) / half > wy_max && k != 0.0)
                {
                    k -= (k > 0.0) ? 1.0 : -1.0; // keep the point in front of the sensor
                }
                const double wy = k / half;
                pt.y = r * wy;
                pt.x = r * std::sqrt(std::max(0.0, 1.0 - wy * wy - wz * wz));
            }
        }

        // 6. the device reports a limited number of points, strongest first
        const size_t max_points = static_cast<size_t>(std::max(0.0, p.max_points));
        if(cand.size() > max_points)
        {
            std::partial_sort(cand.begin(), cand.begin() + max_points, cand.end(),
                [](const RadarPoint& a, const RadarPoint& b){ return a.intensity > b.intensity; });
            cand.resize(max_points);
        }

        return cand;
    }

private:
    struct Member
    {
        uint32_t cell;
        uint32_t ray;
        double power;   // contribution to the cell
        double weight;  // contribution to the angle estimate (power x individual fading)
    };

    static RadarPoint makePoint(double range, double az, double el, double intensity_db, double velocity)
    {
        RadarPoint pt;
        pt.x = range * std::cos(el) * std::cos(az);
        pt.y = range * std::cos(el) * std::sin(az);
        pt.z = range * std::sin(el);
        pt.intensity = intensity_db;
        pt.velocity = velocity;
        return pt;
    }

    /// power-weighted direction of the rays in [begin, end) flagged with `group`
    void centroid(
        const std::vector<RadarReturn>& returns,
        size_t begin, size_t end, int group,
        double& cx, double& cy, double& cz, double& power) const
    {
        cx = cy = cz = power = 0.0;
        for(size_t i = begin; i < end; i++)
        {
            if(m_group[i - begin] != group) continue;
            const RadarReturn& r = returns[m_members[i].ray];
            const double w = m_members[i].weight;
            cx += w * r.dx; cy += w * r.dy; cz += w * r.dz;
            power += w;
        }
        const double n = std::sqrt(cx * cx + cy * cy + cz * cz);
        if(n > 1e-12) { cx /= n; cy /= n; cz /= n; }
    }

    /**
     * The array cannot separate reflectors closer than its angular resolution:
     * everything in one range/Doppler cell collapses to a single direction.
     * A second point appears only for a well separated, comparably strong group.
     */
    void estimateAngles(
        const std::vector<RadarReturn>& returns,
        size_t begin, size_t end,
        double range, double velocity, double cell_value,
        std::vector<RadarPoint>& out)
    {
        const RadarDetectorParams& p = params;
        m_group.assign(end - begin, 0);

        double cx, cy, cz, pw_all;
        centroid(returns, begin, end, 0, cx, cy, cz, pw_all);
        if(!(pw_all > 0.0)) return;

        double pw0 = pw_all, pw1 = 0.0;
        double c1x = 0, c1y = 0, c1z = 0;

        if(p.max_points_per_cell >= 2.0)
        {
            // seed the main target with the strongest ray, split off what is far from it
            size_t best = begin;
            for(size_t i = begin; i < end; i++)
            {
                if(m_members[i].weight > m_members[best].weight) best = i;
            }
            const RadarReturn& rb = returns[m_members[best].ray];
            const double cos_sep = std::cos(p.angular_separation_deg * M_PI / 180.0);
            for(size_t i = begin; i < end; i++)
            {
                const RadarReturn& r = returns[m_members[i].ray];
                if(r.dx * rb.dx + r.dy * rb.dy + r.dz * rb.dz < cos_sep)
                {
                    m_group[i - begin] = 1;
                }
            }
            double t0x, t0y, t0z, t0p;
            centroid(returns, begin, end, 0, t0x, t0y, t0z, t0p);
            centroid(returns, begin, end, 1, c1x, c1y, c1z, pw1);
            if(pw1 >= p.multi_object_ratio * t0p && t0p > 0.0)
            {
                cx = t0x; cy = t0y; cz = t0z; pw0 = t0p;
            } else {
                pw1 = 0.0; // too weak: stays merged into the main target
            }
        }

        const double base_db = 10.0 * std::log10(cell_value);
        auto add_point = [&](double x, double y, double z, double frac)
        {
            RadarPoint pt;
            pt.x = range * x; pt.y = range * y; pt.z = range * z;
            pt.intensity = base_db + 10.0 * std::log10(std::max(frac, 1e-6));
            pt.velocity = velocity;
            out.push_back(pt);
        };

        add_point(cx, cy, cz, pw0 / pw_all);
        if(pw1 > 0.0)
        {
            add_point(c1x, c1y, c1z, pw1 / pw_all);
        }
    }

    static uint64_t mix(uint64_t x)
    {
        x += 0x9E3779B97F4A7C15ull;
        x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
        x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
        return x ^ (x >> 31);
    }

    static double hashUniform(uint64_t key)
    {
        // (0, 1]
        return (static_cast<double>(mix(key) >> 11) + 1.0) / 9007199254740992.0;
    }

    static uint64_t poseKey(const RadarSensorPose& pose, const RadarDetectorParams& p)
    {
        const double pc = std::max(p.speckle_position_cell, 1e-4);
        const double rc = std::max(p.speckle_rotation_cell, 1e-4);
        const double vals[7] = {
            pose.x / pc, pose.y / pc, pose.z / pc,
            pose.qx / rc, pose.qy / rc, pose.qz / rc, pose.qw / rc};
        uint64_t key = 0x1234ABCDull;
        for(double v : vals)
        {
            key = mix(key ^ static_cast<uint64_t>(static_cast<int64_t>(std::floor(v))));
        }
        return key;
    }

    std::mt19937 m_gen;
    std::vector<double> m_power;
    std::vector<double> m_range_azimuth;
    std::vector<double> m_cell_weight;
    std::vector<Member> m_members;
    std::vector<int> m_group;
};

/**
 * Checks one detector parameter before it is applied.
 * Returns an empty string if the value is usable, otherwise the reason.
 */
inline std::string checkDetectorParam(const std::string& field, double value)
{
    if(!std::isfinite(value))
    {
        return field + " must be finite";
    }
    if(field == "range_resolution" && (value < 0.005 || value > 10.0))
    {
        return "range_resolution must be within 0.005 .. 10 m";
    }
    if(field == "doppler_resolution" && value < 0.001)
    {
        return "doppler_resolution must be >= 0.001 m/s";
    }
    if(field == "doppler_bins" && (value < 1.0 || value > 1024.0))
    {
        return "doppler_bins must be within 1 .. 1024";
    }
    if((field == "range_max" || field == "range_min") && (value < 0.0 || value > 1000.0))
    {
        return field + " must be within 0 .. 1000 m";
    }
    if((field == "detection_probability" || field == "leakage_presence"
        || field == "leakage2_presence") && (value < 0.0 || value > 1.0))
    {
        return field + " is a probability, must be within 0 .. 1";
    }
    if((field == "max_points" || field == "false_alarm_rate" || field == "sideband_rate"
        || field == "cfar_guard_cells" || field == "speckle" || field == "glint") && value < 0.0)
    {
        return field + " must be >= 0";
    }
    if(field == "cfar_window_cells" && value < 1.0)
    {
        return "cfar_window_cells must be >= 1";
    }
    return "";
}

} // namespace radarays_gazebo_plugins

#endif // RADARAYS_RADAR_DETECTOR_H
