// Tests of the FMCW detection stage. No Gazebo, no GPU: the detector is header only.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <radarays_gazebo_plugins/radar_detector.h>

using namespace radarays_gazebo_plugins;

namespace
{

const double DEG = M_PI / 180.0;

/// detector without any random component, so results are exact
RadarDetector makeDetector()
{
    RadarDetector det;
    RadarDetectorParams& p = det.params;
    p.range_bias = 0.0;
    p.speckle = 0.0;
    p.glint = 0.0;
    p.intensity_noise_db = 0.0;
    p.azimuth_noise_deg = 0.0;
    p.elevation_noise_deg = 0.0;
    p.elevation_bias_deg = 0.0;
    p.azimuth_fft_size = 0.0;
    p.false_alarm_rate = 0.0;
    p.sideband_rate = 0.0;
    p.leakage_enabled = 0.0;
    p.near_range_rolloff = 0.0;
    return det;
}

/// a small patch of rays around (az, el) at one range
void addTarget(std::vector<RadarReturn>& rays, double range, double az_deg, double el_deg,
    double velocity = 0.0, double power = 1.0)
{
    for(int i = -2; i <= 2; i++)
    {
        for(int j = -1; j <= 1; j++)
        {
            const double az = (az_deg + i) * DEG;
            const double el = (el_deg + 2 * j) * DEG;
            RadarReturn r;
            r.range = range;
            r.dx = std::cos(el) * std::cos(az);
            r.dy = std::cos(el) * std::sin(az);
            r.dz = std::sin(el);
            r.power = power;
            r.radial_velocity = velocity;
            r.solid_angle = 6e-4;
            rays.push_back(r);
        }
    }
}

double rangeOf(const RadarPoint& p) { return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }
double azimuthOf(const RadarPoint& p) { return std::atan2(p.y, p.x) / DEG; }

} // namespace

TEST(RadarDetector, NoReturnsNoPoints)
{
    RadarDetector det = makeDetector();
    EXPECT_TRUE(det.process({}, RadarSensorPose()).empty());
}

TEST(RadarDetector, SingleTargetGivesOnePointOnTheRangeGrid)
{
    RadarDetector det = makeDetector();
    std::vector<RadarReturn> rays;
    addTarget(rays, 5.03, 10.0, 0.0);

    const auto pts = det.process(rays, RadarSensorPose());
    ASSERT_EQ(pts.size(), 1u);

    const double res = det.params.range_resolution;
    EXPECT_NEAR(rangeOf(pts[0]), std::round(5.03 / res) * res, 1e-3);
    EXPECT_NEAR(azimuthOf(pts[0]), 10.0, 0.5);
    EXPECT_NEAR(pts[0].z, 0.0, 0.05);
    EXPECT_FLOAT_EQ(pts[0].velocity, 0.0f);
    // intensity is reported in dB, far above the linear 0..1 ray energy
    EXPECT_GT(pts[0].intensity, 30.0f);
    EXPECT_LT(pts[0].intensity, 120.0f);
}

TEST(RadarDetector, RangeBiasShiftsTheMeasuredRange)
{
    RadarDetector det = makeDetector();
    det.params.range_bias = 0.5;
    std::vector<RadarReturn> rays;
    addTarget(rays, 2.7, 0.0, 0.0);

    const auto pts = det.process(rays, RadarSensorPose());
    ASSERT_EQ(pts.size(), 1u);
    EXPECT_NEAR(rangeOf(pts[0]), 3.2, det.params.range_resolution);
}

TEST(RadarDetector, DopplerIsQuantizedToBins)
{
    RadarDetector det = makeDetector();
    std::vector<RadarReturn> rays;
    addTarget(rays, 6.0, 0.0, 0.0, -0.5);

    const auto pts = det.process(rays, RadarSensorPose());
    ASSERT_EQ(pts.size(), 1u);
    const double dv = det.params.doppler_resolution;
    EXPECT_NEAR(pts[0].velocity, std::round(-0.5 / dv) * dv, 1e-4);
}

TEST(RadarDetector, TargetsCloserThanTheAngularResolutionMerge)
{
    RadarDetector det = makeDetector();
    std::vector<RadarReturn> rays;
    // like the two boxes with a 51 cm gap seen from 2.7 m: about +-10 deg
    addTarget(rays, 4.0, -10.0, 0.0);
    addTarget(rays, 4.0, 10.0, 0.0);

    const auto pts = det.process(rays, RadarSensorPose());
    ASSERT_EQ(pts.size(), 1u);
    EXPECT_NEAR(azimuthOf(pts[0]), 0.0, 1.0);
}

TEST(RadarDetector, WellSeparatedTargetsInOneCellGiveTwoPoints)
{
    RadarDetector det = makeDetector();
    std::vector<RadarReturn> rays;
    addTarget(rays, 4.0, -30.0, 0.0);
    addTarget(rays, 4.0, 30.0, 0.0);

    const auto pts = det.process(rays, RadarSensorPose());
    ASSERT_EQ(pts.size(), 2u);
    EXPECT_NEAR(std::abs(azimuthOf(pts[0])), 30.0, 1.5);
    EXPECT_NEAR(std::abs(azimuthOf(pts[1])), 30.0, 1.5);
    EXPECT_LT(azimuthOf(pts[0]) * azimuthOf(pts[1]), 0.0);
}

TEST(RadarDetector, PointCountIsLimited)
{
    RadarDetector det = makeDetector();
    det.params.max_points = 10;
    std::vector<RadarReturn> rays;
    for(int i = 0; i < 60; i++)
    {
        // every 4th range bin, alternating Doppler: 60 separate peaks
        addTarget(rays, 2.0 + 0.4 * i, (i % 9 - 4) * 8.0, 0.0, (i % 2) ? 0.5 : -0.5);
    }

    det.params.max_points = 1000;
    EXPECT_GT(det.process(rays, RadarSensorPose()).size(), 10u);
    det.params.max_points = 10;
    EXPECT_EQ(det.process(rays, RadarSensorPose()).size(), 10u);
}

TEST(RadarDetector, OutsideFieldOfViewOrRangeIsNotReported)
{
    RadarDetector det = makeDetector();
    std::vector<RadarReturn> rays;
    addTarget(rays, 5.0, 80.0, 0.0);   // azimuth FoV is +-60 deg
    addTarget(rays, 40.0, 0.0, 0.0);   // beyond range_max
    addTarget(rays, 0.2, 0.0, 0.0);    // closer than range_min
    EXPECT_TRUE(det.process(rays, RadarSensorPose()).empty());
}

TEST(RadarDetector, WeakTargetStaysBelowTheThreshold)
{
    RadarDetector det = makeDetector();
    std::vector<RadarReturn> rays;
    addTarget(rays, 5.0, 0.0, 0.0, 0.0, 1e-9);
    EXPECT_TRUE(det.process(rays, RadarSensorPose()).empty());
}

TEST(RadarDetector, FadingIsFrozenWhileTheSensorIsStill)
{
    RadarDetector det = makeDetector();
    det.params.speckle = 1.0;
    det.params.glint = 1.0;
    std::vector<RadarReturn> rays;
    for(int i = 0; i < 12; i++)
    {
        addTarget(rays, 2.0 + 1.3 * i, (i % 5 - 2) * 12.0, 0.0);
    }

    RadarSensorPose pose;
    pose.x = 1.0; pose.y = 2.0; pose.z = 1.2;

    const auto a = det.process(rays, pose);
    const auto b = det.process(rays, pose);
    ASSERT_EQ(a.size(), b.size());
    for(size_t i = 0; i < a.size(); i++)
    {
        EXPECT_FLOAT_EQ(a[i].x, b[i].x);
        EXPECT_FLOAT_EQ(a[i].y, b[i].y);
        EXPECT_FLOAT_EQ(a[i].intensity, b[i].intensity);
    }

    // after moving, the fading pattern changes: over many poses the
    // intensities must not all stay identical
    bool changed = false;
    for(int k = 1; k <= 20 && !changed; k++)
    {
        RadarSensorPose moved = pose;
        moved.x += 0.05 * k;
        const auto c = det.process(rays, moved);
        changed = c.size() != a.size();
        for(size_t i = 0; i < c.size() && !changed; i++)
        {
            changed = std::abs(c[i].intensity - a[i].intensity) > 1e-3;
        }
    }
    EXPECT_TRUE(changed);
}

TEST(RadarDetector, HeatmapsShowTheTargetAboveTheNoiseFloor)
{
    RadarDetector det = makeDetector();
    det.want_range_doppler = true;
    det.want_range_azimuth = true;
    std::vector<RadarReturn> rays;
    addTarget(rays, 6.0, 20.0, 0.0);

    const auto pts = det.process(rays, RadarSensorPose());
    ASSERT_EQ(pts.size(), 1u);

    const double res = det.params.range_resolution;
    const int rb = static_cast<int>(std::lround(6.0 / res));

    // range-Doppler: peak in the zero Doppler column of the target's range bin
    const auto& rd = det.range_doppler;
    ASSERT_EQ(rd.cols, static_cast<int>(det.params.doppler_bins));
    ASSERT_GT(rd.rows, rb);
    ASSERT_EQ(rd.data.size(), static_cast<size_t>(rd.rows) * rd.cols);
    const float peak = rd.data[rb * rd.cols + rd.cols / 2];
    EXPECT_NEAR(peak, pts[0].intensity, 0.5f);
    const float empty = rd.data[(rb + 40) * rd.cols + 3];
    EXPECT_NEAR(empty, det.params.noise_floor_db, 2.0f);
    EXPECT_GT(peak, empty + 8.0f);

    // range-azimuth: strongest column of that range bin lies at +20 deg
    const auto& ra = det.range_azimuth;
    ASSERT_EQ(ra.cols, static_cast<int>(det.params.heatmap_azimuth_bins));
    int best = 0;
    for(int a = 1; a < ra.cols; a++)
    {
        if(ra.data[rb * ra.cols + a] > ra.data[rb * ra.cols + best]) best = a;
    }
    const double az = det.params.fov_azimuth_min + (best + 0.5)
        * (det.params.fov_azimuth_max - det.params.fov_azimuth_min) / ra.cols;
    EXPECT_NEAR(az, 20.0, 3.0);
    // a single reflector shows the same level in both maps and in the point
    EXPECT_NEAR(ra.data[rb * ra.cols + best], pts[0].intensity, 1.0f);
    // blurred by the angular resolution: the neighbouring columns are raised too
    EXPECT_GT(ra.data[rb * ra.cols + best + 2], empty + 3.0f);
}

TEST(RadarDetector, HeatmapsAreNotComputedUnlessRequested)
{
    RadarDetector det = makeDetector();
    std::vector<RadarReturn> rays;
    addTarget(rays, 6.0, 0.0, 0.0);
    det.process(rays, RadarSensorPose());
    EXPECT_TRUE(det.range_doppler.data.empty());
    EXPECT_TRUE(det.range_azimuth.data.empty());
}

TEST(RadarDetector, ParameterValidation)
{
    EXPECT_TRUE(checkDetectorParam("range_resolution", 0.0996).empty());
    EXPECT_FALSE(checkDetectorParam("range_resolution", 0.0).empty());
    EXPECT_FALSE(checkDetectorParam("doppler_bins", 0.0).empty());
    EXPECT_FALSE(checkDetectorParam("detection_probability", 1.5).empty());
    EXPECT_FALSE(checkDetectorParam("gain_db", std::nan("")).empty());
    EXPECT_TRUE(checkDetectorParam("gain_db", 90.0).empty());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
