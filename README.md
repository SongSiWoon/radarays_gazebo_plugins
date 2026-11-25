# radarays_gazebo_plugins

Gazebo Classic (ROS 2 Humble) sensor plugins for [RadaRays](https://github.com/uos/radarays_ros),
extended to imitate the point cloud of an FMCW mmWave radar such as the TI IWR6843.

This is a fork of [uos/radarays_gazebo_plugins](https://github.com/uos/radarays_gazebo_plugins).
Compared to upstream, this version contains:

- the port from ROS 1 to ROS 2 Humble
- a 3D scan (azimuth and elevation) instead of the original 2D spinning radar
- a **detection stage** (`include/radarays_gazebo_plugins/radar_detector.h`) that turns the
  ray traced returns into the sparse detection list a real radar reports
- range-Doppler and range-azimuth **heat maps**
- fixes for crashes and wrong values of the original plugins (see [Changes](#changes))

## Dependencies

| Package | Repository |
|---|---|
| `rmagine` (Embree and/or OptiX backend) | [SongSiWoon/rmagine](https://github.com/SongSiWoon/rmagine) (uos/rmagine 2.4.1 + fix for large meshes in OptiX) |
| `rmagine_gazebo_plugins` | [SongSiWoon/rmagine_gazebo_plugins](https://github.com/SongSiWoon/rmagine_gazebo_plugins) (map update and mesh import fixes) |
| `radarays_ros` | [SongSiWoon/radarays_ros](https://github.com/SongSiWoon/radarays_ros) (ROS 2 port) |

A `.repos` file for [vcstool](https://github.com/dirk-thomas/vcstool) with pinned commits is the
easiest way to get all four.

The plugins run on the CPU (Embree). The GPU backend (OptiX) is optional: it needs CUDA and
the OptiX 7.3 SDK; without them only the CPU plugins are built.

## How it works

1. **Ray tracing** (RadaRays): rays are cast over the scan pattern given in `<ray>`. Every hit is
   shaded with the material of the surface and, with `n_reflections > 1`, reflected and
   refracted (multipath ghosts).
2. **Detection** (`RadarDetector`):
   - returns are accumulated in range bins x Doppler bins (range bias, antenna pattern, range loss)
   - fading tied to the sensor pose: a still sensor reports the same points every frame, a moving
     one does not
   - CFAR along range (cell averaging, smallest-of or greatest-of) plus peak grouping
   - one or two angle estimates per cell: reflectors closer than the angular resolution merge
   - the strongest `max_points` are reported, intensity in dB, velocity quantized to Doppler bins
3. **Output**: `PointCloud2` with fields `x, y, z, intensity, v_doppler` (same layout as
   `ti_mmwave_ros2_pkg`), optional heat maps and ground truth.

Measured against a TI IWR6843 flight recording (trajectory replayed in Gazebo), the default
detector settings give about the same number of points per frame (median 23 vs. 25), the same
range, azimuth, elevation and intensity distributions, and the same frame-to-frame persistence.

## Sensor SDF

```xml
<sensor type="radarays_optix" name="radar">   <!-- or radarays_embree -->
  <update_rate>20</update_rate>
  <ray>
    <scan>
      <!-- ray sampling density, not the resolution of the device -->
      <horizontal><min_angle>-1.047</min_angle><increment>0.01745</increment><samples>121</samples></horizontal>
      <vertical><min_angle>-0.5236</min_angle><increment>0.0349</increment><samples>31</samples></vertical>
    </scan>
    <range><min>0.2</min><max>25.0</max></range>
  </ray>

  <ray_tracing>
    <n_samples>2</n_samples>            <!-- rays per beam -->
    <n_reflections>2</n_reflections>    <!-- 2 = first order multipath -->
    <beam_width>1.5</beam_width>        <!-- [deg] -->
    <self_clearance>0.45</self_clearance> <!-- [m] rays start outside of the own vehicle -->
  </ray_tracing>

  <!-- surfaces without <radarays_material> -->
  <default_material>
    <velocity>0.0</velocity><ambient>0.1</ambient><diffuse>1.0</diffuse><specular>20.0</specular>
  </default_material>

  <!-- any field of RadarDetectorParams, see radar_detector.h -->
  <radar_detector>
    <range_resolution>0.0996</range_resolution>
    <doppler_resolution>0.218</doppler_resolution>
    <cfar_threshold_db>9.0</cfar_threshold_db>
    <max_points>36</max_points>
  </radar_detector>

  <plugin name="radarays_ros" filename="libradarays_optix_ros_gzplugin.so">
    <ros><namespace>drone1</namespace></ros>
    <frame>radar_link</frame>
    <topic>radar/image</topic>
    <pointcloud_topic>sensor/radar/points</pointcloud_topic>
    <doppler_field>v_doppler</doppler_field>
  </plugin>
</sensor>
```

The sensor needs a map plugin in the world (`librmagine_optix_map_gzplugin.so` or
`librmagine_embree_map_gzplugin.so`), see [rmagine_gazebo_plugins](https://github.com/uos/rmagine_gazebo_plugins).
Register the sensor type with `gzserver -s libradarays_optix_sensors_gzregister.so`
(or `libradarays_embree_sensors_gzregister.so`).

## Topics

Relative to the namespace of the ROS plugin.

| Topic | Type | Content |
|---|---|---|
| `<pointcloud_topic>` | `PointCloud2` | detections: `x, y, z, intensity [dB], v_doppler [m/s]` |
| `<pointcloud_topic>_gt` | `PointCloud2` | first hit of every beam, no detection model |
| `heatmap_range_doppler` | `Image` (32FC1, dB) | row = range bin (row 0 = 0 m), column = Doppler bin (centre = 0 m/s) |
| `heatmap_range_azimuth` | `Image` (32FC1, dB) | row = range bin, column = azimuth from `fov_azimuth_min` to `fov_azimuth_max` |
| `<topic>` | `Image` (mono8) | polar image of the original RadaRays |

The heat maps share the scale of the point intensity, empty cells show the noise floor. Heat
maps, ground truth and the polar image are only computed while somebody subscribes.

## Parameters

The ROS node is `<namespace>/radar`. Every detector field is a parameter `det.<field>` and can
be changed at runtime; invalid values are rejected.

```console
ros2 param set /drone1/radar det.cfar_threshold_db 10.0
ros2 param set /drone1/radar det.enabled 0.0     # raw ray returns instead of detections
ros2 param set /drone1/radar n_reflections 3
```

The ray tracing parameters of the original plugin (`n_samples`, `n_reflections`, `beam_width`,
`resolution`, ...) are parameters as well; their defaults come from the SDF.

## Materials

Visuals can carry a `radarays_material`; everything else gets `<default_material>`.

```xml
<visual name="wall">
  ...
  <radarays_material>
    <velocity>0.0</velocity>     <!-- 0 = opaque, > 0 = wave speed inside [m/ns] -->
    <ambient>0.1</ambient>       <!-- angle independent backscatter -->
    <diffuse>1.0</diffuse>       <!-- lobe around the surface normal -->
    <specular>20.0</specular>    <!-- sharpness of that lobe -->
  </radarays_material>
</visual>
```

Materials are looked up by `model::link::visual` and refreshed whenever geometry is added to or
removed from the map. `worlds/radar_demo.world` shows several materials.

## Tests

The detection stage is header only and tested without Gazebo or a GPU:

```console
colcon build --packages-select radarays_gazebo_plugins
./build/radarays_gazebo_plugins/test_radar_detector
```

## Changes

Compared to the upstream plugins:

- point cloud from the detection stage instead of one point per ray hit
- crash when `n_samples` or `n_reflections` change at runtime
- material table out of bounds for models spawned after start
- range cut at `n_cells * resolution` (10 m with the old defaults)
- SDF range limits overwritten by ROS parameter defaults; only some parameters applied at start
- Doppler computed with the world-frame velocity; moving objects ignored
- `record_multi_reflection` ignored by the OptiX plugin
- NaN in the polar image for azimuths without returns
- parameter node shared by all sensors (`/radar`), now in the robot namespace
- OptiX plugin used every CPU core (OpenMP around a few thousand points)
- Embree plugin traced every beam separately; all beams of a pass are now traced in one call
  (same returns, about a third of the CPU time)

## Known limitations

- Ray tracing only: no edge diffraction, so strong corner returns of real radars are missing.
- The carrier frequency is not part of the model; frequency dependent behaviour has to be put
  into the materials and the detector parameters.
- Doppler of moving objects only for the directly hit object and only with OptiX.
- The heat maps leave out the injected near-field (airframe) points and vibration sidebands.

## Credits

RadaRays by Alexander Mock et al., University of Osnabrück:
[paper](https://arxiv.org/abs/2310.03505), [radarays_ros](https://github.com/uos/radarays_ros).
