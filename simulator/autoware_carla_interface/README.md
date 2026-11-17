# autoware_carla_interface

## ROS 2 / Autoware Universe bridge for CARLA simulator

Thanks to <https://github.com/gezp> for ROS 2 Humble support for CARLA Communication.
This ros package enables communication between Autoware and CARLA for autonomous driving simulation.

## Supported Environment

| ubuntu |  ros   | carla  | autoware |
| :----: | :----: | :----: | :------: |
| 22.04  | humble | 0.9.15 |   Main   |

## Setup

### Install

#### Prerequisites

1. **Install CARLA 0.9.15**: Follow the [CARLA Installation Guide](https://carla.readthedocs.io/en/latest/start_quickstart/)

2. **Install CARLA Python Package**: Install [CARLA 0.9.15 ROS 2 Humble communication package](https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04)
   - Option A: Install the wheel using pip
   - Option B: Add the egg file to your `PYTHONPATH`

3. **Download CARLA Lanelet2 Maps**: Get the y-axis inverted maps from [CARLA Autoware Contents](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)

#### Map Setup

1. Download the maps (y-axis inverted version) to an arbitrary location
2. Create the map folder structure in `$HOME/autoware_data/maps`:
   - Rename `point_cloud/Town01.pcd` → `$HOME/autoware_data/maps/Town01/pointcloud_map.pcd`
   - Rename `vector_maps/lanelet2/Town01.osm` → `$HOME/autoware_data/maps/Town01/lanelet2_map.osm`
3. Create `$HOME/autoware_data/maps/Town01/map_projector_info.yaml` with:

   ```yaml
   projector_type: Local
   ```

### Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run

1. Run carla, change map, spawn object if you need
   <!--- cspell:ignore prefernvidia -->

   ```bash
   cd CARLA
   ./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
   ```

2. Run Autoware with CARLA

   ```bash
   ros2 launch autoware_launch e2e_simulator.launch.xml \
       map_path:=$HOME/autoware_data/maps/Town01 \
       vehicle_model:=sample_vehicle \
       sensor_model:=carla_sensor_kit \
       simulator_type:=carla
   ```

   For E2E planning with VAD:

   ```bash
   ros2 launch autoware_launch e2e_simulator.launch.xml \
       map_path:=$HOME/autoware_data/maps/Town01 \
       vehicle_model:=sample_vehicle \
       sensor_model:=carla_sensor_kit \
       simulator_type:=carla \
       use_e2e_planning:=true
   ```

3. Set initial pose (Init by GNSS)
4. Set goal position
5. Wait for planning
6. Engage

### Viewing Multi-Camera View in RViz

The `carla_sensor_kit` includes 6 cameras providing 360-degree coverage (Front, Front-Left, Front-Right, Back, Back-Left, Back-Right). A multi-camera combiner node automatically combines all camera feeds into a single 2x3 grid view.

To view the combined camera feed in RViz:

1. In the **Displays** panel (left side), click the **"Add"** button
2. Select the **"By topic"** tab
3. Navigate to `/sensing/camera/all_cameras/image_raw`
4. Select **"Image"** display type
5. Click **OK**

![Multi-Camera View in RViz](docs/images/rviz_multi_camera_view.png)

The combined view shows all 6 cameras with labels: FL (Front-Left), F (Front), FR (Front-Right), BL (Back-Left), B (Back), BR (Back-Right).

**Note:** If you don't need the multi-camera combiner (to save CPU resources), you can comment out the following line in `launch/autoware_carla_interface.launch.xml`:

```xml
<!-- Multi-camera combiner for RViz visualization -->
<!-- <node pkg="autoware_carla_interface" exec="multi_camera_combiner" output="screen"/> -->
```

### Following the Ego Vehicle with the CARLA Spectator Camera

The `spectator_follow` script locks the CARLA spectator (free) camera to the ego vehicle, so the in-simulator view chases the car automatically instead of having to pan manually. It connects to the running CARLA server, looks up the ego actor by its `role_name`, and updates the spectator transform at a fixed rate. It re-acquires the actor if it is (re)spawned, so it works regardless of when/how the ego vehicle appears (e.g. spawned by `autoware_carla_interface` at startup, or (re)spawned later after an RViz-initiated initial pose).

#### Automatic launch (recommended)

Pass `spectator_follow:=true` to `autoware_carla_interface.launch.xml` (or the top-level `e2e_simulator.launch.xml`) to have it start automatically alongside the CARLA interface, using the same `host`/`port`/`ego_vehicle_role_name` values as the main node:

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 \
  spectator_follow:=true \
  ...
```

Camera framing (`--distance`/`--height`/`--pitch`/`--rate`) is not exposed as launch args; run the script manually (below) if you need to tune it.

#### Manual launch

Run it in a separate terminal while CARLA and the Autoware bridge are running:

```bash
ros2 run autoware_carla_interface spectator_follow
```

Common options (all optional):

| Flag         | Default       | Description                                                 |
| ------------ | ------------- | ----------------------------------------------------------- |
| `--host`     | `localhost`   | CARLA server host                                           |
| `--port`     | `2000`        | CARLA server RPC port                                       |
| `--role`     | `ego_vehicle` | `role_name` attribute of the ego actor to follow            |
| `--distance` | `8.0`         | Meters behind the ego vehicle (use `0` for a top-down view) |
| `--height`   | `4.0`         | Meters above the ego vehicle                                |
| `--pitch`    | `-15.0`       | Camera pitch in degrees (negative looks down)               |
| `--rate`     | `30.0`        | Update rate in Hz                                           |

For a top-down view directly above the ego vehicle:

```bash
ros2 run autoware_carla_interface spectator_follow --distance 0 --height 30 --pitch -90
```

Press `Ctrl+C` to stop the script. It will keep retrying if the ego actor is not yet spawned, and re-acquire it if it is removed and respawned.

## Inner-workings / Algorithms

The `InitializeInterface` class is key to setting up both the CARLA world and the ego vehicle. It fetches configuration parameters through the `autoware_carla_interface.launch.xml`.

The main simulation loop runs within the `carla_ros2_interface` class. This loop ticks simulation time inside the CARLA simulator at `fixed_delta_seconds` time, where data is received and published as ROS 2 messages at frequencies defined in `self.sensor_frequencies`.

Ego vehicle commands from Autoware are processed through the `autoware_raw_vehicle_cmd_converter`, which calibrates these commands for CARLA. The calibrated commands are then fed directly into CARLA control via `CarlaDataProvider`.

### Configurable Parameters for World Loading

All the key parameters can be configured in `autoware_carla_interface.launch.xml`.

| Name                              | Type   | Default Value                                                                     | Description                                                                                                                                                                                                         |
| --------------------------------- | ------ | --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `host`                            | string | "localhost"                                                                       | Hostname for the CARLA server                                                                                                                                                                                       |
| `port`                            | int    | "2000"                                                                            | Port number for the CARLA server                                                                                                                                                                                    |
| `timeout`                         | int    | 20                                                                                | Timeout for the CARLA client                                                                                                                                                                                        |
| `ego_vehicle_role_name`           | string | "ego_vehicle"                                                                     | Role name for the ego vehicle                                                                                                                                                                                       |
| `vehicle_type`                    | string | "vehicle.toyota.prius"                                                            | Blueprint ID of the vehicle to spawn. The Blueprint ID of vehicles can be found in [CARLA Blueprint ID](https://carla.readthedocs.io/en/latest/catalogue_vehicles/)                                                 |
| `spawn_point`                     | string | None                                                                              | Coordinates for spawning the ego vehicle (None is random). Format = [x, y, z, roll, pitch, yaw]                                                                                                                     |
| `sync_mode`                       | bool   | True                                                                              | Boolean flag to set synchronous mode in CARLA                                                                                                                                                                       |
| `fixed_delta_seconds`             | double | 0.1                                                                               | Time step for the simulation (related to client FPS)                                                                                                                                                                |
| `use_traffic_manager`             | bool   | False                                                                             | Boolean flag to set traffic manager in CARLA                                                                                                                                                                        |
| `max_real_delta_seconds`          | double | 0.1                                                                               | Parameter to limit the simulation speed below `fixed_delta_seconds`                                                                                                                                                 |
| `weather`                         | string | "ClearNoon"                                                                       | CARLA weather preset applied on world load. Must match an attribute name of `carla.WeatherParameters` (e.g. `ClearNoon`, `CloudyNoon`, `WetNoon`, `HardRainNoon`, `ClearNight`).                                    |
| `sensor_kit_name`                 | string | "carla_sensor_kit_description"                                                    | Name of the sensor kit package to use for sensor configuration. Should be the \*\_description package containing config/sensor_kit_calibration.yaml                                                                 |
| `use_light_weight_sensor_mapping` | bool   | False                                                                             | If True, uses `sensor_mapping_light_weight.yaml` instead of the default `sensor_mapping.yaml` to reduce simulator load. See [Sensor Mapping (CARLA-specific)](#2-sensor-mapping-carla-specific) for details.        |
| `sensor_mapping_file`             | string | "$(find-pkg-share autoware_carla_interface)/config/sensor_mapping.yaml"           | Path to sensor mapping YAML configuration file. When `use_light_weight_sensor_mapping` is True, this defaults to `config/sensor_mapping_light_weight.yaml`.                                                         |
| `config_file`                     | string | "$(find-pkg-share autoware_carla_interface)/raw_vehicle_cmd_converter.param.yaml" | Control mapping file to be used in `autoware_raw_vehicle_cmd_converter`. Current control are calibrated based on `vehicle.toyota.prius` Blueprints ID in CARLA. Changing the vehicle type may need a recalibration. |
| `traffic_light.publish`           | bool   | False                                                                             | Publish the CARLA traffic-light states on `/perception/traffic_light_recognition/traffic_signals`. See [Publishing CARLA Traffic-Light States](#publishing-carla-traffic-light-states).                             |
| `traffic_light.force_green`       | bool   | False                                                                             | Set every CARLA traffic light to green and freeze it at startup (un-frozen at shutdown).                                                                                                                            |
| `traffic_light.map_path`          | string | ""                                                                                | Lanelet2 map (.osm) the CARLA traffic lights are matched to by position. Empty uses the OpenDRIVE signal id as the group id.                                                                                        |
| `traffic_light.match_distance`    | double | 5.0                                                                               | Maximum head-to-head distance (m) accepted when matching a CARLA traffic light to a lanelet2 head.                                                                                                                  |
| `traffic_light.match_ratio`       | double | 0.6                                                                               | Ambiguity threshold: a match is rejected when nearest > ratio * second-nearest (different signal). Lower is stricter.                                                                                               |
| `traffic_light.id_map`            | string | ""                                                                                | Manual overrides `opendrive_id:group_id[\|group_id...],...`, taking precedence over position matching.                                                                                                              |

### Sensor Configuration

The interface uses the **`carla_sensor_kit`** which provides 6 cameras for 360-degree coverage, LiDAR, IMU, and GNSS sensors. Sensor configurations are dynamically loaded from Autoware sensor kit calibration files through two configuration files:

#### 1. Sensor Kit Calibration (from Autoware sensor kit)

Located in `<sensor_kit_name>_description/config/sensor_kit_calibration.yaml`

Defines sensor positions and orientations relative to `base_link` (rear axle center). Example:

```yaml
sensor_kit_base_link:
  CAM_FRONT/camera_link:
    x: 2.225
    y: 0.000
    z: 1.600
    roll: 0.000
    pitch: 0.000
    yaw: 0.000 # Angles in radians
```

#### 2. Sensor Mapping (CARLA-specific)

Located in `config/sensor_mapping.yaml`

Maps Autoware sensors to CARLA sensor types and parameters. Key sections:

- `default_sensor_kit_name`: Default sensor kit to use (e.g., `carla_sensor_kit_description`)
- `sensor_mappings`: Maps each sensor to CARLA type and ROS topics
- `enabled_sensors`: List of sensors to spawn in CARLA
- `vehicle_config` (optional): Vehicle parameters like wheelbase

Example sensor mapping:

```yaml
sensor_mappings:
  CAM_FRONT/camera_link:
    carla_type: sensor.camera.rgb
    id: CAM_FRONT
    ros_config:
      frame_id: CAM_FRONT/camera_optical_link
      topic_image: /sensing/camera/CAM_FRONT/image_raw
      topic_info: /sensing/camera/CAM_FRONT/camera_info
      frequency_hz: 11
      qos_profile: reliable
    parameters:
      image_size_x: 1600
      image_size_y: 900
      fov: 70.0
```

For CARLA sensor parameters, see [CARLA Sensor Reference](https://carla.readthedocs.io/en/latest/ref_sensors/).

##### Light-Weight Sensor Mapping

For machines with limited GPU/CPU resources, an alternative `config/sensor_mapping_light_weight.yaml` is provided to reduce simulator load. Compared to the default mapping, it:

- Uses a **single front camera** (`CAM_FRONT`) instead of the 6-camera 360-degree setup, with a wider FOV (120°) and lower resolution (1080x720) to roughly cover the area in front of the vehicle.
- Lowers sensor frequencies (e.g., LiDAR/camera at 10 Hz instead of 11 Hz).
- Keeps the same LiDAR, IMU, and GNSS configuration as the default mapping.

When enabled via `use_light_weight_sensor_mapping:=True`, the launch file also skips the `image_transport` republish nodes for the disabled cameras and the `multi_camera_combiner` node automatically.

Example usage:

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/autoware_map/Town01 \
    vehicle_model:=sample_vehicle \
    sensor_model:=carla_sensor_kit \
    simulator_type:=carla \
    use_light_weight_sensor_mapping:=True
```

Note that with this configuration, features that depend on the surround cameras (such as the multi-camera RViz view) are unavailable.

### World Loading

The `carla_ros.py` sets up the CARLA world:

1. **Client Connection**:

   ```python
   client = carla.Client(self.local_host, self.port)
   client.set_timeout(self.timeout)
   ```

2. **Load the Map**:

   Map loaded in CARLA world with map according to `carla_map` parameter.

   ```python
   client.load_world(self.map_name)
   self.world = client.get_world()
   ```

3. **Spawn Ego Vehicle**:

   Vehicle are spawn according to `vehicle_type`, `spawn_point`, and `agent_role_name` parameter.

   ```python
   spawn_point = carla.Transform()
   point_items = self.spawn_point.split(",")
   if len(point_items) == 6:
      spawn_point.location.x = float(point_items[0])
      spawn_point.location.y = float(point_items[1])
      spawn_point.location.z = float(point_items[2]) + 2
      spawn_point.rotation.roll = float(point_items[3])
      spawn_point.rotation.pitch = float(point_items[4])
      spawn_point.rotation.yaw = float(point_items[5])
   CarlaDataProvider.request_new_actor(self.vehicle_type, spawn_point, self.agent_role_name)
   ```

## Traffic Light Recognition

The maps provided by the Carla Simulator ([Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)) currently lack proper traffic light components for Autoware and have different latitude and longitude coordinates compared to the pointcloud map. To enable traffic light recognition, follow the steps below to modify the maps.

- Options to Modify the Map
  - A. Create a New Map from Scratch
  - Use the [TIER IV Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) to create a new map.

  - B. Modify the Existing Carla Lanelet2 Maps
  - Adjust the longitude and latitude of the [Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/) to align with the PCD (origin).
    - Use this [tool](https://github.com/mraditya01/offset_lanelet2/tree/main) to modify the coordinates.
    - Snap Lanelet with PCD and add the traffic lights using the [TIER IV Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/).

- When using the TIER IV Vector Map Builder, you must convert the PCD format from `binary_compressed` to `ascii`. You can use `pcl_tools` for this conversion.
- For reference, an example of Town01 with added traffic lights at one intersection can be downloaded [here](https://drive.google.com/drive/folders/1QFU0p3C8NW71sT5wwdnCKXoZFQJzXfTG?usp=sharing).
- Alternatively, add CARLA's own traffic lights to the existing map with `tools/carla_tl_to_lanelet2.py` and bypass camera recognition, see [Publishing CARLA Traffic-Light States](#publishing-carla-traffic-light-states).

### Publishing CARLA Traffic-Light States

Instead of running camera-based recognition, the bridge can publish the CARLA server's
traffic-light states directly. Setting `traffic_light.publish:=true` publishes an
`autoware_perception_msgs/TrafficLightGroupArray` on
`/perception/traffic_light_recognition/traffic_signals` every tick, stamped with the simulation
time of the frame. Each CARLA light is reported as a circular signal whose color and status follow
the CARLA state: `Red`/`Yellow`/`Green` map to `RED`/`AMBER`/`GREEN` with status `SOLID_ON`, the
known-dark `Off` state maps to status `SOLID_OFF`, and only a state the bridge cannot interpret is
published as `UNKNOWN`/`UNKNOWN`. With the SUMO co-simulation and `tls_manager:=sumo`, the CARLA
lights follow SUMO, so the SUMO signal states reach Autoware the same way.

Autoware keys traffic signals by `traffic_light_group_id`, the id of a `traffic_light`
regulatory element in the lanelet2 map. The bridge resolves which group(s) each CARLA light
belongs to once, on the first publishing tick, in order of precedence:

1. **`traffic_light.id_map` override.** If the light's OpenDRIVE signal id appears in the
   `opendrive_id:group_id[|group_id...],...` map, those group ids are used directly. A malformed
   entry is skipped with a warning; the remaining entries still apply.
2. **Position matching (`traffic_light.map_path`).** Each CARLA light head (the mean of
   `get_light_boxes()`) is matched to the nearest map traffic-light head, and its state is
   published under every regulatory element that references that head.
3. **OpenDRIVE-id fallback.** With no map path, the OpenDRIVE signal id is used directly as the
   group id (correct for maps generated with `tools/carla_tl_to_lanelet2.py`, see below).

Position matching is conservative: if a head of a _different_ signal is nearly as close
(`traffic_light.match_ratio`), exactly as close, or nothing is within
`traffic_light.match_distance`, the light is left unpublished and logged as ambiguous / unmatched.
Check the startup log for the report (`N matched, M ambiguous, K too far, L overridden`); it also
warns when the map has no traffic-light heads at all. If several CARLA lights resolve to the same
group id with different states, the most restrictive one is published
(red > amber > unknown > off > green). No message is published when no light could be resolved.

> Position matching reads the lanelet2 node `local_x`/`local_y` tags (the Autoware map frame) and
> converts each CARLA head with `map = (carla_x, -carla_y)`, the same conversion as the GNSS pose.
> Unlike upstream there is no `map_origin_x`/`map_origin_y` offset; it is not needed for the stock
> CARLA towns (verified on Town01).

`traffic_light.force_green:=true` sets every CARLA traffic light to green and freezes it at
startup, so the ego can pass every signalized stop line (e.g. camera-less closed-loop runs). At
shutdown the lights are un-frozen and each traffic-light group is reset, so the intersections
resume a consistent cycle. It is refused at startup together with `use_sumo:=true` and
`tls_manager:=sumo` or `carla`, where the lights are synchronized with SUMO.

The parameters are validated at startup: a `traffic_light.map_path` that is not a file,
`match_distance <= 0`, or `match_ratio` outside (0, 1] stops the bridge with an error.

#### Adding traffic lights to the CARLA Lanelet2 maps

`tools/carla_tl_to_lanelet2.py` reads the traffic lights of the loaded CARLA world and writes a
copy of a lanelet2 map with, for every CARLA light:

- a `traffic_light` linestring per light head (bottom edge of the head, tagged with its height),
- a `stop_line` linestring across the lane at the CARLA stop position, moved forward along the
  lane by `--stop-line-forward-offset` [m] (default 0),
- a `traffic_light` regulatory element (`refers` = heads, `ref_line` = stop line) whose id is the
  OpenDRIVE signal id (plus `--group-id-offset`),
- a reference to it on the lanelet that contains the stop position (the approach lanelet).

```bash
# CARLA running with Town01 loaded
python3 tools/carla_tl_to_lanelet2.py ~/autoware_map/Town01/lanelet2_map.osm --stop-line-forward-offset 6
# -> ~/autoware_map/Town01/lanelet2_map_with_tl.osm (the input map is not modified)
```

CARLA's stop positions lie about 7 m before the light heads (6.7-8.9 m in Town01), so without an
offset the ego vehicle stops well back from the intersection. With `--stop-line-forward-offset 6`
the Town01 stop lines end up 0.96-7.08 m before the end of their approach lanelet. A stop line
that the offset would move out of its lanelet (into the intersection) is not added and is
reported as `not added` (exit code 2).

Keep the output in the same map directory (the CARLA map name is taken from the directory name).
The tool stops with an error if an OpenDRIVE id collides with an existing lanelet2 id (use
`--group-id-offset`) or if the map already has traffic-light regulatory elements.
`--dump-json` / `--from-json` save and reuse the CARLA data, so the map can also be regenerated
without CARLA.

For SUMO co-simulation, `tools/check_sumo_lanelet2_tl_ids.py NET.net.xml MAP_with_tl.osm` checks
that the SUMO net (`linkSignalID` params) and the generated map refer to the same CARLA lights.
The whole procedure for another town is in the Claude Code skill
`.claude/skills/carla-town-signals/SKILL.md` (repository root).

#### Running with Autoware

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml simulator_type:=carla \
  map_path:=$HOME/autoware_map/Town01 lanelet2_map_file:=lanelet2_map_with_tl.osm \
  vehicle_model:=sample_vehicle sensor_model:=carla_sensor_kit \
  traffic_light.publish:=true
```

In `e2e_simulator.launch.xml` (local change in `autoware_launch`), `traffic_light.map_path`
defaults to the map Autoware loads (`map_path`/`lanelet2_map_file`), and
`traffic_light.publish:=true` disables the camera-based traffic light recognition
(`use_traffic_light_recognition:=false`) so that the bridge is the only publisher of the topic.
`autoware_carla_interface.launch.xml` also skips the CAM_FRONT relays feeding that recognition.
On Town01 the ego stops about 0.5 m before the stop line on red and starts on green.

## SUMO Vehicle Blueprints

With the SUMO co-simulation on Town01, each SUMO vehicle gets the CARLA body closest in size to its
SUMO vType, the same way as in the Vissim co-simulation (design and results:
`docs/SUMO_CARLA_Autoware_ブループリント割り当て_実装計画_v1.0.md`).

- Town01's SUMO vehicle groups (`rou/Town01.rou.xml` in the CARLA repository) now hold the 14 Vissim
  `Town01.inpx` 2D/3D models (their sizes and in-type shares, `vissim_fleet_vtypes.rou.xml`), so both
  co-simulations run the same vehicles and get the same CARLA bodies. The shares between the groups
  (70% cars, 10% trucks, 5% tractors, 5% buses, 10% motorbikes) are unchanged.
- Sizes are the bodies as a LiDAR sees them (`data/carla_fleet/mesh_measurements_carla0.9.15.json`),
  and the rules are `data/carla_fleet/carla_variant_rules.yaml`, shared with the Vissim setup.
  `tools/carla_fleet/build_carla_variants.py --sumocfg Town01.sumocfg` groups the vTypes of all
  `route-files` by size and writes `sumo_integration/data/carla_variants_sumo_town01.yaml`. Trucks,
  tractors and buses are 2-3 overlapping `carlamotors.carlacola` bodies.
- The bridge checks the table before starting SUMO (blueprints present, rules, measurements and the
  sha256 of every `route-files` file of the given `.sumocfg` unchanged, same CARLA version) and looks
  every vehicle up by its vType id, checking its size. A missing or stale table, or a vehicle whose
  vType is not in the table, stops the bridge with `Error: CARLA blueprint assignment: <code>: ...`
  (exit code 1) instead of substituting another blueprint.
- Launch argument `sumo_carla_variants` (default `auto`: `sumo_integration/data/carla_variants_sumo_<sumocfg name, lower case>.yaml`
  if it exists, otherwise the previous choice). Town05 and Town10HD have no table and keep the
  previous choice (the vType id as blueprint, else a random one of `sumo_integration/data/vtypes.json`).
  `sumo_carla_variants:=legacy_vtypes` forces the previous choice; on Town01 buses then have no CARLA
  body at all (no `bus` class in `vtypes.json`), so use it only for comparisons.
- The SUMO scenarios (Town01, Town05, Town10HD: `.sumocfg`, net, routes) are kept in this package under
  `sumo/` and installed to `share/autoware_carla_interface/sumo/`. A relative
  `sumo_cfg_file` (e.g. `sumo_cfg_file:=Town01.sumocfg`) is taken from there; an absolute path is used
  as it is. A missing file stops the bridge at startup with `Error: invalid parameters: sumo_cfg_file ...
  not found`, and the startup log shows the file used (`SUMO config: <path>`).
- On Town01 with Autoware, the bus seen from the side is perceived 12.1-13.1 m long and the truck
  7.56 m (models: 12.14 m, 7.93 m), close to the Vissim results; with `legacy_vtypes` both trucks
  and tractors are a single 5.11 m `carlacola`.

See the start-up guide (`docs/SUMO-CARLA-Autoware_co-sim_起動手順.md`, 2.2.3) for regenerating the
table (only needed after changing the SUMO vehicles, the rules or the CARLA version) and 3.5 for the
meaning of each `<code>`.

## Tips

- Misalignment might occurs during initialization, pressing `init by gnss` button should fix it.
- Changing the `fixed_delta_seconds` can increase the simulation tick (default 0.1 s), some sensor params in `sensor_mapping.yaml` need to be adjusted when it is changed (example: LIDAR rotation frequency should match the FPS).

## Known Issues and Future Works

- **Testing on procedural maps (Adv Digital Twin)**: Currently unable to test due to failures in creating the Adv Digital Twin map.
- **Traffic light recognition**: The default CARLA Lanelet2 maps lack proper traffic light regulatory elements. See the "Traffic Light Recognition" section above for workarounds, or generate them with `tools/carla_tl_to_lanelet2.py` and bypass camera recognition with `traffic_light.publish` (see "Publishing CARLA Traffic-Light States").
- **SUMO vehicle blueprints**: Only Town01 has a table. Trucks, tractors and buses are stacked
  `carlacola` delivery trucks, which looks odd up close, and a stacked vehicle can be perceived as
  two objects while turning (seen once in 83 samples). A bus right behind the ego is perceived about
  5 m long, as only its front is visible (as for any long vehicle).
- **SUMO yellow lights**: With `tls_manager:=sumo`, a CARLA light mapped to SUMO links in different states (e.g. straight `G`, turn `y`) is forced to red, so Autoware may receive red while SUMO shows yellow or green for the straight link.
