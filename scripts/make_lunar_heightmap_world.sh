#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DEM_PATH="${1:-/mnt/c/Users/Username/Downloads/Lunar_LRO_LOLA_Global_LDEM_118m_Mar2014.tif}"
MEDIA_DIR="${2:-$REPO_ROOT/src/togo/togo_gz/media}"
WORLD_DIR="${3:-$REPO_ROOT/src/togo/togo_gz/worlds}"
XOFF="${4:-40000}"
YOFF="${5:-12000}"
SRC_SIZE="${6:-2048}"
OUT_SIZE="${7:-1025}"
WORLD_SIZE_M="${8:-350}"
HEIGHT_M="${9:-35}"

if ! command -v gdal_translate >/dev/null 2>&1; then
  echo "gdal_translate was not found. Install it in WSL with:" >&2
  echo "  sudo apt update" >&2
  echo "  sudo apt install -y gdal-bin" >&2
  exit 1
fi

mkdir -p "$MEDIA_DIR" "$WORLD_DIR"
CROP_TIF="$MEDIA_DIR/lunar_patch_crop.tif"
HEIGHTMAP_PNG="$MEDIA_DIR/lunar_heightmap.png"
WORLD_SDF="$WORLD_DIR/lunar_heightmap.world.sdf"

echo "Cropping DEM patch..."
echo "  input: $DEM_PATH"
echo "  srcwin: x=$XOFF y=$YOFF size=${SRC_SIZE}x${SRC_SIZE}"
gdal_translate \
  -srcwin "$XOFF" "$YOFF" "$SRC_SIZE" "$SRC_SIZE" \
  -outsize "$OUT_SIZE" "$OUT_SIZE" \
  -of GTiff \
  "$DEM_PATH" "$CROP_TIF"

echo "Converting crop to 8-bit grayscale PNG heightmap..."
gdal_translate \
  -of PNG \
  -ot Byte \
  -scale \
  "$CROP_TIF" "$HEIGHTMAP_PNG"

cat > "$WORLD_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="lunar_heightmap_test">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.02 0.02 0.025 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun_low_angle">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0.4 0.25 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.65 0.25 -0.72</direction>
    </light>

    <model name="lunar_terrain_patch">
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <heightmap>
              <uri>../media/lunar_heightmap.png</uri>
              <size>$WORLD_SIZE_M $WORLD_SIZE_M $HEIGHT_M</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <heightmap>
              <uri>../media/lunar_heightmap.png</uri>
              <size>$WORLD_SIZE_M $WORLD_SIZE_M $HEIGHT_M</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.45 0.43 0.39 1</ambient>
            <diffuse>0.62 0.60 0.55 1</diffuse>
            <specular>0.03 0.03 0.03 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
EOF

echo "Done."
echo "  heightmap: $HEIGHTMAP_PNG"
echo "  world:     $WORLD_SDF"
echo ""
echo "Try it with:"
echo "  gz sim -r $WORLD_SDF"
