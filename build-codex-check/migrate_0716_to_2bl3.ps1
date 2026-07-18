$ErrorActionPreference = "Stop"

$workspace = Split-Path $PSScriptRoot -Parent
$instanceDir = Get-ChildItem "$workspace/instances_front&back" -Directory |
    Where-Object { $_.Name -like "0716_*" } |
    Select-Object -First 1
if ($null -eq $instanceDir) {
    throw "0716 instance directory was not found."
}

$instancePath = Join-Path $instanceDir.FullName "algorithm/beacon_image.c"
$boardRepo = "D:/smartcar/21_smartcar/HDUASC-SmartCar-21st-FlyOverMinefield/CYT2BL3_Image"
$boardRelativePath = "project/code/Image/image.c"
$boardPath = Join-Path $boardRepo $boardRelativePath

$instance = [System.IO.File]::ReadAllText($instancePath) -replace "`r`n", "`n"
$board = (& git -C $boardRepo show "HEAD:$boardRelativePath") -join "`n"
if ($LASTEXITCODE -ne 0) {
    throw "Failed to read the committed 2BL3 board adapter."
}

$algorithmMarker = "#define CAR_LAMP_BINARY_THRESHOLD"
$boardTailMarker = "static uint8 s_image_frame"
$debugMarker = "unsigned char beacon_image_debug_threshold("
$supportStartMarker = "static const uint16 g_front_beacon_area_upper_default"
$supportEndMarker = "static void beacon_image_init(void)"

$instanceStart = $instance.IndexOf($algorithmMarker)
$instanceEnd = $instance.IndexOf($debugMarker)
$boardStart = $board.IndexOf($algorithmMarker)
$boardTail = $board.IndexOf($boardTailMarker)
$supportStart = $board.IndexOf($supportStartMarker)
$supportEnd = $board.IndexOf($supportEndMarker)
$hasRuntimeSupport = (($supportStart -ge 0) -and ($supportEnd -gt $supportStart))
if (($instanceStart -lt 0) -or ($instanceEnd -le $instanceStart) -or
    ($boardStart -lt 0) -or ($boardTail -le $boardStart)) {
    throw "Algorithm or board adapter boundary was not found."
}

$prefix = $board.Substring(0, $boardStart)
$core = $instance.Substring($instanceStart, $instanceEnd - $instanceStart).TrimEnd()
$tail = $board.Substring($boardTail)
$support = if ($hasRuntimeSupport) {
    $board.Substring($supportStart, $supportEnd - $supportStart).TrimEnd()
} else {
    ""
}

if ($hasRuntimeSupport) {
$runtimeDefaults = @'
#define BEACON_AREA_GATE_X_COUNT  (9U)
#define BEACON_AREA_GATE_Y_COUNT  (7U)
#define CAR_LAMP_BINARY_THRESHOLD_DEFAULT 200
#define CAR_LAMP_UPPER_THRESHOLD_DEFAULT 150
#define CAR_LAMP_UPPER_Y_DEFAULT 64.0f
#define CAR_LAMP_BRIDGE_MAX_GAP_DEFAULT 4.0f
#define BEACON_TRACK_THRESHOLD_DEFAULT 105
#define BEACON_EDGE_THRESHOLD_DEFAULT 80
#define BEACON_EDGE_MIN_AREA_DEFAULT 2
#define BEACON_EDGE_MAX_AREA_DEFAULT 60
#define BEACON_TOP_EDGE_MAX_AREA_DEFAULT 50
#define BEACON_LOCAL_RING_INNER_DEFAULT 3.0f
#define BEACON_LOCAL_RING_OUTER_DEFAULT 8.0f
#define BEACON_ISOLATED_GRAY_MIN_DEFAULT 120
#define BEACON_ISOLATED_BG_MAX_DEFAULT 2
#define LAMP_NEAR_BEACON_PAD_DEFAULT 8.0f
#define LAMP_NEAR_BEACON_MIN_AREA_DEFAULT 21
#define LAMP_NEAR_BEACON_BACKGROUND_MAX_DEFAULT 20
#define LAMP_NEAR_BEACON_GRAY_MIN_DEFAULT 150
#define BEACON_MIN_COMPONENT_AREA_DEFAULT 6
#define CAR_LAMP_MIN_AREA_DEFAULT 24
#define CAR_LAMP_MAX_AREA_DEFAULT 230
#define CAR_LAMP_FRONT_MAX_AREA_DEFAULT 180
#define CAR_LAMP_MIN_ELONGATION_DEFAULT 1.6f
#define CAR_LAMP_MIN_LENGTH_DEFAULT 12.0f
#define CAR_LAMP_FRONT_MIN_LENGTH_DEFAULT 10.0f
#define B0_MATCH_DISTANCE_DEFAULT 18.0f
#define B0_INIT_CONFIRM_FRAMES_DEFAULT 2
#define BEACON_MAX_MISSES_DEFAULT 3
#define KALMAN_GATE_DISTANCE_DEFAULT 24.0f
#define KALMAN_NEW_TARGET_DISTANCE_DEFAULT 36.0f
#define FILTER_POS_ALPHA_DEFAULT 0.65f
#define FILTER_VEL_ALPHA_DEFAULT 0.30f
#define AREA_GATE_ENABLED_DEFAULT 0
#define STREAM_MODE_DEFAULT IMAGE_STREAM_MODE_RAW
'@
$core = $core.Replace(
    "static temporal_track_t g_car_track;",
    "static temporal_track_t g_car_track;`nstatic temporal_track_t g_stream_beacon_track;")
$core = $core.Replace(
    "int g_beacon_binary_threshold = BEACON_BINARY_THRESHOLD_DEFAULT;",
    $runtimeDefaults + "`n" + $support)
$core = $core.Replace("void beacon_image_init(void)", "static void beacon_image_init(void)")
$core = $core.Replace("void beacon_image_reset_temporal(void)", "static void beacon_image_reset_temporal(void)")
$core = $core.Replace("void beacon_image_process(`n", "static void beacon_image_process(`n")
$core = $core.Replace("static static void beacon_image_reset_temporal(void);", "static void beacon_image_reset_temporal(void);")
$core = $core.Replace(
    "    memset(&g_car_track, 0, sizeof(g_car_track));`n    g_temporal_car_hard_rejected = 0U;",
    "    memset(&g_car_track, 0, sizeof(g_car_track));`n    memset(&g_stream_beacon_track, 0, sizeof(g_stream_beacon_track));`n    g_temporal_car_hard_rejected = 0U;")

$reinforceStart = $core.IndexOf("static void reinforce_tracked_beacon(")
$reinforceEnd = $core.IndexOf("static component_t grow_component(", $reinforceStart)
if (($reinforceStart -lt 0) -or ($reinforceEnd -le $reinforceStart)) {
    throw "Tracked beacon reinforcement function was not found."
}
$reinforce = $core.Substring($reinforceStart, $reinforceEnd - $reinforceStart)
$updatedReinforce = $reinforce.Replace(
    "    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])",
    "    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],`n    const temporal_track_t *track)")
$updatedReinforce = $updatedReinforce.Replace(
    "    if((g_b0_track.confirmed == 0U) ||`n       (g_b0_track.misses >= BEACON_MAX_MISSES))",
    "    if((track == 0) ||`n       (track->confirmed == 0U) ||`n       (track->misses >= BEACON_MAX_MISSES))")
$updatedReinforce = $updatedReinforce.Replace("g_b0_track.x", "track->x")
$updatedReinforce = $updatedReinforce.Replace("g_b0_track.y", "track->y")
$updatedReinforce = $updatedReinforce.Replace("g_b0_track.vx", "track->vx")
$updatedReinforce = $updatedReinforce.Replace("g_b0_track.vy", "track->vy")
$core = $core.Remove($reinforceStart, $reinforceEnd - $reinforceStart)
$core = $core.Insert($reinforceStart, $updatedReinforce)
$core = $core.Replace(
    "    result->beacon_count = 0;`n    g_bad_shape_count = 0U;`n    threshold_beacon_image(image);`n    reinforce_tracked_beacon(image);",
    "    result->beacon_count = 0;`n    g_bad_shape_count = 0U;`n    g_stream_beacon_track = g_b0_track;`n    threshold_beacon_image(image);`n    reinforce_tracked_beacon(image, &g_stream_beacon_track);")

$algorithmCodeStart = $core.IndexOf("static void beacon_image_init(void)")
if ($algorithmCodeStart -lt 0) {
    throw "Board algorithm function block was not found."
}
$algorithmCode = $core.Substring($algorithmCodeStart)
$runtimeMap = [ordered]@{
    "CAR_LAMP_BINARY_THRESHOLD" = "g_image_runtime_params.car_lamp_binary_threshold"
    "CAR_LAMP_UPPER_THRESHOLD" = "g_image_runtime_params.car_lamp_upper_threshold"
    "CAR_LAMP_UPPER_Y" = "g_image_runtime_params.car_lamp_upper_y"
    "CAR_LAMP_BRIDGE_MAX_GAP" = "g_image_runtime_params.car_lamp_bridge_max_gap"
    "BEACON_TRACK_THRESHOLD" = "g_image_runtime_params.beacon_track_threshold"
    "BEACON_EDGE_THRESHOLD" = "g_image_runtime_params.beacon_edge_threshold"
    "BEACON_EDGE_MIN_AREA" = "g_image_runtime_params.beacon_edge_min_area"
    "BEACON_EDGE_MAX_AREA" = "g_image_runtime_params.beacon_edge_max_area"
    "BEACON_TOP_EDGE_MAX_AREA" = "g_image_runtime_params.beacon_top_edge_max_area"
    "BEACON_LOCAL_RING_INNER" = "g_image_runtime_params.beacon_local_ring_inner"
    "BEACON_LOCAL_RING_OUTER" = "g_image_runtime_params.beacon_local_ring_outer"
    "BEACON_ISOLATED_GRAY_MIN" = "g_image_runtime_params.beacon_isolated_gray_min"
    "BEACON_ISOLATED_BG_MAX" = "g_image_runtime_params.beacon_isolated_bg_max"
    "LAMP_NEAR_BEACON_PAD" = "g_image_runtime_params.lamp_near_beacon_pad"
    "LAMP_NEAR_BEACON_MIN_AREA" = "g_image_runtime_params.lamp_near_beacon_min_area"
    "LAMP_NEAR_BEACON_GRAY_MIN" = "g_image_runtime_params.lamp_near_beacon_gray_min"
    "LAMP_NEAR_BEACON_BACKGROUND_MAX" = "g_image_runtime_params.lamp_near_beacon_background_max"
    "BEACON_MIN_COMPONENT_AREA" = "g_image_runtime_params.beacon_min_component_area"
    "CAR_LAMP_MIN_AREA" = "g_image_runtime_params.car_lamp_min_area"
    "CAR_LAMP_MAX_AREA" = "g_image_runtime_params.car_lamp_max_area"
    "CAR_LAMP_MIN_ELONGATION" = "g_image_runtime_params.car_lamp_min_elongation"
    "CAR_LAMP_MIN_LENGTH" = "g_image_runtime_params.car_lamp_min_length"
    "B0_MATCH_DISTANCE" = "g_image_runtime_params.b0_match_distance"
    "B0_INIT_CONFIRM_FRAMES" = "g_image_runtime_params.b0_init_confirm_frames"
    "BEACON_MAX_MISSES" = "g_image_runtime_params.beacon_max_misses"
    "KALMAN_GATE_DISTANCE" = "g_image_runtime_params.kalman_gate_distance"
    "KALMAN_NEW_TARGET_DISTANCE" = "g_image_runtime_params.kalman_new_target_distance"
    "FILTER_POS_ALPHA" = "g_image_runtime_params.filter_pos_alpha"
    "FILTER_VEL_ALPHA" = "g_image_runtime_params.filter_vel_alpha"
}
foreach ($entry in $runtimeMap.GetEnumerator()) {
    $algorithmCode = $algorithmCode.Replace($entry.Key, $entry.Value)
}
$core = $core.Substring(0, $algorithmCodeStart) + $algorithmCode
} else {
    $core = $core.Replace(
        "int g_beacon_binary_threshold = BEACON_BINARY_THRESHOLD_DEFAULT;",
        "int32 g_beacon_binary_threshold = IMAGE_BEACON_BINARY_THRESHOLD_DEFAULT;")
    $core = $core.Replace("void beacon_image_init(void)", "static void beacon_image_init(void)")
    $core = $core.Replace("void beacon_image_reset_temporal(void)", "static void beacon_image_reset_temporal(void)")
    $core = $core.Replace("void beacon_image_process(`n", "static void beacon_image_process(`n")
}

$core = $core.Replace(
    "    circle.radius = sqrtf((float)comp->area / PI_F);`n    circle.valid = 1;",
    "    circle.radius = sqrtf((float)comp->area / PI_F);`n    circle.area = (float)comp->area;`n    circle.valid = 1;")
$oldTemporalBeacon = @'
static void write_temporal_beacon(
    const temporal_track_t *track,
    beacon_result_t *result,
    int matched_index)
{
    (void)matched_index;
    memset(result->temporal_beacons, 0, sizeof(result->temporal_beacons));
    result->temporal_beacons[0].x = track->x;
    result->temporal_beacons[0].y = track->y;
    result->temporal_beacons[0].radius = track->radius;
    result->temporal_beacons[0].valid = 1U;
    result->temporal_beacon_count = 1U;
}
'@
$newTemporalBeacon = @'
static void write_temporal_beacon(
    const temporal_track_t *track,
    beacon_result_t *result,
    int matched_index)
{
    beacon_circle_t old_beacons[BEACON_MAX_BEACON_COUNT];
    int old_count = result->beacon_count;
    int i;
    int out = 1;

    if(old_count > BEACON_MAX_BEACON_COUNT)
    {
        old_count = BEACON_MAX_BEACON_COUNT;
    }
    memcpy(old_beacons, result->beacons, sizeof(old_beacons));
    memset(result->beacons, 0, sizeof(result->beacons));

    result->beacons[0].x = track->x;
    result->beacons[0].y = track->y;
    result->beacons[0].radius = track->radius;
    result->beacons[0].area = track->area;
    result->beacons[0].valid = 1U;

    for(i = 0; (i < old_count) && (out < BEACON_MAX_BEACON_COUNT); i++)
    {
        if(i == matched_index)
        {
            continue;
        }
        result->beacons[out] = old_beacons[i];
        out++;
    }
    result->beacon_count = (unsigned char)out;
}
'@
if (-not $core.Contains($oldTemporalBeacon)) {
    throw "Analyzer temporal beacon writer was not found."
}
$core = $core.Replace($oldTemporalBeacon, $newTemporalBeacon)

$oldTemporalCar = @'
static void write_temporal_car(const temporal_track_t *track, beacon_result_t *result)
{
    result->temporal_car_lamps[0].cx = track->x;
    result->temporal_car_lamps[0].cy = track->y;
    result->temporal_car_lamps[0].width = track->width;
    result->temporal_car_lamps[0].length = track->length;
    result->temporal_car_lamps[0].angle = track->angle;
    result->temporal_car_lamps[0].valid = 1U;
    result->temporal_car_lamp_count = 1U;
}

static void write_current_car_as_temporal(
    const beacon_rect_t *car,
    beacon_result_t *result)
{
    result->temporal_car_lamps[0] = *car;
    result->temporal_car_lamp_count = 1U;
}
'@
$newTemporalCar = @'
static void write_temporal_car(const temporal_track_t *track, beacon_result_t *result)
{
    result->car_lamps[0].cx = track->x;
    result->car_lamps[0].cy = track->y;
    result->car_lamps[0].width = track->width;
    result->car_lamps[0].length = track->length;
    result->car_lamps[0].angle = track->angle;
    result->car_lamps[0].valid = 1U;
    result->car_lamp_count = 1U;
}

static void write_current_car_as_temporal(
    const beacon_rect_t *car,
    beacon_result_t *result)
{
    result->car_lamps[0] = *car;
    result->car_lamp_count = 1U;
}
'@
if (-not $core.Contains($oldTemporalCar)) {
    throw "Analyzer temporal car writer was not found."
}
$core = $core.Replace($oldTemporalCar, $newTemporalCar)
$core = $core.Replace("    result->temporal_car_lamp_count = 0U;`n`n", "")

$adapterStart = $core.IndexOf("static void sync_legacy_beacons(")
$processStart = $core.IndexOf("static void beacon_image_process(`n")
if (($adapterStart -lt 0) -or ($processStart -le $adapterStart)) {
    throw "Analyzer-only output adapter block was not found."
}
$core = $core.Remove($adapterStart, $processStart - $adapterStart)
$core = $core.Replace("    convert_result_to_analyzer_coordinates(result);`n", "")
$core = $core.Replace("    sync_legacy_beacons(result);`n", "")

$forbidden = @(
    "temporal_beacons",
    "temporal_beacon_count",
    "temporal_car_lamps",
    "temporal_car_lamp_count",
    "BEACON_MAX_CIRCLE_COUNT",
    "convert_result_to_analyzer_coordinates",
    "sync_legacy_beacons"
)
foreach ($token in $forbidden) {
    if ($core.Contains($token)) {
        throw "Analyzer-only token remains in board core: $token"
    }
}

$output = $prefix + "static float beacon_area(const beacon_circle_t *beacon);`n" +
          $core + "`n`n" + $tail
[System.IO.File]::WriteAllText(
    $boardPath,
    ($output -replace "`n", "`r`n"),
    [System.Text.UTF8Encoding]::new($false))

Write-Output "Migrated 0716 algorithm and parameters to: $boardPath"
