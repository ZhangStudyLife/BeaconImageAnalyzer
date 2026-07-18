$ErrorActionPreference = "Stop"

$sourceRepo = "D:/smartcar/21_smartcar/HDUASC-SmartCar-21st-FlyOverMinefield/CYT2BL3_Image"
$sourceRevision = "f650483b1a952e126290ec8fb322fa8dcb321cfe"
$sourceRelativePath = "project/code/Image/image.c"
$workspace = Split-Path $PSScriptRoot -Parent
$instanceDir = Get-ChildItem "$workspace/instances_front&back" -Directory |
    Where-Object { $_.Name -like "0716_*" } |
    Select-Object -First 1
if ($null -eq $instanceDir) {
    throw "Target instance directory was not found."
}
$targetDir = Join-Path $instanceDir.FullName "algorithm"
$targetPath = Join-Path $targetDir "beacon_image.c"

$source = (& git -C $sourceRepo show "${sourceRevision}:$sourceRelativePath") -join "`n"
if ($LASTEXITCODE -ne 0) {
    throw "Failed to read the fixed 2BL3 algorithm baseline."
}
$source = $source -replace "`r`n", "`n"
$bodyStart = $source.IndexOf("#define CAR_LAMP_BINARY_THRESHOLD")
$bodyEnd = $source.IndexOf("static uint8 s_image_frame")
if (($bodyStart -lt 0) -or ($bodyEnd -le $bodyStart)) {
    throw "Board algorithm body was not found."
}

$body = $source.Substring($bodyStart, $bodyEnd - $bodyStart).TrimEnd()
$body = $body.Replace("int32 g_beacon_binary_threshold = IMAGE_BEACON_BINARY_THRESHOLD_DEFAULT;", "int g_beacon_binary_threshold = BEACON_BINARY_THRESHOLD_DEFAULT;")
$body = $body.Replace("static void beacon_image_init(void)", "void beacon_image_init(void)")
$body = $body.Replace("static void beacon_image_reset_temporal(void)", "void beacon_image_reset_temporal(void)")
$body = $body.Replace("static void beacon_image_process(`n", "void beacon_image_process(`n")
$body = $body.Replace("result->beacons[slot].area", "beacon_area(&result->beacons[slot])")
$body = $body.Replace("result->beacons[i].area", "beacon_area(&result->beacons[i])")
$body = $body.Replace("    circle.area = (float)comp->area;`n", "")
$body = $body.Replace("    return beacon->area;", "    return PI_F * beacon->radius * beacon->radius;")
$body = $body.Replace("    result->beacons[0].area = track->area;`n", "")
$body = $body.Replace("#define CAR_LAMP_MAX_AREA         100", "#define CAR_LAMP_MAX_AREA         230")
$body = $body.Replace("#define CAR_LAMP_MIN_LENGTH       12.0f", @'
#define CAR_LAMP_MIN_LENGTH       12.0f
#define CAR_LAMP_MIN_WIDTH        3.5f
#define CAR_LAMP_NARROW_MIN_WIDTH 2.7f
#define CAR_LAMP_NARROW_MIN_ELONGATION 3.5f
#define CAR_LAMP_UPPER_MIN_AREA   120
#define CAR_LAMP_UPPER_MIN_LENGTH 22.0f
#define CAR_LAMP_UPPER_MIN_WIDTH  5.5f
#define CAR_LAMP_UPPER_COMPACT_MIN_Y 20.0f
#define CAR_LAMP_UPPER_COMPACT_MIN_AREA 36
#define CAR_LAMP_UPPER_COMPACT_MIN_LENGTH 14.0f
#define CAR_LAMP_UPPER_COMPACT_MIN_WIDTH 3.0f
#define CAR_LAMP_UPPER_COMPACT_MIN_ELONGATION 3.0f
'@)
$body = $body.Replace("#define BEACON_WEAK_FOOTPRINT_MAX 15", @'
#define BEACON_WEAK_FOOTPRINT_MAX 15
#define BEACON_TOP_GLARE_CENTER_Y  25.0f
#define BEACON_TOP_GLARE_MAX_GRAY  190
#define BEACON_TOP_LARGE_GLARE_MIN_AREA 30
#define BEACON_TOP_LARGE_GLARE_MAX_GRAY 250
#define BEACON_UPPER_WEAK_BOTTOM_Y 45
#define BEACON_UPPER_WEAK_MAX_GRAY 150
#define BEACON_VERTICAL_WEAK_MAX_AREA 12
#define BEACON_VERTICAL_WEAK_MAX_GRAY 180
#define BEACON_VERTICAL_GLARE_BOTTOM_Y 50
#define BEACON_VERTICAL_GLARE_MIN_AREA 13
#define BEACON_VERTICAL_GLARE_MAX_AREA 45
#define BEACON_VERTICAL_GLARE_MIN_ELONGATION 1.8f
#define BEACON_VERTICAL_GLARE_MAX_GRAY 190
#define BEACON_SATURATED_VERTICAL_MAX_Y 35.0f
#define BEACON_SATURATED_VERTICAL_MIN_AREA 13
#define BEACON_SATURATED_VERTICAL_MAX_AREA 45
#define BEACON_SATURATED_VERTICAL_MIN_ELONGATION 2.3f
#define BEACON_SATURATED_VERTICAL_MIN_GRAY 240
#define BEACON_LINEAR_MIN_AREA     12
#define BEACON_LINEAR_MIN_MAJOR    13.0f
#define BEACON_LINEAR_MAX_MINOR    4.5f
#define BEACON_LINEAR_MIN_ELONGATION 6.0f
#define BEACON_TINY_LINE_MAX_AREA  3
#define BEACON_TINY_LINE_MAX_MINOR 1.05f
#define BEACON_TINY_LINE_MIN_ELONGATION 1.8f
#define BEACON_TINY_LINE_MAX_GRAY  200
#define BEACON_TINY_TRACK_MAX_AREA 3.0f
#define BEACON_WEAK_CENTER_THRESHOLD 70
#define BEACON_WEAK_CENTER_MIN_X   70
#define BEACON_WEAK_CENTER_MAX_X   172
#define BEACON_WEAK_CENTER_BASE_MAX_X 125
#define BEACON_WEAK_CENTER_NEAR_RIGHT_MAX_X 145
#define BEACON_WEAK_CENTER_FAR_RIGHT_MIN_X 165
#define BEACON_WEAK_CENTER_RIGHT_MIN_GRAY 120
#define BEACON_WEAK_CENTER_FAR_RIGHT_MIN_Y 55
#define BEACON_WEAK_CENTER_FAR_RIGHT_MIN_AREA 7
#define BEACON_WEAK_CENTER_FAR_RIGHT_MAX_AREA 8
#define BEACON_WEAK_CENTER_FAR_RIGHT_MIN_GRAY 150
#define BEACON_WEAK_CENTER_FAR_RIGHT_MAX_ELONGATION 1.2f
#define BEACON_WEAK_CENTER_MIN_Y   45
#define BEACON_WEAK_CENTER_MAX_Y   100
#define BEACON_WEAK_CENTER_MIN_AREA 3
#define BEACON_WEAK_CENTER_UPPER_MIN_AREA 7
#define BEACON_WEAK_CENTER_UPPER_MAX_AREA 8
#define BEACON_WEAK_CENTER_UPPER_MIN_GRAY 110
#define BEACON_WEAK_CENTER_UPPER_MAX_GRAY 120
#define BEACON_WEAK_CENTER_UPPER_MAX_MEAN 94
#define BEACON_WEAK_CENTER_FULL_MIN_Y 58
#define BEACON_WEAK_CENTER_RIGHT_MAX_AREA 6
#define BEACON_WEAK_CENTER_MAX_AREA 12
#define BEACON_WEAK_CENTER_MIN_GRAY 90
#define BEACON_WEAK_CENTER_MAX_BG  10
#define BEACON_WEAK_CENTER_MAX_ELONGATION 2.0f
#define BEACON_WEAK_CENTER_DUPLICATE_DISTANCE 8.0f
#define BEACON_SHAPE_MIN_AREA      6
#define BEACON_SHAPE_RATIO_MIN_AREA 12
#define BEACON_SHAPE_MAX_RATIO_NUM 2
#define BEACON_SHAPE_MAX_RATIO_DEN 1
#define BEACON_SHAPE_FILL_MIN_AREA 12
#define BEACON_SHAPE_MIN_FILL_PERCENT 60
#define BEACON_SHAPE_SMALL_MIN_FILL_PERCENT 50
#define BEACON_BAD_SHAPE_MAX_COUNT 8
#define BEACON_BAD_SHAPE_MATCH_PAD 4
#define BEACON_OUTPUT_DIM_MID_MIN_Y 35.0f
#define BEACON_OUTPUT_DIM_MID_SPLIT_Y 45.0f
#define BEACON_OUTPUT_DIM_MID_MAX_Y 58.0f
#define BEACON_OUTPUT_DIM_MID_MAX_AREA 12.0f
#define BEACON_OUTPUT_DIM_MID_MAX_GRAY 99
#define BEACON_OUTPUT_DIM_UPPER_MAX_GRAY 110
#define BEACON_OUTPUT_SIDE_MIN_Y 30.0f
#define BEACON_OUTPUT_SIDE_MARGIN 16.0f
#define BEACON_OUTPUT_SIDE_MAX_AREA 12.0f
#define BEACON_OUTPUT_SIDE_MAX_GRAY 120
#define BEACON_OUTPUT_LOCAL_RADIUS 4
#define BEACON_WEAK_TOP_MIN_X 30
#define BEACON_WEAK_TOP_MAX_X 160
#define BEACON_WEAK_TOP_MIN_Y 1
#define BEACON_WEAK_TOP_MAX_Y 26
#define BEACON_WEAK_TOP_MIN_AREA 5
#define BEACON_WEAK_TOP_MAX_AREA 7
#define BEACON_WEAK_TOP_MIN_GRAY 105
#define BEACON_WEAK_TOP_MAX_GRAY 130
#define BEACON_WEAK_TOP_MAX_BG 10
#define BEACON_WEAK_TOP_MAX_ELONGATION 2.2f
#define BEACON_WEAK_TOP_MIN_FILL_PERCENT 60
#define BEACON_WEAK_TOP_LAMP_MAX_DX 25.0f
#define BEACON_WEAK_TOP_LAMP_MIN_DY 60.0f
#define BEACON_WEAK_TOP_LAMP_MIN_AREA 120
#define BEACON_BRIGHT_TOP_MIN_Y 15.0f
#define BEACON_BRIGHT_TOP_MAX_Y 30.0f
#define BEACON_BRIGHT_TOP_MIN_AREA 7
#define BEACON_BRIGHT_TOP_MAX_AREA 7
#define BEACON_BRIGHT_TOP_MIN_GRAY 138
#define BEACON_BRIGHT_TOP_MAX_GRAY 150
#define BEACON_BRIGHT_TOP_MAX_MEAN 120
#define BEACON_BRIGHT_TOP_MAX_ELONGATION 2.0f
#define BEACON_BRIGHT_TOP_MIN_FILL_PERCENT 60
#define BEACON_TOP_VERTICAL_MIN_AREA 4
#define BEACON_TOP_VERTICAL_MAX_AREA 4
#define BEACON_TOP_VERTICAL_MIN_GRAY 125
#define BEACON_TOP_VERTICAL_MAX_GRAY 135
#define BEACON_TOP_VERTICAL_MIN_ELONGATION 3.0f
#define BEACON_TOP_VERTICAL_MAX_WIDTH 2
#define BEACON_TOP_VERTICAL_LAMP_MIN_AREA 60
#define BEACON_TOP_VERTICAL_LAMP_MAX_DX 6.0f
#define BEACON_TOP_VERTICAL_LAMP_MIN_DY 50.0f
#define BEACON_TOP_VERTICAL_LAMP_MAX_DY 58.0f
#define BEACON_SATURATED_TOP_MIN_Y 20.0f
#define BEACON_SATURATED_TOP_MAX_Y 40.0f
#define BEACON_SATURATED_TOP_MIN_AREA 18
#define BEACON_SATURATED_TOP_MAX_AREA 20
#define BEACON_SATURATED_TOP_MIN_GRAY 240
#define BEACON_SATURATED_TOP_MIN_FILL_PERCENT 75
#define BEACON_SATURATED_TOP_LAMP_MAX_DX 30.0f
#define BEACON_SATURATED_TOP_LAMP_MIN_DX 20.0f
#define BEACON_SATURATED_TOP_LAMP_MIN_DY 35.0f
#define BEACON_SATURATED_TOP_LAMP_MAX_DY 46.0f
'@)
$oldLampCandidate = @'
static unsigned char is_lamp_candidate(const component_t *comp)
{
    return ((comp != 0) &&
            (comp->valid != 0) &&
            (comp->min_y > COMPONENT_TOP_REJECT_Y) &&
            (comp->max_y < COMPONENT_BOTTOM_REJECT_Y) &&
            (comp->cy >= (float)CAR_LAMP_MIN_CENTER_Y) &&
            (comp->area >= CAR_LAMP_MIN_AREA) &&
            (comp->area <= CAR_LAMP_MAX_AREA) &&
            (comp->elongation >= CAR_LAMP_MIN_ELONGATION) &&
            (comp->major >= CAR_LAMP_MIN_LENGTH)) ? 1 : 0;
}
'@
$newLampCandidate = @'
static unsigned char is_lamp_candidate(const component_t *comp)
{
    if((comp == 0) || (comp->valid == 0) ||
       (comp->min_y <= COMPONENT_TOP_REJECT_Y) ||
       (comp->max_y >= COMPONENT_BOTTOM_REJECT_Y) ||
       (comp->area < CAR_LAMP_MIN_AREA) ||
       (comp->area > CAR_LAMP_MAX_AREA) ||
       (comp->elongation < CAR_LAMP_MIN_ELONGATION) ||
       (comp->major < CAR_LAMP_MIN_LENGTH))
    {
        return 0U;
    }
    if(comp->cy >= (float)CAR_LAMP_MIN_CENTER_Y)
    {
        if(comp->minor >= CAR_LAMP_MIN_WIDTH)
        {
            return 1U;
        }

        return ((comp->minor >= CAR_LAMP_NARROW_MIN_WIDTH) &&
                (comp->elongation >=
                CAR_LAMP_NARROW_MIN_ELONGATION)) ? 1U : 0U;
    }

    if((comp->cy >= CAR_LAMP_UPPER_COMPACT_MIN_Y) &&
       (comp->area >= CAR_LAMP_UPPER_COMPACT_MIN_AREA) &&
       (comp->major >= CAR_LAMP_UPPER_COMPACT_MIN_LENGTH) &&
       (comp->minor >= CAR_LAMP_UPPER_COMPACT_MIN_WIDTH) &&
       (comp->elongation >= CAR_LAMP_UPPER_COMPACT_MIN_ELONGATION))
    {
        return 1U;
    }

    return ((comp->area >= CAR_LAMP_UPPER_MIN_AREA) &&
            (comp->major >= CAR_LAMP_UPPER_MIN_LENGTH) &&
            (comp->minor >= CAR_LAMP_UPPER_MIN_WIDTH)) ? 1U : 0U;
}
'@
$body = $body.Replace($oldLampCandidate, $newLampCandidate)

$reflectionFilter = @'
static component_t g_bad_shape_components[BEACON_BAD_SHAPE_MAX_COUNT];
static unsigned char g_bad_shape_count;

static unsigned char component_average_gray(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    int x;
    int y;
    int count = 0;
    unsigned int sum = 0U;

    if((image == 0) || (comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }
    for(y = comp->min_y; y <= comp->max_y; y++)
    {
        for(x = comp->min_x; x <= comp->max_x; x++)
        {
            if(g_binary[y][x] != 0U)
            {
                sum += image[y][x];
                count++;
            }
        }
    }
    return (count > 0) ? (unsigned char)(sum / (unsigned int)count) : 0U;
}

static unsigned char is_obvious_reflection(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    int width;
    int height;
    unsigned char max_gray;

    if((image == 0) || (comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }

    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    max_gray = component_max_gray(image, comp);
    if((comp->cy < BEACON_TOP_GLARE_CENTER_Y) &&
       (max_gray < BEACON_TOP_GLARE_MAX_GRAY) &&
       ((comp->area > BEACON_TOP_WEAK_AREA_MAX) || (height > width)))
    {
        return 1U;
    }
    if((comp->cy < BEACON_TOP_GLARE_CENTER_Y) &&
       (comp->area >= BEACON_TOP_LARGE_GLARE_MIN_AREA) &&
       (max_gray < BEACON_TOP_LARGE_GLARE_MAX_GRAY))
    {
        return 1U;
    }
    if((comp->min_y < BEACON_UPPER_WEAK_BOTTOM_Y) &&
       (max_gray <= BEACON_UPPER_WEAK_MAX_GRAY) &&
       (comp->area > BEACON_TOP_WEAK_AREA_MAX))
    {
        return 1U;
    }
    if((comp->min_y < BEACON_VERTICAL_GLARE_BOTTOM_Y) &&
       (comp->area >= BEACON_VERTICAL_GLARE_MIN_AREA) &&
       (comp->area <= BEACON_VERTICAL_GLARE_MAX_AREA) &&
       (height > width) &&
       (comp->elongation >= BEACON_VERTICAL_GLARE_MIN_ELONGATION) &&
       (max_gray < BEACON_VERTICAL_GLARE_MAX_GRAY))
    {
        return 1U;
    }
    if((comp->cy < BEACON_SATURATED_VERTICAL_MAX_Y) &&
       (comp->area >= BEACON_SATURATED_VERTICAL_MIN_AREA) &&
       (comp->area <= BEACON_SATURATED_VERTICAL_MAX_AREA) &&
       (height > width) &&
       (comp->elongation >=
        BEACON_SATURATED_VERTICAL_MIN_ELONGATION) &&
       (max_gray >= BEACON_SATURATED_VERTICAL_MIN_GRAY))
    {
        return 1U;
    }
    if((comp->min_y < BEACON_UPPER_WEAK_BOTTOM_Y) &&
       (comp->area <= BEACON_VERTICAL_WEAK_MAX_AREA) &&
       (height > width) &&
       (max_gray < BEACON_VERTICAL_WEAK_MAX_GRAY))
    {
        return 1U;
    }
    if((comp->area >= BEACON_LINEAR_MIN_AREA) &&
       (comp->major >= BEACON_LINEAR_MIN_MAJOR) &&
       (comp->minor <= BEACON_LINEAR_MAX_MINOR) &&
       (comp->elongation >= BEACON_LINEAR_MIN_ELONGATION))
    {
        return 1U;
    }
    if((comp->area <= BEACON_TINY_LINE_MAX_AREA) &&
       (comp->minor <= BEACON_TINY_LINE_MAX_MINOR) &&
       (comp->elongation >= BEACON_TINY_LINE_MIN_ELONGATION) &&
       (max_gray < BEACON_TINY_LINE_MAX_GRAY))
    {
        return 1U;
    }

    return 0U;
}

static unsigned char is_beacon_shape_candidate(const component_t *comp)
{
    int width;
    int height;
    int box_area;

    if((comp == 0) || (comp->valid == 0U) ||
       (comp->area < BEACON_SHAPE_MIN_AREA))
    {
        return 1U;
    }
    if((comp->min_x <= 0) || (comp->max_x >= BEACON_IMAGE_W - 1) ||
       (comp->min_y <= 0) || (comp->max_y >= BEACON_IMAGE_H - 1))
    {
        return 1U;
    }

    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if((comp->area >= BEACON_SHAPE_RATIO_MIN_AREA) &&
       ((width * BEACON_SHAPE_MAX_RATIO_DEN >
         height * BEACON_SHAPE_MAX_RATIO_NUM) ||
        (height * BEACON_SHAPE_MAX_RATIO_DEN >
         width * BEACON_SHAPE_MAX_RATIO_NUM)))
    {
        return 0U;
    }

    box_area = width * height;
    if((comp->area >= BEACON_SHAPE_FILL_MIN_AREA) &&
       (comp->area * 100 <
        box_area * BEACON_SHAPE_MIN_FILL_PERCENT))
    {
        return 0U;
    }
    if((comp->area < BEACON_SHAPE_FILL_MIN_AREA) &&
       (comp->area * 100 <
        box_area * BEACON_SHAPE_SMALL_MIN_FILL_PERCENT))
    {
        return 0U;
    }

    return 1U;
}

static void record_bad_shape_component(const component_t *comp)
{
    if((comp == 0) || (comp->valid == 0U) ||
       (g_bad_shape_count >= BEACON_BAD_SHAPE_MAX_COUNT))
    {
        return;
    }

    g_bad_shape_components[g_bad_shape_count++] = *comp;
}

static unsigned char output_local_max_gray(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    float image_x,
    float image_y)
{
    int x;
    int y;
    int cx = (int)(image_x + 0.5f);
    int cy = (int)(image_y + 0.5f);
    int min_x = cx - BEACON_OUTPUT_LOCAL_RADIUS;
    int max_x = cx + BEACON_OUTPUT_LOCAL_RADIUS;
    int min_y = cy - BEACON_OUTPUT_LOCAL_RADIUS;
    int max_y = cy + BEACON_OUTPUT_LOCAL_RADIUS;
    unsigned char max_gray = 0U;

    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    if(max_x >= BEACON_IMAGE_W) max_x = BEACON_IMAGE_W - 1;
    if(max_y >= BEACON_IMAGE_H) max_y = BEACON_IMAGE_H - 1;
    for(y = min_y; y <= max_y; y++)
    {
        for(x = min_x; x <= max_x; x++)
        {
            if(image[y][x] > max_gray)
            {
                max_gray = image[y][x];
            }
        }
    }
    return max_gray;
}

static void suppress_bad_shape_beacons(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    int input;
    int kept = 0;
    int count;

    if(result == 0)
    {
        return;
    }
    count = result->beacon_count;
    if(count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }

    for(input = 0; input < count; input++)
    {
        int bad_index;
        unsigned char suppress = 0U;
        float image_x = result->beacons[input].x +
                        (float)BEACON_IMAGE_W * 0.5f;
        float image_y = result->beacons[input].y +
                        (float)BEACON_IMAGE_H * 0.5f;
        float area = beacon_area(&result->beacons[input]);
        unsigned char local_max = output_local_max_gray(
            image, image_x, image_y);

        if((area <= BEACON_OUTPUT_DIM_MID_MAX_AREA) &&
           (image_y >= BEACON_OUTPUT_DIM_MID_MIN_Y) &&
           (image_y < BEACON_OUTPUT_DIM_MID_SPLIT_Y) &&
           (local_max <= BEACON_OUTPUT_DIM_UPPER_MAX_GRAY))
        {
            suppress = 1U;
        }
        if((area <= BEACON_OUTPUT_DIM_MID_MAX_AREA) &&
           (image_y >= BEACON_OUTPUT_DIM_MID_SPLIT_Y) &&
           (image_y < BEACON_OUTPUT_DIM_MID_MAX_Y) &&
           (local_max <= BEACON_OUTPUT_DIM_MID_MAX_GRAY))
        {
            suppress = 1U;
        }
        if((area <= BEACON_OUTPUT_SIDE_MAX_AREA) &&
           (image_y >= BEACON_OUTPUT_SIDE_MIN_Y) &&
           ((image_x < BEACON_OUTPUT_SIDE_MARGIN) ||
            (image_x >= (float)BEACON_IMAGE_W -
                        BEACON_OUTPUT_SIDE_MARGIN)) &&
           (local_max <= BEACON_OUTPUT_SIDE_MAX_GRAY))
        {
            suppress = 1U;
        }

        for(bad_index = 0;
            (suppress == 0U) && (bad_index < g_bad_shape_count);
            bad_index++)
        {
            const component_t *bad = &g_bad_shape_components[bad_index];

            if((image_x >= (float)(bad->min_x - BEACON_BAD_SHAPE_MATCH_PAD)) &&
               (image_x <= (float)(bad->max_x + BEACON_BAD_SHAPE_MATCH_PAD)) &&
               (image_y >= (float)(bad->min_y - BEACON_BAD_SHAPE_MATCH_PAD)) &&
               (image_y <= (float)(bad->max_y + BEACON_BAD_SHAPE_MATCH_PAD)))
            {
                suppress = 1U;
                break;
            }
        }

        if(suppress == 0U)
        {
            if(kept != input)
            {
                result->beacons[kept] = result->beacons[input];
            }
            kept++;
        }
    }

    for(input = kept; input < count; input++)
    {
        memset(&result->beacons[input], 0, sizeof(result->beacons[input]));
    }
    result->beacon_count = (unsigned char)kept;
}

static unsigned char is_compact_weak_top_beacon(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    int width;
    int height;

    if((image == 0) || (comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    if((comp->cx < (float)BEACON_WEAK_TOP_MIN_X) ||
       (comp->cx >= (float)BEACON_WEAK_TOP_MAX_X) ||
       (comp->cy < (float)BEACON_WEAK_TOP_MIN_Y) ||
       (comp->cy >= (float)BEACON_WEAK_TOP_MAX_Y) ||
       (comp->area < BEACON_WEAK_TOP_MIN_AREA) ||
       (comp->area > BEACON_WEAK_TOP_MAX_AREA) ||
       (comp->elongation > BEACON_WEAK_TOP_MAX_ELONGATION) ||
       (comp->area * 100 < width * height *
        BEACON_WEAK_TOP_MIN_FILL_PERCENT) ||
       (component_max_gray(image, comp) < BEACON_WEAK_TOP_MIN_GRAY) ||
       (component_max_gray(image, comp) > BEACON_WEAK_TOP_MAX_GRAY) ||
       (local_background_average(image, comp) > BEACON_WEAK_TOP_MAX_BG))
    {
        return 0U;
    }
    return 1U;
}

static unsigned char is_compact_weak_top_beacon_near_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    const component_t *lamp)
{
    if((is_compact_weak_top_beacon(image, comp) == 0U) ||
       (lamp == 0) || (lamp->valid == 0U) ||
       (lamp->area < BEACON_WEAK_TOP_LAMP_MIN_AREA) ||
       (fabsf(comp->cx - lamp->cx) > BEACON_WEAK_TOP_LAMP_MAX_DX) ||
       ((lamp->cy - comp->cy) < BEACON_WEAK_TOP_LAMP_MIN_DY))
    {
        return 0U;
    }
    return 1U;
}

static unsigned char is_compact_bright_top_beacon(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp)
{
    int width;
    int height;
    unsigned char max_gray;

    if((image == 0) || (comp == 0) || (comp->valid == 0U))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    max_gray = component_max_gray(image, comp);
    return ((comp->cy >= BEACON_BRIGHT_TOP_MIN_Y) &&
            (comp->cy < BEACON_BRIGHT_TOP_MAX_Y) &&
            (comp->area >= BEACON_BRIGHT_TOP_MIN_AREA) &&
            (comp->area <= BEACON_BRIGHT_TOP_MAX_AREA) &&
            (comp->elongation <= BEACON_BRIGHT_TOP_MAX_ELONGATION) &&
            (comp->area * 100 >= width * height *
             BEACON_BRIGHT_TOP_MIN_FILL_PERCENT) &&
            (max_gray >= BEACON_BRIGHT_TOP_MIN_GRAY) &&
            (max_gray <= BEACON_BRIGHT_TOP_MAX_GRAY) &&
            (component_average_gray(image, comp) <=
             BEACON_BRIGHT_TOP_MAX_MEAN) &&
            (local_background_average(image, comp) <=
             BEACON_WEAK_TOP_MAX_BG)) ? 1U : 0U;
}

static unsigned char is_vertical_top_beacon_near_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    const component_t *lamp)
{
    int width;

    if((image == 0) || (comp == 0) || (comp->valid == 0U) ||
       (lamp == 0) || (lamp->valid == 0U))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    return ((comp->area >= BEACON_TOP_VERTICAL_MIN_AREA) &&
            (comp->area <= BEACON_TOP_VERTICAL_MAX_AREA) &&
            (width <= BEACON_TOP_VERTICAL_MAX_WIDTH) &&
            (comp->elongation >= BEACON_TOP_VERTICAL_MIN_ELONGATION) &&
            (component_max_gray(image, comp) >=
             BEACON_TOP_VERTICAL_MIN_GRAY) &&
            (component_max_gray(image, comp) <=
             BEACON_TOP_VERTICAL_MAX_GRAY) &&
            (lamp->area >= BEACON_TOP_VERTICAL_LAMP_MIN_AREA) &&
            (fabsf(comp->cx - lamp->cx) <=
             BEACON_TOP_VERTICAL_LAMP_MAX_DX) &&
            ((lamp->cy - comp->cy) >=
             BEACON_TOP_VERTICAL_LAMP_MIN_DY) &&
            ((lamp->cy - comp->cy) <=
             BEACON_TOP_VERTICAL_LAMP_MAX_DY)) ? 1U : 0U;
}

static unsigned char is_saturated_top_beacon_near_lamp(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *comp,
    const component_t *lamp)
{
    int width;
    int height;

    if((image == 0) || (comp == 0) || (comp->valid == 0U) ||
       (lamp == 0) || (lamp->valid == 0U))
    {
        return 0U;
    }
    width = comp->max_x - comp->min_x + 1;
    height = comp->max_y - comp->min_y + 1;
    return ((comp->cy >= BEACON_SATURATED_TOP_MIN_Y) &&
            (comp->cy < BEACON_SATURATED_TOP_MAX_Y) &&
            (comp->area >= BEACON_SATURATED_TOP_MIN_AREA) &&
            (comp->area <= BEACON_SATURATED_TOP_MAX_AREA) &&
            (component_max_gray(image, comp) >=
             BEACON_SATURATED_TOP_MIN_GRAY) &&
            (comp->area * 100 >= width * height *
             BEACON_SATURATED_TOP_MIN_FILL_PERCENT) &&
            (fabsf(comp->cx - lamp->cx) >=
             BEACON_SATURATED_TOP_LAMP_MIN_DX) &&
            (fabsf(comp->cx - lamp->cx) <=
             BEACON_SATURATED_TOP_LAMP_MAX_DX) &&
            ((lamp->cy - comp->cy) >=
             BEACON_SATURATED_TOP_LAMP_MIN_DY) &&
            ((lamp->cy - comp->cy) <=
             BEACON_SATURATED_TOP_LAMP_MAX_DY)) ? 1U : 0U;
}

'@
$beaconInsertMarker = "static void insert_beacon_by_area(`n"
$beaconInsertIndex = $body.IndexOf($beaconInsertMarker)
if ($beaconInsertIndex -lt 0) {
    throw "insert_beacon_by_area was not found."
}
$body = $body.Insert($beaconInsertIndex, $reflectionFilter)
$validMarker = "    if((comp == 0) || (comp->valid == 0))`n    {`n        return;`n    }`n"
$validIndex = $body.IndexOf($validMarker, $beaconInsertIndex + $reflectionFilter.Length)
if ($validIndex -lt 0) {
    throw "Beacon validation block was not found."
}
$reflectionCall = "    if((is_obvious_reflection(image, comp) != 0U) &&`n       (is_vertical_top_beacon_near_lamp(image, comp, lamp) == 0U) &&`n       (is_saturated_top_beacon_near_lamp(image, comp, lamp) == 0U))`n    {`n        return;`n    }`n"
$shapeCall = "    if((is_beacon_shape_candidate(comp) == 0U) &&`n       (is_saturated_top_beacon_near_lamp(image, comp, lamp) == 0U))`n    {`n        record_bad_shape_component(comp);`n    }`n"
$body = $body.Insert($validIndex + $validMarker.Length, $shapeCall)
$body = $body.Insert($validIndex + $validMarker.Length + $shapeCall.Length, $reflectionCall)
$oldTopWeakReject = @'
    if(((((comp->min_y <= BEACON_TOP_WEAK_Y) &&
           (comp->area <= BEACON_TOP_WEAK_AREA_MAX)) ||
          ((comp->min_y < BEACON_EDGE_TOP_Y) &&
           (comp->area >= BEACON_TOP_DIFFUSE_AREA_MIN) &&
           (comp->area <= BEACON_TOP_EDGE_MAX_AREA) &&
           (matches_confirmed_beacon_track(comp) == 0U)) ||
          ((is_side_edge != 0U) &&
           (comp->area >= BEACON_SIDE_DIFFUSE_AREA_MIN) &&
           (comp->area <= BEACON_EDGE_MAX_AREA))) &&
         (component_max_gray(image, comp) < BEACON_TOP_WEAK_GRAY_MIN)))
'@
$newTopWeakReject = @'
    if((is_compact_weak_top_beacon_near_lamp(
            image, comp, lamp) == 0U) &&
       (is_compact_bright_top_beacon(image, comp) == 0U) &&
       (is_vertical_top_beacon_near_lamp(image, comp, lamp) == 0U) &&
       ((((comp->min_y <= BEACON_TOP_WEAK_Y) &&
           (comp->area <= BEACON_TOP_WEAK_AREA_MAX)) ||
          ((comp->min_y < BEACON_EDGE_TOP_Y) &&
           (comp->area >= BEACON_TOP_DIFFUSE_AREA_MIN) &&
           (comp->area <= BEACON_TOP_EDGE_MAX_AREA) &&
           (matches_confirmed_beacon_track(comp) == 0U)) ||
          ((is_side_edge != 0U) &&
           (comp->area >= BEACON_SIDE_DIFFUSE_AREA_MIN) &&
           (comp->area <= BEACON_EDGE_MAX_AREA))) &&
         (component_max_gray(image, comp) < BEACON_TOP_WEAK_GRAY_MIN)))
'@
$body = $body.Replace($oldTopWeakReject, $newTopWeakReject)
$weakCenterHelpers = @'
static unsigned char is_near_existing_beacon(
    const component_t *comp,
    const beacon_result_t *result)
{
    int i;
    int count = result->beacon_count;
    float x = comp->cx - (float)BEACON_IMAGE_W * 0.5f;
    float y = comp->cy - (float)BEACON_IMAGE_H * 0.5f;
    float max_d2 = BEACON_WEAK_CENTER_DUPLICATE_DISTANCE *
                   BEACON_WEAK_CENTER_DUPLICATE_DISTANCE;

    if(count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }
    for(i = 0; i < count; i++)
    {
        float dx;
        float dy;

        if(result->beacons[i].valid == 0U)
        {
            continue;
        }
        dx = result->beacons[i].x - x;
        dy = result->beacons[i].y - y;
        if((dx * dx + dy * dy) <= max_d2)
        {
            return 1U;
        }
    }

    return 0U;
}

static void find_weak_center_beacons(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    const component_t *lamp,
    const component_t *temporal_lamp,
    beacon_result_t *result)
{
    unsigned char x;
    unsigned char y;

    threshold_image(image, BEACON_WEAK_CENTER_THRESHOLD);
    erase_lamp_from_binary(lamp);
    erase_temporal_lamp_from_binary(temporal_lamp);

    for(y = BEACON_WEAK_CENTER_MIN_Y; y < BEACON_WEAK_CENTER_MAX_Y; y++)
    {
        for(x = BEACON_WEAK_CENTER_MIN_X; x < BEACON_WEAK_CENTER_MAX_X; x++)
        {
            component_t comp;

            if((g_binary[y][x] == 0U) || (g_visit[y][x] != 0U))
            {
                continue;
            }

            comp = grow_component(x, y);
            if((comp.valid == 0U) ||
               (comp.cy < (float)BEACON_WEAK_CENTER_MIN_Y) ||
               (comp.cy >= (float)BEACON_WEAK_CENTER_MAX_Y) ||
               (comp.area < BEACON_WEAK_CENTER_MIN_AREA) ||
               ((comp.cy < (float)BEACON_WEAK_CENTER_FULL_MIN_Y) &&
                (comp.cx <
                 (float)BEACON_WEAK_CENTER_NEAR_RIGHT_MAX_X) &&
                ((comp.area < BEACON_WEAK_CENTER_UPPER_MIN_AREA) ||
                 (comp.area > BEACON_WEAK_CENTER_UPPER_MAX_AREA) ||
                 (component_max_gray(image, &comp) <
                  BEACON_WEAK_CENTER_UPPER_MIN_GRAY) ||
                 (component_max_gray(image, &comp) >
                  BEACON_WEAK_CENTER_UPPER_MAX_GRAY) ||
                 (component_average_gray(image, &comp) >
                  BEACON_WEAK_CENTER_UPPER_MAX_MEAN))) ||
               (comp.area > BEACON_WEAK_CENTER_MAX_AREA) ||
               (comp.elongation > BEACON_WEAK_CENTER_MAX_ELONGATION) ||
               (component_max_gray(image, &comp) < BEACON_WEAK_CENTER_MIN_GRAY) ||
               ((comp.cx >= (float)BEACON_WEAK_CENTER_BASE_MAX_X) &&
                (((comp.cx <
                   (float)BEACON_WEAK_CENTER_NEAR_RIGHT_MAX_X) &&
                  ((comp.area > BEACON_WEAK_CENTER_RIGHT_MAX_AREA) ||
                   ((comp.max_y - comp.min_y) <=
                    (comp.max_x - comp.min_x)) ||
                   (component_max_gray(image, &comp) <
                    BEACON_WEAK_CENTER_RIGHT_MIN_GRAY))) ||
                 ((comp.cx >=
                   (float)BEACON_WEAK_CENTER_NEAR_RIGHT_MAX_X) &&
                  ((comp.cx <
                    (float)BEACON_WEAK_CENTER_FAR_RIGHT_MIN_X) ||
                   (comp.cy <
                    (float)BEACON_WEAK_CENTER_FAR_RIGHT_MIN_Y) ||
                   (comp.area <
                    BEACON_WEAK_CENTER_FAR_RIGHT_MIN_AREA) ||
                   (comp.area >
                    BEACON_WEAK_CENTER_FAR_RIGHT_MAX_AREA) ||
                   ((comp.max_y - comp.min_y) !=
                    (comp.max_x - comp.min_x)) ||
                   (comp.elongation >
                    BEACON_WEAK_CENTER_FAR_RIGHT_MAX_ELONGATION) ||
                   (component_max_gray(image, &comp) <
                    BEACON_WEAK_CENTER_FAR_RIGHT_MIN_GRAY))))) ||
               (local_background_average(image, &comp) > BEACON_WEAK_CENTER_MAX_BG) ||
               (is_near_existing_beacon(&comp, result) != 0U))
            {
                continue;
            }

            insert_beacon_by_area(image, &comp, lamp, temporal_lamp, result);
        }
    }
}

'@
$findBeaconsMarker = "static void find_beacons(`n"
$findBeaconsIndex = $body.IndexOf($findBeaconsMarker)
if ($findBeaconsIndex -lt 0) {
    throw "find_beacons was not found."
}
$body = $body.Insert($findBeaconsIndex, $weakCenterHelpers)
$findBeaconsTail = "            insert_beacon_by_area(image, &comp, lamp, temporal_lamp, result);`n        }`n    }`n}`n`nstatic float square_distance"
$findBeaconsReplacement = "            insert_beacon_by_area(image, &comp, lamp, temporal_lamp, result);`n        }`n    }`n`n    find_weak_center_beacons(image, lamp, temporal_lamp, result);`n}`n`nstatic float square_distance"
$body = $body.Replace($findBeaconsTail, $findBeaconsReplacement)
$body = $body.Replace("    result->beacon_count = 0;`n    threshold_beacon_image(image);", "    result->beacon_count = 0;`n    g_bad_shape_count = 0U;`n    threshold_beacon_image(image);")
$body = $body.Replace("    update_temporal_result(result);`n    convert_result_to_analyzer_coordinates(result);", "    suppress_bad_shape_beacons(image, result);`n    update_temporal_result(result);`n    convert_result_to_analyzer_coordinates(result);")
$body = $body.Replace("        if((g_b0_track.confirmed != 0U) &&`n           (g_b0_track.misses < BEACON_MAX_MISSES))", "        if((g_b0_track.confirmed != 0U) &&`n           (g_b0_track.area > BEACON_TINY_TRACK_MAX_AREA) &&`n           (g_b0_track.misses < BEACON_MAX_MISSES))")
$oldUnmatchedBeacon = @'
    if(selected < 0)
    {
        if((g_b0_track.confirmed != 0U) &&
           (g_b0_track.area > BEACON_TINY_TRACK_MAX_AREA) &&
           (g_b0_track.misses < BEACON_MAX_MISSES))
        {
            g_b0_track.x += g_b0_track.vx;
            g_b0_track.y += g_b0_track.vy;
            g_b0_track.misses++;
            write_temporal_beacon(&g_b0_track, result, -1);
            return;
        }
        reset_track(&g_b0_track);
        return;
    }
'@
$newUnmatchedBeacon = @'
    if(selected < 0)
    {
        start_pending_beacon(&g_b0_track, &result->beacons[0]);
        return;
    }
'@
$body = $body.Replace($oldUnmatchedBeacon, $newUnmatchedBeacon)
$oldTemporalBeacon = @'
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
$newTemporalBeacon = @'
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
$body = $body.Replace($oldTemporalBeacon, $newTemporalBeacon)
$oldUpdateCarTrack = @'
static void update_car_track(temporal_track_t *track, const beacon_rect_t *car)
{
    float old_x = track->x;
    float old_y = track->y;
    float predict_x = track->x + track->vx;
    float predict_y = track->y + track->vy;

    track->vx = (1.0f - FILTER_VEL_ALPHA) * track->vx +
                FILTER_VEL_ALPHA * (car->cx - old_x);
    track->vy = (1.0f - FILTER_VEL_ALPHA) * track->vy +
                FILTER_VEL_ALPHA * (car->cy - old_y);
    track->x = FILTER_POS_ALPHA * car->cx +
               (1.0f - FILTER_POS_ALPHA) * predict_x;
    track->y = FILTER_POS_ALPHA * car->cy +
               (1.0f - FILTER_POS_ALPHA) * predict_y;
    track->width = FILTER_POS_ALPHA * car->width +
                   (1.0f - FILTER_POS_ALPHA) * track->width;
    track->length = FILTER_POS_ALPHA * car->length +
                    (1.0f - FILTER_POS_ALPHA) * track->length;
    track->angle = car->angle;
    track->misses = 0U;
}
'@
$newUpdateCarTrack = @'
static void update_car_track(temporal_track_t *track, const beacon_rect_t *car)
{
    float old_x = track->x;
    float old_y = track->y;
    float predict_x = track->x + track->vx;
    float predict_y = track->y + track->vy;

    track->vx = (1.0f - FILTER_VEL_ALPHA) * track->vx +
                FILTER_VEL_ALPHA * (car->cx - old_x);
    track->vy = (1.0f - FILTER_VEL_ALPHA) * track->vy +
                FILTER_VEL_ALPHA * (car->cy - old_y);
    track->x = FILTER_POS_ALPHA * car->cx +
               (1.0f - FILTER_POS_ALPHA) * predict_x;
    track->y = FILTER_POS_ALPHA * car->cy +
               (1.0f - FILTER_POS_ALPHA) * predict_y;
    track->width = FILTER_POS_ALPHA * car->width +
                   (1.0f - FILTER_POS_ALPHA) * track->width;
    track->length = FILTER_POS_ALPHA * car->length +
                    (1.0f - FILTER_POS_ALPHA) * track->length;
    track->angle = car->angle;
    track->misses = 0U;
}
'@
$body = $body.Replace($oldUpdateCarTrack, $newUpdateCarTrack)
$oldTemporalCar = @'
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
'@
$newTemporalCar = @'
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
$body = $body.Replace($oldTemporalCar, $newTemporalCar)
$body = $body.Replace("static void update_temporal_car(beacon_result_t *result)`n{`n    beacon_rect_t *measurement = 0;", "static void update_temporal_car(beacon_result_t *result)`n{`n    beacon_rect_t *measurement = 0;`n`n    result->temporal_car_lamp_count = 0U;")
$body = $body.Replace("        start_pending_car(&g_car_track, measurement);`n        return;", "        start_pending_car(&g_car_track, measurement);`n        write_current_car_as_temporal(measurement, result);`n        return;")
$body = $body.Replace("    if(g_car_track.confirmed != 0U)`n    {`n        write_temporal_car(&g_car_track, result);`n    }", "    if(g_car_track.confirmed != 0U)`n    {`n        write_current_car_as_temporal(measurement, result);`n    }")

$adapter = @'
static void sync_legacy_beacons(beacon_result_t *result)
{
    int i;
    int count = result->beacon_count;

    if(count > BEACON_MAX_BEACON_COUNT)
    {
        count = BEACON_MAX_BEACON_COUNT;
    }
    if(count > BEACON_MAX_CIRCLE_COUNT)
    {
        count = BEACON_MAX_CIRCLE_COUNT;
    }

    memset(result->circles, 0, sizeof(result->circles));
    for(i = 0; i < count; i++)
    {
        result->circles[i] = result->beacons[i];
    }
    result->count = (unsigned char)count;
}

static void convert_result_to_analyzer_coordinates(beacon_result_t *result)
{
    int i;
    int beacon_count = result->beacon_count;
    int temporal_beacon_count = result->temporal_beacon_count;
    int car_lamp_count = result->car_lamp_count;
    int temporal_car_lamp_count = result->temporal_car_lamp_count;

    if(beacon_count > BEACON_MAX_BEACON_COUNT)
    {
        beacon_count = BEACON_MAX_BEACON_COUNT;
    }
    for(i = 0; i < beacon_count; i++)
    {
        result->beacons[i].x = -result->beacons[i].x;
    }

    if(temporal_beacon_count > BEACON_MAX_BEACON_COUNT)
    {
        temporal_beacon_count = BEACON_MAX_BEACON_COUNT;
    }
    for(i = 0; i < temporal_beacon_count; i++)
    {
        result->temporal_beacons[i].x =
            -result->temporal_beacons[i].x;
    }

    if(car_lamp_count > BEACON_MAX_CAR_LAMP_COUNT)
    {
        car_lamp_count = BEACON_MAX_CAR_LAMP_COUNT;
    }
    for(i = 0; i < car_lamp_count; i++)
    {
        result->car_lamps[i].cx = -result->car_lamps[i].cx;
    }

    if(temporal_car_lamp_count > BEACON_MAX_CAR_LAMP_COUNT)
    {
        temporal_car_lamp_count = BEACON_MAX_CAR_LAMP_COUNT;
    }
    for(i = 0; i < temporal_car_lamp_count; i++)
    {
        result->temporal_car_lamps[i].cx =
            -result->temporal_car_lamps[i].cx;
    }
}

'@
$processMarker = "void beacon_image_process(`n"
$processIndex = $body.IndexOf($processMarker)
if ($processIndex -lt 0) {
    throw "beacon_image_process was not found."
}
$body = $body.Insert($processIndex, $adapter)
$body = $body.Replace("    update_temporal_result(result);`n}", "    suppress_bad_shape_beacons(image, result);`n    update_temporal_result(result);`n    convert_result_to_analyzer_coordinates(result);`n    sync_legacy_beacons(result);`n}")

$prefix = @'
#include "beacon_image.h"
#include "beacon_image_config.h"

#include <math.h>
#include <string.h>

/* Preserve board-side output limits while keeping the analyzer ABI fixed. */
#undef BEACON_MAX_BEACON_COUNT
#define BEACON_MAX_BEACON_COUNT 3
#undef BEACON_MAX_CAR_LAMP_COUNT
#define BEACON_MAX_CAR_LAMP_COUNT 1

static float beacon_area(const beacon_circle_t *beacon);

'@

$suffix = @'

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    (void)image;
    return (unsigned char)g_beacon_binary_threshold;
}

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    if(binary == 0)
    {
        return;
    }
    if(image == 0)
    {
        memset(binary, 0, BEACON_IMAGE_W * BEACON_IMAGE_H);
        return;
    }

    threshold_car_lamp_image(image);
    memcpy(binary, g_binary, sizeof(g_binary));
}
'@

$output = $prefix + $body + $suffix
[System.IO.Directory]::CreateDirectory($targetDir) | Out-Null
$utf8 = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($targetPath, $output, $utf8)
