#include "image_horizon.h"

#include <math.h>
#include <string.h>

#define IMAGE_HORIZON_BOARD_COUNT          (2U)
#define IMAGE_HORIZON_TABLE_INTERVALS      (256U)
#define IMAGE_HORIZON_TABLE_SIZE           (IMAGE_HORIZON_TABLE_INTERVALS + 1U)
#define IMAGE_HORIZON_DEG_TO_RAD           (0.01745329251994329577f)
#define IMAGE_HORIZON_CENTER_X             (89.05679508319916f)
#define IMAGE_HORIZON_CENTER_Y             (69.5f)
#define IMAGE_HORIZON_SCALE                (93.5f)
#define IMAGE_HORIZON_THETA_K1             (2.136653726640037f)
#define IMAGE_HORIZON_THETA_K3             (-1.210544244169379f)
#define IMAGE_HORIZON_THETA_K5             (0.6350455170908825f)
#define IMAGE_HORIZON_ROLL_MIN_DEG         (-18.5663319f)
#define IMAGE_HORIZON_ROLL_MAX_DEG         (21.1464043f)
#define IMAGE_HORIZON_PITCH_MIN_DEG        (-10.6507406f)
#define IMAGE_HORIZON_PITCH_MAX_DEG        (36.6099815f)
#define IMAGE_HORIZON_HEIGHT_MIN_MM         (656.097656f)
#define IMAGE_HORIZON_HEIGHT_MAX_MM         (1261.45825f)
#define IMAGE_HORIZON_HEIGHT_ZERO_MM        (1119.0491510546249f)
#define IMAGE_HORIZON_DISTANCE_MM           (7047.706875656069f)
#define IMAGE_HORIZON_ZERO_EPSILON         (1.0e-7f)

static const float s_attitude_to_camera_normal[3][3] =
{
    {0.00346433916224295f, -0.50925728267113f, -0.0019043835510911922f},
    {0.3206116243770298f, 0.043452357107652585f, -0.2858130784381615f},
    {-0.5320478058095025f, 0.03541420100980411f, -0.5195605495364598f}
};

float g_image_horizon_y[IMAGE_HORIZON_WIDTH];
uint8 g_image_horizon_column_valid[IMAGE_HORIZON_WIDTH];
uint8 g_image_horizon_valid;
uint8 g_image_horizon_extrapolated;

static float s_normalized_x[IMAGE_HORIZON_WIDTH];
static float s_normalized_y[IMAGE_HORIZON_HEIGHT];
static float s_radial_factor[IMAGE_HORIZON_TABLE_SIZE];
static float s_ray_z[IMAGE_HORIZON_TABLE_SIZE];
static float s_radius2_to_table;
static uint8 s_initialized;

static float image_horizon_max(float first, float second)
{
    return (first > second) ? first : second;
}
static float image_horizon_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int image_horizon_round(float value)
{
    return (value >= 0.0f) ? (int)(value + 0.5f) : (int)(value - 0.5f);
}

static float image_horizon_value(uint16 x, uint16 y, const float normal[3])
{
    float radius2 = s_normalized_x[x] * s_normalized_x[x] +
                    s_normalized_y[y] * s_normalized_y[y];
    float table_position = radius2 * s_radius2_to_table;
    uint16 index;
    float fraction;
    float radial;
    float ray_z;

    if(table_position <= 0.0f)
    {
        index = 0U;
        fraction = 0.0f;
    }
    else if(table_position >= (float)IMAGE_HORIZON_TABLE_INTERVALS)
    {
        index = IMAGE_HORIZON_TABLE_INTERVALS - 1U;
        fraction = 1.0f;
    }
    else
    {
        index = (uint16)table_position;
        fraction = table_position - (float)index;
    }

    radial = s_radial_factor[index] +
             fraction * (s_radial_factor[index + 1U] - s_radial_factor[index]);
    ray_z = s_ray_z[index] + fraction * (s_ray_z[index + 1U] - s_ray_z[index]);
    return s_normalized_x[x] * radial * normal[0] +
           s_normalized_y[y] * radial * normal[1] +
           ray_z * normal[2];
}

static uint8 image_horizon_root_in_interval(uint16 x,
                                            uint16 y,
                                            const float normal[3],
                                            float *root)
{
    float first = image_horizon_value(x, y, normal);
    float second = image_horizon_value(x, (uint16)(y + 1U), normal);
    float magnitude;

    if(image_horizon_abs(first) <= IMAGE_HORIZON_ZERO_EPSILON)
    {
        *root = (float)y;
        return 1U;
    }
    if(image_horizon_abs(second) <= IMAGE_HORIZON_ZERO_EPSILON)
    {
        *root = (float)(y + 1U);
        return 1U;
    }
    if((first > 0.0f) == (second > 0.0f))
    {
        return 0U;
    }

    magnitude = image_horizon_abs(first) + image_horizon_abs(second);
    *root = (float)y + image_horizon_abs(first) / magnitude;
    return 1U;
}

static uint8 image_horizon_find_root(uint16 x,
                                     float reference_y,
                                     const float normal[3],
                                     float *root)
{
    int center = image_horizon_round(reference_y);
    int distance;

    if(center < 0)
    {
        center = 0;
    }
    else if(center >= (int)IMAGE_HORIZON_HEIGHT)
    {
        center = (int)IMAGE_HORIZON_HEIGHT - 1;
    }

    for(distance = 0; distance < (int)IMAGE_HORIZON_HEIGHT; distance++)
    {
        int lower = center - distance - 1;
        int upper = center + distance;
        float lower_root = 0.0f;
        float upper_root = 0.0f;
        uint8 lower_valid = 0U;
        uint8 upper_valid = 0U;

        if(lower >= 0)
        {
            lower_valid = image_horizon_root_in_interval(x, (uint16)lower, normal, &lower_root);
        }
        if(upper < ((int)IMAGE_HORIZON_HEIGHT - 1))
        {
            upper_valid = image_horizon_root_in_interval(x, (uint16)upper, normal, &upper_root);
        }
        if((lower_valid != 0U) || (upper_valid != 0U))
        {
            if((lower_valid != 0U) && (upper_valid != 0U))
            {
                *root = (image_horizon_abs(lower_root - reference_y) <=
                         image_horizon_abs(upper_root - reference_y)) ?
                        lower_root : upper_root;
            }
            else
            {
                *root = (lower_valid != 0U) ? lower_root : upper_root;
            }
            return 1U;
        }
    }
    return 0U;
}

static uint8 image_horizon_find_seed(const float normal[3], uint16 *seed_x, float *seed_y)
{
    int center_x = image_horizon_round(IMAGE_HORIZON_CENTER_X);
    int offset;

    if(center_x < 0)
    {
        center_x = 0;
    }
    else if(center_x >= (int)IMAGE_HORIZON_WIDTH)
    {
        center_x = (int)IMAGE_HORIZON_WIDTH - 1;
    }

    for(offset = 0; offset < (int)IMAGE_HORIZON_WIDTH; offset++)
    {
        int left = center_x - offset;
        int right = center_x + offset;

        if((left >= 0) &&
           (image_horizon_find_root((uint16)left,
                                    IMAGE_HORIZON_CENTER_Y,
                                    normal,
                                    seed_y) != 0U))
        {
            *seed_x = (uint16)left;
            return 1U;
        }
        if((offset != 0) && (right < (int)IMAGE_HORIZON_WIDTH) &&
           (image_horizon_find_root((uint16)right,
                                    IMAGE_HORIZON_CENTER_Y,
                                    normal,
                                    seed_y) != 0U))
        {
            *seed_x = (uint16)right;
            return 1U;
        }
    }
    return 0U;
}

void image_horizon_init(void)
{
    uint16 index;
    float max_x;
    float max_y;
    float radius2_max;

    memset(g_image_horizon_y, 0, sizeof(g_image_horizon_y));
    memset(g_image_horizon_column_valid, 0, sizeof(g_image_horizon_column_valid));
    g_image_horizon_valid = 0U;
    g_image_horizon_extrapolated = 0U;

    for(index = 0U; index < IMAGE_HORIZON_WIDTH; index++)
    {
        s_normalized_x[index] = ((float)index - IMAGE_HORIZON_CENTER_X) /
                                IMAGE_HORIZON_SCALE;
    }
    for(index = 0U; index < IMAGE_HORIZON_HEIGHT; index++)
    {
        s_normalized_y[index] = ((float)index - IMAGE_HORIZON_CENTER_Y) /
                                IMAGE_HORIZON_SCALE;
    }

    max_x = image_horizon_max(image_horizon_abs(s_normalized_x[0]),
                              image_horizon_abs(s_normalized_x[IMAGE_HORIZON_WIDTH - 1U]));
    max_y = image_horizon_max(image_horizon_abs(s_normalized_y[0]),
                              image_horizon_abs(s_normalized_y[IMAGE_HORIZON_HEIGHT - 1U]));
    radius2_max = max_x * max_x + max_y * max_y;
    s_radius2_to_table = (float)IMAGE_HORIZON_TABLE_INTERVALS / radius2_max;

    for(index = 0U; index < IMAGE_HORIZON_TABLE_SIZE; index++)
    {
        float radius2 = radius2_max * (float)index /
                        (float)IMAGE_HORIZON_TABLE_INTERVALS;
        float radius = sqrtf(radius2);
        float theta = radius *
                      (IMAGE_HORIZON_THETA_K1 +
                       radius2 * (IMAGE_HORIZON_THETA_K3 +
                                  radius2 * IMAGE_HORIZON_THETA_K5));

        s_radial_factor[index] = (radius > 1.0e-8f) ?
                                 (sinf(theta) / radius) : IMAGE_HORIZON_THETA_K1;
        s_ray_z[index] = cosf(theta);
    }
    s_initialized = 1U;
}

void image_horizon_update(uint8 board_id,
                          float roll_deg,
                          float pitch_deg,
                          float height_mm,
                          uint8 attitude_valid,
                          uint8 height_valid)
{
    float roll;
    float pitch;
    float roll_rad;
    float pitch_rad;
    float sin_roll;
    float cos_roll;
    float sin_pitch;
    float cos_pitch;
    float height_compensation;
    float attitude_vector[3];
    float normal[3];
    float seed_y;
    float previous_y;
    uint16 seed_x;
    uint16 valid_count = 0U;
    int x;
    uint8 row;
    uint8 axis;

    if(s_initialized == 0U)
    {
        image_horizon_init();
    }
    if((attitude_valid == 0U) || (height_valid == 0U) ||
       (board_id >= IMAGE_HORIZON_BOARD_COUNT) ||
       (roll_deg != roll_deg) || (pitch_deg != pitch_deg) ||
       (height_mm != height_mm))
    {
        g_image_horizon_valid = 0U;
        g_image_horizon_extrapolated = 0U;
        return;
    }

    roll = (board_id == 0U) ? roll_deg : -roll_deg;
    pitch = (board_id == 0U) ? pitch_deg : -pitch_deg;
    g_image_horizon_extrapolated =
        ((roll < IMAGE_HORIZON_ROLL_MIN_DEG) ||
         (roll > IMAGE_HORIZON_ROLL_MAX_DEG) ||
         (pitch < IMAGE_HORIZON_PITCH_MIN_DEG) ||
         (pitch > IMAGE_HORIZON_PITCH_MAX_DEG) ||
         (height_mm < IMAGE_HORIZON_HEIGHT_MIN_MM) ||
         (height_mm > IMAGE_HORIZON_HEIGHT_MAX_MM)) ? 1U : 0U;

    roll_rad = roll * IMAGE_HORIZON_DEG_TO_RAD;
    pitch_rad = pitch * IMAGE_HORIZON_DEG_TO_RAD;
    sin_roll = sinf(roll_rad);
    cos_roll = cosf(roll_rad);
    sin_pitch = sinf(pitch_rad);
    cos_pitch = cosf(pitch_rad);
    height_compensation = (IMAGE_HORIZON_HEIGHT_ZERO_MM - height_mm) /
                          IMAGE_HORIZON_DISTANCE_MM;
    attitude_vector[0] = -sin_pitch + height_compensation * cos_pitch;
    attitude_vector[1] = sin_roll * cos_pitch +
                         height_compensation * sin_roll * sin_pitch;
    attitude_vector[2] = cos_roll * cos_pitch +
                         height_compensation * cos_roll * sin_pitch;
    for(row = 0U; row < 3U; row++)
    {
        normal[row] = 0.0f;
        for(axis = 0U; axis < 3U; axis++)
        {
            normal[row] += s_attitude_to_camera_normal[row][axis] *
                           attitude_vector[axis];
        }
    }

    memset(g_image_horizon_column_valid, 0, sizeof(g_image_horizon_column_valid));
    if(image_horizon_find_seed(normal, &seed_x, &seed_y) == 0U)
    {
        g_image_horizon_valid = 0U;
        return;
    }

    g_image_horizon_y[seed_x] = seed_y;
    g_image_horizon_column_valid[seed_x] = 1U;
    valid_count = 1U;
    previous_y = seed_y;
    for(x = (int)seed_x - 1; x >= 0; x--)
    {
        float root;
        if(image_horizon_find_root((uint16)x, previous_y, normal, &root) == 0U)
        {
            break;
        }
        g_image_horizon_y[x] = root;
        g_image_horizon_column_valid[x] = 1U;
        previous_y = root;
        valid_count++;
    }

    previous_y = seed_y;
    for(x = (int)seed_x + 1; x < (int)IMAGE_HORIZON_WIDTH; x++)
    {
        float root;
        if(image_horizon_find_root((uint16)x, previous_y, normal, &root) == 0U)
        {
            break;
        }
        g_image_horizon_y[x] = root;
        g_image_horizon_column_valid[x] = 1U;
        previous_y = root;
        valid_count++;
    }
    g_image_horizon_valid = (valid_count >= 2U) ? 1U : 0U;
}

uint8 image_horizon_get_y(uint16 x, float *y)
{
    if((y == 0) || (g_image_horizon_valid == 0U) ||
       (x >= IMAGE_HORIZON_WIDTH) || (g_image_horizon_column_valid[x] == 0U))
    {
        return 0U;
    }
    *y = g_image_horizon_y[x];
    return 1U;
}

void image_horizon_draw_overlay(uint8 *image, uint8 gray)
{
    uint16 x;
    int previous_y = 0;
    uint8 previous_valid = 0U;

    if((image == 0) || (g_image_horizon_valid == 0U))
    {
        return;
    }

    for(x = 0U; x < IMAGE_HORIZON_WIDTH; x++)
    {
        int y;
        int first;
        int last;
        int draw_y;

        if(g_image_horizon_column_valid[x] == 0U)
        {
            previous_valid = 0U;
            continue;
        }
        y = image_horizon_round(g_image_horizon_y[x]);
        if(y < 0)
        {
            y = 0;
        }
        else if(y >= (int)IMAGE_HORIZON_HEIGHT)
        {
            y = (int)IMAGE_HORIZON_HEIGHT - 1;
        }
        first = ((previous_valid != 0U) && (previous_y < y)) ? previous_y : y;
        last = ((previous_valid != 0U) && (previous_y > y)) ? previous_y : y;
        for(draw_y = first; draw_y <= last; draw_y++)
        {
            uint32 pixel = (uint32)draw_y * IMAGE_HORIZON_WIDTH + x;
            if(image[pixel] < gray)
            {
                image[pixel] = gray;
            }
        }
        previous_y = y;
        previous_valid = 1U;
    }
}
