#include "beacon_image.h"

#include <math.h>
#include <string.h>

#define IMAGE_SEED_MIN_PEAK 90U
#define IMAGE_SEED_MIN_CONTRAST 45
#define IMAGE_TOP_EXCLUSION_Y 20U
#define IMAGE_REGION_FLOOR 18
#define IMAGE_REGION_DROP 120
#define IMAGE_MAX_COMPONENT_AREA 5000U
#define IMAGE_MAX_COMPONENT_SIDE 24
#define IMAGE_TRACK_KEEP_FRAMES 8
#define IMAGE_TRACK_MATCH_DISTANCE 18.0f
#define IMAGE_TRACK_MERGE_DISTANCE 5.0f
#define IMAGE_PI 3.1415926f

static beacon_image_context_t g_default_context;

void beacon_image_context_init(beacon_image_context_t *context)
{
    if (context == 0)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
}

void beacon_image_init(void)
{
    beacon_image_context_init(&g_default_context);
}

static void clear_result(beacon_result_t *result)
{
    if (result == 0)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
}

static int clamp_int(int value, int low, int high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

static int abs_int(int value)
{
    return (value < 0) ? -value : value;
}

static void begin_visit_pass(beacon_image_context_t *context)
{
    context->current_visit_stamp++;
    if (context->current_visit_stamp == 0U)
    {
        memset(context->visit_stamp, 0, sizeof(context->visit_stamp));
        context->current_visit_stamp = 1U;
    }
}

static unsigned char is_visited(const beacon_image_context_t *context, unsigned char x, unsigned char y)
{
    return (context->visit_stamp[y][x] == context->current_visit_stamp) ? 1U : 0U;
}

static void mark_visited(beacon_image_context_t *context, unsigned char x, unsigned char y)
{
    context->visit_stamp[y][x] = context->current_visit_stamp;
}

static unsigned char local_ring_mean(const beacon_image_context_t *context,
                                     unsigned char center_x,
                                     unsigned char center_y)
{
    int x;
    int y;
    int x0 = (int)center_x - 5;
    int y0 = (int)center_y - 5;
    int x1 = (int)center_x + 5;
    int y1 = (int)center_y + 5;
    unsigned int sum = 0U;
    unsigned int count = 0U;

    x0 = clamp_int(x0, 0, BEACON_IMAGE_W - 1);
    y0 = clamp_int(y0, 0, BEACON_IMAGE_H - 1);
    x1 = clamp_int(x1, 0, BEACON_IMAGE_W - 1);
    y1 = clamp_int(y1, 0, BEACON_IMAGE_H - 1);

    for (y = y0; y <= y1; ++y)
    {
        for (x = x0; x <= x1; ++x)
        {
            int distance = abs_int(x - (int)center_x);
            const int dy = abs_int(y - (int)center_y);

            if (dy > distance)
            {
                distance = dy;
            }
            if (distance < 3 || distance > 5)
            {
                continue;
            }

            sum += context->frame[y][x];
            ++count;
        }
    }

    return (count == 0U) ? 0U : (unsigned char)(sum / count);
}

static unsigned char is_local_peak(const beacon_image_context_t *context,
                                   unsigned char x,
                                   unsigned char y,
                                   unsigned char *ring_mean)
{
    int nx;
    int ny;
    const unsigned char peak = context->frame[y][x];
    unsigned char local_background;

    if (y < IMAGE_TOP_EXCLUSION_Y || peak < IMAGE_SEED_MIN_PEAK)
    {
        return 0U;
    }

    for (ny = (int)y - 1; ny <= (int)y + 1; ++ny)
    {
        if (ny < 0 || ny >= BEACON_IMAGE_H)
        {
            continue;
        }
        for (nx = (int)x - 1; nx <= (int)x + 1; ++nx)
        {
            if (nx < 0 || nx >= BEACON_IMAGE_W || (nx == (int)x && ny == (int)y))
            {
                continue;
            }
            if (context->frame[ny][nx] > peak)
            {
                return 0U;
            }
        }
    }

    local_background = local_ring_mean(context, x, y);
    if (ring_mean != 0)
    {
        *ring_mean = local_background;
    }

    return (((int)peak - (int)local_background) >= IMAGE_SEED_MIN_CONTRAST) ? 1U : 0U;
}

static beacon_image_candidate_t grow_candidate(beacon_image_context_t *context,
                                               unsigned char seed_x,
                                               unsigned char seed_y,
                                               unsigned char ring_mean)
{
    unsigned int head = 0U;
    unsigned int tail = 0U;
    const unsigned char seed_peak = context->frame[seed_y][seed_x];
    int grow_threshold = (int)seed_peak - IMAGE_REGION_DROP;
    beacon_image_candidate_t candidate;

    memset(&candidate, 0, sizeof(candidate));
    candidate.min_x = seed_x;
    candidate.max_x = seed_x;
    candidate.min_y = seed_y;
    candidate.max_y = seed_y;
    candidate.peak = seed_peak;
    candidate.seed_x = seed_x;
    candidate.seed_y = seed_y;
    candidate.ring_mean = ring_mean;

    if (grow_threshold > 60)
    {
        grow_threshold = 60;
    }
    grow_threshold = clamp_int(grow_threshold, IMAGE_REGION_FLOOR, seed_peak);

    context->queue_x[tail] = seed_x;
    context->queue_y[tail] = seed_y;
    ++tail;
    mark_visited(context, seed_x, seed_y);

    while (head < tail)
    {
        static const signed char dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
        static const signed char dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
        int i;
        const unsigned char x = context->queue_x[head];
        const unsigned char y = context->queue_y[head];
        ++head;

        candidate.area++;
        candidate.sum_x += x;
        candidate.sum_y += y;
        if (x < candidate.min_x)
        {
            candidate.min_x = x;
        }
        if (x > candidate.max_x)
        {
            candidate.max_x = x;
        }
        if (y < candidate.min_y)
        {
            candidate.min_y = y;
        }
        if (y > candidate.max_y)
        {
            candidate.max_y = y;
        }
        if (context->frame[y][x] > candidate.peak)
        {
            candidate.peak = context->frame[y][x];
        }

        for (i = 0; i < 8; ++i)
        {
            const int nx = (int)x + dx[i];
            const int ny = (int)y + dy[i];
            unsigned char pixel;

            if (nx < 0 || nx >= BEACON_IMAGE_W || ny < 0 || ny >= BEACON_IMAGE_H)
            {
                continue;
            }
            if (is_visited(context, (unsigned char)nx, (unsigned char)ny) != 0U)
            {
                continue;
            }

            pixel = context->frame[ny][nx];
            if ((int)pixel < grow_threshold ||
                pixel <= (unsigned char)clamp_int((int)ring_mean + 12, 0, 255))
            {
                continue;
            }

            mark_visited(context, (unsigned char)nx, (unsigned char)ny);
            context->queue_x[tail] = (unsigned char)nx;
            context->queue_y[tail] = (unsigned char)ny;
            ++tail;
        }
    }

    return candidate;
}

static float candidate_center_x(const beacon_image_candidate_t *candidate)
{
    return (float)candidate->sum_x / (float)candidate->area;
}

static float candidate_center_y(const beacon_image_candidate_t *candidate)
{
    return (float)candidate->sum_y / (float)candidate->area;
}

static unsigned char candidate_too_close(const beacon_image_candidate_t *a,
                                         const beacon_image_candidate_t *b)
{
    const float dx = candidate_center_x(a) - candidate_center_x(b);
    const float dy = candidate_center_y(a) - candidate_center_y(b);

    return ((dx * dx) + (dy * dy) < 16.0f) ? 1U : 0U;
}

static unsigned char valid_candidate_geometry(const beacon_image_candidate_t *candidate)
{
    const int width = ((int)candidate->max_x - (int)candidate->min_x) + 1;
    const int height = ((int)candidate->max_y - (int)candidate->min_y) + 1;
    const int max_side = (width > height) ? width : height;
    const int min_side = (width < height) ? width : height;
    const float center_y = candidate_center_y(candidate);

    if (candidate->area < BEACON_IMAGE_MIN_COMPONENT_AREA ||
        candidate->area > IMAGE_MAX_COMPONENT_AREA)
    {
        return 0U;
    }
    if (center_y < (float)IMAGE_TOP_EXCLUSION_Y)
    {
        return 0U;
    }
    if (max_side > IMAGE_MAX_COMPONENT_SIDE)
    {
        return 0U;
    }
    if (min_side <= 0 || max_side > ((min_side * 5) + 2))
    {
        return 0U;
    }
    return 1U;
}

static int score_candidate(const beacon_image_candidate_t *candidate)
{
    const int width = ((int)candidate->max_x - (int)candidate->min_x) + 1;
    const int height = ((int)candidate->max_y - (int)candidate->min_y) + 1;
    const int box_area = width * height;
    const int contrast = (int)candidate->peak - (int)candidate->ring_mean;
    int size_score = candidate->area;
    int fill_score = 0;

    if (size_score > 24)
    {
        size_score = 24;
    }
    if (box_area > 0)
    {
        fill_score = ((int)candidate->area * 450) / box_area;
    }

    return (contrast * 20) +
           ((int)candidate->peak * 7) +
           fill_score +
           (size_score * 50);
}

static void insert_candidate(beacon_image_context_t *context,
                             beacon_image_candidate_t candidate,
                             unsigned char *candidate_count)
{
    int i;
    int pos;

    if (valid_candidate_geometry(&candidate) == 0U)
    {
        return;
    }
    candidate.score = score_candidate(&candidate);

    for (i = 0; i < (int)(*candidate_count); ++i)
    {
        if (candidate_too_close(&candidate, &context->candidates[i]) != 0U)
        {
            if (candidate.score > context->candidates[i].score)
            {
                context->candidates[i] = candidate;
            }
            return;
        }
    }

    pos = (int)(*candidate_count);
    if (pos >= BEACON_IMAGE_MAX_CANDIDATES)
    {
        pos = BEACON_IMAGE_MAX_CANDIDATES - 1;
        if (candidate.score <= context->candidates[pos].score)
        {
            return;
        }
    }
    else
    {
        (*candidate_count)++;
    }

    for (i = pos - 1; i >= 0 && context->candidates[i].score < candidate.score; --i)
    {
        context->candidates[i + 1] = context->candidates[i];
    }
    context->candidates[i + 1] = candidate;
}

static void find_candidates(beacon_image_context_t *context, unsigned char *candidate_count)
{
    int x;
    int y;

    *candidate_count = 0U;
    begin_visit_pass(context);

    for (y = IMAGE_TOP_EXCLUSION_Y; y < BEACON_IMAGE_H; ++y)
    {
        for (x = 0; x < BEACON_IMAGE_W; ++x)
        {
            unsigned char ring_mean = 0U;
            beacon_image_candidate_t candidate;

            if (is_visited(context, (unsigned char)x, (unsigned char)y) != 0U)
            {
                continue;
            }
            if (is_local_peak(context, (unsigned char)x, (unsigned char)y, &ring_mean) == 0U)
            {
                continue;
            }

            candidate = grow_candidate(context, (unsigned char)x, (unsigned char)y, ring_mean);
            insert_candidate(context, candidate, candidate_count);
        }
    }
}

static float distance_sq(float ax, float ay, float bx, float by)
{
    const float dx = ax - bx;
    const float dy = ay - by;

    return (dx * dx) + (dy * dy);
}

static int choose_best_for_track(const beacon_image_context_t *context,
                                 unsigned char track_index,
                                 const unsigned char *used,
                                 unsigned char candidate_count)
{
    unsigned char i;
    int best = -1;
    float best_score = -1000000.0f;

    for (i = 0U; i < candidate_count; ++i)
    {
        const float cx = candidate_center_x(&context->candidates[i]);
        const float cy = candidate_center_y(&context->candidates[i]);
        float score = (float)context->candidates[i].score;

        if (used[i] != 0U)
        {
            continue;
        }

        if (context->tracks[track_index].active != 0U)
        {
            const float dist2 = distance_sq(cx, cy, context->tracks[track_index].x, context->tracks[track_index].y);
            if (dist2 > (IMAGE_TRACK_MATCH_DISTANCE * IMAGE_TRACK_MATCH_DISTANCE))
            {
                continue;
            }
            score -= dist2 * 15.0f;
        }

        if (score > best_score)
        {
            best_score = score;
            best = (int)i;
        }
    }

    return best;
}

static float candidate_radius(const beacon_image_candidate_t *candidate)
{
    return sqrtf((float)candidate->area / IMAGE_PI);
}

static void output_tracks(const beacon_image_context_t *context,
                          unsigned char matched_this_frame,
                          beacon_result_t *result)
{
    unsigned char i;
    unsigned char j;
    unsigned char order[BEACON_IMAGE_TRACK_COUNT];

    clear_result(result);

    for (i = 0U; i < BEACON_IMAGE_TRACK_COUNT; ++i)
    {
        order[i] = i;
    }

    for (i = 0U; i < (BEACON_IMAGE_TRACK_COUNT - 1U); ++i)
    {
        for (j = (unsigned char)(i + 1U); j < BEACON_IMAGE_TRACK_COUNT; ++j)
        {
            const unsigned char left = order[i];
            const unsigned char right = order[j];
            unsigned char swap_order = 0U;

            if (context->tracks[left].active == 0U && context->tracks[right].active != 0U)
            {
                swap_order = 1U;
            }
            else if (context->tracks[left].active != 0U && context->tracks[right].active != 0U)
            {
                const float left_rank = (float)context->tracks[left].score + (context->tracks[left].radius * 900.0f);
                const float right_rank = (float)context->tracks[right].score + (context->tracks[right].radius * 900.0f);
                if (right_rank > left_rank)
                {
                    swap_order = 1U;
                }
            }

            if (swap_order != 0U)
            {
                const unsigned char temp = order[i];
                order[i] = order[j];
                order[j] = temp;
            }
        }
    }

    for (i = 0U; i < BEACON_IMAGE_TRACK_COUNT; ++i)
    {
        const unsigned char track_index = order[i];
        beacon_circle_t *circle;

        if (context->tracks[track_index].active == 0U)
        {
            continue;
        }
        if (context->tracks[track_index].missing != 0U)
        {
            continue;
        }
        if (result->count >= BEACON_MAX_CIRCLE_COUNT)
        {
            break;
        }

        circle = &result->circles[result->count];
        circle->x = BEACON_IMAGE_TARGET_PIXEL_X - context->tracks[track_index].x;
        circle->y = context->tracks[track_index].y - BEACON_IMAGE_TARGET_PIXEL_Y;
        circle->radius = context->tracks[track_index].radius;
        circle->valid = 1U;
        result->count++;
    }
}

static void drop_stale_tracks_after_jump(beacon_image_context_t *context, unsigned char candidate_count)
{
    unsigned char track_index;

    if (candidate_count == 0U)
    {
        return;
    }

    for (track_index = 0U; track_index < BEACON_IMAGE_TRACK_COUNT; ++track_index)
    {
        unsigned char candidate_index;
        unsigned char has_near_candidate = 0U;

        if (context->tracks[track_index].active == 0U ||
            context->tracks[track_index].missing == 0U)
        {
            continue;
        }

        for (candidate_index = 0U; candidate_index < candidate_count; ++candidate_index)
        {
            const float cx = candidate_center_x(&context->candidates[candidate_index]);
            const float cy = candidate_center_y(&context->candidates[candidate_index]);
            if (distance_sq(context->tracks[track_index].x,
                            context->tracks[track_index].y,
                            cx,
                            cy) <= (IMAGE_TRACK_MATCH_DISTANCE * IMAGE_TRACK_MATCH_DISTANCE))
            {
                has_near_candidate = 1U;
                break;
            }
        }

        if (has_near_candidate == 0U)
        {
            context->tracks[track_index].active = 0U;
        }
    }
}

static void update_tracks(beacon_image_context_t *context,
                          unsigned char candidate_count,
                          beacon_result_t *result)
{
    unsigned char used[BEACON_IMAGE_MAX_CANDIDATES];
    unsigned char track_index;
    unsigned char i;
    unsigned char matched_this_frame = 0U;

    memset(used, 0, sizeof(used));

    for (track_index = 0U; track_index < BEACON_IMAGE_TRACK_COUNT; ++track_index)
    {
        const int selected = choose_best_for_track(context, track_index, used, candidate_count);

        if (selected >= 0)
        {
            const unsigned char candidate_index = (unsigned char)selected;
            context->tracks[track_index].x = candidate_center_x(&context->candidates[candidate_index]);
            context->tracks[track_index].y = candidate_center_y(&context->candidates[candidate_index]);
            context->tracks[track_index].radius = candidate_radius(&context->candidates[candidate_index]);
            context->tracks[track_index].score = context->candidates[candidate_index].score;
            context->tracks[track_index].missing = 0U;
            context->tracks[track_index].active = 1U;
            used[candidate_index] = 1U;
            matched_this_frame = 1U;
        }
        else if (context->tracks[track_index].active != 0U)
        {
            context->tracks[track_index].missing++;
            if (context->tracks[track_index].missing > IMAGE_TRACK_KEEP_FRAMES)
            {
                context->tracks[track_index].active = 0U;
            }
        }
    }

    for (track_index = 0U; track_index < BEACON_IMAGE_TRACK_COUNT; ++track_index)
    {
        unsigned char other_index;

        if (context->tracks[track_index].active == 0U)
        {
            continue;
        }
        for (other_index = (unsigned char)(track_index + 1U); other_index < BEACON_IMAGE_TRACK_COUNT; ++other_index)
        {
            if (context->tracks[other_index].active == 0U)
            {
                continue;
            }
            if (distance_sq(context->tracks[track_index].x,
                            context->tracks[track_index].y,
                            context->tracks[other_index].x,
                            context->tracks[other_index].y) <=
                (IMAGE_TRACK_MERGE_DISTANCE * IMAGE_TRACK_MERGE_DISTANCE))
            {
                if (context->tracks[track_index].score >= context->tracks[other_index].score)
                {
                    context->tracks[other_index].active = 0U;
                }
                else
                {
                    context->tracks[track_index].active = 0U;
                    break;
                }
            }
        }
    }

    drop_stale_tracks_after_jump(context, candidate_count);

    for (i = 0U; i < candidate_count; ++i)
    {
        unsigned char free_track = 0xFFU;

        if (used[i] != 0U)
        {
            continue;
        }
        for (track_index = 0U; track_index < BEACON_IMAGE_TRACK_COUNT; ++track_index)
        {
            if (context->tracks[track_index].active == 0U)
            {
                free_track = track_index;
                break;
            }
        }
        if (free_track == 0xFFU)
        {
            continue;
        }

        context->tracks[free_track].x = candidate_center_x(&context->candidates[i]);
        context->tracks[free_track].y = candidate_center_y(&context->candidates[i]);
        context->tracks[free_track].radius = candidate_radius(&context->candidates[i]);
        context->tracks[free_track].score = context->candidates[i].score;
        context->tracks[free_track].missing = 0U;
        context->tracks[free_track].active = 1U;
        used[i] = 1U;
        matched_this_frame = 1U;
    }

    output_tracks(context, matched_this_frame, result);
}

static void process_frame(beacon_image_context_t *context, beacon_result_t *result)
{
    unsigned char candidate_count = 0U;

    memset(context->candidates, 0, sizeof(context->candidates));
    find_candidates(context, &candidate_count);
    update_tracks(context, candidate_count, result);
}

void beacon_image_process_with_context(
    beacon_image_context_t *context,
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    clear_result(result);
    if (context == 0 || image == 0 || result == 0)
    {
        return;
    }

    memcpy(context->frame[0], image[0], BEACON_IMAGE_W * BEACON_IMAGE_H);
    process_frame(context, result);
}

void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result)
{
    beacon_image_process_with_context(&g_default_context, image, result);
}

unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;
    int max_value = 0;
    unsigned long sum = 0;
    const int pixel_count = BEACON_IMAGE_W * BEACON_IMAGE_H;
    int mean_value;
    int threshold;

    if (image == 0)
    {
        return 255U;
    }

    for (y = 0; y < BEACON_IMAGE_H; ++y)
    {
        for (x = 0; x < BEACON_IMAGE_W; ++x)
        {
            const int value = image[y][x];
            sum += (unsigned long)value;
            if (value > max_value)
            {
                max_value = value;
            }
        }
    }

    if (max_value < IMAGE_SEED_MIN_PEAK)
    {
        return 255U;
    }

    mean_value = (int)(sum / (unsigned long)pixel_count);
    threshold = mean_value + (max_value - mean_value) * 45 / 100;
    threshold = clamp_int(threshold, IMAGE_REGION_FLOOR, 245);
    return (unsigned char)threshold;
}

void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W])
{
    int x;
    int y;
    const unsigned char threshold = beacon_image_debug_threshold(image);

    if (image == 0 || binary == 0)
    {
        return;
    }

    for (y = 0; y < BEACON_IMAGE_H; ++y)
    {
        for (x = 0; x < BEACON_IMAGE_W; ++x)
        {
            binary[y][x] = (threshold != 255U && image[y][x] >= threshold) ? 255U : 0U;
        }
    }
}
