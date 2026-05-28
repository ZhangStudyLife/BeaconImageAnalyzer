#ifndef BEACON_IMAGE_CONFIG_H_
#define BEACON_IMAGE_CONFIG_H_

/* Normal threshold used for beacon segmentation after car-lamp masking. */
#define BEACON_BINARY_THRESHOLD       200
#define BEACON_BEACON_LOW_THRESHOLD   150

/* High threshold used to isolate the very bright car lamp strip. */
#define BEACON_CAR_LAMP_THRESHOLD     200

/* Connected-component area filters. */
#define BEACON_MIN_COMPONENT_AREA     8
#define BEACON_MAX_COMPONENT_AREA     5000
#define BEACON_MIN_LAMP_AREA          24
#define BEACON_MAX_LAMP_AREA          1200
#define BEACON_MIN_LAMP_ELONGATION    1.8f
#define BEACON_MIN_LAMP_LENGTH        8.0f
#define BEACON_LAMP_MASK_PAD          4
#define BEACON_EDGE_LAMP_MIN_AREA     18
#define BEACON_EDGE_LAMP_MARGIN       2
#define BEACON_EDGE_LAMP_MIN_SPAN     5
#define BEACON_MAX_BEACON_AREA        260
#define BEACON_MAX_BEACON_ELONGATION  2.8f
#define BEACON_EDGE_BEACON_MIN_AREA   8
#define BEACON_EDGE_BEACON_MARGIN     1
#define BEACON_DUPLICATE_DISTANCE     5.0f
#define BEACON_TRACK_MATCH_DISTANCE   36.0f

/* Queue size = image pixel count. */
#define BEACON_QUEUE_SIZE             (BEACON_IMAGE_W * BEACON_IMAGE_H)

#endif
