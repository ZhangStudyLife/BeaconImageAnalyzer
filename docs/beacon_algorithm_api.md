# Beacon Algorithm Interface Specification

## Overview

BeaconImageAnalyzer loads user-written C algorithms at runtime to detect
infrared beacons in 188x120 grayscale images. Your algorithm must compile
as a single C file that exports the functions described below.

## Prerequisites

- gcc (MinGW or MSYS2) must be installed and available in PATH,
  or located at `C:/msys64/mingw64/bin/gcc.exe` or `C:/code/msys64/mingw64/bin/gcc.exe`
- Your `.c` file must `#include "beacon_image.h"` (provided in the `algorithm/` folder)

## Image Format

```
Width:  188 pixels
Height: 120 pixels
Pixel:  unsigned char (0-255 grayscale)

Input array: const unsigned char image[120][188]
image[y][x] = grayscale value at row y, column x
Row 0 is the top, column 0 is the left
```

## Coordinate System

Your output coordinates use **IMAGE CENTER coordinates**, NOT pixel coordinates:

```
Origin:  pixel (94, 60) = center of the image
X axis:  positive = LEFT in the image
Y axis:  positive = DOWN in the image
```

Conversion formulas:

```
algo_x = 94 - pixel_x      (pixel -> algo)
algo_y = pixel_y - 60       (pixel -> algo)

pixel_x = 94 - algo_x      (algo -> pixel)
pixel_y = 60 + algo_y       (algo -> pixel)
```

Example: If you detect a beacon at pixel (50, 30):

```
algo_x = 94 - 50 = 44.0
algo_y = 30 - 60 = -30.0
```

## Required Functions

### `beacon_image_init` (REQUIRED)

```c
void beacon_image_init(void);
```

Called once when your algorithm is first loaded. Use this to initialize
any static/global state your algorithm needs. If you have no state, leave
the body empty.

### `beacon_image_process` (REQUIRED)

```c
void beacon_image_process(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    beacon_result_t *result
);
```

Called once per frame. This is the main entry point.

**Parameters:**

| Parameter | Description |
|-----------|-------------|
| `image`   | 120x188 grayscale image. `image[y][x]`. Read-only. |
| `result`  | Output structure you must fill. Already zeroed before the call. |

You must fill `result->beacons[]` and `result->beacon_count`:

```c
result->beacons[i].x      = algo center X (see coordinate system)
result->beacons[i].y      = algo center Y (see coordinate system)
result->beacons[i].radius = circle radius in pixels
result->beacons[i].valid  = 1 if this entry is a real detection

result->beacon_count = number of valid beacon entries (0 to 8)
```

Optionally fill `result->car_lamps[]` and `result->car_lamp_count`
for car lamp detections (most users can leave these as 0).

**Maximum counts:**

| Macro | Value | Description |
|-------|-------|-------------|
| `BEACON_MAX_BEACON_COUNT` | 8 | Maximum beacon detections per frame |
| `BEACON_MAX_CAR_LAMP_COUNT` | 2 | Maximum car lamp detections per frame |

## Optional Functions

### `beacon_image_debug_threshold` (OPTIONAL)

```c
unsigned char beacon_image_debug_threshold(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W]
);
```

If exported, called to get the current binary threshold for the
debug binary view. Return the threshold value (0-255).
If not exported, the binary view will not update.

### `beacon_image_debug_binary` (OPTIONAL)

```c
void beacon_image_debug_binary(
    const unsigned char image[BEACON_IMAGE_H][BEACON_IMAGE_W],
    unsigned char binary[BEACON_IMAGE_H][BEACON_IMAGE_W]
);
```

If exported, called to produce a binary image for the debug view.
Fill `binary[y][x]` with 0 or 255.
If not exported, the binary view will show a black image.

## Data Structures

### `beacon_circle_t`

```c
typedef struct {
    float x;               // algo center X
    float y;               // algo center Y
    float radius;          // circle radius in pixels
    unsigned char valid;   // 1 = valid detection, 0 = ignore
} beacon_circle_t;
```

### `beacon_rect_t`

```c
typedef struct {
    float cx;              // algo center X
    float cy;              // algo center Y
    float width;           // rectangle width
    float length;          // rectangle length
    float angle;           // angle in degrees
    unsigned char valid;   // 1 = valid, 0 = ignore
} beacon_rect_t;
```

### `beacon_result_t`

```c
typedef struct {
    beacon_circle_t circles[8];     // legacy field, can ignore
    unsigned char count;            // legacy field, can ignore
    beacon_circle_t beacons[8];     // <-- fill this
    unsigned char beacon_count;     // <-- fill this
    beacon_rect_t car_lamps[2];     // optional
    unsigned char car_lamp_count;   // optional
} beacon_result_t;
```

## Linking Notes

Your `.c` file is compiled into a shared library (DLL) at runtime.

**You may use:**

- Standard C library (`string.h`, `math.h`, `stdlib.h`, etc.)
- Static/global variables (your state is isolated per load)
- `#include "beacon_image.h"` (always available)
- `#include "beacon_image_config.h"` (threshold constants, optional)

**You must NOT:**

- Use C++ features (compiled with `-std=c11`)
- Depend on Qt, OpenCV, Windows API, or any external library
- Use multi-threading
- Allocate resources that require cleanup on unload

## Compilation

The analyzer compiles your file with:

```
gcc -shared -O2 -std=c11 -I<your_dir> -I<algorithm_dir> -o output.dll your_file.c -lm
```

Where:

| Placeholder | Description |
|-------------|-------------|
| `<your_dir>` | Directory containing your `.c` file |
| `<algorithm_dir>` | The `algorithm/` folder (for `beacon_image.h`) |

## Error Handling

If your algorithm crashes (access violation, divide by zero, etc.),
the application will catch the crash and return an empty result for
that frame. The application will NOT terminate. However, you should
test your code carefully to avoid undefined behavior.

## Quick Start

1. Copy `docs/beacon_image_template.c` to your working directory
2. Modify `beacon_image_process()` to implement your detection logic
3. In the application, use **File > Import Algorithm C File** to load your `.c` file
4. The application compiles and loads your algorithm automatically
5. Iterate: modify your `.c` file, re-import, and see updated results
