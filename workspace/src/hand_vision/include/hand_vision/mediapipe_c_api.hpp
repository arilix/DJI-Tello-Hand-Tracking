#pragma once
// MediaPipe 0.10.x C API — struct layout reverse-engineered dari Python ctypes bindings.
// Struct sizes diverifikasi: sizeof(BaseOptionsC)=56, NormalizedLandmarkC=40,
// HandLandmarkerResultC=48, HandLandmarkerOptionsC=88.

#include <cstdint>
#include <cstddef>

// ─── BaseOptions ──────────────────────────────────────────────────────────────
struct MpBaseOptions {
    const char* model_asset_buffer;        // [0]  8 bytes
    uint32_t    model_asset_buffer_count;  // [8]  4 bytes
    uint32_t    _pad0;                     // [12] 4 bytes padding
    const char* model_asset_path;          // [16] 8 bytes
    int32_t     delegate;                  // [24] 4 bytes  (0=CPU)
    int32_t     host_environment;          // [28] 4 bytes  (0)
    int32_t     host_system;               // [32] 4 bytes  (0)
    uint32_t    _pad1;                     // [36] 4 bytes padding
    const char* host_version;              // [40] 8 bytes  (null ok)
    const char* ca_bundle_path;            // [48] 8 bytes  (null ok)
};  // total 56 bytes

static_assert(sizeof(MpBaseOptions) == 56, "MpBaseOptions size mismatch");

// ─── RunningMode ─────────────────────────────────────────────────────────────
enum MpRunningMode : int32_t { MP_IMAGE = 1, MP_VIDEO = 2, MP_LIVE_STREAM = 3 };

// ─── Landmark ────────────────────────────────────────────────────────────────
struct MpNormalizedLandmark {
    float        x, y, z;        // [0]  12 bytes
    bool         has_visibility;  // [12] 1 byte
    uint8_t      _pad0[3];        // [13] 3 bytes padding
    float        visibility;      // [16] 4 bytes
    bool         has_presence;    // [20] 1 byte
    uint8_t      _pad1[3];        // [21] 3 bytes padding
    float        presence;        // [24] 4 bytes
    uint32_t     _pad2;           // [28] 4 bytes padding
    const char*  name;            // [32] 8 bytes
};  // total 40 bytes

static_assert(sizeof(MpNormalizedLandmark) == 40, "MpNormalizedLandmark size mismatch");

struct MpNormalizedLandmarks {
    MpNormalizedLandmark* landmarks;        // [0]  8 bytes
    uint32_t              landmarks_count;  // [8]  4 bytes
    uint32_t              _pad;             // [12] 4 bytes padding
};  // total 16 bytes

static_assert(sizeof(MpNormalizedLandmarks) == 16, "MpNormalizedLandmarks size mismatch");

struct MpLandmark {
    float        x, y, z;
    bool         has_visibility;
    uint8_t      _pad0[3];
    float        visibility;
    bool         has_presence;
    uint8_t      _pad1[3];
    float        presence;
    uint32_t     _pad2;
    const char*  name;
};  // total 40 bytes

static_assert(sizeof(MpLandmark) == 40, "MpLandmark size mismatch");

struct MpLandmarks {
    MpLandmark*  landmarks;
    uint32_t     landmarks_count;
    uint32_t     _pad;
};  // total 16 bytes

static_assert(sizeof(MpLandmarks) == 16, "MpLandmarks size mismatch");

// ─── Category ────────────────────────────────────────────────────────────────
struct MpCategory {
    int32_t      index;          // [0]  4 bytes
    float        score;          // [4]  4 bytes
    const char*  category_name;  // [8]  8 bytes
    const char*  display_name;   // [16] 8 bytes
};  // total 24 bytes

static_assert(sizeof(MpCategory) == 24, "MpCategory size mismatch");

struct MpCategories {
    MpCategory*  categories;
    uint32_t     categories_count;
    uint32_t     _pad;
};  // total 16 bytes

static_assert(sizeof(MpCategories) == 16, "MpCategories size mismatch");

// ─── HandLandmarkerResult ─────────────────────────────────────────────────────
struct MpHandLandmarkerResult {
    MpCategories*          handedness;               // [0]  8 bytes
    uint32_t               handedness_count;         // [8]  4 bytes
    uint32_t               _pad0;                    // [12] 4 bytes
    MpNormalizedLandmarks* hand_landmarks;           // [16] 8 bytes
    uint32_t               hand_landmarks_count;     // [24] 4 bytes
    uint32_t               _pad1;                    // [28] 4 bytes
    MpLandmarks*           hand_world_landmarks;     // [32] 8 bytes
    uint32_t               hand_world_landmarks_count; // [40] 4 bytes
    uint32_t               _pad2;                    // [44] 4 bytes
};  // total 48 bytes

static_assert(sizeof(MpHandLandmarkerResult) == 48, "MpHandLandmarkerResult size mismatch");

// ─── HandLandmarkerOptions ───────────────────────────────────────────────────
using MpHandResultCallback = void(*)(
    int32_t                  status,
    MpHandLandmarkerResult*  result,
    void*                    image,
    int64_t                  timestamp_ms
);

struct MpHandLandmarkerOptions {
    MpBaseOptions           base_options;                     // [0]   56 bytes
    MpRunningMode           running_mode;                     // [56]   4 bytes
    int32_t                 num_hands;                        // [60]   4 bytes
    float                   min_hand_detection_confidence;   // [64]   4 bytes
    float                   min_hand_presence_confidence;    // [68]   4 bytes
    float                   min_tracking_confidence;         // [72]   4 bytes
    uint32_t                _pad;                            // [76]   4 bytes
    MpHandResultCallback    result_callback;                 // [80]   8 bytes
};  // total 88 bytes

static_assert(sizeof(MpHandLandmarkerOptions) == 88, "MpHandLandmarkerOptions size mismatch");

// ─── C API functions ──────────────────────────────────────────────────────────
// Semua CStatusFunction mengembalikan int status (0=sukses) dan mengambil
// char** error_msg sebagai argumen terakhir.

extern "C" {

int MpHandLandmarkerCreate(
    MpHandLandmarkerOptions* options,
    void**                   handle_out,
    char**                   error_msg);

int MpHandLandmarkerDetectImage(
    void*                    handle,
    void*                    image,
    void*                    processing_opts,   // nullable
    MpHandLandmarkerResult*  result,
    char**                   error_msg);

void MpHandLandmarkerCloseResult(MpHandLandmarkerResult* result);

int MpHandLandmarkerClose(void* handle, char** error_msg);

int MpImageCreateFromUint8Data(
    int32_t   format,
    int32_t   width,
    int32_t   height,
    uint8_t*  data,
    int32_t   data_size,
    void**    image_out,
    char**    error_msg);

void MpImageFree(void* image);

void MpErrorFree(char* error_msg);

}  // extern "C"
