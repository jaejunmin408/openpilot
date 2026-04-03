# UDP Packet Spec

This package replays the model output as a compact UDP packet intended for control integration.

## Byte order
- Little-endian

## Header layout
- `magic`: `4s` = `ALPA`
- `version`: `uint16`
- `flags`: `uint16`
- `tx_seq`: `uint32`
- `plan_seq`: `uint32`
- `sample_id`: `uint32`
- `source_t0_us`: `uint64`
- `tx_time_us`: `uint64`
- `coord_mode`: `uint16`
  - `0 = local`
  - `1 = world`
- `num_points`: `uint16`
- `dt_s`: `float32`

Header size: `44 bytes`

## Point layout
Repeated `num_points` times:
- `x_m`: `float32`
- `y_m`: `float32`
- `yaw_rad`: `float32`
- `v_mps`: `float32`
- `curvature`: `float32`

Point size: `20 bytes`

## Trailer
- `crc32`: `uint32`
  - computed over `header + point payload`

## Current default replay settings
- `coord_mode = local`
- `dt_s = 0.020`
- `num_points = 25`
- `control_horizon = 0.500 s`

## C reference struct
```c
#pragma pack(push, 1)
typedef struct {
  char magic[4];
  uint16_t version;
  uint16_t flags;
  uint32_t tx_seq;
  uint32_t plan_seq;
  uint32_t sample_id;
  uint64_t source_t0_us;
  uint64_t tx_time_us;
  uint16_t coord_mode;
  uint16_t num_points;
  float dt_s;
} ControlPacketHeader;

typedef struct {
  float x_m;
  float y_m;
  float yaw_rad;
  float v_mps;
  float curvature;
} ControlPoint;
#pragma pack(pop)
```
