# Control Team UDP Replay Package

This package contains the artifacts needed to replay the `chunk_0011` model output toward the control stack without requiring live Thor-to-control-PC UDP.

## What is included
- `chunk0011_model_plan_bank.npz`
  - planner output bank derived from actual model inference over all `597` samples
- `packet_previews/`
  - first few packets encoded as binary `.bin`
- `packet_preview_first_packets.json`
  - decoded preview of those packets
- `PACKET_SPEC.md`
  - byte-level UDP format
- `run_localhost_replay.sh`
  - quick launcher for localhost replay
- `viewer/`
  - HTML replay viewer for visual inspection
  - note: `viewer/images_bank` is a symlink to the request-bank images to avoid duplicating the cached PNG set
- `tools/udp_replay_player.py`
  - configurable UDP replay sender

## Model/runtime used
- LLM: `/alpamayo_vlm_engines/alpa1.5/llm.engine`
- ViT: `/alpamayo_vlm_engines/alpa1.5_visual_fp8_rebuild`
- FM: `/root/test/output/alpamayo15_fm_one_step_mxfp8/alpamayo15_fm_one_step_mxfp8_thor.plan`

## Coordinate frame
- Default replay packet uses `local` ego frame
- The underlying NPZ also contains `ref_x_world_enu`, `ref_y_world_enu`, `ref_yaw_world_rad`, so the replay player can also emit `world` packets if needed

## NPZ schema
- `sample_id [N]`
- `t0_us [N]`
- `t_rel_s [N]`
- `plan_dt_s`
- `plan_points`
- `traj_x_local [N, T]`
- `traj_y_local [N, T]`
- `traj_yaw_local [N, T]`
- `traj_v_mps [N, T]`
- `traj_curvature [N, T]`
- `history_x_local [N, H]`
- `history_y_local [N, H]`
- `ref_x_world_enu [N]`
- `ref_y_world_enu [N]`
- `ref_yaw_world_rad [N]`

## Quick start
```bash
cd "/root/TensorRT-Edge-LLM-v060/output/deliverables/control_team/chunk0011_udp_replay_package"
./run_localhost_replay.sh
```

Equivalent direct command:
```bash
python "./tools/udp_replay_player.py" \
  --plan-bank "./chunk0011_model_plan_bank.npz" \
  --host 127.0.0.1 \
  --port 5001 \
  --control-dt 0.02 \
  --control-points 25 \
  --coord-mode local
```

## Tuning knobs
The replay packet shape is not fixed by the model. These values can be changed at replay time:
- `--control-dt`
- `--control-points`
- `--coord-mode local|world`
- `--loop`
- `--playback-rate`

For example, `25 points @ 0.02s` gives a `0.5s` control horizon, while `50 points @ 0.02s` gives a `1.0s` control horizon.

## Visual inspection
If the viewer is being served already:
- `http://localhost:8769/chunk0011_model_timeline_viewer.html`

Or open the files under `viewer/`.
