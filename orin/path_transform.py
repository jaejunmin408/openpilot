"""ADCM reference path transform helpers — adcm_trajectory_sender 와 web_viz/server 가
동일한 anchor-기반 좌표 변환을 적용하기 위한 공용 모듈.

sender 의 첫 cur_pose 와 server 가 받는 첫 ADCM 패킷의 ego 는 같은 값이므로
같은 함수로 같은 path_world 를 만들면 viz 의 ref path 와 sender 가 추종하는 path 가
정확히 일치한다.
"""

import json
import math

import numpy as np


def make_transform_params(anchor_pose, json_first_pose):
    """JSON 좌표계 점을 anchor_pose 기준 GPS 프레임으로 매핑하는 파라미터 반환.

    매핑:
      rx   = (x − jx)·cos(δ) − (y − jy)·sin(δ) + ax
      ry   = (x − jx)·sin(δ) + (y − jy)·cos(δ) + ay
      ryaw = yaw + δ_yaw
    """
    ax, ay, ayaw = anchor_pose
    jx, jy, jyaw = json_first_pose
    delta_yaw = ayaw - jyaw
    return (ax, ay, jx, jy, delta_yaw, math.cos(delta_yaw), math.sin(delta_yaw))


def apply_transform_to_path(transform, path_json):
    """JSON path (N, 3) 을 GPS 프레임으로 일괄 변환."""
    ax, ay, jx, jy, delta_yaw, cos_d, sin_d = transform
    dx = path_json[:, 0] - jx
    dy = path_json[:, 1] - jy
    rx = dx * cos_d - dy * sin_d + ax
    ry = dx * sin_d + dy * cos_d + ay
    ryaw = path_json[:, 2] + delta_yaw
    return np.stack([rx, ry, ryaw], axis=1)


def find_closest_index(path_world, x, y):
    """차의 (x, y) 에서 가장 가까운 path 점 인덱스."""
    d2 = (path_world[:, 0] - x) ** 2 + (path_world[:, 1] - y) ** 2
    return int(np.argmin(d2))


def signed_lateral_error(path_world, idx, x, y):
    """path tangent 의 왼쪽이 양수인 부호 있는 lateral 거리."""
    px, py, pyaw = path_world[idx]
    perp_x, perp_y = -math.sin(pyaw), math.cos(pyaw)
    return (x - px) * perp_x + (y - py) * perp_y


def load_path_json(path):
    """JSON 파일 읽고 (path_array (N, 3), first_pose, raw_dict) 반환.

    sender 는 raw_dict 에서 speed_mps, point_spacing_m 등 추가 메타를 꺼내 쓰고,
    server 는 path_array + first_pose 만 사용한다.
    """
    with open(path) as f:
        data = json.load(f)
    pts = data.get("path")
    if not pts:
        raise ValueError(f"JSON 'path' 키가 비어있거나 없음: {path}")
    path_json = np.array([(p["x"], p["y"], p["yaw"]) for p in pts], dtype=np.float64)
    first_pose = (float(path_json[0, 0]), float(path_json[0, 1]), float(path_json[0, 2]))
    return path_json, first_pose, data
