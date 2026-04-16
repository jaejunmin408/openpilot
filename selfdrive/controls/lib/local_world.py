import bisect
import math
from collections import deque

HISTORY_SECONDS = 6.0
POSE_RATE_HZ = 20
HISTORY_LEN = int(HISTORY_SECONDS * POSE_RATE_HZ)
MAX_DT_SEC = 0.5


class LocalWorld:
  """livePose 적분으로 로컬 월드 frame (x, y, yaw) 유지 + 과거 pose 조회.

  Caller 가 매 livePose 메시지마다 update(live_pose, t_ns) 호출.
  외부 의존성 없음 (SubMaster 미소유).

  좌표계: 첫 valid livePose 시점의 차 위치를 원점으로 하는 로컬 frame.
    - x, y: 적분된 위치 (단위: m)
    - yaw: orientationNED.z 그대로 사용 (locationd 가 이미 필터)
  """

  def __init__(self, history_len: int = HISTORY_LEN):
    self._buf: deque = deque(maxlen=history_len)
    self._x = 0.0
    self._y = 0.0
    self._yaw = 0.0
    self._last_t_ns: int | None = None
    self._initialized = False

  def update(self, live_pose, t_ns: int) -> None:
    if not self._initialized:
      if live_pose.orientationNED.valid:
        self._yaw = live_pose.orientationNED.z
        self._initialized = True
        self._last_t_ns = t_ns
        self._buf.append((t_ns, 0.0, 0.0, self._yaw))
      return

    dt = (t_ns - self._last_t_ns) / 1e9
    if dt <= 0 or dt > MAX_DT_SEC:
      self._last_t_ns = t_ns
      return

    if not (live_pose.orientationNED.valid and live_pose.velocityDevice.valid):
      self._last_t_ns = t_ns
      return

    yaw_new = live_pose.orientationNED.z
    yaw_mid = 0.5 * (self._yaw + yaw_new)
    c, s = math.cos(yaw_mid), math.sin(yaw_mid)
    vx = live_pose.velocityDevice.x
    vy = live_pose.velocityDevice.y
    self._x += (vx * c - vy * s) * dt
    self._y += (vx * s + vy * c) * dt
    self._yaw = yaw_new
    self._last_t_ns = t_ns
    self._buf.append((t_ns, self._x, self._y, self._yaw))

  def current(self) -> tuple[int, float, float, float] | None:
    return self._buf[-1] if self._buf else None

  def at(self, t_ns: int) -> tuple[int, float, float, float] | None:
    """t_ns 시점 pose 를 선형 보간으로 반환. 범위 밖이면 가장 가까운 끝점."""
    if not self._buf:
      return None

    samples = list(self._buf)
    timestamps = [s[0] for s in samples]
    idx = bisect.bisect_left(timestamps, t_ns)

    if idx == 0:
      return samples[0]
    if idx >= len(samples):
      return samples[-1]

    t1, x1, y1, yaw1 = samples[idx - 1]
    t2, x2, y2, yaw2 = samples[idx]
    alpha = (t_ns - t1) / (t2 - t1)
    x = x1 + alpha * (x2 - x1)
    y = y1 + alpha * (y2 - y1)
    sin_avg = (1.0 - alpha) * math.sin(yaw1) + alpha * math.sin(yaw2)
    cos_avg = (1.0 - alpha) * math.cos(yaw1) + alpha * math.cos(yaw2)
    yaw = math.atan2(sin_avg, cos_avg)
    return (t_ns, x, y, yaw)

  def history(self) -> list[tuple[int, float, float, float]]:
    """현재 buffer 내용 snapshot. 가장 오래된 → 최신 순."""
    return list(self._buf)

  def is_initialized(self) -> bool:
    return self._initialized

  def __len__(self) -> int:
    return len(self._buf)
