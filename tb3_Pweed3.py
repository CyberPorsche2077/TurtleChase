#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tb3_connect.py  -  คอม Ubuntu (VS Code) -> TurtleBot3 ฝั่งจับ (PURSUER) ผ่าน SSH

  python3 tb3_connect.py                    # เช็คการเชื่อมต่อ + topic
  python3 tb3_connect.py --bringup          # สั่ง bringup แล้วเช็ค topic
  python3 tb3_connect.py --bringup --move   # bringup แล้วรันอัลกอริทึมฝั่งจับเลย
  python3 tb3_connect.py --move             # ถ้า bringup ค้างอยู่แล้ว
  python3 tb3_connect.py --stop             # หยุด bringup แล้วออก

--move จะอัปโหลดสคริปต์ฝั่งจับ (SEARCH / CHASE / LOST) ไปรันบน Raspberry Pi ของหุ่น
อินพุต /scan + /odom  ->  เอาต์พุต /cmd_vel  แล้วแสดงสถานะกลับมาที่เทอร์มินัลนี้
สคริปต์ค้างการเชื่อมต่อไว้จนกว่าคุณจะกด Ctrl+C เอง (กด Ctrl+C = สั่งหยุดหุ่นก่อนออกเสมอ)

ติดตั้ง:  python3 -m venv ~/tb3_env && source ~/tb3_env/bin/activate && pip install paramiko
บน Pi ต้องมี numpy:  sudo apt install python3-numpy
ข้อควรระวัง: ส่ง /cmd_vel แบบ geometry_msgs/Twist (ใช้ได้กับ ROS 2 Humble)
"""
import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import time

try:
    import paramiko
except ImportError:
    sys.exit("ยังไม่ได้ติดตั้ง paramiko  ->  pip install paramiko  (แนะนำให้ทำใน venv)")

# ============================ TODO: กรอกค่าของคุณ ============================
CONFIG = {
    "wifi_ssid": 'HUAWEI12i',

    # --- ROS 2 บนหุ่น ---
    "ros_distro": 'humble',
    "turtlebot3_model": 'burger',
    "lds_model": 'LDS-02',               # เว้นว่างถ้าไม่ต้องใช้ (ดู e-Manual ของ ROBOTIS)
    "extra_setup": '~/turtlebot3_ws/install/setup.bash',             # workspace ที่ build เองบน Pi (ถ้าไม่มีเว้นว่าง)
    "ros_domain_id": 10,

    # --- SSH ไปที่ Raspberry Pi ---
    "host": '192.168.43.223',
    "user": 'wiriya',
    "password": 'Wiriya0446!',   # แนะนำ: export TB3_PASSWORD='xxxx'
    "key_file": None,              # เช่น "~/.ssh/id_ed25519" (ถ้าใช้จะไม่ใช้รหัสผ่าน)
    "hostname": '',
    "ssh_port": 22,
    "timeout": 8,

    # --- ฝั่งจับ (PURSUER) ---
    "start_delay": 3,                # s นับถอยหลังก่อนหุ่นเริ่มเคลื่อนที่ (Ctrl+C เพื่อยกเลิก)
    "pursuer": {
        "flip_front": False,         # True ถ้าหุ่นวิ่ง "ผิดด้าน" (ด้านหน้าจริงตรงข้ามกับ +x ของ ROS):
                                     #   จะกลับ scan 180 องศา และกลับเครื่องหมายความเร็วเดินหน้าให้
        "v_max": 0.20,               # m/s ความเร็วสูงสุด (burger ทำได้ 0.22) เริ่มต้นแนะนำต่ำ ๆ ก่อน
        "w_max": 2.0,                # rad/s (burger ทำได้ 2.84)
        "evader_speed": 0.20,        # m/s ความเร็วที่สมมติให้ฝั่งหนี (ใช้คำนวณจุดดัก)
        "catch_radius": 0.40,        # m รัศมีจับตามกติกา (วัดจากศูนย์กลางฝั่งหนี)
        "stop_when_caught": True,    # เข้ารัศมีจับแล้วหยุดนิ่ง
        "max_time": 0.0,             # s หยุดเองเมื่อครบเวลา (0 = ไม่จำกัด)
        # ตอนเริ่ม: สมมติว่าฝั่งหนีอยู่ด้านหน้า (วางหุ่นหันหน้าเข้าหากันและไม่มีอะไรบัง)
        "start_front_prior": True,
        "start_prior_angle_deg": 40.0,   # ครึ่งมุมของกรวยด้านหน้า
        "start_prior_max": 3.0,          # m ระยะไกลสุดที่คาดว่าฝั่งหนีอยู่
        # การตรวจจับ: ก้อนเล็กที่ LiDAR เห็น ถ้าหุ่นฝั่งหนีเห็นเป็นก้อนแคบ/กว้างกว่านี้ให้ปรับ
        "detect_max_width": 0.24,        # m ความกว้างสูงสุดของก้อนที่ถือว่า "อาจเป็นหุ่น" (ตัวแยกของนิ่งด้านล่างจะคัดกำแพง/เสาแคบ ๆ ออกเอง)
        "detect_min_points": 2,          # จำนวนจุด LiDAR ขั้นต่ำในก้อน (ลดเป็น 1 ได้ถ้าก้อนเล็กมาก แต่สัญญาณลวงจะเพิ่ม)
        "confirm_n": 3,                  # ต้องเห็นติดกันกี่ scan ถึงยืนยันว่าเป็นฝั่งหนี
        "min_goal_dist": 0.5,            # m ตอน SEARCH/LOST เลือกเป้าที่ไกลจากหุ่นอย่างน้อยเท่านี้ (กันหยุด-ออกตัวถี่ ๆ)
        # ระยะเผื่อรอบสิ่งกีดขวาง: ห้ามเดินใกล้กว่า hard_clearance_m, ช่วงถึง inflate_m เดินได้แต่พยายามเลี่ยง
        "hard_clearance_m": 0.10,        # m (ต้องมากกว่ารัศมีหุ่น 0.07) ถ้าหุ่นชนขอบสิ่งกีดขวางให้เพิ่มค่านี้
        "inflate_m": 0.15,               # m
        "front_slow_m": 0.14,            # m เริ่มลดความเร็วเมื่อด้านหน้าใกล้กว่านี้
        "coast_tau": 0.35,               # s ตอนฝั่งหนีหาย ความเร็วที่ประมาณไว้จะลดลงเรื่อย ๆ ด้วยค่าคงที่นี้
        # แยก "ของนิ่ง" (กำแพง/เสาแคบ ๆ) ออกจาก "ฝั่งหนีที่กำลังเคลื่อนที่" จากการเห็นผิวเดิมซ้ำที่ตำแหน่งเดิม
        "static_min_age": 0.8,           # s ต้องเห็นก้อนนี้มาอย่างน้อยเท่านี้ถึงตัดสินได้ (ยิ่งน้อยยิ่งไวแต่เสี่ยงตัดสินผิด)
        "static_max_age": 3.0,           # s ประวัติย้อนหลังที่ใช้เทียบ
        "static_match": 0.30,            # สัดส่วนจุดที่ซ้ำตำแหน่งเดิม >= ค่านี้ = ของนิ่ง
        "max_classify_w": 0.8,           # rad/s หุ่นหมุนเร็วกว่านี้ไม่ตัดสินนิ่ง/ขยับในรอบนั้น (scan เพี้ยน)
        "wheel_max": 0.22,               # m/s ความเร็วล้อสูงสุด (จำกัด v ตอนเลี้ยวแรง ๆ ไม่ให้ล้อนอกเกินขีดจำกัด)
    },
}
# ============================================================================

EXPECTED_TOPICS = ["/scan", "/cmd_vel", "/odom"]

# ---------- สคริปต์ฝั่งจับ ที่จะถูกอัปโหลดไปรันบนหุ่น (rclpy + numpy) ----------
REMOTE_SCRIPT = r'''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pursuer_remote.py  -  สมองฝั่งจับ (PURSUER) รันบน Raspberry Pi ของหุ่น (ROS 2 Humble)
อินพุต : /scan (LiDAR)  /odom
เอาต์พุต: /cmd_vel (Twist)
state machine: SEARCH / CHASE / LOST
  SEARCH : target belief grid + negative information + เลือกเป้า max(belief / เวลาเดินทาง)
  CHASE  : ตรวจจับก้อนเล็กจาก scan + Kalman (ความเร็วคงที่) + ตัดหน้า (intercept)
  LOST   : belief เริ่มใหม่จากตำแหน่ง/ความเร็วสุดท้าย + momentum prior
แมพ: occupancy grid สร้างเองจาก /scan + /odom (กริดอยู่ในเฟรมที่จุดเริ่มต้นของหุ่น = (0,0), หัวหุ่น = แกน +x)
"""
import heapq
import json
import math
import signal
import sys
import threading
import time
from collections import deque

import numpy as np

DEFAULTS = {
    "v_max": 0.20,               # m/s ความเร็วสูงสุดที่ใช้ (burger ทำได้ 0.22)
    "w_max": 2.0,                # rad/s (burger ทำได้ 2.84)
    "evader_speed": 0.20,        # m/s ความเร็วที่สมมติให้ฝั่งหนี (ใช้ทำนายจุดดัก)
    "catch_radius": 0.30,        # m รัศมีจับตามกติกา (วัดจากศูนย์กลางฝั่งหนี)
    "res": 0.075,                # m ขนาดเซลล์ของกริด
    "half_size": 3.0,            # m กริดครอบคลุม +-half_size รอบจุดเริ่ม
    "hard_clearance_m": 0.10,    # m ห้ามเดินใกล้สิ่งกีดขวางกว่านี้ (>= รัศมีหุ่น 0.07 + เผื่อ)
    "inflate_m": 0.15,           # m ช่วง hard..inflate_m เดินได้แต่ถูกปรับ cost เพิ่ม (พยายามเลี่ยง ถ้าจำเป็นก็ผ่าน)
    "soft_cost": 1.0,            # cost เพิ่มต่อก้าวในช่วงดังกล่าว
    "front_slow_m": 0.14,        # m ระยะด้านหน้าที่เริ่มลดความเร็ว (กันชน)
    "coast_tau": 0.35,           # s ค่าคงที่การลดความเร็วเมื่อ track หาย (ไม่เดาตำแหน่งต่อด้วยความเร็วเดิม)
    "target_radius": 0.07,       # m รัศมีหุ่นฝั่งหนี (ใช้ชดเชยตำแหน่งศูนย์กลางจากจุดที่ LiDAR ชน)
    "lidar_min": 0.12,
    "lidar_max": 3.5,
    "detect_max_width": 0.24,    # m ความกว้างสูงสุดของก้อนที่ถือว่าเป็นหุ่น (ตัวหุ่น ~0.14)
    "detect_min_points": 2,      # จำนวนจุด LiDAR ขั้นต่ำในก้อน
    "confirm_n": 3,              # ต้องเห็นติดกันกี่ scan ถึงยืนยัน track
    "lost_time": 0.6,            # s ที่ track หายแล้วเข้า LOST
    "lost_momentum_time": 4.0,   # s ที่ใช้ momentum prior หลังหาย
    "lookahead": 0.30,           # m pure pursuit
    "min_goal_dist": 0.5,        # m SEARCH/LOST เลือกเป้าที่ไกลจากหุ่นอย่างน้อยเท่านี้ (กันหยุด-ออกตัวถี่ ๆ)
    "plan_period": 0.4,          # s วางแผนเส้นทางใหม่ทุกกี่วินาที
    "belief_blur_m": 0.9,        # m ขนาดหน้าต่างรวม belief ตอนเลือกเป้า
    "start_front_prior": True,   # ตอนเริ่ม: ฝั่งหนีอยู่ด้านหน้าในกรวย (วางหุ่นหันหน้าเข้าหากัน)
    "start_prior_angle_deg": 40.0,
    "start_prior_min": 0.4,
    "start_prior_max": 3.0,
    "stop_when_caught": True,    # ถึงรัศมีจับแล้วหยุดนิ่ง
    "flip_front": False,         # True = ด้านหน้าจริงของหุ่นตรงข้ามกับ +x ของ ROS (กลับ scan 180 องศา + กลับเครื่องหมาย v)
    "max_time": 0.0,             # s หยุดเองเมื่อครบเวลา (0 = ไม่จำกัด)
    "scan_timeout": 0.6,         # s ไม่ได้ scan ใหม่เกินนี้ = หยุดหุ่น
    "status_period": 1.0,        # s พิมพ์สถานะทุกกี่วินาที
    # --- แยก "ของนิ่ง" ออกจาก "ฝั่งหนีที่กำลังเคลื่อนที่" ---
    "static_min_age": 0.8,       # s ต้องเห็นก้อนนี้มาอย่างน้อยเท่านี้ ถึงตัดสินได้ว่านิ่งหรือขยับ
    "static_max_age": 3.0,       # s ประวัติย้อนหลังที่ใช้เทียบ
    "static_match": 0.30,        # สัดส่วนจุดที่ซ้ำตำแหน่งเดิม >= ค่านี้ = ของนิ่ง (ฝั่งหนีที่วิ่งอยู่จะซ้ำเกือบ 0)
    "max_classify_w": 0.8,       # rad/s หุ่นหมุนเร็วกว่านี้ scan จะเพี้ยน -> ไม่ตัดสินนิ่ง/ขยับในรอบนั้น
    "wheel_max": 0.22,           # m/s ความเร็วล้อสูงสุด (จำกัด v ให้ล้อข้างนอกไม่เกินขีดจำกัดตอนเลี้ยวแรง ๆ)
}
CFG = dict(DEFAULTS)
ANG = np.deg2rad(np.arange(360))

# ค่าคงที่ที่คำนวณจาก CFG (ตั้งใน setup())
V_MAX = W_MAX = EV_SPEED = CATCH_R = RES = HALF = INFL = TARGET_R = 0.0
LIDAR_MIN = LIDAR_MAX = MAX_W = LOST_T = LOST_MOM_T = LOOKAHEAD = PLAN_PERIOD = BELIEF_DT = 0.0
MIN_PTS = CONFIRM_N = BLUR_K = N = 0
MIN_GOAL = 0.5
HARD = 1
SOFT_COST = 1.0
FRONT_SLOW = 0.14
COAST_TAU = 0.35
GX = GY = None


def setup(cfg=None):
    global HARD, SOFT_COST, FRONT_SLOW, COAST_TAU, MIN_GOAL, CFG, V_MAX, W_MAX, EV_SPEED, CATCH_R, RES, HALF, INFL, TARGET_R, LIDAR_MIN, LIDAR_MAX
    global MAX_W, LOST_T, LOST_MOM_T, LOOKAHEAD, PLAN_PERIOD, BELIEF_DT, MIN_PTS, CONFIRM_N, BLUR_K, N, GX, GY
    CFG = dict(DEFAULTS)
    if cfg:
        CFG.update({k: v for k, v in cfg.items() if k in DEFAULTS})
    V_MAX, W_MAX, EV_SPEED = CFG["v_max"], CFG["w_max"], CFG["evader_speed"]
    CATCH_R, RES, HALF = CFG["catch_radius"], CFG["res"], CFG["half_size"]
    N = int(round(2 * HALF / RES))
    INFL = max(1, int(round(CFG["inflate_m"] / RES)))
    TARGET_R, LIDAR_MIN, LIDAR_MAX = CFG["target_radius"], CFG["lidar_min"], CFG["lidar_max"]
    MAX_W, MIN_PTS, CONFIRM_N = CFG["detect_max_width"], CFG["detect_min_points"], CFG["confirm_n"]
    LOST_T, LOST_MOM_T, LOOKAHEAD = CFG["lost_time"], CFG["lost_momentum_time"], CFG["lookahead"]
    PLAN_PERIOD = CFG["plan_period"]
    MIN_GOAL = CFG["min_goal_dist"]
    HARD = max(1, int(round(CFG["hard_clearance_m"] / RES)))
    SOFT_COST, FRONT_SLOW, COAST_TAU = CFG["soft_cost"], CFG["front_slow_m"], CFG["coast_tau"]
    BELIEF_DT = RES / V_MAX          # diffusion 1 เซลล์ต่อช่วงนี้ = ความเร็วสูงสุด
    k = int(round(CFG["belief_blur_m"] / RES))
    BLUR_K = k + 1 if k % 2 == 0 else k
    c = (np.arange(N) + 0.5) * RES - HALF
    GX, GY = np.meshgrid(c, c, indexing="ij")


setup()


# =============================== HELPERS ===============================
def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def to_cell(x, y):
    return (min(max(int((x + HALF) // RES), 0), N - 1), min(max(int((y + HALF) // RES), 0), N - 1))


def cell_xy(c):
    return ((c[0] + 0.5) * RES - HALF, (c[1] + 0.5) * RES - HALF)


def in_grid(x, y):
    return -HALF + RES <= x < HALF - RES and -HALF + RES <= y < HALF - RES


def shift(a, dx, dy):
    out = np.zeros_like(a)
    out[max(dx, 0):N + min(dx, 0), max(dy, 0):N + min(dy, 0)] = \
        a[max(-dx, 0):N + min(-dx, 0), max(-dy, 0):N + min(-dy, 0)]
    return out


def dilate(mask, r):
    out = mask.copy()
    if r <= 0:
        return out
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            if (dx or dy) and dx * dx + dy * dy <= r * r + 1:
                out |= shift(mask, dx, dy)
    return out


def box_sum(a, k):
    r = k // 2
    p = np.pad(a, ((r + 1, r), (r + 1, r)))
    c = p.cumsum(0).cumsum(1)
    return c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]


_NB = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
       (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]


def dijkstra(cost, blocked, start):
    cl, bl = cost.tolist(), blocked.tolist()
    INF = float("inf")
    dist = [[INF] * N for _ in range(N)]
    prev = [-1] * (N * N)
    sx, sy = start
    dist[sx][sy] = 0.0
    pq = [(0.0, sx, sy)]
    while pq:
        d, x, y = heapq.heappop(pq)
        if d > dist[x][y]:
            continue
        for dx, dy, l in _NB:
            nx, ny = x + dx, y + dy
            if nx < 0 or ny < 0 or nx >= N or ny >= N or bl[nx][ny]:
                continue
            if dx and dy and (bl[x + dx][y] or bl[x][y + dy]):
                continue
            nd = d + l * RES * cl[nx][ny]
            if nd < dist[nx][ny]:
                dist[nx][ny] = nd
                prev[nx * N + ny] = x * N + y
                heapq.heappush(pq, (nd, nx, ny))
    return np.array(dist), prev


def extract(prev, cell):
    out, k = [], cell[0] * N + cell[1]
    while k != -1:
        out.append((k // N, k % N))
        k = prev[k]
    out.reverse()
    return out


def nearest_reachable(dist, c, rad=5):
    best, bd = None, 1e9
    for dx in range(-rad, rad + 1):
        for dy in range(-rad, rad + 1):
            x, y = c[0] + dx, c[1] + dy
            if 0 <= x < N and 0 <= y < N and np.isfinite(dist[x, y]):
                s = math.hypot(dx, dy)
                if s < bd:
                    best, bd = (x, y), s
    return best


def closest_reachable(dist, xy):
    """เซลล์ที่เดินไปถึงและอยู่ใกล้พิกัด xy ที่สุด (ทั้งกริด)"""
    m = np.isfinite(dist)
    if not m.any():
        return None
    d2 = np.where(m, (GX - xy[0]) ** 2 + (GY - xy[1]) ** 2, np.inf)
    i = np.unravel_index(int(np.argmin(d2)), d2.shape)
    return (int(i[0]), int(i[1]))


# =============================== OCCUPANCY MAP ===============================
class OccMap:
    def __init__(self):
        self.lo = np.zeros((N, N), np.float32)
        self.visible = np.zeros((N, N), bool)

    def update(self, pose, ranges, skip):
        x, y, yaw = pose
        a = yaw + ANG
        fin = np.isfinite(ranges)
        rr = np.where(fin, ranges, LIDAR_MAX)
        s = np.arange(0.05, LIDAR_MAX, 0.5 * RES)
        margin = np.where(fin, np.where(skip, 1.6 * RES, 0.8 * RES), 0.0)
        valid = s[None, :] < (rr - margin)[:, None]
        px = x + np.cos(a)[:, None] * s[None, :]
        py = y + np.sin(a)[:, None] * s[None, :]
        ix = np.floor((px + HALF) / RES).astype(int)
        iy = np.floor((py + HALF) / RES).astype(int)
        valid &= (ix >= 0) & (ix < N) & (iy >= 0) & (iy < N)
        free = np.zeros((N, N), bool)
        free[ix[valid], iy[valid]] = True
        self.visible = free.copy()
        hx = np.floor((x + rr * np.cos(a) + HALF) / RES).astype(int)
        hy = np.floor((y + rr * np.sin(a) + HALF) / RES).astype(int)
        hm = fin & ~skip & (hx >= 0) & (hx < N) & (hy >= 0) & (hy < N)
        hit = np.zeros((N, N), bool)
        hit[hx[hm], hy[hm]] = True
        self.lo[free & ~hit] -= 0.25
        self.lo[hit] += 0.9
        np.clip(self.lo, -4, 4, out=self.lo)

    def occ(self):
        return self.lo > 0.5

    def unknown(self):
        return np.abs(self.lo) < 1e-6


# =============================== DETECTION ===============================
def runs(mask):
    n = len(mask)
    if not mask.any() or mask.all():
        return []
    st = int(np.argmin(mask))
    m = np.roll(mask, -st)
    out, i = [], 0
    while i < n:
        if m[i]:
            j = i
            while j + 1 < n and m[j + 1]:
                j += 1
            out.append((np.arange(i, j + 1) + st) % n)
            i = j + 1
        else:
            i += 1
    return out


def detect(ranges, pose, prior):
    """หาก้อนเล็ก ๆ ที่น่าจะเป็นหุ่นอีกตัว จาก scan 360 ray (index 0 = หน้าหุ่น)
       (1) cluster แยกตาม range jump, ความกว้าง <= detect_max_width, ข้างซ้ายขวาเป็นพื้นหลังที่ไกลกว่า,
           และไม่ใช่ของนิ่งที่อยู่ในแมพแล้ว
       (2) จุดที่ชนในเซลล์ที่แมพเคยยืนยันว่า 'ว่าง' (map difference)
       คืน list ของ {"c": (x,y), "pts": จุดของก้อน, "idx": ดัชนี ray, "strong": เห็นในที่ที่เคยว่างแน่ ๆ}"""
    x, y, yaw = pose
    a = yaw + ANG
    fin = np.isfinite(ranges)
    rr = np.where(fin, ranges, np.inf)
    r0 = np.where(fin, ranges, 0.0)
    px, py = x + r0 * np.cos(a), y + r0 * np.sin(a)
    hix = np.clip(np.floor((px + HALF) / RES).astype(int), 0, N - 1)
    hiy = np.clip(np.floor((py + HALF) / RES).astype(int), 0, N - 1)
    occ_prior = prior.lo > 0.5
    cands = []

    def add(idx, strong=False):
        pts = np.stack([px[idx], py[idx]], 1)
        c = pts.mean(0)
        d = c - np.array([x, y])
        nd = max(float(np.hypot(*d)), 1e-6)
        c = c + d / nd * (TARGET_R * 0.7)
        for q in cands:
            if math.hypot(q["c"][0] - c[0], q["c"][1] - c[1]) < 0.15:
                q["strong"] = q["strong"] or strong
                return
        cands.append({"c": (float(c[0]), float(c[1])), "pts": pts, "idx": np.asarray(idx), "strong": strong})

    nxt = np.roll(rr, -1)
    with np.errstate(invalid="ignore"):
        thr = 0.05 + 0.06 * np.minimum(rr, nxt)
        link = np.isfinite(rr) & np.isfinite(nxt) & (np.abs(rr - nxt) < thr)
    brk = np.where(~link)[0]
    for a_, b_ in zip(brk, np.roll(brk, -1)):
        start, end = (a_ + 1) % 360, b_
        n = (end - start) % 360 + 1
        if n < MIN_PTS or n > 40:
            continue
        idx = (start + np.arange(n)) % 360
        if not fin[idx].all():
            continue
        if math.hypot(px[idx[-1]] - px[idx[0]], py[idx[-1]] - py[idx[0]]) > MAX_W:
            continue
        rm = rr[idx].mean()
        if not (rr[(start - 1) % 360] > rm + 0.08 and rr[(end + 1) % 360] > rm + 0.08):
            continue
        if occ_prior[hix[idx], hiy[idx]].mean() >= 0.5:
            continue
        add(idx, strong=False)

    strong = prior.lo < -1.0
    er = strong.copy()
    for dx, dy, _ in _NB:
        er &= shift(strong, dx, dy)
    unexp = fin & er[hix, hiy]
    for idx in runs(unexp):
        if MIN_PTS <= len(idx) <= 40 and \
                math.hypot(px[idx[-1]] - px[idx[0]], py[idx[-1]] - py[idx[0]]) <= MAX_W + 0.06:
            add(idx, strong=True)
    return cands


# =============================== แยก "ของนิ่ง" / "กำลังเคลื่อนที่" ===============================
class MotionFilter:
    """ตัดสินจากการ "เห็นผิวเดิมซ้ำที่ตำแหน่งเดิม" : ก้อนที่เป็นของนิ่ง (กำแพงแคบ ๆ เสา) จุด LiDAR ของมันจะซ้ำตำแหน่ง
       เดิมที่เห็นเมื่อ >= static_min_age วินาทีก่อน แต่ฝั่งหนีที่วิ่งอยู่จะเลื่อนไปแล้ว (ซ้ำเกือบ 0)
       ก้อนที่เพิ่งเห็นไม่ถึง static_min_age -> 'unknown' (ยังไม่ตัดสิน ไม่ไล่)"""
    RF = 0.04

    def __init__(self):
        self.h = deque()

    def classify(self, dets, now):
        tmin, tmax = CFG["static_min_age"], CFG["static_max_age"]
        ref = [p for t, p in self.h if tmin <= now - t <= tmax and len(p)]
        if not ref:
            return ["unknown"] * len(dets)
        R = np.concatenate(ref)
        cells = np.unique(np.floor(R / self.RF).astype(int), axis=0)
        S = set()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                S.update(map(tuple, (cells + np.array([dx, dy])).tolist()))
        out = []
        for d in dets:
            c = d["c"]
            if not (np.hypot(R[:, 0] - c[0], R[:, 1] - c[1]) < 0.5).any():
                out.append("unknown")           # ไม่เคยเห็นอะไรแถวนี้มาก่อน -> ยังตัดสินไม่ได้
                continue
            ci = np.floor(d["pts"] / self.RF).astype(int).tolist()
            f = sum(tuple(q) in S for q in ci) / max(len(ci), 1)
            out.append("static" if f >= CFG["static_match"] else "dyn")
        return out

    def push(self, dets, now):
        pts = np.concatenate([d["pts"] for d in dets]) if dets else np.zeros((0, 2))
        self.h.append((now, pts))
        while self.h and now - self.h[0][0] > CFG["static_max_age"] + 0.5:
            self.h.popleft()


# =============================== KALMAN TRACKER (เวลาจริง) ===============================
class Tracker:
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
    R = np.eye(2) * 0.07 ** 2
    SA = 0.4
    GATE = 0.6
    VCAP = 0.35

    def __init__(self):
        self.reset()

    def reset(self):
        self.active = False
        self.x = np.zeros(4)
        self.P = np.eye(4)
        self.t = 0.0
        self.last_seen_t = 0.0
        self.last_seen = None
        self.hist = []          # [(t, (x, y))]
        self.origin = None
        self.max_disp = 0.0     # ระยะไกลสุดที่ target เคยเคลื่อนห่างจากจุดที่เริ่ม track (ของนิ่งจะใกล้ 0)

    def unseen(self, now):
        return now - self.last_seen_t

    def step(self, cands, now, prefer=None, static_cands=()):
        if self.active:
            dt = max(1e-3, min(now - self.t, 1.0))
            self.t = now
            x_prev = self.x.copy()
            F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
            g = np.array([[dt ** 4 / 4, dt ** 3 / 2], [dt ** 3 / 2, dt ** 2]]) * self.SA ** 2
            Q = np.zeros((4, 4))
            Q[np.ix_([0, 2], [0, 2])] = g
            Q[np.ix_([1, 3], [1, 3])] = g
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + Q
            best, bd = None, self.GATE
            for c in cands:
                d = math.hypot(c[0] - self.x[0], c[1] - self.x[1])
                if d < bd:
                    best, bd = c, d
            for c in static_cands:                 # ก้อนนิ่ง: ยอมรับเฉพาะที่ตรงตำแหน่งที่ทำนายจริง ๆ (ฝั่งหนีหยุดพอดี)
                d = math.hypot(c[0] - self.x[0], c[1] - self.x[1])
                if d < min(0.15, bd):
                    best, bd = c, d
            if best is None:
                # ไม่เห็น: ความเร็วลดลงเรื่อย ๆ (ไม่เดาต่อด้วยความเร็วเดิม -> ไม่เกิด "เป้าผี" ที่เลยไปไกล)
                k = math.exp(-dt / COAST_TAU)
                self.x[:2] = x_prev[:2] + x_prev[2:] * dt * k
                self.x[2:] = x_prev[2:] * k
                return False
            z = np.array(best)
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.x = self.x + K @ (z - self.H @ self.x)
            self.P = (np.eye(4) - K @ self.H) @ self.P
            sp = math.hypot(self.x[2], self.x[3])
            if sp > self.VCAP:
                self.x[2:] *= self.VCAP / sp
            self.last_seen_t = now
            self.last_seen = self.x.copy()
            if self.origin is not None:
                self.max_disp = max(self.max_disp, math.hypot(self.x[0] - self.origin[0], self.x[1] - self.origin[1]))
            return True
        pool = list(cands)
        if prefer is not None:
            pool.sort(key=lambda c: -prefer(c))
        z = None
        if self.hist:
            for c in pool:
                if math.hypot(c[0] - self.hist[-1][1][0], c[1] - self.hist[-1][1][1]) < 0.30:
                    z = c
                    break
        if z is not None:
            self.hist.append((now, z))
        elif pool:
            self.hist = [(now, pool[0])]
        else:
            self.hist = []
        if len(self.hist) >= CONFIRM_N:
            t1, p1 = self.hist[0]
            t3, p3 = self.hist[-1]
            v = (np.array(p3) - np.array(p1)) / max(t3 - t1, 1e-3)
            sp = float(np.hypot(*v))
            if sp > self.VCAP:
                v = v * self.VCAP / sp
            self.x = np.array([p3[0], p3[1], v[0], v[1]])
            self.P = np.diag([0.06 ** 2, 0.06 ** 2, 0.3 ** 2, 0.3 ** 2])
            self.active, self.t, self.last_seen_t, self.hist = True, now, now, []
            self.last_seen = self.x.copy()
            self.origin, self.max_disp = self.x[:2].copy(), 0.3        # ผ่านการตัดสินว่า "เคลื่อนที่" มาแล้ว
            return True
        return False


# =============================== TARGET BELIEF ===============================
class Belief:
    def __init__(self):
        self.set_uniform()

    def set_uniform(self):
        m = np.zeros((N, N))
        m[2:-2, 2:-2] = 1.0
        self.b = m / m.sum()

    def set_front_prior(self, pose, ang_deg, rmin, rmax):
        """ตอนเริ่ม: รู้แค่ว่าฝั่งหนีอยู่ด้านหน้า (วางหุ่นหันหน้าเข้าหากัน) -> belief กระจุกในกรวยด้านหน้า"""
        x, y, yaw = pose
        dx, dy = GX - x, GY - y
        r = np.hypot(dx, dy)
        ang = (np.arctan2(dy, dx) - yaw + np.pi) % (2 * np.pi) - np.pi
        m = (r >= rmin) & (r <= rmax) & (np.abs(ang) <= math.radians(ang_deg))
        if m.sum() == 0:
            self.set_uniform()
        else:
            self.b = m / m.sum()

    def set_blob(self, pos, sigma):
        g = np.exp(-((GX - pos[0]) ** 2 + (GY - pos[1]) ** 2) / (2 * sigma ** 2))
        self.b = g / g.sum()

    def normalize(self, free):
        s = self.b.sum()
        if s < 1e-12:
            self.b = free.astype(float)
            s = self.b.sum()
        self.b /= max(s, 1e-12)

    def predict(self, free, momentum=None):
        W = {}
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    w = 0.20
                elif dx == 0 or dy == 0:
                    w = 0.15
                else:
                    w = 0.05
                if momentum is not None and (dx or dy):
                    u = np.array([dx, dy]) / math.hypot(dx, dy)
                    w *= math.exp(1.6 * float(u @ momentum))
                W[(dx, dy)] = w
        s = sum(W.values())
        nb = np.zeros((N, N))
        for (dx, dy), w in W.items():
            nb += (w / s) * shift(self.b, dx, dy)
        self.b = nb * free
        self.normalize(free)

    def negative(self, visible, free, p_miss=0.5):
        self.b[visible] *= p_miss
        if self.b.sum() < 1e-9:
            self.b = (free & ~visible).astype(float)
        self.normalize(free)


# =============================== INTERCEPT / PLAN ===============================
def intercept(T, lo, tr_x, stable):
    """ตัดหน้า: จุดแรกบนวิถีที่ทำนาย (ตามทิศ Kalman) ที่เราไปถึงทันฝั่งหนี (ใช้เวลาเดินทางตามเส้นทางจริง)
       ถ้าทิศมุ่งหน้านิ่ง (เดาว่ากำลังไปเส้นชัย) มองวิถีไกลขึ้น -> จุดดักไปอยู่ระหว่างฝั่งหนีกับเส้นชัย"""
    pe, ve = tr_x[:2], tr_x[2:]
    sp = math.hypot(*ve)
    if sp > 0.10:
        spd = min(sp, EV_SPEED)
        if stable:                       # ทิศนิ่ง = กำลังมุ่งไปที่ใดที่หนึ่งด้วยความเร็วใกล้เต็ม
            spd = max(spd, 0.7 * EV_SPEED)
        u = ve / sp
    else:
        u, spd = np.zeros(2), 0.0        # แทบไม่ขยับ: เล็งที่ตัวมันตรง ๆ
    horizon = 8.0 if stable else 2.5
    out = []
    for t in np.arange(0.0, horizon + 1e-9, 0.2):
        p = pe + u * spd * t
        if not in_grid(p[0], p[1]):
            break
        c = to_cell(p[0], p[1])
        if lo[c] > 0.5:
            break
        tp = max(0.0, T[c] * V_MAX - 0.8 * CATCH_R) / V_MAX
        out.append((tp - t, p))
    if not out:
        return pe
    for s, p in out:
        if s <= 0.3:
            return p
    return min(out, key=lambda o: o[0])[1]


def compute_plan(snap):
    lo, pose, mode = snap["lo"], snap["pose"], snap["mode"]
    occ = lo > 0.5
    start = to_cell(pose[0], pose[1])
    dist = prev = None
    near = (GX - pose[0]) ** 2 + (GY - pose[1]) ** 2 <= max(2 * RES, 0.15) ** 2
    for hard_r in (HARD, 1, 0):                      # ถ้าติดจนไม่มีทางไป ค่อยลดระยะ hard ลง
        hard = dilate(occ, hard_r)
        hard &= ~(near & ~dilate(occ, min(1, hard_r)))
        soft = dilate(occ, max(INFL, hard_r)) & ~hard
        cost = 1.0 + 0.3 * (np.abs(lo) < 1e-6) + SOFT_COST * soft
        dist, prev = dijkstra(cost, hard, start)
        if np.isfinite(dist).sum() > 20:
            break
    T = dist / V_MAX
    res = {"path": [], "goal": None, "target": None, "reach": np.isfinite(dist)}
    goal = None
    if mode == "CHASE" and snap["tr_x"] is not None:
        tgt = intercept(T, lo, snap["tr_x"], snap["stable"])
        res["target"] = (float(tgt[0]), float(tgt[1]))
        goal = nearest_reachable(dist, to_cell(tgt[0], tgt[1])) or closest_reachable(dist, tgt)
    else:
        with np.errstate(divide="ignore", invalid="ignore"):
            score = box_sum(snap["belief"], BLUR_K) / (T + 1.5)
        score = np.nan_to_num(score, nan=0.0, posinf=0.0)
        far = np.isfinite(dist) & (dist >= MIN_GOAL)          # เป้าต้องไกลพอ จะได้ไม่ถึงแล้วหยุดรอแผนใหม่
        sc_far = np.where(far, score, 0.0)
        if far.any() and sc_far.max() > 1e-12:
            score = sc_far
        best = np.unravel_index(int(np.argmax(score)), score.shape)
        best = (int(best[0]), int(best[1]))
        cur = snap["goal"]
        if cur is not None:
            cx, cy = cell_xy(cur)
            if math.hypot(cx - pose[0], cy - pose[1]) < 0.45 or not np.isfinite(dist[cur]) \
                    or score[best] > 1.3 * score[cur]:
                cur = None
        if cur is None:
            cur = best
        res["goal"] = cur
        goal = cur if np.isfinite(dist[cur]) else \
            (nearest_reachable(dist, cur) or closest_reachable(dist, cell_xy(cur)))
    if goal is not None and np.isfinite(dist[goal]):
        res["path"] = [cell_xy(c) for c in extract(prev, goal)]
    return res


# =============================== BRAIN ===============================
class Brain:
    def __init__(self, async_plan=False, log=print):
        self.log = log
        self.async_plan = async_plan
        self.map = OccMap()
        self.tracker = Tracker()
        self.mf = MotionFilter()
        self.belief = Belief()
        self.mode = "SEARCH"
        self.path, self.goal, self.target_xy = [], None, None
        self.momentum = None
        self.lost_until = 0.0
        self.t_prev = None
        self.belief_acc = 0.0
        self.last_plan_t = -1e9
        self._busy = False
        self.reach = None
        self.last_scan = None
        self.pose = (0.0, 0.0, 0.0)
        self.caught = False
        self.catch_since = None
        self.vel_hist = deque(maxlen=12)
        self.first_scan = True
        self.stable = False
        self.lost_info = None
        self.v_s = np.zeros(2)
        self._vs_init = False
        self.force_plan = False
        self.plan_ms = 0.0
        self.stop_reason = ""
        self.last_cmd = (0.0, 0.0)
        self._prev_mode = "SEARCH"

    # ---------- เมื่อได้ scan ใหม่ ----------
    def on_scan(self, scan, pose, now):
        self.pose, self.last_scan = pose, scan
        dt = 0.0 if self.t_prev is None else min(now - self.t_prev, 1.0)
        self.t_prev = now
        if self.first_scan:
            self.first_scan = False
            if CFG["start_front_prior"]:
                self.belief.set_front_prior(pose, CFG["start_prior_angle_deg"],
                                            CFG["start_prior_min"], CFG["start_prior_max"])
        dets = detect(scan, pose, self.map)
        fast_turn = abs(self.last_cmd[1]) > CFG["max_classify_w"]          # หมุนเร็ว: scan เพี้ยน ไม่ตัดสินรอบนี้
        labels = ["unknown"] * len(dets) if fast_turn else self.mf.classify(dets, now)
        # ก้อนที่โผล่ในที่ที่ "เห็นว่าว่างมาก่อน" ถือว่าเป็นของที่เพิ่งเข้ามา (รวมฝั่งหนีที่หยุดนิ่งอยู่ในที่ที่เราเคยมองเห็นว่าว่าง)
        labels = ["dyn" if d["strong"] else l for d, l in zip(dets, labels)]
        skip = np.zeros(360, bool)
        for d, l in zip(dets, labels):
            if l != "static":                       # ของนิ่งเข้าแมพตามปกติ / ก้อนที่อาจเป็นฝั่งหนีไม่ใส่ลงแมพ
                for i in d["idx"]:
                    for k in (-1, 0, 1):
                        skip[(i + k) % 360] = True
        self.map.update(pose, scan, skip)
        free = ~dilate(self.map.occ(), 1)
        tr = self.tracker
        dyn = [d["c"] for d, l in zip(dets, labels) if l == "dyn"]
        unk = [d["c"] for d, l in zip(dets, labels) if l == "unknown"]
        sta = [d["c"] for d, l in zip(dets, labels) if l == "static"]
        updated = tr.step(dyn if not tr.active else dyn + unk, now,
                          prefer=lambda c: self.belief.b[to_cell(c[0], c[1])], static_cands=sta)
        if tr.active and tr.max_disp < 0.25 and sta and \
                min(math.hypot(c[0] - tr.x[0], c[1] - tr.x[1]) for c in sta) < 0.25:
            self.log("[track] ทิ้งเป้า: เป็นของนิ่ง (ไม่เคยเคลื่อนที่)")      # เช่น reacquire ไปจับกำแพงแคบ ๆ
            tr.reset()
            updated = False
            self._vs_init = False
        if not fast_turn:
            self.mf.push(dets, now)
        if not tr.active and self.lost_info is not None and (dyn or unk):
            self.reacquire(dyn + unk, now)
        if tr.active:
            if updated:
                self.v_s = tr.x[2:].copy() if not self._vs_init else 0.6 * self.v_s + 0.4 * tr.x[2:]
                self._vs_init = True
            else:
                self.v_s = self.v_s * math.exp(-dt / COAST_TAU)
        if updated and tr.active:
            sp = math.hypot(tr.x[2], tr.x[3])
            if sp > 0.05:
                self.vel_hist.append(tr.x[2:] / sp)
        vh = list(self.vel_hist)
        self.stable = len(vh) >= 5 and float(np.hypot(*np.mean(vh, axis=0))) > 0.93

        if tr.active and tr.unseen(now) <= LOST_T:
            self.mode = "CHASE"
            self.lost_until = 0.0
            self.lost_info = None
        else:
            if tr.active:
                self.begin_lost(free, now)
            self.mode = "LOST" if now < self.lost_until else "SEARCH"
            self.belief_acc += dt
            while self.belief_acc >= BELIEF_DT:
                self.belief.predict(free, self.momentum if self.mode == "LOST" else None)
                self.belief_acc -= BELIEF_DT
            self.belief.negative(self.map.visible, free)
            if self.reach is not None:      # ตัดเซลล์ที่เดินไปไม่ถึง (เช่น นอกกำแพง) ออกจาก belief
                nb = self.belief.b * dilate(self.reach, 2)
                if nb.sum() > 1e-6:
                    self.belief.b = nb / nb.sum()

        if self.mode != self._prev_mode:
            self.log(f"[mode] {self._prev_mode} -> {self.mode}")
            self._prev_mode = self.mode
        if CFG["stop_when_caught"] and self.mode == "CHASE":
            d = math.hypot(tr.x[0] - pose[0], tr.x[1] - pose[1])
            if d <= 0.9 * CATCH_R and tr.unseen(now) < 0.3:
                if self.catch_since is None:
                    self.catch_since = now
                elif now - self.catch_since >= 0.5:
                    self.caught = True
            else:
                self.catch_since = None
        self.request_plan(now)

    def begin_lost(self, free, now):
        """โหมด C: เริ่ม belief ใหม่จากตำแหน่ง+ความเร็วสุดท้ายที่เห็น (momentum prior)"""
        tr = self.tracker
        ls = tr.last_seen if tr.last_seen is not None else tr.x
        steps = int(tr.unseen(now) / BELIEF_DT)
        self.belief.set_blob(ls[:2], 0.12)
        sp = math.hypot(ls[2], ls[3])
        self.momentum = np.array([ls[2], ls[3]]) / sp if sp > 0.05 else None
        for _ in range(steps):
            self.belief.predict(free, self.momentum)
        self.lost_until = now + LOST_MOM_T
        self.lost_info = (np.array(ls, float), float(tr.last_seen_t))
        self.goal = None
        self.vel_hist.clear()
        self._vs_init = False
        tr.reset()

    def reacquire(self, cands, now):
        """เจอฝั่งหนีอีกครั้งหลังเพิ่งหายไป: กลับเข้า CHASE ทันทีถ้าก้อนที่เห็นอยู่ในรัศมีที่ฝั่งหนีเดินไปถึงได้
           (ไม่ต้องรอยืนยัน CONFIRM_N scan ใหม่ ซึ่งทำให้หุ่นลังเลหลังฝั่งหนีโผล่พ้นสิ่งกีดขวาง)"""
        ls, t_ls = self.lost_info
        el = now - t_ls
        if el > LOST_MOM_T + LOST_T + 1.0:
            self.lost_info = None
            return
        rad = 0.35 + EV_SPEED * el
        best = min(cands, key=lambda c: math.hypot(c[0] - ls[0], c[1] - ls[1]))
        if math.hypot(best[0] - ls[0], best[1] - ls[1]) <= rad:
            tr = self.tracker
            tr.x = np.array([best[0], best[1], 0.5 * ls[2], 0.5 * ls[3]])
            tr.P = np.diag([0.06 ** 2, 0.06 ** 2, 0.3 ** 2, 0.3 ** 2])
            tr.active, tr.t, tr.last_seen_t, tr.hist = True, now, now, []
            tr.last_seen = tr.x.copy()
            tr.origin, tr.max_disp = tr.x[:2].copy(), 0.0
            self.lost_info = None
            self._vs_init = False
            self.log("[track] เจอฝั่งหนีอีกครั้ง -> กลับ CHASE ทันที")

    # ---------- วางแผน ----------
    def request_plan(self, now):
        if self._busy or (not self.force_plan and now - self.last_plan_t < PLAN_PERIOD):
            return
        self.force_plan = False
        self.last_plan_t = now
        tr = self.tracker
        snap = {"lo": self.map.lo.copy(), "belief": self.belief.b.copy(), "pose": self.pose,
                "mode": self.mode, "goal": self.goal, "stable": self.stable,
                "tr_x": np.concatenate([tr.x[:2], self.v_s]) if tr.active else None}
        if self.async_plan:
            self._busy = True
            threading.Thread(target=self._worker, args=(snap,), daemon=True).start()
        else:
            t0 = time.time()
            self._apply(compute_plan(snap))
            self.plan_ms = (time.time() - t0) * 1000

    def _worker(self, snap):
        try:
            t0 = time.time()
            res = compute_plan(snap)
            self.plan_ms = (time.time() - t0) * 1000
            self._apply(res)
        except Exception as e:                      # noqa
            self.log(f"[plan] error: {e}")
        finally:
            self._busy = False

    def _apply(self, res):
        self.path, self.target_xy, self.reach = res["path"], res["target"], res["reach"]
        if res["goal"] is not None or self.mode != "CHASE":
            self.goal = res["goal"]

    # ---------- คุมการเคลื่อนที่ ----------
    def control(self, pose):
        v, w = self._control(pose)
        self.last_cmd = (v, w)
        return v, w

    def _control(self, pose):
        self.stop_reason = ""
        if self.caught:
            self.stop_reason = "caught"
            return 0.0, 0.0
        if not self.path:
            self.stop_reason = "no_path"
            self.force_plan = True
            return 0.0, 0.0
        pts = np.asarray(self.path)
        d = np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])
        k = int(d.argmin())
        if self.mode != "CHASE" and d[-1] < 0.55:       # ใกล้ถึงเป้าแล้ว: วางแผนเป้าถัดไปล่วงหน้า
            self.force_plan = True
        tgt = pts[-1]
        for j in range(k, len(pts)):
            if d[j] >= LOOKAHEAD:
                tgt = pts[j]
                break
        if d[-1] < 0.06:
            self.stop_reason = "end_of_path"
            self.force_plan = True
            return 0.0, 0.0
        e = wrap(math.atan2(tgt[1] - pose[1], tgt[0] - pose[0]) - pose[2])
        w = float(np.clip(3.0 * e, -W_MAX, W_MAX))
        if abs(e) < 1.3:
            v = V_MAX * max(0.0, math.cos(e))
        else:
            v = 0.0
            self.stop_reason = "turn_in_place"
        if self.last_scan is not None:
            r = np.where(np.isfinite(self.last_scan), self.last_scan, LIDAR_MAX)
            if r[np.r_[0:20, 340:360]].min() < FRONT_SLOW:     # กันชนด้านหน้า
                v *= 0.4
                w += 1.5 if r[30:90].mean() > r[270:330].mean() else -1.5
                self.stop_reason = self.stop_reason or "obstacle_slow"
        w = float(np.clip(w, -W_MAX, W_MAX))
        v = min(v, max(0.0, CFG["wheel_max"] - abs(w) * 0.08))   # ล้อนอกตอนเลี้ยว = v + |w|*0.08 ต้องไม่เกินขีดจำกัดล้อ
        return v, w

    def status(self, pose, now):
        tr = self.tracker
        s = f"mode={self.mode:6s} pose=({pose[0]:+.2f},{pose[1]:+.2f},{math.degrees(pose[2]):+4.0f}deg)"
        if tr.active:
            s += f" evader=({tr.x[0]:+.2f},{tr.x[1]:+.2f}) v=({tr.x[2]:+.2f},{tr.x[3]:+.2f}) unseen={tr.unseen(now):.1f}s"
        if self.mode == "CHASE" and self.target_xy is not None:
            s += f" intercept=({self.target_xy[0]:+.2f},{self.target_xy[1]:+.2f})"
        v, w = self.last_cmd
        s += f" cmd=({v:.2f},{w:+.2f}) path={len(self.path)}pts plan={self.plan_ms:.0f}ms"
        if self.stop_reason:
            s += f" [{self.stop_reason}]"
        if self.caught:
            s += "  *** CAUGHT ***"
        return s


# =============================== ROS 2 NODE ===============================
def scan_to_360(ranges, angle_min, inc, range_min, range_max, flip):
    """แปลง LaserScan -> array 360 ช่อง (index 0 = หน้าหุ่นตามที่อัลกอริทึมถือ, CCW), ค่าไม่ถูกต้อง = inf"""
    r = np.asarray(ranges, float)
    n = len(r)
    d = np.radians(np.arange(360))
    idx = np.round(((d - angle_min) % (2 * np.pi)) / inc).astype(int) % n
    out = r[idx]
    ok = np.isfinite(out) & (out >= max(range_min, LIDAR_MIN)) & (out <= min(range_max, LIDAR_MAX))
    out = np.where(ok, out, np.inf)
    return np.roll(out, -180) if flip else out


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def run_node(cfg):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    flip = bool(CFG["flip_front"])

    class PursuerNode(Node):
        def __init__(self):
            super().__init__("pursuer_brain")
            self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.create_subscription(LaserScan, "/scan", self.on_scan_msg, qos_profile_sensor_data)
            self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
            self.create_timer(0.1, self.loop)
            self.brain = Brain(async_plan=True, log=lambda s: print(s, flush=True))
            self.odom = None
            self.o0 = None
            self.scan = None
            self.scan_t = 0.0
            self.scan_new = False
            self.t0 = time.monotonic()
            self.last_status = 0.0
            self.done = False
            self.warned = False

        def send(self, v, w):
            m = Twist()
            m.linear.x = float(-v if flip else v)
            m.angular.z = float(w)
            self.pub.publish(m)

        def on_odom(self, m):
            p = m.pose.pose
            yaw = yaw_of(p.orientation) + (math.pi if flip else 0.0)
            self.odom = (p.position.x, p.position.y, wrap(yaw))
            if self.o0 is None:
                self.o0 = self.odom

        def on_scan_msg(self, m):
            self.scan = scan_to_360(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, flip)
            self.scan_t = time.monotonic()
            self.scan_new = True

        def alg_pose(self):
            x, y, yaw = self.odom
            dx, dy = x - self.o0[0], y - self.o0[1]
            c, s = math.cos(-self.o0[2]), math.sin(-self.o0[2])
            return (c * dx - s * dy, s * dx + c * dy, wrap(yaw - self.o0[2]))

        def loop(self):
            now = time.monotonic()
            if self.odom is None or self.scan is None:
                if not self.warned and now - self.t0 > 3.0:
                    print("รอข้อมูล /scan และ /odom ...", flush=True)
                    self.warned = True
                self.send(0.0, 0.0)
                return
            pose = self.alg_pose()
            if self.scan_new:
                self.scan_new = False
                self.brain.on_scan(self.scan, pose, self.scan_t)
            v, w = self.brain.control(pose)
            if now - self.scan_t > CFG["scan_timeout"]:
                v = w = 0.0
            if CFG["max_time"] and now - self.t0 > CFG["max_time"]:
                if not self.done:
                    print("ครบ max_time -> หยุด", flush=True)
                self.done = True
            if self.done:
                v = w = 0.0
            self.send(v, w)
            if now - self.last_status >= CFG["status_period"]:
                self.last_status = now
                print(f"[{now - self.t0:5.1f}s] " + self.brain.status(pose, self.scan_t), flush=True)

    def _raise(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _raise)
    signal.signal(signal.SIGHUP, _raise)

    rclpy.init()
    node = PursuerNode()
    print("เริ่มทำงาน: ฝั่งจับ (PURSUER)  กด Ctrl+C เพื่อหยุด" + ("  [flip_front]" if flip else ""), flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("ถูกยกเลิก -> สั่งหยุดหุ่น", flush=True)
    finally:
        for _ in range(5):
            node.send(0.0, 0.0)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


def main():
    cfg = {}
    if "--cfg" in sys.argv:
        with open(sys.argv[sys.argv.index("--cfg") + 1]) as f:
            cfg = json.load(f)
    setup(cfg)
    run_node(cfg)


if __name__ == "__main__":
    main()
'''
REMOTE_PATH = "/tmp/tb3_pursuer.py"
REMOTE_CFG = "/tmp/tb3_pursuer_cfg.json"


# ------------------------------- ฝั่งคอม (Ubuntu) -------------------------------
def current_ssid():
    if shutil.which("iwgetid"):
        try:
            return subprocess.check_output(["iwgetid", "-r"], text=True).strip() or None
        except Exception:                                   # noqa
            pass
    if shutil.which("nmcli"):
        try:
            out = subprocess.check_output(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"], text=True)
            for line in out.splitlines():
                if line.startswith("yes:"):
                    return line.split(":", 1)[1] or None
        except Exception:                                   # noqa
            pass
    return None


def check_pc_wifi():
    ssid, want = current_ssid(), CONFIG["wifi_ssid"]
    if ssid is None:
        print("[PC] ไม่พบการเชื่อมต่อ Wi-Fi (อาจใช้สาย LAN อยู่ หรือไม่มี nmcli/iwgetid)")
    elif ssid == want:
        print(f"[PC] Wi-Fi: {ssid}  OK")
    else:
        print(f"[PC] Wi-Fi ที่ต่ออยู่คือ '{ssid}' แต่ตั้งค่าไว้ '{want}' -> ตรวจว่าเป็นวงเดียวกับหุ่นหรือไม่")


def check_port(host, port, timeout):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError as e:
        print(f"    เชื่อมต่อ {host}:{port} ไม่ได้ ({e})")
        return False


# ------------------------------- SSH -------------------------------
def connect():
    host = CONFIG["host"]
    if host.startswith("TODO"):
        print("[pursuer] ยังไม่ได้กรอก IP ใน CONFIG")
        return None
    print(f"[pursuer] ติดต่อ {host} ...")
    if not check_port(host, CONFIG["ssh_port"], CONFIG["timeout"]):
        print("[pursuer] ไม่พบพอร์ต SSH -> เช็ค: หุ่นเปิดอยู่? อยู่ Wi-Fi เดียวกัน? IP ถูก? SSH เปิดบน Pi?")
        return None
    key = os.path.expanduser(CONFIG["key_file"]) if CONFIG["key_file"] else None
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        cli.connect(CONFIG["host"], port=CONFIG["ssh_port"], username=CONFIG["user"],
                    password=None if key else os.environ.get("TB3_PASSWORD", CONFIG["password"]),
                    key_filename=key, timeout=CONFIG["timeout"],
                    banner_timeout=CONFIG["timeout"], look_for_keys=bool(key), allow_agent=False)
    except paramiko.AuthenticationException:
        print("[pursuer] user/รหัสผ่าน/คีย์ไม่ถูกต้อง")
        return None
    except Exception as e:                                  # noqa
        print(f"[pursuer] SSH ไม่สำเร็จ: {e}")
        return None
    cli.get_transport().set_keepalive(30)       # กันการเชื่อมต่อหลุดตอนค้างไว้
    print("[pursuer] SSH เชื่อมต่อสำเร็จ")
    return cli


def ros_prefix():
    """คำสั่งตั้ง environment ของ ROS 2 ก่อนรันทุกครั้ง"""
    parts = [f"source /opt/ros/{CONFIG['ros_distro']}/setup.bash"]
    if CONFIG["extra_setup"]:
        parts.append(f"[ -f {CONFIG['extra_setup']} ] && source {CONFIG['extra_setup']}")
    parts.append(f"export ROS_DOMAIN_ID={CONFIG['ros_domain_id']}")
    parts.append(f"export TURTLEBOT3_MODEL={CONFIG['turtlebot3_model']}")
    parts.append("export PYTHONIOENCODING=utf-8")
    if CONFIG["lds_model"]:
        parts.append(f"export LDS_MODEL={CONFIG['lds_model']}")
    return "; ".join(parts)


def run(cli, cmd, timeout=20):
    """รันคำสั่งบนหุ่น คืน (exit_code, stdout, stderr)"""
    _, so, se = cli.exec_command(f"bash -lc '{cmd}'", timeout=timeout)
    out, err = so.read().decode(errors="ignore"), se.read().decode(errors="ignore")
    return so.channel.recv_exit_status(), out.strip(), err.strip()


def run_stream(cli, cmd):
    """รันคำสั่งบนหุ่นแล้วแสดงผลแบบ real-time (ใช้ pty: ปิด channel = หุ่นได้รับสัญญาณให้หยุด)"""
    ch = cli.get_transport().open_session()
    ch.get_pty()
    ch.exec_command(f"bash -lc '{cmd}'")
    try:
        while True:
            if ch.recv_ready():
                sys.stdout.write(ch.recv(4096).decode(errors="ignore"))
                sys.stdout.flush()
            elif ch.exit_status_ready():
                while ch.recv_ready():
                    sys.stdout.write(ch.recv(4096).decode(errors="ignore"))
                sys.stdout.flush()
                return ch.recv_exit_status()
            else:
                time.sleep(0.05)
    finally:
        if not ch.closed:
            ch.close()


def inspect_robot(cli):
    _, out, _ = run(cli, "hostname; uname -m; grep PRETTY_NAME /etc/os-release")
    print("[pursuer] ข้อมูลเครื่อง:\n    " + out.replace("\n", "\n    "))
    first = out.splitlines()[0] if out else ""
    if CONFIG["hostname"] and CONFIG["hostname"] != first:
        print(f"[pursuer] เตือน: hostname ไม่ตรงกับที่คาดไว้ ('{CONFIG['hostname']}') อาจต่อผิดตัว")
    _, out, err = run(cli, f"{ros_prefix()}; ros2 --help > /dev/null && echo ROS2_OK")
    print("[pursuer] ROS 2: " + ("พร้อมใช้" if "ROS2_OK" in out else f"ใช้ไม่ได้ -> เช็ค ros_distro/การติดตั้ง ({err[:120]})"))
    _, out, _ = run(cli, "python3 -c \"import numpy; print(numpy.__version__)\"")
    if out.strip():
        print(f"[pursuer] numpy บน Pi: {out.strip()}")
    else:
        print("[pursuer] เตือน: Pi ยังไม่มี numpy (--move จะรันไม่ได้) -> sudo apt install python3-numpy")
    if CONFIG["ros_distro"] not in ("humble", "foxy", "galactic"):
        print("[pursuer] เตือน: ROS 2 รุ่นนี้ TurtleBot3 อาจใช้ TwistStamped ที่ /cmd_vel "
              "สคริปต์ควบคุมอาจไม่ทำงาน")


def bringup(cli):
    # หมายเหตุ: ใช้ "[r]obot" เพื่อไม่ให้ pgrep/pkill จับโดนเชลล์ของตัวเอง
    _, out, _ = run(cli, 'pgrep -f "[r]obot.launch.py" | head -1')
    if out.strip():
        print(f"[pursuer] bringup ทำงานอยู่แล้ว (PID {out.strip()}) -> ไม่สั่งซ้ำ "
              "(ถ้าต้องการเริ่มใหม่ให้รัน --stop ก่อน)")
        return
    run(cli, f"{ros_prefix()}; setsid nohup ros2 launch turtlebot3_bringup robot.launch.py "
             f"> /tmp/tb3_bringup.log 2>&1 < /dev/null &", timeout=10)
    print("[pursuer] สั่ง bringup แล้ว (log: /tmp/tb3_bringup.log บนหุ่น)")


def stop_bringup(cli):
    run(cli, 'pkill -f "[t]urtlebot3_bringup"; pkill -f "[r]obot.launch.py"; pkill -f "[t]urtlebot3_node"')
    print("[pursuer] สั่งหยุด bringup แล้ว")


def check_topics(cli):
    code, out, err = run(cli, f"{ros_prefix()}; ros2 topic list", timeout=25)
    if code != 0:
        print(f"[pursuer] อ่าน topic ไม่สำเร็จ: {err[:150]}")
        return False
    topics = out.split()
    for t in EXPECTED_TOPICS:
        print(f"[pursuer]   {t:10s} {'พบ' if t in topics else 'ไม่พบ (ยังไม่ได้ bringup?)'}")
    return all(t in topics for t in EXPECTED_TOPICS)


def wait_odom(cli, total=30):
    """รอจนได้ข้อมูลจริงจาก /odom (topic มีอยู่ไม่ได้แปลว่ามีข้อมูลไหลมา)"""
    t0 = time.time()
    while time.time() - t0 < total:
        _, out, _ = run(cli, f"{ros_prefix()}; timeout 4 ros2 topic echo /odom --once "
                             "--qos-reliability best_effort 2>/dev/null | head -3", timeout=15)
        if out.strip():
            print("[pursuer] /odom มีข้อมูลไหลเข้ามาแล้ว")
            return True
        print("[pursuer] รอ /odom ... (ยังไม่มีข้อมูล)")
    print("[pursuer] /odom ไม่มีข้อมูลหลังรอ %d วินาที -> log ของ bringup ท้ายสุด:" % total)
    _, out, _ = run(cli, "tail -n 25 /tmp/tb3_bringup.log")
    print("    " + out.replace("\n", "\n    "))
    return False


# ------------------------------- ฝั่งจับ: --move -------------------------------
def stop_motion(cli):
    """ส่งคำสั่งหยุดฉุกเฉิน: ฆ่าสคริปต์ฝั่งจับ + publish ความเร็ว 0"""
    try:
        run(cli, 'pkill -f "[t]b3_pursuer.py"; '
                 f"{ros_prefix()}; "
                 "for i in 1 2 3; do timeout 5 ros2 topic pub --once /cmd_vel "
                 "geometry_msgs/msg/Twist \"{}\" > /dev/null 2>&1; done", timeout=25)
        print("[pursuer] ส่งคำสั่งหยุดหุ่นแล้ว")
    except Exception as e:                                  # noqa
        print("[pursuer] ส่งคำสั่งหยุดไม่สำเร็จ:", e, "-> ให้ยกหุ่น/ปิดสวิตช์เอง")


def run_pursuer(cli):
    p = CONFIG["pursuer"]
    if p["v_max"] > 0.22 or p["w_max"] > 2.84:
        print("[pursuer] v_max/w_max เกินขีดจำกัดของ burger (0.22 m/s, 2.84 rad/s) -> ยกเลิก")
        return
    sftp = cli.open_sftp()
    with sftp.file(REMOTE_PATH, "w") as f:
        f.write(REMOTE_SCRIPT)
    with sftp.file(REMOTE_CFG, "w") as f:
        f.write(json.dumps(p))
    sftp.close()
    print(f"\n[pursuer] อัปโหลดสคริปต์ฝั่งจับแล้ว (v_max={p['v_max']} m/s, catch_radius={p['catch_radius']} m"
          f"{', flip_front' if p['flip_front'] else ''})")
    for i in range(int(CONFIG["start_delay"]), 0, -1):
        print(f"[pursuer] หุ่นจะเริ่มเคลื่อนที่ใน {i} ... (Ctrl+C เพื่อยกเลิก)")
        time.sleep(1)
    code = run_stream(cli, f"{ros_prefix()}; python3 -u {REMOTE_PATH} --cfg {REMOTE_CFG}")
    print(f"[pursuer] สคริปต์ฝั่งจับจบ (exit code {code})")


# ------------------------------- main -------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bringup", action="store_true", help="สั่ง bringup แล้วเช็ค topic")
    ap.add_argument("--stop", action="store_true", help="หยุด bringup แล้วออก")
    ap.add_argument("--move", action="store_true", help="รันอัลกอริทึมฝั่งจับควบคุมหุ่น")
    a = ap.parse_args()

    check_pc_wifi()
    cli = connect()
    if cli is None:
        return
    try:
        inspect_robot(cli)
        if a.stop:
            stop_bringup(cli)
            return
        if a.bringup:
            bringup(cli)
            time.sleep(3)
        ok = check_topics(cli)
        if ok and (a.bringup or a.move):
            ok = wait_odom(cli)
        if a.move:
            if ok:
                run_pursuer(cli)
            else:
                print("[pursuer] topic/odom ไม่พร้อม -> ข้ามการควบคุมหุ่น (ลองใส่ --bringup)")
        print("\n[pursuer] เชื่อมต่อค้างไว้ กด Ctrl+C เพื่อออก")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[pursuer] ได้รับ Ctrl+C")
    finally:
        if a.move:
            stop_motion(cli)
        cli.close()
        print("[pursuer] ปิดการเชื่อมต่อ")


if __name__ == "__main__":
    main()
