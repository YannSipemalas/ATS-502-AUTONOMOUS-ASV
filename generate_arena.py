#!/usr/bin/env python3
"""
Generator arena.world (Gazebo Sim) sesuai denah "ARENA SIMULASI GAZEBO (4m x 30m)".

Arena persegi 4.5 x 4.5 m (desain asli). Hanya POSISI yang mengikuti denah. Origin world = pojok kiri-bawah arena.
Semua koordinat diambil dari piksel gambar referensi lalu dikonversi ke meter:
    X = (px - 153) / 735 * 4.5     Y = (915 - py) / 735 * 4.5

File ini adalah SATU-SATUNYA sumber layout. Selain worlds/arena.world, script
ini juga menulis config/arena_layout.json yang dibaca oleh:
  - scripts/arena_markers_pub.py  (marker RViz)
  - run_rviz.sh                   (static TF world -> odom = pose spawn robot)
sehingga Gazebo, odometri dan RViz selalu memakai pose yang sama.

Jalankan:  python3 generate_arena.py
"""
import json
import math
from pathlib import Path

# ===================== PARAMETER =====================
ROBOT_L = 0.17          # panjang robot (m)
ROBOT_W = 0.16          # lebar robot (m)
BALL_R = 0.02           # radius bola (m), bola ~4 cm (desain asli)
CAPTURE_H = 0.15        # tinggi kotak capture biru/hijau (m)
LINE_W = 0.05           # lebar garis lintasan hitam (m)
START_LINE_H = 0.00005  # ketebalan vertikal garis start (m)

ARENA_X = 4.5           # arena persegi desain asli (30 cm x skala 15)
ARENA_Y = 4.5


def px(u, v):
    """Piksel gambar referensi -> meter world."""
    return (round((u - 153) / 735 * ARENA_X, 3), round((915 - v) / 735 * ARENA_Y, 3))


# Lintasan (searah panah): start kanan-bawah -> naik sisi kanan (berkelok)
# -> atas ke kiri -> turun sisi kiri -> bawah kembali ke start.
TRACK = [px(*p) for p in [
    (790, 775), (795, 700), (785, 650), (760, 615), (770, 570), (795, 530),
    (790, 460), (770, 410), (720, 375), (650, 365), (550, 365), (450, 368),
    (380, 380), (320, 405), (275, 445), (250, 500), (238, 560), (235, 630),
    (235, 700), (240, 765), (300, 795), (400, 825), (480, 855), (580, 875),
    (680, 882), (745, 878),
]]

# Bola: HIJAU di LUAR lintasan, MERAH di DALAM. Sisi kiri & kanan masing-masing
# 3 pasang dengan pola zigzag (mengikuti kelokan jalur), sisi atas 4 pasang.
GREEN = {
    "top_1": px(465, 330), "top_2": px(510, 330), "top_3": px(556, 330), "top_4": px(600, 330),
    "left_1": px(245, 465), "left_2": px(208, 548), "left_3": px(205, 628),
    "right_1": px(803, 532), "right_2": px(773, 598), "right_3": px(797, 680),
    "start_1": px(760, 840), "start_2": px(760, 861), "start_3": px(760, 882),
}
# Bola hijau di samping start dinonaktifkan: tidak dibuat di Gazebo,
# tidak ditampilkan sebagai marker, dan tidak dihitung sebagai rintangan navigasi.
DISABLED_GREEN = {"start_1", "start_2", "start_3"}
RED = {
    "top_1": px(465, 373), "top_2": px(510, 373), "top_3": px(556, 373), "top_4": px(600, 373),
    "left_1": px(290, 472), "left_2": px(253, 550), "left_3": px(248, 622),
    "right_1": px(760, 522), "right_2": px(730, 598), "right_3": px(753, 678),
}

# Jarak pusat-ke-pusat bola pada setiap gate. Pemindahan simetris menjaga
# titik tengah celah tetap pada lintasan; jarak bebas antarsurface menjadi
# 0.70 - 2 * BALL_R = 0.66 m.
PAIR_CENTER_DISTANCE = 0.70
PAIR_HALF = PAIR_CENTER_DISTANCE / 2
for _k in RED:
    (_xg, _yg), (_xr, _yr) = GREEN[_k], RED[_k]
    _mx, _my = (_xg + _xr) / 2, (_yg + _yr) / 2
    _d = math.hypot(_xg - _xr, _yg - _yr)
    _ux, _uy = (_xg - _xr) / _d, (_yg - _yr) / _d
    GREEN[_k] = (round(_mx + _ux * PAIR_HALF, 4), round(_my + _uy * PAIR_HALF, 4))
    RED[_k] = (round(_mx - _ux * PAIR_HALF, 4), round(_my - _uy * PAIR_HALF, 4))
    _actual_distance = math.dist(GREEN[_k], RED[_k])
    if not math.isclose(_actual_distance, PAIR_CENTER_DISTANCE, abs_tol=0.0001):
        raise ValueError(
            f"Jarak pusat bola gate {_k} tidak sesuai: {_actual_distance:.4f} m")


def rect(u0, v0, u1, v1):
    (x0, y0), (x1, y1) = px(u0, v0), px(u1, v1)
    return (x0 + x1) / 2, (y0 + y1) / 2, abs(x1 - x0), abs(y1 - y0)


# Kotak: nama -> (x, y, z_center, sx, sy, sz, rgba)
# Sisi kiri-bawah lintasan: BIRU di atas garis (dalam lintasan), HIJAU di
# bawahnya (dekat garis bawah), persis seperti denah.
_bx, _by, _bsx, _bsy = rect(242, 714, 335, 771)
_gx, _gy, _gsx, _gsy = rect(302, 830, 395, 887)
_sx, _sy = px(790, 800); _ssx = _ssy = 0
BOXES = {
    "box_start_finish":  (_sx, _sy, START_LINE_H / 2, 0.25, 0.03, START_LINE_H, "1 0 0 1"),
    "box_capture_blue":  (_bx, _by, CAPTURE_H / 2, 0.20, 0.20, CAPTURE_H, "0 0.4 1 1"),
    "box_capture_green": (_gx, _gy, CAPTURE_H / 2, 0.20, 0.20, CAPTURE_H, "0 0.8 0 1"),
}

# Spawn robot: DI BELAKANG garis START. Arah lintasan setelah start = +Y
# (naik sisi kanan), jadi "belakang" = -Y. Origin robot = poros roda; badan
# menjorok ~0.20 m ke depan, maka mundur 0.40 m agar seluruh badan belum
# melewati garis (ujung depan robot ~0.20 m di belakang garis).
START_BACK = 0.40
ROBOT_SPAWN = (_sx, _sy - START_BACK, 1.5708)


def gui_section():
    """Gazebo GUI tanpa publisher keyboard yang bisa menimpa /cmd_vel navigasi."""
    return """
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>2.25 -3.5 5.5 0 0.9 1.5708</camera_pose>
        <camera_clip><near>0.01</near><far>100</far></camera_clip>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="CameraTracking" name="Camera Tracking">
        <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui>
      </plugin>
      <plugin filename="WorldControl" name="World control">
        <gz-gui><title>World control</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">72</property><property type="string" key="state">floating</property><property type="double" key="width">121</property><property type="double" key="z">1</property><property type="double" key="x">0</property><property type="double" key="y">0</property></gz-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>true</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">110</property><property type="double" key="width">290</property><property type="string" key="state">floating</property><property type="double" key="z">1</property><property type="double" key="x">0</property><property type="double" key="y">62</property></gz-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree">
        <gz-gui/>
      </plugin>
    </gui>
"""
def track_models():
    """Garis lintasan hitam di lantai (segmen box tipis, tanpa collision)."""
    import math
    s = ""
    for i in range(len(TRACK) - 1):
        (x0, y0), (x1, y1) = TRACK[i], TRACK[i + 1]
        L = math.hypot(x1 - x0, y1 - y0) + LINE_W
        yaw = math.atan2(y1 - y0, x1 - x0)
        s += f"""    <model name="track_{i:02d}"><static>true</static><link name="link"><pose>{(x0+x1)/2:.4f} {(y0+y1)/2:.4f} 0.0005 0 0 {yaw:.4f}</pose>
      <visual name="v"><geometry><box><size>{L:.4f} {LINE_W} 0.001</size></box></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material></visual></link></model>
"""
    return s


def box_model(name, x, y, z, sx, sy, sz, rgba):
    geo = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
    return f"""    <model name="{name}"><static>true</static><link name="link"><pose>{x:.4f} {y:.4f} {z:.8f} 0 0 0</pose>
      <collision name="c">{geo}</collision>
      <visual name="v">{geo}<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>
"""


def ball_model(name, x, y, rgba):
    geo = f"<geometry><sphere><radius>{BALL_R}</radius></sphere></geometry>"
    return f"""    <model name="{name}"><static>true</static><link name="link"><pose>{x:.4f} {y:.4f} {BALL_R} 0 0 0</pose>
      <collision name="c">{geo}</collision>
      <visual name="v">{geo}<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>
"""


def main():
    parts = [f"""<?xml version="1.0" ?>
<!-- AUTO-GENERATED oleh generate_arena.py. Arena {ARENA_X:.1f} x {ARENA_Y:.1f} m
     untuk robot {ROBOT_L*100:.0f} x {ROBOT_W*100:.0f} cm. Origin = pojok kiri bawah. -->
<sdf version="1.9">
  <world name="arena_scaled">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.2 0.2 0.25 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <!-- LANTAI -->
""", gui_section(), box_model("ground_plane_arena", ARENA_X / 2, ARENA_Y / 2, -0.005, ARENA_X, ARENA_Y, 0.01, "0.9 0.9 0.9 1"),
        "\n    <!-- ZONA KOTAK -->\n"]
    for n, b in BOXES.items():
        parts.append(box_model(n, *b))
    parts.append("\n    <!-- BOLA HIJAU (luar lintasan) -->\n")
    for n, (x, y) in GREEN.items():
        if n not in DISABLED_GREEN:
            parts.append(ball_model(f"ball_green_{n}", x, y, "0 1 0 1"))
    parts.append("\n    <!-- BOLA MERAH (dalam lintasan) -->\n")
    for n, (x, y) in RED.items():
        parts.append(ball_model(f"ball_red_{n}", x, y, "1 0 0 1"))
    rx, ry, ryaw = ROBOT_SPAWN
    parts.append(f"""
    <!-- ROBOT -->
    <include>
      <uri>model://robot</uri>
      <name>robot</name>
      <pose>{rx:.4f} {ry:.4f} 0.002 0 0 {ryaw}</pose>
    </include>
""")
    parts.append("  </world>\n</sdf>\n")

    out = Path(__file__).resolve().parent / "worlds" / "arena.world"
    out.write_text("".join(parts))

    layout = {
        "arena": [ARENA_X, ARENA_Y], "ball_r": BALL_R, "track": TRACK,
        "green": {n: point for n, point in GREEN.items() if n not in DISABLED_GREEN},
        "red": RED,
        "boxes": {n: list(b[:6]) + [[float(c) for c in b[6].split()[:3]]] for n, b in BOXES.items()},
        "spawn": {"x": rx, "y": ry, "yaw": ryaw},
    }
    jout = Path(__file__).resolve().parent / "config" / "arena_layout.json"
    jout.write_text(json.dumps(layout, indent=2))

    print(f"Arena      : {ARENA_X:.1f} x {ARENA_Y:.1f} m")
    print(f"Bola       : hijau {len(GREEN) - len(DISABLED_GREEN)} aktif "
          f"({len(DISABLED_GREEN)} dinonaktifkan), merah {len(RED)}")
    print(f"Spawn robot: x={rx:.3f} y={ry:.3f} yaw={ryaw}")
    print(f"Ditulis ke : {out}\n             {jout}")


if __name__ == "__main__":
    main()
