"""Read the versioned simulator socket directly; never map or write hardware SHM."""

import ctypes as c
import os
from pathlib import Path
import select
import socket
import struct
import time

ROOT = Path(__file__).resolve().parents[1]


class State(c.Structure):
    _fields_ = [
        ("published_ns", c.c_uint64),
        ("run_id", c.c_uint64),
        ("sequence", c.c_uint64),
        ("transport_drops", c.c_uint64),
        ("finished", c.c_bool),
        ("time", c.c_double),
        *[
            (name, c.c_double * 3)
            for name in (
                "position_gt",
                "torque_gt",
                "toe_position",
                "contact_force",
                "contact_torque",
                "object_position",
                "velocity_gt",
            )
        ],
        ("object_quaternion", c.c_double * 4),
        ("paused", c.c_bool),
        *[
            (name, c.c_uint64)
            for name in (
                "applied_input_sequence",
                "applied_pause_sequence",
                "applied_step_sequence",
                "perturb_owner",
            )
        ],
        ("perturb_active", c.c_int),
        ("perturb_wrench", c.c_double * 6),
        ("contact", c.c_bool),
        ("contact_event_sequence", c.c_uint64),
        ("contact_event_time", c.c_double),
    ]


class SimulationReader:
    def __init__(self, session):
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket.settimeout(3)
        self.socket.connect(f"/tmp/qhrr05-simulation-{os.getuid()}/{session[1:]}.sock")
        self.socket.send(struct.pack("=QIIII", 0x5148525253494D31, 1, 1, 0, 0))
        self.state = State()

    def read(self, timeout=0.1):
        deadline = time.monotonic() + timeout
        while True:
            ready, _, _ = select.select([self.socket], [], [], max(0, deadline - time.monotonic()))
            if not ready:
                return self.state
            packet = self.socket.recv(4096)
            if not packet:
                return self.state
            magic, version, kind, size, _ = struct.unpack("=QIIII", packet[:24])
            assert magic == 0x5148525253494D31 and version == 1 and kind == 3
            assert len(packet) == 24 + size and size == c.sizeof(State), (size, c.sizeof(State))
            self.state = State.from_buffer_copy(packet[24:])
            if time.monotonic() >= deadline:
                return self.state


class GlCamera(c.Structure):
    _fields_ = [
        ("pos", c.c_float * 3),
        ("forward", c.c_float * 3),
        ("up", c.c_float * 3),
        *[(name, c.c_float) for name in ("center", "width", "bottom", "top", "near", "far")],
        ("orthographic", c.c_int),
    ]


class Projection:
    def __init__(self):
        self.library = c.CDLL(str(ROOT / "build-filament/native/lib/libmujoco.so.3.16.0"))
        lib = self.library
        lib.mj_loadXML.argtypes = [c.c_char_p, c.c_void_p, c.c_void_p, c.c_int]
        lib.mj_loadXML.restype = c.c_void_p
        lib.mj_makeData.argtypes = [c.c_void_p]
        lib.mj_makeData.restype = c.c_void_p
        self.model = lib.mj_loadXML(
            str(ROOT / "models/qhrr-05/scene_pbr.xml").encode(), None, None, 0
        )
        assert self.model
        self.data = lib.mj_makeData(self.model)
        for name in ("mj_kinematics", "mj_comPos", "mj_camlight"):
            getattr(lib, name).argtypes = [c.c_void_p, c.c_void_p]
        lib.mj_setState.argtypes = [c.c_void_p, c.c_void_p, c.c_void_p, c.c_uint]
        lib.mj_name2id.argtypes = [c.c_void_p, c.c_int, c.c_char_p]
        lib.mj_local2Global.argtypes = [c.c_void_p] * 5 + [c.c_int, c.c_int]
        lib.mj_ray.argtypes = [c.c_void_p, c.c_void_p, c.c_void_p, c.c_void_p,
                              c.c_void_p, c.c_bool, c.c_int, c.c_void_p, c.c_void_p]
        lib.mj_ray.restype = c.c_double
        lib.mj_id2name.argtypes = [c.c_void_p, c.c_int, c.c_int]
        lib.mj_id2name.restype = c.c_char_p
        lib.mjv_defaultFreeCamera.argtypes = [c.c_void_p, c.c_void_p]
        lib.mjv_camera2GLCamera.argtypes = [c.c_void_p, c.c_void_p, c.c_void_p]
        lib.mjv_camera2GLCamera.restype = GlCamera
        self.camera = c.create_string_buffer(512)
        lib.mjv_defaultFreeCamera(self.model, self.camera)

    def pixel(self, state, width, height, body=None):
        lib = self.library
        lib.mj_setState(self.model, self.data, state.position_gt, 2)  # mjSTATE_QPOS
        lib.mj_setState(self.model, self.data, state.object_position, 1 << 10)
        lib.mj_setState(self.model, self.data, state.object_quaternion, 1 << 11)
        lib.mj_kinematics(self.model, self.data)
        lib.mj_comPos(self.model, self.data)
        lib.mj_camlight(self.model, self.data)
        camera = lib.mjv_camera2GLCamera(self.model, self.data, self.camera)
        if body:
            body_id = lib.mj_name2id(self.model, 1, body.encode())  # mjOBJ_BODY
            assert body_id > 0
            point = (c.c_double * 3)()
            matrix = (c.c_double * 9)()
            # A fixed near-foot point can be occluded by the contact box in the new CAD.
            # Pick a point whose camera ray actually hits the requested visible link.
            quat = (c.c_double * 4)(1, 0, 0, 0)
            origin = (c.c_double * 3)(*camera.pos)
            groups = (c.c_ubyte * 6)(1, 1, 1, 0, 1, 1)
            expected_geom = b"qhrr_calf_visual"
            found = False
            for z in (-.08, -.12, -.04, -.16, -.20):
                for xoffset in (0, -.01, .01):
                    local = (c.c_double * 3)(xoffset, 0, z)
                    lib.mj_local2Global(self.data, point, matrix, local, quat, body_id, 0)
                    ray = (c.c_double * 3)(*(point[i] - origin[i] for i in range(3)))
                    hit = c.c_int(-1)
                    distance = lib.mj_ray(self.model, self.data, origin, ray, groups,
                                          True, -1, c.byref(hit), None)
                    if distance >= 0 and lib.mj_id2name(self.model, 5, hit.value) == expected_geom:
                        found = True
                        break
                if found:
                    break
            assert found, f"No unoccluded pick point on {body}"
        else:
            point = state.object_position
        rel = [point[i] - camera.pos[i] for i in range(3)]
        right = [
            camera.forward[1] * camera.up[2] - camera.forward[2] * camera.up[1],
            camera.forward[2] * camera.up[0] - camera.forward[0] * camera.up[2],
            camera.forward[0] * camera.up[1] - camera.forward[1] * camera.up[0],
        ]
        dot = lambda a, b: sum(x * y for x, y in zip(a, b))
        depth = dot(rel, camera.forward)
        half_y = (camera.top - camera.bottom) / 2
        half_x = half_y * width / height
        x = (1 + (dot(rel, right) * camera.near / depth - camera.center) / half_x) * width / 2
        y = (
            (
                1
                - (dot(rel, camera.up) * camera.near / depth - (camera.top + camera.bottom) / 2)
                / half_y
            )
            * height
            / 2
        )
        assert 0 < x < width and 0 < y < height, (x, y, width, height)
        return round(x), round(y)
