# BSD 3-Clause License
#
# Copyright (c) 2026 JLG Industries, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the JLG Industries, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import pytest

from vda5050_connector_py.geo_conversion import GeoConversionError
from vda5050_connector_py.geo_conversion import WireCoordinateTranslator
from vda5050_msgs.msg import ControlPoint
from vda5050_msgs.msg import EdgeState
from vda5050_msgs.msg import NodeState
from vda5050_msgs.msg import OrderState
from vda5050_msgs.msg import Visualization

LON0 = -77.373513
LAT0 = 39.439381
METERS_PER_DEG = 100000.0
# Map axes rotated 0.1 rad counter-clockwise from ENU.
MAP_YAW = 0.1


class FakeConverter:
    """Planar stand-in for navsat_transform: map = R(-MAP_YAW) * scaled ENU offset."""

    def __init__(self, ready=True):
        self.ready = ready

    def lon_lat_to_map(self, lon_lats):
        if not self.ready:
            raise GeoConversionError("not ready")
        points = []
        for lon, lat in lon_lats:
            e = (lon - LON0) * METERS_PER_DEG
            n = (lat - LAT0) * METERS_PER_DEG
            c, s = math.cos(-MAP_YAW), math.sin(-MAP_YAW)
            points.append((c * e - s * n, s * e + c * n))
        return points, -MAP_YAW

    def map_to_lon_lat(self, x, y):
        if not self.ready:
            raise GeoConversionError("not ready")
        c, s = math.cos(MAP_YAW), math.sin(MAP_YAW)
        e, n = c * x - s * y, s * x + c * y
        return LON0 + e / METERS_PER_DEG, LAT0 + n / METERS_PER_DEG

    def enu_to_map_yaw(self, lon, lat):
        return -MAP_YAW


def get_order():
    return {
        "order_id": "order_1",
        "order_update_id": 0,
        "nodes": [
            {
                "node_id": "n1",
                "sequence_id": 0,
                "released": True,
                "node_position": {
                    "x": LON0 + 0.0001,
                    "y": LAT0,
                    "theta": 0,
                    "allowed_deviation_x_y": 0.5,
                    "map_id": "WGS84",
                },
                "actions": [],
            },
            {
                "node_id": "n2",
                "sequence_id": 2,
                "released": True,
                "actions": [],
            },
        ],
        "edges": [
            {
                "edge_id": "e1",
                "sequence_id": 1,
                "released": True,
                "start_node_id": "n1",
                "end_node_id": "n2",
                "orientation": 0.5,
                "orientation_type": "GLOBAL",
                "length": 12,
                "trajectory": {
                    "degree": 1,
                    "knot_vector": [0, 0, 1, 1],
                    "control_points": [
                        {"x": LON0, "y": LAT0 + 0.0001, "weight": 1.0},
                        {"x": LON0, "y": LAT0},
                    ],
                },
                "actions": [],
            },
        ],
    }


def test_order_to_map_converts_positions_and_headings():
    translator = WireCoordinateTranslator(FakeConverter())
    original = get_order()

    order = translator.order_to_map(original)

    position = order["nodes"][0]["node_position"]
    assert position["x"] == pytest.approx(10.0 * math.cos(MAP_YAW))
    assert position["y"] == pytest.approx(-10.0 * math.sin(MAP_YAW))
    assert position["theta"] == pytest.approx(-MAP_YAW)
    assert position["allowed_deviation_x_y"] == 0.5
    assert "node_position" not in order["nodes"][1]

    cp = order["edges"][0]["trajectory"]["control_points"][0]
    assert cp["x"] == pytest.approx(10.0 * math.sin(MAP_YAW))
    assert cp["y"] == pytest.approx(10.0 * math.cos(MAP_YAW))
    assert cp["weight"] == 1.0
    assert order["edges"][0]["orientation"] == pytest.approx(0.5 - MAP_YAW)
    assert order["edges"][0]["length"] == 12

    assert original == get_order()


def test_order_to_map_tangential_orientation_unchanged():
    order = get_order()
    order["edges"][0]["orientation_type"] = "TANGENTIAL"

    converted = WireCoordinateTranslator(FakeConverter()).order_to_map(order)

    assert converted["edges"][0]["orientation"] == 0.5


def test_order_to_map_without_coordinates_is_passthrough():
    order = get_order()
    del order["nodes"][0]["node_position"]
    del order["edges"][0]["trajectory"]
    order["edges"][0]["orientation_type"] = "TANGENTIAL"

    converter = FakeConverter(ready=False)
    assert WireCoordinateTranslator(converter).order_to_map(order) == order


def test_order_to_map_rejects_when_not_ready():
    with pytest.raises(GeoConversionError):
        WireCoordinateTranslator(FakeConverter(ready=False)).order_to_map(get_order())


def test_instant_actions_init_position_to_map():
    instant_actions = {
        "actions": [
            {
                "action_type": "initPosition",
                "action_id": "a1",
                "blocking_type": "HARD",
                "action_parameters": [
                    {"key": "x", "value": LON0 + 0.0001},
                    {"key": "y", "value": str(LAT0)},
                    {"key": "theta", "value": 1.0},
                    {"key": "mapId", "value": "WGS84"},
                ],
            },
            {
                "action_type": "startPause",
                "action_id": "a2",
                "blocking_type": "HARD",
                "action_parameters": [{"key": "x", "value": 1.0}, {"key": "y", "value": 2.0}],
            },
        ]
    }

    converted = WireCoordinateTranslator(FakeConverter()).instant_actions_to_map(
        instant_actions
    )

    params = {p["key"]: p["value"] for p in converted["actions"][0]["action_parameters"]}
    assert params["x"] == pytest.approx(10.0 * math.cos(MAP_YAW))
    assert params["y"] == pytest.approx(-10.0 * math.sin(MAP_YAW))
    assert params["theta"] == pytest.approx(1.0 - MAP_YAW)
    assert params["mapId"] == "WGS84"
    assert converted["actions"][1] == instant_actions["actions"][1]


def test_state_to_wire_round_trip_restores_original_order_positions():
    translator = WireCoordinateTranslator(FakeConverter(), map_id="WGS84")
    wire_order = get_order()
    map_order = translator.order_to_map(wire_order)

    state = OrderState(order_id="order_1")
    state.agv_position.position_initialized = True
    state.agv_position.x = 10.0 * math.cos(MAP_YAW)
    state.agv_position.y = -10.0 * math.sin(MAP_YAW)
    state.agv_position.theta = -MAP_YAW
    state.agv_position.map_id = "lidar_map"

    node_state = NodeState(node_id="n1", sequence_id=0)
    map_position = map_order["nodes"][0]["node_position"]
    node_state.node_position.x = map_position["x"]
    node_state.node_position.y = map_position["y"]
    node_state.node_position.theta = map_position["theta"]
    state.node_states.append(node_state)

    edge_state = EdgeState(edge_id="e1", sequence_id=1)
    edge_state.trajectory.control_points = [
        ControlPoint(x=cp["x"], y=cp["y"])
        for cp in map_order["edges"][0]["trajectory"]["control_points"]
    ]
    state.edge_states.append(edge_state)

    wire_state = translator.state_to_wire(state)

    assert wire_state.agv_position.x == pytest.approx(LON0 + 0.0001)
    assert wire_state.agv_position.y == pytest.approx(LAT0)
    assert wire_state.agv_position.theta == pytest.approx(0.0)
    assert wire_state.agv_position.map_id == "WGS84"

    wire_node = wire_order["nodes"][0]["node_position"]
    assert wire_state.node_states[0].node_position.x == wire_node["x"]
    assert wire_state.node_states[0].node_position.y == wire_node["y"]
    assert wire_state.node_states[0].node_position.theta == 0.0

    wire_cps = wire_order["edges"][0]["trajectory"]["control_points"]
    assert wire_state.edge_states[0].trajectory.control_points[0].x == wire_cps[0]["x"]
    assert wire_state.edge_states[0].trajectory.control_points[0].y == wire_cps[0]["y"]

    # Input message must not be modified.
    assert state.agv_position.map_id == "lidar_map"


def test_state_to_wire_unknown_node_is_converted():
    translator = WireCoordinateTranslator(FakeConverter())
    state = OrderState(order_id="unknown")
    node_state = NodeState(node_id="n1", sequence_id=0)
    node_state.node_position.x = 10.0 * math.cos(MAP_YAW)
    node_state.node_position.y = -10.0 * math.sin(MAP_YAW)
    state.node_states.append(node_state)

    wire_state = translator.state_to_wire(state)

    assert wire_state.node_states[0].node_position.x == pytest.approx(LON0 + 0.0001)
    assert wire_state.node_states[0].node_position.y == pytest.approx(LAT0)
    assert wire_state.node_states[0].node_position.theta == pytest.approx(MAP_YAW)


def test_state_to_wire_not_ready_reports_uninitialized_position():
    translator = WireCoordinateTranslator(FakeConverter(ready=False))
    state = OrderState()
    state.agv_position.position_initialized = True
    state.agv_position.localization_score = 0.9
    state.agv_position.x = 12.0
    state.agv_position.y = 34.0

    wire_state = translator.state_to_wire(state)

    assert not wire_state.agv_position.position_initialized
    assert wire_state.agv_position.localization_score == 0.0
    assert wire_state.agv_position.x == 0.0
    assert wire_state.agv_position.y == 0.0


def test_visualization_to_wire():
    translator = WireCoordinateTranslator(FakeConverter())
    visualization = Visualization()
    visualization.agv_position.position_initialized = True
    visualization.agv_position.x = 0.0
    visualization.agv_position.y = 0.0
    visualization.agv_position.theta = math.pi - 0.05

    wire = translator.visualization_to_wire(visualization)

    assert wire.agv_position.x == pytest.approx(LON0)
    assert wire.agv_position.y == pytest.approx(LAT0)
    assert wire.agv_position.theta == pytest.approx(-math.pi + 0.05)
