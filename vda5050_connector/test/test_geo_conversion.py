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
from types import SimpleNamespace

import pytest

from vda5050_connector_py.geo_conversion import GeoConversionError
from vda5050_connector_py.geo_conversion import GeoCoordinateTranslator
from vda5050_connector_py.geo_conversion import NavSatGeoConverter
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
# Exaggerated stand-in for UTM grid convergence, which varies with longitude.
CONVERGENCE_PER_DEG = 0.5


def fake_enu_to_map_yaw(lon, lat):
    return -MAP_YAW - CONVERGENCE_PER_DEG * (lon - LON0)


class FakeConverter:
    """Planar stand-in for navsat_transform: map = R(-MAP_YAW) * scaled ENU offset."""

    def __init__(self, ready=True):
        self.ready = ready

    def lon_lat_to_map(self, lon_lats):
        if not self.ready:
            raise GeoConversionError("not ready")
        poses = []
        for lon, lat in lon_lats:
            e = (lon - LON0) * METERS_PER_DEG
            n = (lat - LAT0) * METERS_PER_DEG
            c, s = math.cos(-MAP_YAW), math.sin(-MAP_YAW)
            poses.append((c * e - s * n, s * e + c * n, fake_enu_to_map_yaw(lon, lat)))
        return poses

    def map_to_lon_lat(self, x, y):
        if not self.ready:
            raise GeoConversionError("not ready")
        c, s = math.cos(MAP_YAW), math.sin(MAP_YAW)
        e, n = c * x - s * y, s * x + c * y
        return LON0 + e / METERS_PER_DEG, LAT0 + n / METERS_PER_DEG

    def enu_to_map_yaw(self, lon, lat):
        return fake_enu_to_map_yaw(lon, lat)


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
    translator = GeoCoordinateTranslator(FakeConverter())
    original = get_order()
    n1_yaw = fake_enu_to_map_yaw(LON0 + 0.0001, LAT0)

    order = translator.order_to_map(original)

    position = order["nodes"][0]["node_position"]
    assert position["x"] == pytest.approx(10.0 * math.cos(MAP_YAW))
    assert position["y"] == pytest.approx(-10.0 * math.sin(MAP_YAW))
    assert position["theta"] == pytest.approx(n1_yaw)
    assert position["allowed_deviation_x_y"] == 0.5
    assert "node_position" not in order["nodes"][1]

    cp = order["edges"][0]["trajectory"]["control_points"][0]
    assert cp["x"] == pytest.approx(10.0 * math.sin(MAP_YAW))
    assert cp["y"] == pytest.approx(10.0 * math.cos(MAP_YAW))
    assert cp["weight"] == 1.0
    # GLOBAL edge orientation uses the yaw offset at its start node.
    assert order["edges"][0]["orientation"] == pytest.approx(0.5 + n1_yaw)
    assert order["edges"][0]["length"] == 12

    assert original == get_order()


def make_node(node_id, sequence_id, lon):
    return {
        "node_id": node_id,
        "sequence_id": sequence_id,
        "released": True,
        "node_position": {"x": lon, "y": LAT0, "theta": 1.0, "map_id": "WGS84"},
        "actions": [],
    }


def make_edge(edge_id, sequence_id, start_node_id, end_node_id, control_point_lons=()):
    edge = {
        "edge_id": edge_id,
        "sequence_id": sequence_id,
        "released": True,
        "start_node_id": start_node_id,
        "end_node_id": end_node_id,
        "actions": [],
    }
    if control_point_lons:
        edge["trajectory"] = {
            "degree": 1,
            "knot_vector": [0, 0, 1, 1],
            "control_points": [{"x": lon, "y": LAT0} for lon in control_point_lons],
        }
    return edge


def make_order(order_id, order_update_id, nodes, edges):
    return {
        "order_id": order_id,
        "order_update_id": order_update_id,
        "nodes": nodes,
        "edges": edges,
    }


def make_state(order_id, order_update_id, node_keys=(), edge_keys=()):
    state = OrderState(order_id=order_id, order_update_id=order_update_id)
    state.node_states = [NodeState(node_id=n, sequence_id=s) for n, s in node_keys]
    state.edge_states = [EdgeState(edge_id=e, sequence_id=s) for e, s in edge_keys]
    for edge_state in state.edge_states:
        edge_state.trajectory.control_points = [ControlPoint(), ControlPoint()]
    return state


def test_order_update_converts_stitch_node_identically():
    base = make_order(
        "order_1", 0, [make_node("A", 0, LON0), make_node("B", 2, LON0 + 0.001)],
        [make_edge("e1", 1, "A", "B")],
    )
    update = make_order(
        "order_1", 1, [make_node("B", 2, LON0 + 0.001), make_node("C", 4, LON0 + 0.002)],
        [make_edge("e2", 3, "B", "C")],
    )
    translator = GeoCoordinateTranslator(FakeConverter())

    base_b = translator.order_to_map(base)["nodes"][1]["node_position"]
    update_b = translator.order_to_map(update)["nodes"][0]["node_position"]

    # The controller's stitching check compares node positions exactly.
    assert update_b == base_b


# In the state tests below the converter is disabled after the orders are translated, so a
# reported coordinate can only come from a stored original (fallback conversion yields zeros).


def test_rejected_stitch_update_does_not_replace_accepted_positions():
    converter = FakeConverter()
    translator = GeoCoordinateTranslator(converter)
    translator.order_to_map(make_order(
        "o", 0, [make_node("A", 0, LON0), make_node("B", 2, LON0 + 0.001)],
        [make_edge("e1", 1, "A", "B")],
    ))
    translator.state_to_wgs84(make_state("o", 0))
    # Moved stitch node: the controller rejects this update.
    translator.order_to_map(make_order(
        "o", 1, [make_node("B", 2, LON0 + 0.0011), make_node("C", 4, LON0 + 0.002)],
        [make_edge("e2", 3, "B", "C")],
    ))
    converter.ready = False

    state = translator.state_to_wgs84(make_state("o", 0, [("B", 2)]))

    assert state.node_states[0].node_position.x == LON0 + 0.001


def test_corrected_update_with_same_id_is_used_once_accepted():
    converter = FakeConverter()
    translator = GeoCoordinateTranslator(converter)
    translator.order_to_map(make_order(
        "o", 0, [make_node("A", 0, LON0), make_node("B", 2, LON0 + 0.001)],
        [make_edge("e1", 1, "A", "B")],
    ))
    translator.state_to_wgs84(make_state("o", 0))
    translator.order_to_map(make_order(
        "o", 1, [make_node("B", 2, LON0 + 0.0011), make_node("C", 4, LON0 + 0.003)],
        [make_edge("e2", 3, "B", "C")],
    ))
    translator.order_to_map(make_order(
        "o", 1, [make_node("B", 2, LON0 + 0.001), make_node("C", 4, LON0 + 0.002)],
        [make_edge("e2", 3, "B", "C")],
    ))
    converter.ready = False

    state = translator.state_to_wgs84(make_state("o", 1, [("B", 2), ("C", 4)]))

    assert state.node_states[0].node_position.x == LON0 + 0.001
    assert state.node_states[1].node_position.x == LON0 + 0.002


def test_stale_update_does_not_replace_accepted_positions():
    converter = FakeConverter()
    translator = GeoCoordinateTranslator(converter)
    translator.order_to_map(make_order(
        "o", 2, [make_node("A", 0, LON0), make_node("H", 2, LON0 + 0.001)],
        [make_edge("e1", 1, "A", "H")],
    ))
    translator.state_to_wgs84(make_state("o", 2))
    translator.order_to_map(make_order(
        "o", 1, [make_node("A", 0, LON0), make_node("H", 2, LON0 + 0.005)],
        [make_edge("e1", 1, "A", "H")],
    ))
    converter.ready = False

    state = translator.state_to_wgs84(make_state("o", 2, [("H", 2)]))

    assert state.node_states[0].node_position.x == LON0 + 0.001


def test_duplicate_update_id_does_not_replace_accepted_positions():
    converter = FakeConverter()
    translator = GeoCoordinateTranslator(converter)
    translator.order_to_map(make_order(
        "o", 1, [make_node("A", 0, LON0), make_node("B", 2, LON0 + 0.001)],
        [make_edge("e1", 1, "A", "B")],
    ))
    translator.state_to_wgs84(make_state("o", 1))
    translator.order_to_map(make_order(
        "o", 1, [make_node("A", 0, LON0), make_node("B", 2, LON0 + 0.005)],
        [make_edge("e1", 1, "A", "B")],
    ))
    converter.ready = False

    state = translator.state_to_wgs84(make_state("o", 1, [("B", 2)]))

    assert state.node_states[0].node_position.x == LON0 + 0.001


def test_rejected_new_orders_do_not_evict_active_order():
    converter = FakeConverter()
    translator = GeoCoordinateTranslator(converter)
    translator.order_to_map(make_order(
        "active", 0, [make_node("A", 0, LON0), make_node("B", 2, LON0 + 0.001)],
        [make_edge("e1", 1, "A", "B")],
    ))
    translator.state_to_wgs84(make_state("active", 0))
    for i in range(10):
        translator.order_to_map(make_order(
            f"rejected_{i}", 0, [make_node("B", 0, LON0 + 0.004)], [],
        ))
    converter.ready = False

    state = translator.state_to_wgs84(make_state("active", 0, [("B", 2)]))

    assert state.node_states[0].node_position.x == LON0 + 0.001


def test_stitched_base_entries_are_preserved():
    converter = FakeConverter()
    translator = GeoCoordinateTranslator(converter)
    translator.order_to_map(make_order(
        "o", 0, [make_node("A", 0, LON0), make_node("B", 2, LON0 + 0.001)],
        [make_edge("e1", 1, "A", "B", (LON0, LON0 + 0.001))],
    ))
    translator.state_to_wgs84(make_state("o", 0))
    translator.order_to_map(make_order(
        "o", 1, [make_node("B", 2, LON0 + 0.001), make_node("C", 4, LON0 + 0.002)],
        [make_edge("e2", 3, "B", "C", (LON0 + 0.001, LON0 + 0.002))],
    ))
    converter.ready = False

    state = translator.state_to_wgs84(
        make_state("o", 1, [("A", 0), ("B", 2), ("C", 4)], [("e1", 1), ("e2", 3)])
    )

    assert [n.node_position.x for n in state.node_states] == [
        LON0, LON0 + 0.001, LON0 + 0.002
    ]
    assert [cp.x for cp in state.edge_states[0].trajectory.control_points] == [
        LON0, LON0 + 0.001
    ]
    assert [cp.x for cp in state.edge_states[1].trajectory.control_points] == [
        LON0 + 0.001, LON0 + 0.002
    ]


def test_accepted_entries_no_longer_reported_are_dropped():
    converter = FakeConverter()
    translator = GeoCoordinateTranslator(converter)
    translator.order_to_map(make_order(
        "o", 0, [make_node("A", 0, LON0), make_node("B", 2, LON0 + 0.001)],
        [make_edge("e1", 1, "A", "B")],
    ))
    translator.state_to_wgs84(make_state("o", 0, [("A", 0), ("B", 2)], [("e1", 1)]))
    translator.order_to_map(make_order(
        "o", 1, [make_node("B", 2, LON0 + 0.001), make_node("C", 4, LON0 + 0.002)],
        [make_edge("e2", 3, "B", "C")],
    ))
    translator.state_to_wgs84(make_state("o", 1, [("C", 4)], [("e2", 3)]))
    translator.order_to_map(make_order(
        "o", 2, [make_node("C", 4, LON0 + 0.002), make_node("D", 6, LON0 + 0.003)],
        [make_edge("e3", 5, "C", "D")],
    ))
    converter.ready = False

    state = translator.state_to_wgs84(make_state("o", 2, [("D", 6)], [("e3", 5)]))

    assert state.node_states[0].node_position.x == LON0 + 0.003
    # Only the newest update and the entries still reported remain.
    assert set(translator._accepted_order["nodes"]) == {("C", 4), ("D", 6)}
    assert set(translator._accepted_order["edges"]) == {("e3", 5)}


def test_navsat_converter_measures_yaw_on_every_call():
    converter = NavSatGeoConverter.__new__(NavSatGeoConverter)
    yaws = iter([0.1, 0.2])
    converter.lon_lat_to_map = lambda lon_lats: [(0.0, 0.0, next(yaws))]

    assert converter.enu_to_map_yaw(LON0, LAT0) == 0.1
    # e.g. after a SetDatum call the next message must use the new transform.
    assert converter.enu_to_map_yaw(LON0, LAT0) == 0.2


def test_nan_to_ll_result_reports_position_as_not_initialized():
    converter = NavSatGeoConverter.__new__(NavSatGeoConverter)
    converter._to_ll_client = None
    converter._to_ll_type = SimpleNamespace(Request=SimpleNamespace)
    converter._point_type = SimpleNamespace
    converter._call = lambda client, request: SimpleNamespace(
        ll_point=SimpleNamespace(latitude=math.nan, longitude=math.nan)
    )
    visualization = Visualization()
    visualization.agv_position.position_initialized = True
    visualization.agv_position.x = math.nan
    visualization.agv_position.y = math.nan

    with pytest.raises(GeoConversionError):
        converter.map_to_lon_lat(math.nan, math.nan)
    wgs84 = GeoCoordinateTranslator(converter).visualization_to_wgs84(visualization)

    assert not wgs84.agv_position.position_initialized
    assert wgs84.agv_position.x == 0.0
    assert wgs84.agv_position.y == 0.0


class FakeFuture:
    def __init__(self, response, completes):
        self._response = response
        self._completes = completes

    def add_done_callback(self, callback):
        if self._completes:
            callback(self)

    def result(self):
        return self._response


class FakeClient:
    """Stand-in for an rclpy service client; `respond` maps a request to a response."""

    srv_name = "fake_service"

    def __init__(self, respond=lambda request: None, ready=True, completes=True):
        self.respond = respond
        self.ready = ready
        self.completes = completes
        self.requests = []
        self.removed = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return FakeFuture(self.respond(request), self.completes)

    def remove_pending_request(self, future):
        self.removed.append(future)


def planar_from_ll_array(request):
    """Answer fromLLArray like navsat_transform would for the FakeConverter's map frame."""
    c, s = math.cos(-MAP_YAW), math.sin(-MAP_YAW)
    map_points = []
    for p in request.ll_points:
        e = (p.longitude - LON0) * METERS_PER_DEG
        n = (p.latitude - LAT0) * METERS_PER_DEG
        map_points.append(SimpleNamespace(x=c * e - s * n, y=s * e + c * n, z=0.0))
    return SimpleNamespace(map_points=map_points)


def make_navsat_converter(from_ll_array_client, timeout_sec=1.0):
    converter = NavSatGeoConverter.__new__(NavSatGeoConverter)
    converter._timeout_sec = timeout_sec
    converter._geo_point_type = SimpleNamespace
    converter._from_ll_array_type = SimpleNamespace(Request=SimpleNamespace)
    converter._from_ll_array_client = from_ll_array_client
    return converter


def test_call_returns_service_response():
    client = FakeClient(respond=lambda request: "response")
    converter = make_navsat_converter(client)

    assert converter._call(client, "request") == "response"
    assert client.requests == ["request"]


def test_call_raises_when_service_unavailable():
    client = FakeClient(ready=False)

    with pytest.raises(GeoConversionError, match="not available"):
        make_navsat_converter(client)._call(client, "request")
    assert client.requests == []


def test_call_times_out_and_drops_pending_request():
    client = FakeClient(completes=False)

    with pytest.raises(GeoConversionError, match="did not respond"):
        make_navsat_converter(client, timeout_sec=0.01)._call(client, "request")
    assert len(client.removed) == 1


def test_call_raises_when_service_call_fails():
    client = FakeClient(respond=lambda request: None)

    with pytest.raises(GeoConversionError, match="call failed"):
        make_navsat_converter(client)._call(client, "request")


def test_lon_lat_to_map_converts_points_with_per_point_yaw():
    client = FakeClient(respond=planar_from_ll_array)
    converter = make_navsat_converter(client)

    poses = converter.lon_lat_to_map([(LON0 + 0.0001, LAT0), (LON0, LAT0 + 0.0001)])

    # One request: the points followed by a north probe per point.
    request = client.requests[0]
    sent = [value for p in request.ll_points for value in (p.longitude, p.latitude)]
    assert sent == pytest.approx([
        LON0 + 0.0001, LAT0,
        LON0, LAT0 + 0.0001,
        LON0 + 0.0001, LAT0 + 0.0001,
        LON0, LAT0 + 0.0002,
    ])
    assert poses[0] == pytest.approx(
        (10.0 * math.cos(MAP_YAW), -10.0 * math.sin(MAP_YAW), -MAP_YAW)
    )
    assert poses[1] == pytest.approx(
        (10.0 * math.sin(MAP_YAW), 10.0 * math.cos(MAP_YAW), -MAP_YAW)
    )


@pytest.mark.parametrize(
    "response",
    [
        SimpleNamespace(map_points=[]),
        SimpleNamespace(map_points=[SimpleNamespace(x=0.0, y=0.0, z=0.0)] * 2),
    ],
    ids=["empty", "zeros"],
)
def test_lon_lat_to_map_raises_when_navsat_not_ready(response):
    converter = make_navsat_converter(FakeClient(respond=lambda request: response))

    with pytest.raises(GeoConversionError, match="not ready"):
        converter.lon_lat_to_map([(LON0, LAT0)])


def test_lon_lat_to_map_rejects_invalid_coordinates_without_calling_service():
    client = FakeClient(respond=planar_from_ll_array)

    with pytest.raises(ValueError):
        make_navsat_converter(client).lon_lat_to_map([(LON0, 91.0)])
    assert client.requests == []


def test_order_to_map_rejects_global_edge_without_start_node_position():
    order = get_order()
    del order["nodes"][0]["node_position"]

    with pytest.raises(ValueError):
        GeoCoordinateTranslator(FakeConverter()).order_to_map(order)


def test_order_to_map_tangential_orientation_unchanged():
    order = get_order()
    order["edges"][0]["orientation_type"] = "TANGENTIAL"

    converted = GeoCoordinateTranslator(FakeConverter()).order_to_map(order)

    assert converted["edges"][0]["orientation"] == 0.5


def test_order_to_map_without_coordinates_is_passthrough():
    order = get_order()
    del order["nodes"][0]["node_position"]
    del order["edges"][0]["trajectory"]
    order["edges"][0]["orientation_type"] = "TANGENTIAL"

    converter = FakeConverter(ready=False)
    assert GeoCoordinateTranslator(converter).order_to_map(order) == order


def test_order_to_map_rejects_when_not_ready():
    with pytest.raises(GeoConversionError):
        GeoCoordinateTranslator(FakeConverter(ready=False)).order_to_map(get_order())


def test_order_to_map_converts_embedded_init_position_actions():
    def init_position(action_id):
        return {
            "action_type": "initPosition",
            "action_id": action_id,
            "blocking_type": "HARD",
            "action_parameters": [
                {"key": "x", "value": LON0 + 0.0001},
                {"key": "y", "value": LAT0},
                {"key": "theta", "value": 1.0},
            ],
        }

    order = get_order()
    order["nodes"][1]["actions"] = [init_position("node_action")]
    order["edges"][0]["actions"] = [init_position("edge_action")]

    converted = GeoCoordinateTranslator(FakeConverter()).order_to_map(order)

    for action in (converted["nodes"][1]["actions"][0], converted["edges"][0]["actions"][0]):
        params = {p["key"]: p["value"] for p in action["action_parameters"]}
        assert params["x"] == pytest.approx(10.0 * math.cos(MAP_YAW))
        assert params["y"] == pytest.approx(-10.0 * math.sin(MAP_YAW))
        assert params["theta"] == pytest.approx(1.0 + fake_enu_to_map_yaw(LON0 + 0.0001, LAT0))


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

    converted = GeoCoordinateTranslator(FakeConverter()).instant_actions_to_map(
        instant_actions
    )

    params = {p["key"]: p["value"] for p in converted["actions"][0]["action_parameters"]}
    assert params["x"] == pytest.approx(10.0 * math.cos(MAP_YAW))
    assert params["y"] == pytest.approx(-10.0 * math.sin(MAP_YAW))
    assert params["theta"] == pytest.approx(1.0 + fake_enu_to_map_yaw(LON0 + 0.0001, LAT0))
    assert params["mapId"] == "WGS84"
    assert converted["actions"][1] == instant_actions["actions"][1]


def test_state_to_wgs84_round_trip_restores_original_order_positions():
    translator = GeoCoordinateTranslator(FakeConverter(), map_id="WGS84")
    wgs84_order = get_order()
    map_order = translator.order_to_map(wgs84_order)

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

    wgs84_state = translator.state_to_wgs84(state)

    assert wgs84_state.agv_position.x == pytest.approx(LON0 + 0.0001)
    assert wgs84_state.agv_position.y == pytest.approx(LAT0)
    assert wgs84_state.agv_position.theta == pytest.approx(
        -MAP_YAW - fake_enu_to_map_yaw(LON0 + 0.0001, LAT0)
    )
    assert wgs84_state.agv_position.map_id == "WGS84"

    original_node = wgs84_order["nodes"][0]["node_position"]
    assert wgs84_state.node_states[0].node_position.x == original_node["x"]
    assert wgs84_state.node_states[0].node_position.y == original_node["y"]
    assert wgs84_state.node_states[0].node_position.theta == 0.0

    original_cps = wgs84_order["edges"][0]["trajectory"]["control_points"]
    assert wgs84_state.edge_states[0].trajectory.control_points[0].x == original_cps[0]["x"]
    assert wgs84_state.edge_states[0].trajectory.control_points[0].y == original_cps[0]["y"]

    # Input message must not be modified.
    assert state.agv_position.map_id == "lidar_map"


def test_state_to_wgs84_unknown_node_is_converted():
    translator = GeoCoordinateTranslator(FakeConverter())
    state = OrderState(order_id="unknown")
    node_state = NodeState(node_id="n1", sequence_id=0)
    node_state.node_position.x = 10.0 * math.cos(MAP_YAW)
    node_state.node_position.y = -10.0 * math.sin(MAP_YAW)
    state.node_states.append(node_state)

    wgs84_state = translator.state_to_wgs84(state)

    assert wgs84_state.node_states[0].node_position.x == pytest.approx(LON0 + 0.0001)
    assert wgs84_state.node_states[0].node_position.y == pytest.approx(LAT0)
    assert wgs84_state.node_states[0].node_position.theta == pytest.approx(
        -fake_enu_to_map_yaw(LON0 + 0.0001, LAT0)
    )


def test_state_to_wgs84_not_ready_reports_uninitialized_position():
    translator = GeoCoordinateTranslator(FakeConverter(ready=False))
    state = OrderState()
    state.agv_position.position_initialized = True
    state.agv_position.localization_score = 0.9
    state.agv_position.x = 12.0
    state.agv_position.y = 34.0

    wgs84_state = translator.state_to_wgs84(state)

    assert not wgs84_state.agv_position.position_initialized
    assert wgs84_state.agv_position.localization_score == 0.0
    assert wgs84_state.agv_position.x == 0.0
    assert wgs84_state.agv_position.y == 0.0


def test_visualization_to_wgs84():
    translator = GeoCoordinateTranslator(FakeConverter())
    visualization = Visualization()
    visualization.agv_position.position_initialized = True
    visualization.agv_position.x = 0.0
    visualization.agv_position.y = 0.0
    visualization.agv_position.theta = math.pi - 0.05

    wgs84 = translator.visualization_to_wgs84(visualization)

    assert wgs84.agv_position.x == pytest.approx(LON0)
    assert wgs84.agv_position.y == pytest.approx(LAT0)
    assert wgs84.agv_position.theta == pytest.approx(-math.pi + 0.05)
