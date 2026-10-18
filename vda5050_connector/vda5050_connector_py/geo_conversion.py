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

"""
Translation of VDA5050 coordinates between WGS84 (MQTT side) and the robot map frame.

In ``wgs84`` mode the fleet manager exchanges positions as:

* ``x``: longitude [deg]
* ``y``: latitude [deg]
* ``theta``: ENU yaw [rad], 0 = East, counter-clockwise positive, range [-pi, pi]

Everything inside ROS keeps using metric coordinates in the robot ``map`` frame. The
conversion relies on robot_localization's navsat_transform ``fromLLArray``/``toLL``
services, so it always matches the frame the robot localizes in.

Distances (``allowedDeviationXY``, edge ``length``, ...) stay in meters.
"""

import copy
import math
import threading

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

COORDINATE_SYSTEM_CARTESIAN = "cartesian"
COORDINATE_SYSTEM_WGS84 = "wgs84"
SUPPORTED_COORDINATE_SYSTEMS = (COORDINATE_SYSTEM_CARTESIAN, COORDINATE_SYSTEM_WGS84)

INIT_POSITION_ACTION = "initPosition"

# Roughly 11 m north; used to measure the map-frame direction of true north.
_NORTH_PROBE_DEG = 1e-4
_MIN_PROBE_DISTANCE_M = 1.0
# Received coordinates not yet reported by the controller survive this many state messages.
_UNREPORTED_GRACE_STATES = 5
_MAX_UNCONFIRMED_CANDIDATES = 4


class GeoConversionError(Exception):
    """Raised when navsat_transform cannot convert coordinates (not ready, unavailable)."""


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _validate_lon_lat(lon: float, lat: float):
    if not (-180.0 <= lon <= 180.0 and -90.0 <= lat <= 90.0):
        raise ValueError(
            f"Invalid WGS84 coordinate (x=longitude: {lon}, y=latitude: {lat})"
        )


class NavSatGeoConverter:
    """Converts coordinates through robot_localization navsat_transform services."""

    def __init__(self, node, timeout_sec: float):
        # Deferred so robot_localization is only loaded in wgs84 mode.
        from geographic_msgs.msg import GeoPoint
        from geometry_msgs.msg import Point
        from robot_localization.srv import FromLLArray, ToLL

        self._geo_point_type = GeoPoint
        self._point_type = Point
        self._from_ll_array_type = FromLLArray
        self._to_ll_type = ToLL
        self._timeout_sec = timeout_sec

        # Separate group so responses are processed while a subscription callback waits.
        callback_group = MutuallyExclusiveCallbackGroup()
        self._from_ll_array_client = node.create_client(
            FromLLArray, "fromLLArray", callback_group=callback_group
        )
        self._to_ll_client = node.create_client(ToLL, "toLL", callback_group=callback_group)

    def _call(self, client, request):
        if not client.service_is_ready():
            raise GeoConversionError(f"Service '{client.srv_name}' is not available")

        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self._timeout_sec):
            client.remove_pending_request(future)
            raise GeoConversionError(
                f"Service '{client.srv_name}' did not respond within {self._timeout_sec}s"
            )
        result = future.result()
        if result is None:
            raise GeoConversionError(f"Service '{client.srv_name}' call failed")
        return result

    def lon_lat_to_map(self, lon_lats: list) -> list:
        """
        Convert (lon, lat) pairs to the map frame.

        Returns
        -------
            One (x, y, enu_to_map_yaw) tuple per point. The yaw offset [rad] is measured at
            that point, so a location always converts to the same heading.

        """
        for lon, lat in lon_lats:
            _validate_lon_lat(lon, lat)

        request = self._from_ll_array_type.Request()
        request.ll_points = [
            self._geo_point_type(latitude=lat, longitude=lon, altitude=0.0)
            for lon, lat in lon_lats
        ] + [
            self._geo_point_type(latitude=lat + _NORTH_PROBE_DEG, longitude=lon, altitude=0.0)
            for lon, lat in lon_lats
        ]

        response = self._call(self._from_ll_array_client, request)
        # navsat_transform answers with an empty array (or zeros) until its transform is ready.
        if len(response.map_points) != len(request.ll_points):
            raise GeoConversionError("navsat_transform is not ready (datum/transform unavailable)")

        count = len(lon_lats)
        poses = []
        for origin, north in zip(response.map_points[:count], response.map_points[count:]):
            if not all(math.isfinite(v) for v in (origin.x, origin.y, north.x, north.y)):
                raise GeoConversionError("fromLLArray returned non-finite map coordinates")
            dx = north.x - origin.x
            dy = north.y - origin.y
            if math.hypot(dx, dy) < _MIN_PROBE_DISTANCE_M:
                raise GeoConversionError(
                    "navsat_transform is not ready (datum/transform unavailable)"
                )
            poses.append((origin.x, origin.y, normalize_angle(math.atan2(dy, dx) - math.pi / 2.0)))

        return poses

    def map_to_lon_lat(self, x: float, y: float) -> tuple:
        """Convert a map (x, y) point to (lon, lat)."""
        request = self._to_ll_type.Request()
        request.map_point = self._point_type(x=x, y=y, z=0.0)
        ll_point = self._call(self._to_ll_client, request).ll_point
        if ll_point.latitude == 0.0 and ll_point.longitude == 0.0:
            raise GeoConversionError("navsat_transform is not ready (datum/transform unavailable)")
        # NaN/out-of-range results (e.g. from a NaN pose) are a robot-side failure, not bad input.
        try:
            _validate_lon_lat(ll_point.longitude, ll_point.latitude)
        except ValueError as ex:
            raise GeoConversionError(f"toLL returned an invalid result: {ex}") from ex
        return ll_point.longitude, ll_point.latitude

    def enu_to_map_yaw(self, lon: float, lat: float) -> float:
        """Yaw offset [rad] to add to an ENU heading to obtain a map-frame heading."""
        # Measured on every call so a datum change or navsat restart is picked up immediately.
        return self.lon_lat_to_map([(lon, lat)])[0][2]


class GeoCoordinateTranslator:
    """Translates VDA5050 payload coordinates between WGS84 and the map frame."""

    def __init__(self, converter, map_id: str = "", logger=None):
        self._converter = converter
        self._map_id = map_id
        self._logger = logger
        self._lock = threading.Lock()
        # Originals received from the fleet manager, matched to states by the map value the
        # controller reports, since the bridge does not see which orders were accepted:
        # (order_id, "node"|"edge", element_id, sequence_id) ->
        #     {"confirmed": entry | None, "unconfirmed": [entry]}, entry = (map_value, original,
        #     last_seen_state). "confirmed" is the candidate a state last reported.
        self._candidates = {}
        self._state_count = 0

    # Inbound (fleet manager -> robot)

    def order_to_map(self, order: dict) -> dict:
        """Convert a snake_case order dict from WGS84 to the map frame."""
        order = copy.deepcopy(order)

        positioned_nodes = [node for node in order.get("nodes", []) if "node_position" in node]
        node_positions = [node["node_position"] for node in positioned_nodes]
        control_points = [
            cp
            for edge in order.get("edges", [])
            for cp in edge.get("trajectory", {}).get("control_points", [])
        ]
        global_edges = [
            edge
            for edge in order.get("edges", [])
            if edge.get("orientation_type") == "GLOBAL" and "orientation" in edge
        ]

        original_nodes = {
            (node["node_id"], node["sequence_id"]): copy.deepcopy(node.get("node_position"))
            for node in order.get("nodes", [])
        }
        original_edges = {
            (edge["edge_id"], edge["sequence_id"]): copy.deepcopy(
                edge.get("trajectory", {}).get("control_points")
            )
            for edge in order.get("edges", [])
        }

        points = node_positions + control_points
        node_yaws = {}
        if points:
            map_poses = self._converter.lon_lat_to_map(
                [(float(p["x"]), float(p["y"])) for p in points]
            )
            for point, (x, y, _) in zip(points, map_poses):
                point["x"] = x
                point["y"] = y
            for node, (_, _, yaw) in zip(positioned_nodes, map_poses):
                node_yaws[node["node_id"]] = yaw
                position = node["node_position"]
                if "theta" in position:
                    position["theta"] = normalize_angle(float(position["theta"]) + yaw)

        for edge in global_edges:
            if edge["start_node_id"] not in node_yaws:
                raise ValueError(
                    f"Cannot convert GLOBAL orientation of edge '{edge['edge_id']}' "
                    "without a start node position"
                )
            yaw = node_yaws[edge["start_node_id"]]
            edge["orientation"] = normalize_angle(float(edge["orientation"]) + yaw)

        for element in order.get("nodes", []) + order.get("edges", []):
            self._init_position_actions_to_map(element.get("actions", []))

        self._remember_candidates(
            order.get("order_id", ""),
            [
                (
                    ("node", node["node_id"], node["sequence_id"]),
                    _node_map_value(node.get("node_position")),
                    original_nodes[(node["node_id"], node["sequence_id"])],
                )
                for node in order.get("nodes", [])
            ]
            + [
                (
                    ("edge", edge["edge_id"], edge["sequence_id"]),
                    _edge_map_value(edge.get("trajectory", {}).get("control_points", [])),
                    original_edges[(edge["edge_id"], edge["sequence_id"])],
                )
                for edge in order.get("edges", [])
            ],
        )
        return order

    def instant_actions_to_map(self, instant_actions: dict) -> dict:
        """Convert initPosition parameters of a snake_case instantActions dict to the map frame."""
        instant_actions = copy.deepcopy(instant_actions)
        actions_field = "instant_actions" if "instant_actions" in instant_actions else "actions"
        self._init_position_actions_to_map(instant_actions.get(actions_field, []))
        return instant_actions

    def _init_position_actions_to_map(self, actions: list):
        for action in actions:
            if action.get("action_type") != INIT_POSITION_ACTION:
                continue
            params = {p["key"]: p for p in action.get("action_parameters", [])}
            if "x" not in params or "y" not in params:
                continue
            (x, y), yaw = self._lon_lat_to_map_single(
                float(params["x"]["value"]), float(params["y"]["value"])
            )
            params["x"]["value"] = x
            params["y"]["value"] = y
            if "theta" in params:
                params["theta"]["value"] = normalize_angle(float(params["theta"]["value"]) + yaw)

    # Outbound (robot -> fleet manager)

    def state_to_wgs84(self, state):
        """Return a copy of an OrderState ROS message with WGS84 coordinates."""
        state = copy.deepcopy(state)
        self._agv_position_to_wgs84(state.agv_position)

        with self._lock:
            node_originals = [
                self._find_original(
                    (state.order_id, "node", ns.node_id, ns.sequence_id),
                    _node_map_value(ns.node_position),
                )
                for ns in state.node_states
            ]
            edge_originals = [
                self._find_original(
                    (state.order_id, "edge", es.edge_id, es.sequence_id),
                    _edge_map_value(es.trajectory.control_points),
                )
                for es in state.edge_states
            ]
            self._prune_candidates(state)

        for node_state, (found, original_position) in zip(state.node_states, node_originals):
            if found:
                if original_position is not None:
                    node_state.node_position.x = float(original_position["x"])
                    node_state.node_position.y = float(original_position["y"])
                    node_state.node_position.theta = float(original_position.get("theta", 0.0))
            else:
                self._node_position_to_wgs84(node_state.node_position)

        for edge_state, (found, original_control_points) in zip(
            state.edge_states, edge_originals
        ):
            if found:
                for cp, original_cp in zip(
                    edge_state.trajectory.control_points, original_control_points or []
                ):
                    cp.x = float(original_cp["x"])
                    cp.y = float(original_cp["y"])
            else:
                for cp in edge_state.trajectory.control_points:
                    self._control_point_to_wgs84(cp)

        return state

    def visualization_to_wgs84(self, visualization):
        """Return a copy of a Visualization ROS message with WGS84 coordinates."""
        visualization = copy.deepcopy(visualization)
        self._agv_position_to_wgs84(visualization.agv_position)
        return visualization

    # Helpers

    def _lon_lat_to_map_single(self, lon: float, lat: float) -> tuple:
        x, y, yaw = self._converter.lon_lat_to_map([(lon, lat)])[0]
        return (x, y), yaw

    def _pose_to_wgs84(self, x: float, y: float, theta: float) -> tuple:
        lon, lat = self._converter.map_to_lon_lat(x, y)
        yaw = self._converter.enu_to_map_yaw(lon, lat)
        return lon, lat, normalize_angle(theta - yaw)

    def _node_position_to_wgs84(self, position):
        try:
            position.x, position.y, position.theta = self._pose_to_wgs84(
                position.x, position.y, position.theta
            )
        except GeoConversionError as ex:
            self._warn(f"Reporting node position as zero: {ex}")
            position.x = 0.0
            position.y = 0.0
            position.theta = 0.0

    def _control_point_to_wgs84(self, control_point):
        try:
            control_point.x, control_point.y = self._converter.map_to_lon_lat(
                control_point.x, control_point.y
            )
        except GeoConversionError as ex:
            self._warn(f"Reporting trajectory control point as zero: {ex}")
            control_point.x = 0.0
            control_point.y = 0.0

    def _warn(self, message: str):
        if self._logger:
            self._logger.warning(message, throttle_duration_sec=5.0)

    def _agv_position_to_wgs84(self, agv_position):
        if self._map_id:
            agv_position.map_id = self._map_id

        if not agv_position.position_initialized:
            agv_position.x = 0.0
            agv_position.y = 0.0
            agv_position.theta = 0.0
            return

        try:
            agv_position.x, agv_position.y, agv_position.theta = self._pose_to_wgs84(
                agv_position.x, agv_position.y, agv_position.theta
            )
        except GeoConversionError as ex:
            # Never report map-frame meters as degrees.
            self._warn(f"Reporting position as not initialized: {ex}")
            agv_position.position_initialized = False
            agv_position.localization_score = 0.0
            agv_position.x = 0.0
            agv_position.y = 0.0
            agv_position.theta = 0.0

    def _remember_candidates(self, order_id: str, candidates: list):
        with self._lock:
            for element, map_value, original in candidates:
                slot = self._candidates.setdefault(
                    (order_id,) + element, {"confirmed": None, "unconfirmed": []}
                )
                if slot["confirmed"] and slot["confirmed"][0] == map_value:
                    continue
                unconfirmed = [entry for entry in slot["unconfirmed"] if entry[0] != map_value]
                unconfirmed.insert(0, (map_value, original, self._state_count))
                slot["unconfirmed"] = unconfirmed[:_MAX_UNCONFIRMED_CANDIDATES]

    def _find_original(self, key: tuple, map_value: tuple) -> tuple:
        slot = self._candidates.get(key)
        if slot is None:
            return False, None
        match = slot["confirmed"]
        if match is None or match[0] != map_value:
            match = next((e for e in slot["unconfirmed"] if e[0] == map_value), None)
            if match is None:
                return False, None
            # Reported by a state, so accepted: keep it out of the bounded alternatives.
            slot["unconfirmed"].remove(match)
        slot["confirmed"] = (match[0], match[1], self._state_count)
        return True, match[1]

    def _prune_candidates(self, state):
        reported = {
            (state.order_id, "node", n.node_id, n.sequence_id) for n in state.node_states
        } | {(state.order_id, "edge", e.edge_id, e.sequence_id) for e in state.edge_states}
        oldest_kept = self._state_count - _UNREPORTED_GRACE_STATES
        for key in list(self._candidates):
            if key in reported:
                continue
            # Traversed or rejected entries are dropped once they are past the grace period.
            slot = self._candidates[key]
            slot["unconfirmed"] = [e for e in slot["unconfirmed"] if e[2] > oldest_kept]
            if slot["confirmed"] and slot["confirmed"][2] <= oldest_kept:
                slot["confirmed"] = None
            if slot["confirmed"] is None and not slot["unconfirmed"]:
                del self._candidates[key]
        self._state_count += 1


def _node_map_value(position) -> tuple:
    """Map-frame node position as the controller reports it in node states."""
    if position is None:
        return (0.0, 0.0, 0.0)
    if isinstance(position, dict):
        return (
            float(position["x"]),
            float(position["y"]),
            float(position.get("theta", 0.0)),
        )
    return (position.x, position.y, position.theta)


def _edge_map_value(control_points) -> tuple:
    """Map-frame trajectory control points as the controller reports them in edge states."""
    return tuple(
        (float(cp["x"]), float(cp["y"])) if isinstance(cp, dict) else (cp.x, cp.y)
        for cp in control_points
    )
