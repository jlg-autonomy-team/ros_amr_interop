#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
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
#    * Neither the name of the InOrbit, Inc. nor the names of its
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

# Python dependencies
from paho.mqtt import client as mqtt_client
from paho.mqtt.client import error_string
import copy
import json
import os
# JLG_CHANGES_START
import time
# JLG_CHANGES_END

# ROS dependencies / utils
# JLG_CHANGES_START
import rclpy
# JLG_CHANGES_END
from rclpy.node import Node

# JLG_CHANGES_START
from talos_msgs.srv import DTCUnlatch
from talos_msgs.msg import DTC
# JLG_CHANGES_END

from vda5050_connector_py.utils import get_vda5050_mqtt_topic
from vda5050_connector_py.utils import get_vda5050_ros2_topic
from vda5050_connector_py.utils import json_camel_to_snake_case
from vda5050_connector_py.utils import read_str_parameter, read_int_parameter
# JLG_CHANGES_START
from vda5050_connector_py.utils import read_bool_parameter
# JLG_CHANGES_END
from vda5050_connector_py.utils import convert_ros_message_to_json
from vda5050_connector_py.utils import get_vda5050_ts
# JLG_CHANGES_START
from vda5050_connector_py.utils import has_unique_ids
from vda5050_connector_py.utils import validate_vda5050_payload
# JLG_CHANGES_END

from vda5050_connector_py.vda5050_controller import DEFAULT_PROTOCOL_VERSION, SUPPORTED_PROTOCOL_VERSIONS

# ROS msgs / srvs / actions
from vda5050_msgs.msg import Action as VDAAction
from vda5050_msgs.msg import ActionParameter as VDAActionParameter
from vda5050_msgs.msg import Connection as VDAConnection
from vda5050_msgs.msg import ControlPoint as VDAControlPoint
from vda5050_msgs.msg import Edge as VDAEdge
from vda5050_msgs.msg import InstantActions as VDAInstantActions
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import NodePosition as VDANodePosition
from vda5050_msgs.msg import Order as VDAOrder
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import Trajectory as VDATrajectory
from vda5050_msgs.msg import Visualization as VDAVisualization

NODE_NAME = "mqtt_bridge"


def generate_vda_order_msg(order):
    """
    Convert an Order message into a ROS2 Order message represented as a dict.

    Args:
    ----
        order (VDAOrder): VDA5050 Order message

    Returns
    -------
        Order dict message for building a ROS2 Order object

    """
    vda_order = copy.deepcopy(order)
    for node in vda_order["nodes"]:
        # Force all numbers to float. Values with no decimals are
        # interpret as integers, causing the validation errors
        for k in ["x", "y", "theta"]:
            node["node_position"][k] = float(node["node_position"][k])
        node["node_position"] = VDANodePosition(**node["node_position"])
        for action in node["actions"]:
            if "action_parameters" in action:
                action["action_parameters"] = [
                    VDAActionParameter(
                        key=action_parameter["key"],
                        value=str(action_parameter["value"]),
                    )
                    for action_parameter in action["action_parameters"]
                ]
        node["actions"] = [VDAAction(**action) for action in node["actions"]]

    vda_order["nodes"] = [VDANode(**node) for node in vda_order["nodes"]]
    for edge in vda_order["edges"]:
        for action in edge["actions"]:
            if "action_parameters" in action:
                action["action_parameters"] = [
                    VDAActionParameter(
                        key=action_parameter["key"],
                        value=str(action_parameter["value"]),
                    )
                    for action_parameter in action["action_parameters"]
                ]
        edge["actions"] = [VDAAction(**action) for action in edge["actions"]]

        # Force all numbers to float. Values with no decimals are
        # interpreted as integers, causing the validation errors.
        for k in [
            "max_speed",
            "max_height",
            "min_height",
            "orientation",
            "max_rotation_speed",
            "length",
        ]:
            if k in edge:
                edge[k] = float(edge[k])

        if "trajectory" in edge:
            edge["trajectory"] = VDATrajectory(
                # JLG_CHANGES_START
                degree=int(edge["trajectory"]["degree"]),
                # JLG_CHANGES_END
                knot_vector=edge["trajectory"]["knot_vector"],
                control_points=[
                    VDAControlPoint(
                        x=float(cp["x"]),
                        y=float(cp["y"]),
                        # JLG_CHANGES_START
                        # orientation=float(cp["orientation"]),
                        # JLG_CHANGES_END
                        weight=float(cp.get("weight", 1)),
                    )
                    for cp in edge["trajectory"]["control_points"]
                ],
            )
    vda_order["edges"] = [VDAEdge(**edge) for edge in vda_order["edges"]]
    # TODO(@leandropineda): Consider returning a ROS2 Order message
    return vda_order


def generate_vda_instant_action_msg(instant_action):
    """
    Convert an Instant Action message into a ROS2 Instant Action message represented as a dict.

    Args:
    ----
        instant_action (VDAInstantActions): VDA5050 Instant Action message

    Returns
    -------
        Instant Action dict message for building a ROS2 Instant Action object

    """
    # Values on the `instant_action` parameter will be modified. Create
    # a copy of the object to avoid overrides
    vda_instant_action = copy.deepcopy(instant_action)

    # HACK: VDA5050 instantActions message schema differs from v1 to v2. In particular,
    # the ``instantActions`` field from v1 has been renamed to ``actions`` on v2.
    is_v1 = "instant_actions" in vda_instant_action.keys()

    instant_actions_field = "instant_actions" if is_v1 else "actions"

    for action in vda_instant_action[instant_actions_field]:
        try:
            # Force all action parameter values to string. VDA5050 supports
            # value types `array`, `boolean`, `number` and `string` but
            # VDAActionParameter expects str values
            for action_parameters in action["action_parameters"]:
                action_parameters["value"] = str(action_parameters["value"])
            action["action_parameters"] = [
                VDAActionParameter(**action_parameter)
                for action_parameter in action["action_parameters"]
            ]
        except KeyError:
            # Action parameters are not required
            pass

    vda_instant_action["actions"] = [
        VDAAction(**action) for action in vda_instant_action[instant_actions_field]
    ]

    # HACK: As the internal representation of all messages
    # is v2 only, remove the v1 ``instant_actions`` field
    if is_v1:
        vda_instant_action.pop("instant_actions")

    # TODO(@leandropineda): Consider returning a ROS2 Order message
    return vda_instant_action


def generate_vda5050_topic_alias(vda_version):
    """
    Create an alias for the current vda5050 version. The aliases are needed to
    create the mqtt topics.

    Args:
    ----
        vda_version (string): VDA5050 version with format x.x.x.

    Raises:
    ------
        ValueError if the alias is not within the supported values.

    Returns
    -------
        The alias of the version. For example, for the version '2.0.0', the alias is
        'v2'
    """
    if vda_version in SUPPORTED_PROTOCOL_VERSIONS:
        return f"v{vda_version[0]}"
    else:
        raise ValueError(
            f"Invalid protocol major version. Supported versions are: {SUPPORTED_PROTOCOL_VERSIONS},"
            f"but got {vda_version}"
        )

class MQTTBridge(Node):
    """Translates VDA5050 MQTT messages from and to ROS2."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        # Declare Node configuration parameter. Use default values if no parameters
        # are defined on launchfile. Provide the parameter when running the launchfile
        # by using ``foo.launch.py mqtt_address:=localhost mqtt_port:=1883 ...``
        self.mqtt_address = read_str_parameter(self, "mqtt_address", "localhost")
        self.mqtt_port = read_int_parameter(self, "mqtt_port", 1883)
        self.mqtt_username = read_str_parameter(self, "mqtt_username", "")
        self.mqtt_password = read_str_parameter(self, "mqtt_password", "")
        # JLG_CHANGES_START
        self.mqtt_ca_cert = read_str_parameter(
                    self, "mqtt_ca_cert", "/etc/ssl/certs/ca-certificates.crt"
                )
        self.mqtt_client_certificate = read_str_parameter(self, "mqtt_client_certificate", "")
        self.mqtt_client_key = read_str_parameter(self, "mqtt_client_key", "")
        # JLG_CHANGES_END

        self.vda5050_version = read_str_parameter(self, "vda5050_protocol_version", "2.0.0")
        self.vda5050_version_alias = generate_vda5050_topic_alias(self.vda5050_version)

        self._manufacturer_name = read_str_parameter(
            self, "manufacturer_name", "robots"
        )
        self._serial_number = read_str_parameter(self, "serial_number", "robot_1")

        self._interface_name = read_str_parameter(self, "interface_name", "uagv")
        # JLG_CHANGES_START
        self.enable_vda5050_validation = read_bool_parameter(self, "enable_vda5050_validation", True)
        self.invalid_order_dtc = read_int_parameter(self, "invalid_order_dtc", 2460)
        self.broker_comm_loss_dtc = read_int_parameter(
            self, "broker_comm_loss_dtc", 2456
        )
        # JLG_CHANGES_END

        self.configure()

    def configure(self):
        # Configure MQTT
        # JLG_CHANGES_START
        self.mqtt_client = mqtt_client.Client(
            callback_api_version=mqtt_client.CallbackAPIVersion.VERSION2,
            clean_session=True,
            reconnect_on_failure=True,
        )
        # JLG_CHANGES_END
        self.mqtt_client.on_connect = self.on_connect_mqtt
        self.mqtt_client.on_message = self.on_message_mqtt
        self.mqtt_client.on_disconnect = self.on_disconnect_mqtt

        # Enable TLS if username is provided
        if self.mqtt_username:
            self.mqtt_client.tls_set(
                ca_certs=self.mqtt_ca_cert,
                # JLG_CHANGES_START
                certfile=self.mqtt_client_certificate or None,
                keyfile=self.mqtt_client_key or None,
                # JLG_CHANGES_END
            )
            # JLG_CHANGES_START
            # Azure Event Grid requires the CONNECT username to be set to the
            # client authentication name even when using certificate auth.
            # The password is only sent for SAS-token auth; for cert auth it
            # is omitted (username_pw_set with password=None -> u1/p0).
            self.mqtt_client.username_pw_set(
                username=self.mqtt_username,
                password=self.mqtt_password if self.mqtt_password else None,
            )
            # JLG_CHANGES_END

        # Configure will message or last testament message
        will_topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )

        # NOTE: will payload cannot be set dynamically or updated
        # without reconnecting, so some values are fixed. For the
        # timestamp, instead of using the time the connector started
        # the ts 0 is used which is easy to identify.
        will_payload = convert_ros_message_to_json(
            VDAConnection(
                header_id=0,
                version=self.vda5050_version,
                timestamp="1970-01-01T12:00:00.00Z",
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                connection_state=VDAConnection.CONNECTIONBROKEN,
            )
        )
        self.mqtt_client.will_set(
            topic=will_topic, payload=will_payload, qos=1, retain=True
        )

        # Keep a copy of the last VDA5050 Connection message
        self._last_connection_msg = None

        # JLG_CHANGES_START
        # Create DTC publisher
        self.dtc_pub = self.create_publisher(
            msg_type=DTC,
            topic="diagnostics/dtc",
            qos_profile=10,
        )
        # Create DTC latching/unlatching services
        self.dtc_unlatch_client = self.create_client(
            DTCUnlatch, "diagnostics/unlatch_dtc"
        )
        self.dtc_force_latch_client = self.create_client(
            DTCUnlatch, "diagnostics/force_latch_dtc"
        )
        # JLG_CHANGES_END

        # Connect to MQTT broker
        # JLG_CHANGES_START
        # self.mqtt_client.enable_logger()
        self.mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.mqtt_client.max_inflight_messages_set(20)
        self.mqtt_client.max_queued_messages_set(100)

        self.mqtt_client.connect_async(host=self.mqtt_address, port=int(self.mqtt_port), keepalive=60)
        # JLG_CHANGES_END
        self.mqtt_client.loop_start()

        self.on_configure()

        self.logger.info(f"Node {NODE_NAME} has started successfully.")

    # JLG_CHANGES_START
    def on_connect_mqtt(self, client, userdata, flags, rc, properties=None): 
    # JLG_CHANGES_END
        """MQTT client connect callback."""
        if rc == 0:
            self.logger.info("Connected to MQTT Broker!")
            self.mqtt_client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic="order",
                    major_version=self.vda5050_version_alias,
                    interface_name=self._interface_name
                )
            )
            self.mqtt_client.subscribe(
                get_vda5050_mqtt_topic(
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    topic="instantActions",
                    major_version=self.vda5050_version_alias,
                    interface_name=self._interface_name
                )
            )
            self._publish_connection(
                msg=VDAConnection(
                    header_id=0,
                    version=self.vda5050_version,
                    timestamp=get_vda5050_ts(),
                    manufacturer=self._manufacturer_name,
                    serial_number=self._serial_number,
                    connection_state=VDAConnection.ONLINE,
                )
            )
        else:
            self.logger.error(f"Failed to connect, return code {rc}")

    def on_message_mqtt(self, client, userdata, msg):
        """MQTT client message callback."""

        # JLG_CHANGES_START
        # First unlatch invalid order DTC
        if self.enable_vda5050_validation:
            self.call_dtc_unlatch(self.invalid_order_dtc)
        # JLG_CHANGES_END

        try:
            msg_json = json_camel_to_snake_case(msg.payload)
            self.logger.debug(f"Received '{msg_json}' from '{msg.topic}' topic")
        except json.decoder.JSONDecodeError:
            self.logger.error(f"Failed to decode message: '{msg.payload}'")
            # JLG_CHANGES_START
            self.call_dtc_force_latch(self.invalid_order_dtc)
            # JLG_CHANGES_STOP
            return

        # JLG_CHANGES_START
        if self.enable_vda5050_validation:

            # Check IDs uniqueness
            if has_unique_ids(msg_json) is False:
                self.logger.warn(f"❌ Invalid VDA5050 message: duplicate or empty orderID/nodeID/actionID/edgeID found inside the order (IDs must be unique within their respective types).")
                self.call_dtc_force_latch(self.invalid_order_dtc)
                return

            try:
                if msg.topic.endswith("order"):
                    order_validation = validate_vda5050_payload(
                        "order", json.loads(msg.payload)
                    )
                    if order_validation:
                        self.logger.warn(f"❌ Invalid VDA5050 order message")
                        for loc, e in order_validation:
                            self.logger.warn(f"❌ At {loc if loc else '<root>'}: {e}")
                        self.call_dtc_force_latch(self.invalid_order_dtc)
                        return
                    self.logger.info("✅ Valid VDA5050 order message")
                    vda_order_msg = VDAOrder(**generate_vda_order_msg(msg_json))
                    self._order_pub.publish(msg=vda_order_msg)
                if msg.topic.endswith("instantActions"):
                    instant_actions_validation = validate_vda5050_payload(
                        "instantActions", json.loads(msg.payload)
                    )
                    if instant_actions_validation:
                        self.logger.warn(f"❌ Invalid VDA5050 instantActions message")
                        for loc, e in instant_actions_validation:
                            self.logger.warn(f"❌ At {loc if loc else '<root>'}: {e}")
                        self.call_dtc_force_latch(self.invalid_order_dtc)
                        return
                    self.logger.info("✅ Valid VDA5050 instantActions message")
                    vda_instant_actions_message = VDAInstantActions(
                        **generate_vda_instant_action_msg(msg_json)
                    )
                    self._instant_actions_pub.publish(msg=vda_instant_actions_message)
            except Exception as ex:
                self.logger.warn(
                    f"Ignoring malformed VDA5050 message: {type(ex).__name__}: {ex}."
                )
                self.call_dtc_force_latch(self.invalid_order_dtc)
                return
        # JLG_CHANGES_END
        else:
            try:
                if msg.topic.endswith("order"):
                    vda_order_msg = VDAOrder(**generate_vda_order_msg(msg_json))
                    self._order_pub.publish(msg=vda_order_msg)
                if msg.topic.endswith("instantActions"):
                    vda_instant_actions_message = VDAInstantActions(
                        **generate_vda_instant_action_msg(msg_json)
                    )
                    self._instant_actions_pub.publish(msg=vda_instant_actions_message)
            # JLG_CHANGES_START
            except Exception as ex:
                self.logger.warn(
                    f"Ignoring malformed VDA5050 message: {type(ex).__name__}: {ex}."
                )
                return
            # JLG_CHANGES_END

    # JLG_CHANGES_START
    def on_disconnect_mqtt(self, client, userdata, disconnect_flags, rc, properties=None):
    # JLG_CHANGES_END
        """MQTT client disconnect callback."""
        if rc != 0:
            # JLG_CHANGES_START
            self.logger.info(
                f"MQTT client disconnected "
                f"(rc: {rc}, {error_string(rc)}, flags: {disconnect_flags}). "
                "Trying to reconnect."
            )
            # JLG_CHANGES_END

            # JLG_CHANGES_START
            #while not self.mqtt_client.is_connected():
            #    try:
            #        self.mqtt_client.reconnect()
            #    except OSError:
            #        pass
            # JLG_CHANGES_END
        else:
            self.logger.info("Disconnected from MQTT Broker!")

    def on_configure(self):
        """
        Subscribe to relevant ROS2 topics.

        This method registers callbacks for translating
        ROS2 messages to VDA5050 MQTT messages.
        """
        self.logger.info("Configuring ROS topics")
        self._state_sub = self.create_subscription(
            msg_type=VDAOrderState,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="state",
                interface_name=self._interface_name
            ),
            callback=self._publish_state,
            qos_profile=10,
        )

        self._connection_sub = self.create_subscription(
            msg_type=VDAConnection,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="connection",
                interface_name=self._interface_name
            ),
            callback=self._publish_connection,
            qos_profile=10,
        )

        self._visualization_sub = self.create_subscription(
            msg_type=VDAVisualization,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="visualization",
                interface_name=self._interface_name
            ),
            callback=self._publish_visualization,
            qos_profile=10,
        )

        self._order_pub = self.create_publisher(
            msg_type=VDAOrder,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="order",
                interface_name=self._interface_name
            ),
            qos_profile=10,
        )

        self._instant_actions_pub = self.create_publisher(
            msg_type=VDAInstantActions,
            topic=get_vda5050_ros2_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="instantActions",
                interface_name=self._interface_name
            ),
            qos_profile=10,
        )
        self.logger.info("Finished configuring ROS topics")

    def on_shutdown(self):
        """Perform all necessary teardown steps."""
        self.logger.info("Publishing offline Connection message")

        offline_message = VDAConnection(
            header_id=0,
            version=self.vda5050_version,
            timestamp=get_vda5050_ts(),
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            connection_state=VDAConnection.OFFLINE,
        )

        # Use the latest Connection message `header_id` if available
        if self._last_connection_msg:
            offline_message.header_id = self._last_connection_msg.header_id + 1

        self._publish_connection(msg=offline_message)

        self.logger.info("Unsubscribing from MQTT topics")
        self.mqtt_client.unsubscribe(
            get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="order",
                major_version=self.vda5050_version_alias,
                interface_name=self._interface_name
            )
        )
        self.mqtt_client.unsubscribe(
            get_vda5050_mqtt_topic(
                manufacturer=self._manufacturer_name,
                serial_number=self._serial_number,
                topic="instantActions",
                major_version=self.vda5050_version_alias,
                interface_name=self._interface_name
            )
        )

        self.mqtt_client.disconnect()
        # JLG_CHANGES_START
        self.mqtt_client.loop_stop()
        # JLG_CHANGES_END

    def _publish_to_topic(self, msg, topic):
        """
        Publish a ROS2 message to an MQTT topic.

        Args:
        ----
            msg (Any): VDA5050 ROS2 message.
            topic (str): topic for publishing the VDA5050 MQTT message.

        """
        json_msg = convert_ros_message_to_json(msg)
        self.logger.debug(f"Publishing MQTT message to topic {topic}: {json_msg}")
        # JLG_CHANGES_START
        result = self.mqtt_client.publish(topic, json_msg)
        if result.rc != mqtt_client.MQTT_ERR_SUCCESS:
            err_msg = f"Failed to publish message to topic {topic}: {error_string(result.rc)}"
            
            dtc = DTC()
            dtc.dtc = self.broker_comm_loss_dtc
            
            self.logger.error(err_msg)
            self.dtc_pub.publish(dtc)
        # JLG_CHANGES_END

    def _publish_state(self, msg: VDAOrderState):
        """
        Publish ROS2 OrderState message to the corresponding VDA5050 MQTT topic.

        Args:
        ----
            msg (VDAOrderState): VDA5050 ROS2 OrderState message.

        """
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="state",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

    def _publish_connection(self, msg: VDAConnection):
        """
        Publish VDA5050 ROS2 Connection message to the corresponding VDA5050 MQTT topic.

        Also updates a local copy of the last published VDA5050 Connection message to keep track
        of the latest ``header_id`` field. This is used to publish an offline Connection message
        when tearing down the node.

        Args:
        ----
            msg (VDAConnection): VDA5050 ROS2 Connection message.

        """
        # Update the last connection message
        self._last_connection_msg = msg
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="connection",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

    def _publish_visualization(self, msg: VDAVisualization):
        """
        Publish ROS2 Visualization message to the corresponding VDA5050 MQTT topic.

        Args:
        ----
            msg (VDAVisualization): VDA5050 ROS2 Visualization message.

        """
        topic = get_vda5050_mqtt_topic(
            manufacturer=self._manufacturer_name,
            serial_number=self._serial_number,
            topic="visualization",
            major_version=self.vda5050_version_alias,
            interface_name=self._interface_name
        )
        self._publish_to_topic(msg, topic)

    # JLG_CHANGES_START
    def call_dtc_unlatch(self, dtc: int) -> None:
        """
        Call the DTC unlatch service.

        Args:
            dtc: DTC number to unlatch

        Returns:
            bool: Success status of the service call
        """
        if not self.dtc_unlatch_client.service_is_ready():
            self.logger.error("DTC unlatch service not available")
            return

        request = DTCUnlatch.Request()
        dtc_msg = DTC()
        dtc_msg.dtc = dtc
        request.dtc = dtc_msg

        try:
            future = self.dtc_unlatch_client.call_async(request)
            future.add_done_callback(lambda f: self._log_future(f, f"DTC {dtc} unlatch"))
        except Exception as e:
            self.logger.error(f"Failed to send DTC unlatch request: {e}")

    def call_dtc_force_latch(self, dtc: int) -> None:
        """
        Call the DTC force latch service.

        Args:
            dtc_data: DTC message data to send

        Returns:
            bool: Success status of the service call
        """
        if not self.dtc_force_latch_client.service_is_ready():
            self.logger.error("DTC force latch service not available")
            return

        request = DTCUnlatch.Request()
        dtc_msg = DTC()
        dtc_msg.dtc = dtc
        request.dtc = dtc_msg

        try:
            future = self.dtc_force_latch_client.call_async(request)
            future.add_done_callback(lambda f: self._log_future(f, f"DTC {dtc} force latch"))
        except Exception as e:
            self.logger.error(f"Failed to send DTC force latch request: {e}")

    def _log_future(self, future, label: str) -> None:
        try:
            resp = future.result()
            if resp.success:
                self.logger.info(f"{label} succeeded")
            else:
                self.logger.error(f"{label} failed")
        except Exception as e:
            self.logger.error(f"{label} failed: {e}")

    # JLG_CHANGES_END
