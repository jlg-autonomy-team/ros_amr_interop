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

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.task import Future

from uuid import uuid4

from vda5050_connector_py.vda5050_controller import VDA5050Controller
from vda5050_connector_py.vda5050_controller import OrderAcceptModes
from vda5050_connector_py.vda5050_controller import OrderExecutionErrors
from vda5050_connector_py.vda5050_controller import OrderRejectErrors
from vda5050_connector_py.utils import get_vda5050_ts
from action_msgs.msg import GoalStatus
from vda5050_connector.action import NavigateToNode

from vda5050_msgs.msg import Order
from vda5050_msgs.msg import Node
from vda5050_msgs.msg import Edge
from vda5050_msgs.msg import NodePosition
from vda5050_msgs.msg import Action
from vda5050_msgs.msg import ActionParameter
from vda5050_msgs.msg import AGVPosition
from vda5050_msgs.msg import CurrentAction
from vda5050_msgs.msg import EdgeState
from vda5050_msgs.msg import NodeState


def get_order_new(order_id=str(uuid4()), order_update_id=0):
    return Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=order_update_id,
        nodes=[
            Node(
                node_id="node1",
                sequence_id=0,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node2",
                sequence_id=2,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node3",
                sequence_id=4,
                released=True,
                node_position=NodePosition(
                    x=-0.38,
                    y=1.89,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node4",
                sequence_id=6,
                released=True,
                node_position=NodePosition(
                    x=-0.17,
                    y=1.74,
                    theta=-2.6,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node1",
                sequence_id=8,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
        ],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=1,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge2",
                sequence_id=3,
                released=True,
                start_node_id="node2",
                end_node_id="node3",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge3",
                sequence_id=5,
                released=True,
                start_node_id="node3",
                end_node_id="node4",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
            Edge(
                edge_id="edge4",
                sequence_id=7,
                released=True,
                start_node_id="node4",
                end_node_id="node1",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )


def get_order_update(order_id=str(uuid4()), order_update_id=0):
    return Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=order_update_id,
        nodes=[
            Node(
                node_id="node1",
                sequence_id=8,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
            Node(
                node_id="node2",
                sequence_id=10,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
            ),
        ],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=9,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            )
        ],
    )


def get_navigation_result_future():
    future = Future()
    future.set_result(
        NavigateToNode.Impl.GetResultService.Response(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=NavigateToNode.Result(),
        )
    )
    return future


def get_stitch_orders(order_id=str(uuid4())):
    action1 = Action(
        action_type="foo",
        action_id=str(uuid4()),
        action_description="Foo description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="foo", value="bar")
        ]
    )
    action2 = Action(
        action_type="bar",
        action_id=str(uuid4()),
        action_description="Bar description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="foo", value="bar")
        ]
    )
    action3 = Action(
        action_type="foobar",
        action_id=str(uuid4()),
        action_description="FooBar description",
        blocking_type="NONE",
        action_parameters=[
            ActionParameter(key="abc", value="xyz")
        ]
    )
    base_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=0,
        nodes=[
            Node(
                node_id="node1",
                sequence_id=0,
                released=True,
                node_position=NodePosition(
                    x=2.0,
                    y=0.95,
                    theta=-0.66,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action1
                ],
            ),
            Node(
                node_id="node2",
                sequence_id=2,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action2
                ],
            ),
        ],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=1,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )

    stitch_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=1,
        nodes=[
            Node(
                node_id="node2",
                sequence_id=2,
                released=True,
                node_position=NodePosition(
                    x=1.18,
                    y=-1.76,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action2
                ],
            ),
            Node(
                node_id="node3",
                sequence_id=4,
                released=True,
                node_position=NodePosition(
                    x=-0.38,
                    y=1.89,
                    theta=0.0,
                    allowed_deviation_x_y=0.0,
                    allowed_deviation_theta=0.0,
                    map_id="map",
                ),
                actions=[
                    action3
                ],
            ),
        ],
        edges=[
            Edge(
                edge_id="edge2",
                sequence_id=3,
                released=True,
                start_node_id="node2",
                end_node_id="node3",
                max_speed=10.0,
                max_height=10.0,
                min_height=1.0,
            ),
        ],
    )

    return [base_order, stitch_order]


def get_stitch_orders_no_actions_on_stitch_node(order_id=str(uuid4())):
    """Build a base / stitch order pair where the stitch node (node2) has no actions."""
    action1 = Action(
        action_type="foo",
        action_id=str(uuid4()),
        action_description="Foo description",
        blocking_type="NONE",
    )
    action3 = Action(
        action_type="foobar",
        action_id=str(uuid4()),
        action_description="FooBar description",
        blocking_type="NONE",
    )
    node1 = Node(
        node_id="node1",
        sequence_id=0,
        released=True,
        node_position=NodePosition(x=2.0, y=0.95, theta=-0.66, map_id="map"),
        actions=[action1],
    )
    node3 = Node(
        node_id="node3",
        sequence_id=4,
        released=True,
        node_position=NodePosition(x=-0.38, y=1.89, theta=0.0, map_id="map"),
        actions=[action3],
    )

    def build_node2():
        return Node(
            node_id="node2",
            sequence_id=2,
            released=True,
            node_position=NodePosition(x=1.18, y=-1.76, theta=0.0, map_id="map"),
        )

    base_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=0,
        nodes=[node1, build_node2()],
        edges=[
            Edge(
                edge_id="edge1",
                sequence_id=1,
                released=True,
                start_node_id="node1",
                end_node_id="node2",
            ),
        ],
    )

    stitch_order = Order(
        header_id=0,
        timestamp=get_vda5050_ts(),
        version="1.1.1",
        manufacturer="MANUFACTURER",
        serial_number="SERIAL_NUMBER",
        order_id=order_id,
        order_update_id=1,
        nodes=[build_node2(), node3],
        edges=[
            Edge(
                edge_id="edge2",
                sequence_id=3,
                released=True,
                start_node_id="node2",
                end_node_id="node3",
            ),
        ],
    )

    return [base_order, stitch_order]


def test_vda5050_controller_node_new_order(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):

    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate used navigation goal parameters
    spy_send_adapter_navigate_to_node = mocker.spy(
        node, "send_adapter_navigate_to_node"
    )

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # generate an order and let the node process it
    order_id = str(uuid4())
    order = get_order_new(order_id)
    node.process_order(order)

    rclpy.spin_once(node)

    spy_accept_order.assert_called_once_with(order=order, mode=OrderAcceptModes.NEW)

    rclpy.spin_once(adapter_node)

    # check node states were properly updated
    assert node._current_order == order
    assert node._current_state.order_id == order_id
    assert node._current_state.order_update_id == 0

    # The order has 5 nodes and 4 edges but the first edge and node
    # are processed as soon as the order is accepted.
    assert len(node._current_state.node_states) == 4
    assert len(node._current_state.edge_states) == 4

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 0

    # Assert the first navigation goal was sent to the adapter,
    # and that the parameters matches order's first edge and second node.
    # Note: the standard assumes the vehicle is on the first node already,
    # so the first navigation command is to the second order node.
    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[0], node=order.nodes[1]
    )

    # Future for invoking adapter navigation goal result callback
    future = get_navigation_result_future()

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[1], node=order.nodes[2]
    )

    assert len(node._current_state.node_states) == 3
    assert len(node._current_state.edge_states) == 3
    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 2

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[2], node=order.nodes[3]
    )

    assert len(node._current_state.node_states) == 2
    assert len(node._current_state.edge_states) == 2
    assert node._current_state.last_node_id == "node3"
    assert node._current_state.last_node_sequence_id == 4

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[3], node=order.nodes[4]
    )
    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1
    assert node._current_state.last_node_id == "node4"
    assert node._current_state.last_node_sequence_id == 6

    spy_send_adapter_navigate_to_node.reset_mock()
    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    node._on_active_order()

    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 8


def test_vda5050_controller_node_update_order(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate used navigation goal parameters
    spy_send_adapter_navigate_to_node = mocker.spy(
        node, "send_adapter_navigate_to_node"
    )

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # Send first new order
    order_id = str(uuid4())
    order = get_order_new(order_id)
    node.process_order(order)

    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)

    # Simulate the adapter reached navigation goals
    future = get_navigation_result_future()

    # The NEW order contains 5 nodes and 4 edges. The first node (in deviation range)
    # is processed and remove, and 4 nodes are send to navigate to.
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    node._navigate_to_node_result_callback(future)
    node._on_active_order()
    # Finish initial order

    spy_accept_order.reset_mock()
    spy_send_adapter_navigate_to_node.reset_mock()

    # Send update order
    order = get_order_update(order_id, 1)  # Same order id
    node.process_order(order)
    node._on_active_order()

    spy_accept_order.assert_called_once_with(order=order, mode=OrderAcceptModes.UPDATE)

    # check node states were properly updated
    assert node._current_order == order
    assert node._current_state.order_id == order_id
    assert node._current_state.order_update_id == 1

    # The order has 2 nodes and 1 edges but the first edge and node
    # are processed as soon as the order is accepted.
    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1

    assert node._current_state.last_node_id == "node1"
    assert node._current_state.last_node_sequence_id == 8

    # Assert the first navigation goal was sent to the adapter,
    # and that the parameters matches order's first edge and second node.
    # Note: the standard assumes the vehicle is on the first node already,
    # so the first navigation command is to the second order node.
    spy_send_adapter_navigate_to_node.assert_called_once_with(
        edge=order.edges[0], node=order.nodes[1]
    )

    # Future for invoking adapter navigation goal result callback
    future = get_navigation_result_future()

    # Simulate the adapter reached navigation goal
    node._navigate_to_node_result_callback(future)
    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 10


def test_vda5050_controller_node_stitch_order(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate accept order is called correctly
    spy_accept_order = mocker.spy(node, "_accept_order")

    # Get the base order and the stitch order, both with two nodes
    # and one edge each. The resulting (stitched) order should contain
    # three nodes and two edges:
    # (A -> B) + (B -> C) = A -> B -> C
    # All three nodes have an action.
    [base_order, stitch_order] = get_stitch_orders()

    node.process_order(base_order)

    rclpy.spin_once(node)
    rclpy.spin_once(adapter_node)

    # Simulate the adapter reached navigation goals
    future = get_navigation_result_future()

    # The base order contains 2 nodes and 1 edge. The first node (in deviation range)
    # is processed and removed, and 1 node is sent to navigate to.
    node._navigate_to_node_result_callback(future)

    assert len(node._current_order.nodes) == 2
    assert len(node._current_order.edges) == 1
    assert len(node._current_state.action_states) == 2

    spy_accept_order.reset_mock()
    # Process the stitching order
    node.process_order(stitch_order)
    spy_accept_order.assert_called_once_with(order=stitch_order, mode=OrderAcceptModes.STITCH)

    assert node._current_state.order_id == stitch_order.order_id
    assert node._current_state.order_update_id == 1

    assert len(node._current_order.nodes) == 3
    assert len(node._current_order.edges) == 2

    assert len(node._current_state.node_states) == 1
    assert len(node._current_state.edge_states) == 1
    assert len(node._current_state.action_states) == 3

    assert node._current_state.last_node_id == "node2"
    assert node._current_state.last_node_sequence_id == 2

    node._navigate_to_node_result_callback(future)
    assert len(node._current_state.node_states) == 0
    assert len(node._current_state.edge_states) == 0
    assert len(node._current_state.action_states) == 3

    assert node._current_state.last_node_id == "node3"
    assert node._current_state.last_node_sequence_id == 4


def test_vda5050_controller_node_stitch_order_without_actions_on_stitch_node(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """Stitching on an action-less node must not drop the already tracked action states."""
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    spy_accept_order = mocker.spy(node, "_accept_order")

    [base_order, stitch_order] = get_stitch_orders_no_actions_on_stitch_node()
    action1 = base_order.nodes[0].actions[0]
    action3 = stitch_order.nodes[1].actions[0]

    node.process_order(base_order)
    assert len(node._current_state.action_states) == 1

    node._update_action_status(action1.action_id, CurrentAction.FINISHED)

    spy_accept_order.reset_mock()
    node.process_order(stitch_order)
    spy_accept_order.assert_called_once_with(order=stitch_order, mode=OrderAcceptModes.STITCH)

    action_states = {
        action_state.action_id: action_state
        for action_state in node._current_state.action_states
    }
    assert len(action_states) == 2
    assert action_states[action1.action_id].action_status == CurrentAction.FINISHED
    assert action_states[action3.action_id].action_status == CurrentAction.WAITING


def test_vda5050_controller_execute_node_actions_restores_untracked_action(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """An action without a matching action state is restored instead of crashing the node."""
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    mock_send_action = mocker.patch.object(node, "send_adapter_process_vda_action")

    action = Action(
        action_type="foo",
        action_id=str(uuid4()),
        action_description="Foo description",
        blocking_type="HARD",
    )
    node._current_node_actions = [action]
    node._current_state.action_states = []

    node._execute_node_actions()

    assert len(node._current_state.action_states) == 1
    assert node._current_state.action_states[0].action_id == action.action_id
    assert node._current_state.action_states[0].action_status == CurrentAction.WAITING
    mock_send_action.assert_called_once_with(action)


def test_vda5050_controller_on_active_order_reports_unhandled_exceptions(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """An unhandled exception is reported once instead of killing the controller."""
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    mocker.patch.object(node, "_has_current_order", side_effect=RuntimeError("boom"))

    node._on_active_order()
    node._on_active_order()

    internal_errors = [
        error
        for error in node._current_state.errors
        if error.error_type == OrderExecutionErrors.INTERNAL_ERROR.value
    ]
    assert len(internal_errors) == 1
    assert "boom" in internal_errors[0].error_description


def test_vda5050_controller_node_reject_order(
    mocker,
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    node = VDA5050Controller()
    node.logger.set_level(LoggingSeverity.DEBUG)

    # add a spy to validate that the order has been rejected
    spy_reject_order = mocker.spy(node, "_reject_order")

    # UPDATE test fail - lower order_update_id

    # Send first new order
    order_id = str(uuid4())
    order = get_order_new(order_id, 1)
    node.process_order(order)

    # Simulate the adapter reached navigation goals
    future = get_navigation_result_future()

    # The NEW order contains 5 nodes and 4 edges. The first node (in deviation range)
    # is processed and remove, and 4 nodes are send to navigate to.
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    node._navigate_to_node_result_callback(future)
    # Finish initial order

    spy_reject_order.reset_mock()

    order = get_order_update(order_id, 0)  # Same order id, lower order_update_id
    node.process_order(order)

    spy_reject_order.assert_called_once_with(
        order=order,
        error=OrderRejectErrors.ORDER_UPDATE_ERROR,
        description="New update id 0 lower than old update id 1",
    )


def _make_navigate_through_nodes_feedback(x, y):
    """Create a minimal feedback message stub with an AGVPosition at (x, y)."""

    class _Feedback:
        position = AGVPosition(x=x, y=y)

    class _FeedbackMsg:
        feedback = _Feedback()

    return _FeedbackMsg()


def _build_navigate_through_nodes_state(node):
    """
    Set up a VDA5050Controller with three nodes and two edges for
    navigate-through-nodes single-pop and no-pop tests.

    Nodes are placed on a straight line along the X axis:
      node1 @ (0, 0)   sequence_id=0  – already traversed (last_node)
      node2 @ (10, 0)  sequence_id=2  – first running node
      node3 @ (20, 0)  sequence_id=4  – second running node (destination)

    Edges:
      edge1 sequence_id=1 (node1 -> node2)
      edge2 sequence_id=3 (node2 -> node3)

    The controller's _running_nodes, _running_edges, _unexecuted_nodes,
    _unexecuted_edges, and _current_state.node_states / edge_states are
    populated accordingly.  last_node is set to "node1" (already traversed).
    """
    nodes = [
        Node(
            node_id="node1",
            sequence_id=0,
            released=True,
            node_position=NodePosition(x=0.0, y=0.0, map_id="map"),
        ),
        Node(
            node_id="node2",
            sequence_id=2,
            released=True,
            node_position=NodePosition(x=10.0, y=0.0, map_id="map"),
        ),
        Node(
            node_id="node3",
            sequence_id=4,
            released=True,
            node_position=NodePosition(x=20.0, y=0.0, map_id="map"),
        ),
    ]
    edges = [
        Edge(
            edge_id="edge1",
            sequence_id=1,
            released=True,
            start_node_id="node1",
            end_node_id="node2",
        ),
        Edge(
            edge_id="edge2",
            sequence_id=3,
            released=True,
            start_node_id="node2",
            end_node_id="node3",
        ),
    ]

    # The first node is already reached; the remaining two are in the running lists.
    node._running_nodes = list(nodes[1:])  # node2, node3
    node._running_edges = list(edges)      # edge1, edge2
    node._unexecuted_nodes = list(nodes[1:])
    node._unexecuted_edges = list(edges)

    node._current_state.node_states = [
        NodeState(node_id=n.node_id, sequence_id=n.sequence_id, released=n.released)
        for n in nodes[1:]
    ]
    node._current_state.edge_states = [
        EdgeState(edge_id=e.edge_id, sequence_id=e.sequence_id, released=e.released)
        for e in edges
    ]
    node._current_state.last_node_id = "node1"
    node._current_state.last_node_sequence_id = 0

    return nodes, edges


def _build_navigate_through_nodes_state_multi_pop(node):
    """
    Set up a VDA5050Controller with four nodes and three edges for
    the multi-pop feedback test.

    Nodes are clustered very close together so a single robot position
    can be within the arrival radius (0.5 m) of multiple consecutive nodes:
      node1 @ (0.0, 0)   sequence_id=0  – already traversed (last_node)
      node2 @ (0.1, 0)   sequence_id=2  – 1st running node (0.1 m from origin)
      node3 @ (0.2, 0)   sequence_id=4  – 2nd running node (0.2 m from origin)
      node4 @ (20.0, 0)  sequence_id=6  – final destination (far away)

    With the robot at (0.1, 0):
      - (0.1,0) is within 0.5 m of node2 (dist=0.0)  → pop
      - (0.1,0) is within 0.5 m of node3 (dist=0.1)  → pop
      - loop exits because only node4 remains (_running_nodes length drops to 1)

    Edges:
      edge1 sequence_id=1, edge2 sequence_id=3, edge3 sequence_id=5
    """
    nodes = [
        Node(
            node_id="node1",
            sequence_id=0,
            released=True,
            node_position=NodePosition(x=0.0, y=0.0, map_id="map"),
        ),
        Node(
            node_id="node2",
            sequence_id=2,
            released=True,
            node_position=NodePosition(x=0.1, y=0.0, map_id="map"),
        ),
        Node(
            node_id="node3",
            sequence_id=4,
            released=True,
            node_position=NodePosition(x=0.2, y=0.0, map_id="map"),
        ),
        Node(
            node_id="node4",
            sequence_id=6,
            released=True,
            node_position=NodePosition(x=20.0, y=0.0, map_id="map"),
        ),
    ]
    edges = [
        Edge(
            edge_id="edge1",
            sequence_id=1,
            released=True,
            start_node_id="node1",
            end_node_id="node2",
        ),
        Edge(
            edge_id="edge2",
            sequence_id=3,
            released=True,
            start_node_id="node2",
            end_node_id="node3",
        ),
        Edge(
            edge_id="edge3",
            sequence_id=5,
            released=True,
            start_node_id="node3",
            end_node_id="node4",
        ),
    ]

    # node1 is already at last_node; node2/node3/node4 are the running nodes.
    node._running_nodes = list(nodes[1:])   # node2, node3, node4
    node._running_edges = list(edges)       # edge1, edge2, edge3
    node._unexecuted_nodes = list(nodes[1:])
    node._unexecuted_edges = list(edges)

    node._current_state.node_states = [
        NodeState(node_id=n.node_id, sequence_id=n.sequence_id, released=n.released)
        for n in nodes[1:]
    ]
    node._current_state.edge_states = [
        EdgeState(edge_id=e.edge_id, sequence_id=e.sequence_id, released=e.released)
        for e in edges
    ]
    node._current_state.last_node_id = "node1"
    node._current_state.last_node_sequence_id = 0

    return nodes, edges


def test_navigate_through_nodes_feedback_callback_single_pop(
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """
    Feedback that puts the robot within radius of the first running node
    (node2) but not the second (node3) should pop exactly one node/edge.
    """
    controller = VDA5050Controller()
    controller.logger.set_level(LoggingSeverity.DEBUG)

    _build_navigate_through_nodes_state(controller)

    # Position is at node2 exactly – within the default 0.5 m radius.
    feedback_msg = _make_navigate_through_nodes_feedback(x=10.0, y=0.0)
    controller._navigate_through_nodes_feedback_callback(feedback_msg)

    # node2 / edge1 should have been popped; node3 / edge2 remain.
    assert len(controller._running_nodes) == 1
    assert controller._running_nodes[0].node_id == "node3"

    assert len(controller._running_edges) == 1
    assert controller._running_edges[0].edge_id == "edge2"

    assert len(controller._unexecuted_nodes) == 1
    assert controller._unexecuted_nodes[0].node_id == "node3"
    assert len(controller._unexecuted_edges) == 1
    assert controller._unexecuted_edges[0].edge_id == "edge2"

    assert len(controller._current_state.node_states) == 1
    assert controller._current_state.node_states[0].node_id == "node3"

    assert len(controller._current_state.edge_states) == 1
    assert controller._current_state.edge_states[0].edge_id == "edge2"

    assert controller._current_state.last_node_id == "node2"
    assert controller._current_state.last_node_sequence_id == 2


def test_navigate_through_nodes_feedback_callback_multi_pop(
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """
    When the robot's reported position is within the arrival radius of
    multiple consecutive running nodes, the callback must pop all of them
    in a single invocation (multi-pop).

    Setup: four nodes where node2 (0.1 m) and node3 (0.2 m) are very close
    to the robot position (0.1, 0), so both are within the 0.5 m radius.
    node4 is far away (20 m). The loop must:
      1. pop node2/edge1 on the first iteration
      2. pop node3/edge2 on the second iteration
      3. stop because only node4 remains (_running_nodes length drops to 1)
    """
    controller = VDA5050Controller()
    controller.logger.set_level(LoggingSeverity.DEBUG)

    _build_navigate_through_nodes_state_multi_pop(controller)

    # Robot at (0.1, 0) – within 0.5 m of both node2 (dist=0.0) and
    # node3 (dist=0.1), but not node4 (dist≈19.9).
    feedback_msg = _make_navigate_through_nodes_feedback(x=0.1, y=0.0)
    controller._navigate_through_nodes_feedback_callback(feedback_msg)

    # Both node2/edge1 AND node3/edge2 must have been popped.
    # Only node4/edge3 remains.
    assert len(controller._running_nodes) == 1
    assert controller._running_nodes[0].node_id == "node4"

    assert len(controller._running_edges) == 1
    assert controller._running_edges[0].edge_id == "edge3"

    assert len(controller._unexecuted_nodes) == 1
    assert controller._unexecuted_nodes[0].node_id == "node4"

    assert len(controller._unexecuted_edges) == 1
    assert controller._unexecuted_edges[0].edge_id == "edge3"

    assert len(controller._current_state.node_states) == 1
    assert controller._current_state.node_states[0].node_id == "node4"

    assert len(controller._current_state.edge_states) == 1
    assert controller._current_state.edge_states[0].edge_id == "edge3"

    # last_node must reflect the second popped node (node3), not just the first.
    assert controller._current_state.last_node_id == "node3"
    assert controller._current_state.last_node_sequence_id == 4


def test_navigate_through_nodes_feedback_callback_no_pop_outside_radius(
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """
    Feedback with position outside the arrival radius of the first running
    node should leave all state untouched.
    """
    controller = VDA5050Controller()
    controller.logger.set_level(LoggingSeverity.DEBUG)

    _build_navigate_through_nodes_state(controller)

    # Position is far from both nodes (midpoint between node1 and node2).
    feedback_msg = _make_navigate_through_nodes_feedback(x=5.0, y=0.0)
    controller._navigate_through_nodes_feedback_callback(feedback_msg)

    # Nothing should be popped.
    assert len(controller._running_nodes) == 2
    assert len(controller._running_edges) == 2

    assert len(controller._current_state.node_states) == 2
    assert len(controller._current_state.edge_states) == 2

    assert controller._current_state.last_node_id == "node1"
    assert controller._current_state.last_node_sequence_id == 0


def test_navigate_through_nodes_result_callback_success_multi_skipped(
    adapter_node,
    action_server_nav_to_node,
    action_server_process_vda_action,
    service_get_state,
    service_supported_actions,
):
    """
    When a successful navigation result arrives and multiple intermediate nodes
    still remain in ``_running_nodes`` (proximity-based feedback never detected
    them as reached), the result callback must:

    * remove ALL traversed edge states from ``edge_states`` and
      ``_unexecuted_edges``,
    * remove skipped (intermediate) node states from ``node_states`` and
      ``_unexecuted_nodes``,
    * clear both ``_running_nodes`` and ``_running_edges``, and
    * leave only the final destination node for ``_process_node`` to handle
      (verified through its state side-effects on ``node_states`` and
      ``last_node_id``/``last_node_sequence_id``).

    Setup uses the multi-pop fixture: node2 / node3 / node4 in
    ``_running_nodes``; edge1 / edge2 / edge3 in ``_running_edges``.
    Neither node2 nor node3 was ever detected as reached by the feedback
    callback, so both are "skipped" intermediate nodes.
    """
    controller = VDA5050Controller()
    controller.logger.set_level(LoggingSeverity.DEBUG)

    _build_navigate_through_nodes_state_multi_pop(controller)

    # Simulate a successful action result: no error, status not ABORTED.
    class _MockResultResponse:
        class _Result:
            error = False
            error_code = 0

        result = _Result()
        status = GoalStatus.STATUS_SUCCEEDED

    future = Future()
    future.set_result(_MockResultResponse())

    # Invoke the implementation directly to bypass goal-handle teardown.
    controller._navigate_through_nodes_result_callback_impl(future)

    # Both running lists must be emptied by the callback.
    assert controller._running_nodes == []
    assert controller._running_edges == []

    # All three edges were traversed → edge_states and _unexecuted_edges
    # must be empty.
    assert controller._current_state.edge_states == []
    assert controller._unexecuted_edges == []

    # _process_node(node4) removes the final node from node_states and
    # updates last_node, so node_states and _unexecuted_nodes must be empty.
    assert controller._current_state.node_states == []
    assert controller._unexecuted_nodes == []

    # last_node must reflect the final destination (node4), not an
    # intermediate skipped node.
    assert controller._current_state.last_node_id == "node4"
    assert controller._current_state.last_node_sequence_id == 6
