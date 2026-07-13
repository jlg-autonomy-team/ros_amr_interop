# VDA5050 Connector — Software Specification

| | |
|---|---|
| **Package** | `vda5050_connector` |
| **Version** | 1.1.1 |
| **Document status** | Baseline, derived from the current implementation |
| **Applies to** | ROS 2 (ament_cmake), VDA5050 protocol versions 1.1.0 and 2.0.0 |
| **License** | BSD 3-Clause |

This document is a requirements-style specification of the `vda5050_connector`
package as implemented in this repository, including the JLG-specific
extensions (marked `JLG_CHANGES` in the source). Requirement IDs use the
prefix of the component they belong to: **GEN** (general), **BRG** (MQTT
bridge), **CTRL** (controller), **ADP** (adapter), **IF** (interfaces),
**CFG** (configuration), **UTL** (utilities), **TST** (testing).

Requirement keywords follow RFC 2119 ("shall" = mandatory as implemented,
"should" = recommended, "may" = optional).

---

## 1. Purpose and Scope

### 1.1 Purpose

The VDA5050 Connector is a ROS 2 package that bridges a VDA5050 Master
Control (MC) — a central fleet manager communicating over MQTT — and the
ROS 2 API of an AGV/AMR. It implements the robot-side behavior of the
[VDA5050](https://github.com/VDA5050/VDA5050/blob/main/VDA5050_EN.md)
standardized interface: order validation and execution, instant action
processing, and periodic state/connection/visualization/factsheet reporting.

### 1.2 System Context

```
                MQTT (VDA5050 JSON)          ROS 2 (vda5050_msgs)         ROS 2 (actions/services)
Master Control <====================> MQTT Bridge <==============> Controller <===============> Adapter <--> Robot API
                                        (Python)                    (Python)                    (C++ base +
                                                                                                 vendor plugins)
```

The package provides three cooperating ROS 2 nodes:

1. **MQTT Bridge** (`mqtt_bridge`, Python): translates VDA5050 JSON messages
   on MQTT topics to/from ROS 2 `vda5050_msgs` topics.
2. **Controller** (`controller`, Python): implements the VDA5050 order and
   instant-action state machines; assembles and publishes robot state.
3. **Adapter** (`adapter`, C++ library/node): robot-specific integration
   layer. Exposes services/actions consumed by the controller and delegates
   robot operations to `pluginlib` handler plugins.

### 1.3 Scope of Delivery

- **GEN-01** The package shall build with `ament_cmake` and shall be both an
  executable package (bridge + controller nodes, launch files) and a library
  package: it shall export the adapter headers (`include/vda5050_connector/`)
  and the `adapter` shared library so vendors can build their own adapter
  packages against it.
- **GEN-02** The package shall generate and export the ROS interfaces listed
  in §5 (`rosidl_generate_interfaces`, member of `rosidl_interface_packages`).
- **GEN-03** The package shall depend on: `rclcpp`, `rclcpp_action`, `rclpy`,
  `pluginlib`, `tf2`, `geometry_msgs`, `vda5050_msgs` (ROS 2 messages for
  VDA5050 v2), `python3-paho-mqtt`, and `rosidl_runtime_py`. The JLG
  extensions additionally require `talos_msgs` (DTC diagnostics), `jsonschema`
  (payload validation), and the VDA5050 JSON schema files installed to
  `share/vda5050_connector/config/json_schemas` (see CFG-10).
- **GEN-04** Protocol versions `1.1.0` and `2.0.0` shall be supported
  (`SUPPORTED_PROTOCOL_VERSIONS`); the internal representation of all messages
  shall be VDA5050 v2 (`vda5050_msgs`).

---

## 2. MQTT Bridge (`vda5050_connector_py/mqtt_bridge.py`)

### 2.1 Node and Connectivity

- **BRG-01** The bridge shall run as ROS 2 node `mqtt_bridge` and shall use
  the `paho-mqtt` client (callback API v2, `clean_session=True`,
  `reconnect_on_failure=True`) with a threaded network loop (`loop_start`).
- **BRG-02** The bridge shall connect asynchronously (`connect_async`,
  keepalive 60 s) to the broker at parameters `mqtt_address:mqtt_port`, with
  automatic reconnection using exponential backoff bounded to [1 s, 30 s]
  (`reconnect_delay_set`), max 20 in-flight messages and a 100-message
  outbound queue.
- **BRG-03** If `mqtt_username` is non-empty, the bridge shall enable TLS 1.2
  using the CA bundle from environment variable
  `VDA5050_CONNECTOR_TLS_CA_CERT` (default
  `/etc/ssl/certs/ca-certificates.crt`) and shall authenticate with
  `mqtt_username`/`mqtt_password`.
- **BRG-04** Before connecting, the bridge shall register an MQTT Last Will
  message on the `connection` topic (QoS 1, retained) containing a VDA5050
  Connection message with `connectionState=CONNECTIONBROKEN` and fixed
  timestamp `1970-01-01T12:00:00.00Z`, so the MC detects unexpected
  disconnects.
- **BRG-05** On successful connect, the bridge shall subscribe to the `order`
  and `instantActions` MQTT topics and shall publish a Connection message
  with `connectionState=ONLINE`.
- **BRG-06** On graceful shutdown, the bridge shall publish a Connection
  message with `connectionState=OFFLINE` (continuing the `header_id` sequence
  of the last Connection message it relayed), unsubscribe from `order` and
  `instantActions`, disconnect, and stop the network loop.
- **BRG-07** On unsolicited disconnect (`rc != 0`), the bridge shall log the
  return code and rely on the paho automatic reconnect (BRG-02); it shall not
  block in a manual reconnect loop.

### 2.2 Topic Structure

- **BRG-08** MQTT topics shall follow the VDA5050 suggested hierarchy
  `{interfaceName}/{majorVersion}/{manufacturer}/{serialNumber}/{topic}`
  (e.g. `uagv/v2/robots/robot_1/order`), where `majorVersion` is derived from
  parameter `vda5050_protocol_version` as `v<major>` (`generate_vda5050_topic_alias`);
  an unsupported version shall raise `ValueError` at startup.
- **BRG-09** Valid `topic` values are exactly: `order`, `state`,
  `visualization`, `instantActions`, `connection`, `factsheet`; any other
  value shall raise `ValueError` (UTL-05).
- **BRG-10** The corresponding ROS 2 topics shall use the identical structure
  prefixed with `/` (e.g. `/uagv/v2/robots/robot_1/order`).

### 2.3 MQTT → ROS 2 (inbound)

- **BRG-11** For each inbound MQTT message the bridge shall convert the JSON
  payload keys from camelCase to snake_case recursively.
- **BRG-12** Payloads that fail JSON decoding shall be logged and dropped
  (and, with validation enabled, shall latch the invalid-order DTC, BRG-17).
- **BRG-13** Inbound `order` payloads shall be converted to
  `vda5050_msgs/Order` and republished on the ROS `order` topic; inbound
  `instantActions` payloads shall be converted to
  `vda5050_msgs/InstantActions` and republished on the ROS `instantActions`
  topic. Conversion shall:
  - coerce node-position and edge numeric fields (`x`, `y`, `theta`,
    `max_speed`, `max_height`, `min_height`, `orientation`,
    `max_rotation_speed`, `length`) to `float`;
  - coerce all action-parameter values to `str`;
  - build `Trajectory` messages with integer `degree`, float control-point
    coordinates and a default control-point `weight` of 1;
  - accept both the v1 field name `instantActions` and the v2 field name
    `actions` for the instant-actions array, normalizing to v2.
- **BRG-14** Any exception while converting/publishing an inbound message
  shall be caught and logged, the message shall be ignored, and (validation
  enabled) the invalid-order DTC shall be latched; malformed traffic shall
  never crash the bridge.

### 2.4 Inbound Payload Validation (JLG extension)

- **BRG-15** When parameter `enable_vda5050_validation` is true (default),
  each inbound `order` / `instantActions` payload shall be validated against
  the corresponding VDA5050 JSON schema (`order.schema` /
  `instantActions.schema`, JSON Schema draft 2020-12 with format checking)
  before conversion. All validation errors shall be logged with their JSON
  path; invalid payloads shall not be forwarded to ROS.
- **BRG-16** Additionally, all `order_id`/`node_id`/`edge_id`/`action_id`
  values in the payload shall be non-empty and mutually unique
  (`has_unique_ids`); violation shall reject the payload.
- **BRG-17** The bridge shall report validation health through the Talos DTC
  diagnostic interface:
  - on every inbound MQTT message it shall first unlatch DTC
    `invalid_order_dtc` (default **2460**) via service
    `diagnostics/unlatch_dtc`;
  - on any decode/validation/conversion failure it shall force-latch that DTC
    via `diagnostics/force_latch_dtc`;
  - both calls shall be asynchronous, tolerate an unavailable service (log
    and continue), and log the service result.

### 2.5 ROS 2 → MQTT (outbound)

- **BRG-18** The bridge shall subscribe to the ROS `state`, `connection`, and
  `visualization` topics and republish each message on the matching MQTT
  topic, converting the ROS message to JSON with all keys converted from
  snake_case to camelCase.
- **BRG-19** If an MQTT publish fails (`rc != MQTT_ERR_SUCCESS`), the bridge
  shall log the error and publish DTC `broker_comm_loss_dtc` (default
  **2456**) on topic `diagnostics/dtc` (JLG extension).

---

## 3. Controller (`vda5050_connector_py/vda5050_controller.py`)

### 3.1 Node, Lifecycle, Interfaces

- **CTRL-01** The controller shall run as node `controller` in namespace
  `vda5050` and shall be spun by a `MultiThreadedExecutor`
  (`scripts/vda5050_controller.py`).
- **CTRL-02** On startup the controller shall block until the adapter
  interfaces are available, logging an error each 1 s wait cycle:
  `NavigateToNode` and `ProcessVDAAction` action servers, `GetState` and
  `SupportedActions` services, and — only when
  `enable_navigate_through_nodes` is true — the `NavigateThroughNodes`
  action server.
- **CTRL-03** All controller↔adapter interfaces shall be named
  `{namespace}/{manufacturer_name}/{robot_name}/{interface}`, where the
  interface leaf names are configurable parameters (defaults:
  `adapter/get_state`, `adapter/supported_actions`, `adapter/vda_action`,
  `adapter/nav_to_node`, and fixed `adapter/nav_through_nodes`).
- **CTRL-04** The controller shall subscribe to the ROS `order` and
  `instantActions` topics and publish to the ROS `state`, `connection`,
  `visualization`, and `factsheet` topics (topic naming per BRG-10).
- **CTRL-05** The controller shall maintain, with `header_id` monotonically
  increasing per message type and VDA5050 ISO-8601 UTC timestamps:
  - an `OrderState` published every `state_pub_period` (default 5 s) and
    immediately on significant events (order accepted/rejected, errors,
    action results, `new_base_request`);
  - a `Connection` message with `connectionState=ONLINE` published every
    `connection_pub_period` (default 15 s);
  - a `Visualization` message published every `visualization_pub_period`
    (default 1 s), refreshing AGV position and velocity from the adapter
    (synchronous `GetState` call) before each publish;
  - a `Factsheet` published on demand (CTRL-13).
- **CTRL-06** The initial `OrderState` shall carry the configured protocol
  version, manufacturer, serial number, `last_node_id = starting_node_id`,
  `operating_mode=AUTOMATIC`, and a safety state of `e_stop=NONE`,
  `field_violation=false`.

### 3.2 Adapter State Aggregation

- **CTRL-07** `get_state_from_adapter` shall support synchronous and
  asynchronous `GetState` calls. Each response shall update the current
  order state's robot-specific fields: `agv_position`, `velocity`, `loads`,
  `driving`, `paused`, `distance_since_last_node`, `battery_state`, and (JLG
  extension) `information`, `operating_mode`, and `safety_state`; it shall
  also update the Visualization message's position/velocity.
- **CTRL-08** Adapter-reported errors shall replace previously reported
  adapter errors, while controller-generated errors (order-reject, action,
  and order-execution error types) shall be preserved across state refreshes.
- **CTRL-09 (JLG)** The retained error list shall be capped at the most
  recent **25** entries (`MAX_RETAINED_ERRORS`) whenever errors are updated,
  to bound state-message growth.

### 3.3 Instant Actions

- **CTRL-10** On receipt of an `InstantActions` message, each action shall be
  appended to `action_states` with status `WAITING` and processed as follows:
  - `cancelOrder`: stored as the pending cancel action, handled by the order
    state machine (CTRL-27);
  - `stateRequest`: set `RUNNING`, request adapter state asynchronously, then
    mark `FINISHED` and publish the updated state;
  - `factsheetRequest`: populate (once) and publish the factsheet, mark
    `FINISHED`;
  - any other type: forwarded to the adapter via the `ProcessVDAAction`
    action (CTRL-11).
- **CTRL-10a (JLG)** An instant action whose `action_id` already exists in
  `action_states` shall be recognized as a duplicate, logged, and skipped.
- **CTRL-11** VDA actions shall only be sent to the adapter while in state
  `WAITING`; on send they shall transition to `INITIALIZING`, then track the
  adapter's feedback (`action_status` updates) and final result (status plus
  `result_description`, the latter recorded only for terminal states
  `FINISHED`/`FAILED`).
- **CTRL-12** If the adapter rejects a VDA action goal, the action shall be
  marked `FAILED` with result description "Action type … is not supported"
  (JLG behavior). If an action finishes `FAILED`, an `actionFailed` error
  (level `WARNING`, reference `action_id`) shall be published once and then
  removed from the retained error list.
- **CTRL-12a (JLG)** Blocking behavior: when an accepted VDA action has
  `blocking_type != NONE`, the controller shall set an *active block* (no new
  navigation/action dispatch, CTRL-26) and shall flag *retry current node* if
  navigation was active or a retry was already pending. The block shall be
  released when the action result arrives.
- **CTRL-12b (JLG)** Pause handling: successful completion of a `startPause`
  / `stopPause` action shall set / clear the *active pause* flag; while
  paused the order state machine shall not dispatch work (CTRL-26), and a
  navigation result arriving while paused shall set the retry flag rather
  than advance the order.
- **CTRL-13** The factsheet shall be populated once (guarded by
  `header_id != 0`) from ROS parameters under the `factsheet.*` namespace
  (type specification, physical parameters, protocol limits, AGV geometry —
  wheel definitions and 2D/3D envelopes keyed by `ids` arrays — and load
  specification), plus the AGV actions returned by the adapter's
  `SupportedActions` service for `protocol_features`. Malformed
  `polygon_points` entries shall be logged and skipped.
- **CTRL-14** Updating the status of an unknown `action_id` shall publish a
  transient `actionNotFound` error (level `WARNING`, reference `action_id`)
  without modifying retained errors.

### 3.4 Order Acceptance (VDA5050 Figure 8)

- **CTRL-15** On receipt of an `Order` the controller shall decide among
  ACCEPT (modes `NEW` / `UPDATE` / `STITCH`), REJECT, or DISCARD:
  - different `order_id` with no active order and first node in deviation
    range → `NEW`;
  - different `order_id` with an active order → reject `orderUpdateError`
    ("There is an active running order.");
  - different `order_id`, first node out of deviation range → reject
    `noRouteError` (note: deviation-range checking is currently a stub that
    always passes, CTRL-16);
  - same `order_id`, same `order_update_id` → discard silently (log only);
  - same `order_id`, lower `order_update_id` → reject `orderUpdateError`;
  - same `order_id`, higher `order_update_id`, stitch nodes matching → `STITCH`
    if an order is active, else `UPDATE`;
  - stitch mismatch → reject `orderUpdateError`.
- **CTRL-16** Message-level validation hooks (`order_msg_is_valid`,
  `instant_action_msg_is_valid`) and deviation-range checking
  (`_first_node_in_deviation_range`) are placeholder implementations that
  currently accept everything; full implementations remain TODO. (Inbound
  schema validation is instead performed by the bridge, BRG-15/16.)
- **CTRL-17** Stitch validation (`_match_stitch_nodes`) shall require the
  last released node of the current base and the first node of the new base
  to have identical `node_id`, `sequence_id`, node position (x, y, theta,
  allowed deviations, map id/description), and identical action lists
  (type, id, blocking type, description, and all parameters, in order).
- **CTRL-18** An active order exists (`_has_current_order`) iff any action
  state other than a pending `cancelOrder` is non-terminal, or any node/edge
  states remain.
- **CTRL-19** Accepting an order shall update the state atomically:
  `order_id`, `order_update_id`, node/edge states derived from the order
  (preserving base states only on STITCH, and avoiding duplication of the
  stitch node), action states initialized to `WAITING` for all base
  node/edge actions, `new_base_request=false`, and removal of previous
  order-reject and order-execution errors. On `NEW`, prior action states
  shall be deleted.
- **CTRL-20** On NEW/UPDATE acceptance, the first node of the order (where
  the standard assumes the robot already is) shall be processed immediately
  (CTRL-24); on STITCH the stitch node shall not be re-processed and its
  node state shall be removed. **(JLG)** The unexecuted node/edge lists used
  by multi-node navigation shall be (re)initialized from released
  nodes/edges accordingly.
- **CTRL-21** Rejection shall publish an error of the given type with level
  `WARNING`, a description embedding order id/update id and reason, and
  references: `order_id` + `order_update_id` for `orderUpdateError`,
  first `node_id` for `noRouteError`. Reject errors persist until a
  subsequent order is accepted.

### 3.5 Order Execution State Machine

- **CTRL-22** A periodic timer (period `execute_order_period`, default
  0.1 s) shall drive execution with the following precedence:
  1. pending `cancelOrder` → run cancel procedure (CTRL-27);
  2. **(JLG)** navigation error flagged → run kill procedure (CTRL-28);
  3. no active order → idle;
  4. **(JLG)** active pause or active block → idle;
  5. pending current-node actions → execute them (CTRL-25);
  6. not navigating → dispatch next navigation goal: multi-node planning
     (CTRL-29 ff.) when `enable_navigate_through_nodes` is true, otherwise
     single-edge traversal (CTRL-23).
- **CTRL-23** Single-node mode: the next edge is the one with
  `sequence_id == last_node_sequence_id + 1`; if absent, idle. If the edge is
  not released (horizon), the controller shall set `new_base_request=true`
  (published once) and wait. Otherwise the node with `sequence_id + 2` shall
  be sent as a `NavigateToNode` goal — including **(JLG)** `final=true` when
  it is the last remaining node — unless that node is already the current
  goal and no retry is pending.
- **CTRL-24** On arrival at a node (`_process_node`): its node state shall be
  removed, `last_node_id`/`last_node_sequence_id` updated, **(JLG)** the node
  removed from the unexecuted list, and its actions queued as the current
  node actions. Completion of the last node state shall be logged as order
  finished.
- **CTRL-25** Node actions shall execute per VDA5050 Figure 15: terminal
  actions are pruned; remaining actions are grouped by blocking type; `HARD`
  actions run strictly serially (first one only per tick); then `SOFT` and
  `NONE` actions are dispatched in parallel. **(JLG)** Presence of a
  `precisionLocation` action shall set the *only-take-next-node* flag used
  by multi-node planning (CTRL-29).
- **CTRL-26** While an active pause or active block is set, no navigation
  goals or node actions shall be dispatched (see CTRL-12a/12b).

### 3.6 Order Cancellation and Failure Handling

- **CTRL-27** `cancelOrder` (VDA5050 Figure 9): with no active order, the
  cancel action shall be marked `FAILED` and a `noOrderToCancel` error
  (level `WARNING`, reference `action_id`) published transiently. With an
  active order the procedure shall, over successive timer ticks: mark the
  cancel action `RUNNING`; fail all `WAITING` actions; cancel all running
  VDA action goals; cancel any running navigation goal (both NavigateToNode
  and NavigateThroughNodes handles); then clear node/edge states and
  order-reject errors, reset `new_base_request`, mark the cancel action
  `FINISHED`, and reset the current order to the sentinel `order_id="-1"`.
- **CTRL-28 (JLG)** `_kill_order` shall perform the same teardown as
  CTRL-27 driven by internal failures (navigation error or rejected goal)
  rather than an MC request: fail waiting actions, cancel running
  action/navigation goals, clear the navigation-error flag once quiescent,
  clear node/edge states, and reset the current order. Errors reported to
  the MC prior to the kill (GOAL_REJECTED / NAVIGATION_ERROR) remain until
  the next accepted order.
- **CTRL-28a** Navigation goal outcomes shall be handled as follows (both
  navigation actions):
  - goal rejected by adapter → publish `goalRejectedError` (WARNING,
    reference `node_id`) and kill the order;
  - result with `error=true` → set the navigation-error flag and publish
    `navigationError` with the adapter's `error_code` (WARNING, reference
    `node_id`); the kill procedure follows via CTRL-22;
  - goal `STATUS_ABORTED` → publish `noRouteError` "Failed to reach current
    node." (WARNING, reference `node_id`) and stop advancing;
  - success while cancelling / paused / blocked / retry pending → do not
    advance (pause additionally sets the retry flag);
  - success otherwise → remove the traversed edge state(s) and process the
    reached node (CTRL-24).

### 3.7 Multi-Node Navigation (JLG extension)

- **CTRL-29** When `enable_navigate_through_nodes` is true the controller
  shall batch navigation using the `NavigateThroughNodes` action:
  `_process_goal_list` shall take the contiguous run of released edges and
  the released nodes up to and including the first node that has actions
  (truncating both lists to equal length). If the *only-take-next-node* flag
  is set (previous node had `precisionLocation`), exactly one edge/node pair
  shall be taken instead. An empty batch shall fall back to horizon
  handling (`new_base_request`, as in CTRL-23).
- **CTRL-30** The goal shall carry the batched `edges` and `nodes`; when
  parameter `include_start_node_in_path` is true, the node the robot is
  currently at (matched by `last_node_sequence_id` in the current order)
  shall be prepended to the goal's node list (goal message only — tracking
  state is unaffected, and the node is not duplicated if already first).
- **CTRL-31** During execution, feedback (AGV position) shall be used to pop
  intermediate nodes: while more than one node/edge remain in the running
  lists, a node whose position lies within `node_arrived_radius` (default
  0.5 m, compared with squared distances) shall be treated as reached — its
  node/edge states removed, `last_node_id`/`last_node_sequence_id` updated,
  and the node/edge removed from running and unexecuted lists. The final
  node shall only be processed from the action result.
- **CTRL-32** On successful result, the last batched node shall be processed
  via CTRL-24 and remaining batched edges cleared from the unexecuted list.
  The goal handle shall be cleared only after all state updates complete, so
  the periodic state machine cannot race and re-dispatch the same goal.
  Failure outcomes follow CTRL-28a.

---

## 4. Adapter (C++ base class, `adapter::AdapterNode`)

### 4.1 Node and Plugin Architecture

- **ADP-01** `AdapterNode` shall be an `rclcpp::Node` (default name
  `adapter`, namespace `vda5050`) intended for subclassing/instantiation by
  vendor adapter packages; all configuration hooks are `virtual`.
- **ADP-02** The adapter shall load robot-specific behavior exclusively via
  `pluginlib` against three exported base classes:
  - `adapter::StateHandler` — populate robot data in the shared state;
  - `adapter::NavToNode` — execute navigation to a node (single instance);
  - `adapter::VDAAction` — execute one VDA action type (one instance per
    action name).
- **ADP-03** Plugins shall be declared through ROS parameters (CFG-05):
  `state_handler_names` (string array), `nav_to_node.handler` (string;
  missing value logs a warning and leaves navigation unhandled), and
  `vda_action_handlers` (string array of action names, each with a
  `<snake_case(action)>.handler` class name plus supported-info parameters).
  For each VDA action the adapter shall also read `description`,
  `result_description`, scopes (`scopes.instant/node/edge` booleans) and
  parameter definitions (`parameters` array; per parameter: `data_type`
  (default OBJECT), `description`, `is_optional`) and store them in the
  handler's `AGVAction` support record.
- **ADP-04** Each created handler shall be composed with the node pointer,
  the shared `SafeState`, and the robot name, then `configure()`d;
  pluginlib instantiation failures shall propagate (throw).

### 4.2 Shared State

- **ADP-05** The adapter shall own a `SafeState`: an `OrderState` guarded by
  a `std::shared_mutex` allowing concurrent readers and exclusive writers,
  with member-wise `set_parameter`, append helpers for `information`
  (**JLG:** `vda5050_msgs/JLGInfo`), `loads`, and `errors`, plus `clear()`
  (arrays only) and `reset()` (whole message).
- **ADP-06** Serving `GetState` shall first clear the state arrays and run
  `execute()` on every registered state handler, then return the assembled
  `OrderState` (fresh snapshot per request).

### 4.3 Service and Action Servers

- **ADP-07** The adapter shall expose, under the same
  `{namespace}/{manufacturer}/{robot_name}/` prefix as CTRL-03:
  `GetState` and `SupportedActions` services, and `NavigateToNode` and
  `ProcessVDAAction` action servers. (`NavigateThroughNodes` is defined as an
  interface (IF-02) but has no server in the base class; adapters supporting
  multi-node navigation must implement it.)
- **ADP-08** `SupportedActions` shall return the `AGVAction` support record
  of every configured VDA action handler (consumed by the controller's
  factsheet, CTRL-13).
- **ADP-09** `ProcessVDAAction` goal admission: goals whose `action_type` has
  no registered handler shall be rejected; goals whose handler is currently
  `INITIALIZING`/`RUNNING`/`PAUSED` shall be rejected (one concurrent
  execution per action type). Accepted goals shall execute in a detached
  thread: the handler is `reset()` with the action and goal handle, then
  `execute()` runs its state machine (WAITING → INITIALIZING → RUNNING →
  … → FINISHED/FAILED), publishing a `CurrentAction` feedback on every state
  change. A handler lookup failure during execution shall abort the goal
  with status `FAILED` and result description "Action invalid or
  unsupported.". Cancellation shall be delegated to the handler's `cancel()`
  (base implementation: force `FAILED`, accept).
- **ADP-10** `NavigateToNode` goal admission: a goal shall be rejected while
  the shared state reports `driving=true`; otherwise accepted and executed
  in a detached thread via the NavToNode handler (`reset(node, handle)` then
  `execute()`; exceptions logged). Cancellation shall be delegated to the
  handler's `cancel()`. The handler shall keep the shared `driving` flag
  current via `update_driving_state`.

---

## 5. ROS Interface Definitions

- **IF-01** `action/NavigateToNode.action`:
  - Goal: `bool final` **(JLG:** true when the target is the order's last
    node**)**, `vda5050_msgs/Edge edge`, `vda5050_msgs/Node node`;
  - Result: `bool error`, `uint32 error_code` **(JLG)**;
  - Feedback: `vda5050_msgs/AGVPosition position`,
    `vda5050_msgs/Velocity velocity`.
- **IF-02 (JLG)** `action/NavigateThroughNodes.action`:
  - Goal: `vda5050_msgs/Edge[] edges`, `vda5050_msgs/Node[] nodes`;
  - Result: `bool error`, `uint32 error_code`;
  - Feedback: `vda5050_msgs/AGVPosition position`,
    `vda5050_msgs/Velocity velocity`.
- **IF-03** `action/ProcessVDAAction.action`:
  - Goal: `vda5050_msgs/Action action`;
  - Result: `vda5050_msgs/CurrentAction result`;
  - Feedback: `vda5050_msgs/CurrentAction current_action`.
- **IF-04** `srv/GetState.srv`: empty request → `vda5050_msgs/OrderState state`.
- **IF-05** `srv/SupportedActions.srv`: empty request →
  `vda5050_msgs/AGVAction[] agv_actions`.
- **IF-06** Error taxonomy emitted by the controller (all level `WARNING`):

  | error_type | Source | Trigger |
  |---|---|---|
  | `validationOrder` | order reject | order message invalid |
  | `orderUpdateError` | order reject | active order conflict / stale or mismatched update |
  | `noRouteError` | order reject / execution | first node unreachable; navigation goal aborted |
  | `goalRejectedError` **(JLG)** | execution | adapter rejected a navigation goal |
  | `navigationError` **(JLG)** | execution | navigation result returned `error=true` |
  | `actionNotFound` | actions | status update for unknown action id |
  | `actionFailed` | actions | VDA action ended `FAILED` |
  | `noOrderToCancel` | cancel | `cancelOrder` with no active order |

---

## 6. Configuration Parameters

### 6.1 MQTT Bridge

| Parameter | Type | Default | Requirement |
|---|---|---|---|
| `mqtt_address` | string | `localhost` | BRG-02 |
| `mqtt_port` | int | `1883` | BRG-02 |
| `mqtt_username` / `mqtt_password` | string | `""` | BRG-03 |
| `vda5050_protocol_version` | string | `2.0.0` | BRG-08 |
| `manufacturer_name` | string | `robots` | BRG-08 |
| `serial_number` | string | `robot_1` | BRG-08 |
| `interface_name` | string | `uagv` | BRG-08 |
| `enable_vda5050_validation` **(JLG)** | bool | `true` | BRG-15 |
| `invalid_order_dtc` **(JLG)** | int | `2460` | BRG-17 |
| `broker_comm_loss_dtc` **(JLG)** | int | `2456` | BRG-19 |

### 6.2 Controller

| Parameter | Type | Default | Requirement |
|---|---|---|---|
| `robot_name` | string | `robot_1` | CTRL-03 |
| `manufacturer_name` | string | `robots` | CTRL-03 |
| `serial_number` | string | `robot_1` | CTRL-04 |
| `protocol_version` | string | `2.0.0` | CTRL-05 |
| `starting_node_id` | string | `""` | CTRL-06 |
| `interface_name` | string | `uagv` | CTRL-04 |
| `get_state_svc_name` | string | `adapter/get_state` | CTRL-03 |
| `supported_actions_svc_name` | string | `adapter/supported_actions` | CTRL-03 |
| `vda_action_act_name` | string | `adapter/vda_action` | CTRL-03 |
| `nav_to_node_act_name` | string | `adapter/nav_to_node` | CTRL-03 |
| `state_pub_period` | double (s) | `5.0` | CTRL-05 |
| `connection_pub_period` | double (s) | `15.0` | CTRL-05 |
| `visualization_pub_period` | double (s) | `1.0` | CTRL-05 |
| `execute_order_period` | double (s) | `0.1` | CTRL-22 |
| `enable_navigate_through_nodes` **(JLG)** | bool | `false` | CTRL-29 |
| `node_arrived_radius` **(JLG)** | double (m) | `0.5` | CTRL-31 |
| `include_start_node_in_path` **(JLG)** | bool | `false` | CTRL-30 |
| `factsheet.*` | mixed | zeros / empty | CTRL-13 |

### 6.3 Adapter

- **CFG-05** `robot_name`, `manufacturer_name`, `serial_number`, the four
  interface-name parameters (same defaults as controller), plus the plugin
  declarations of ADP-03.

### 6.4 Launch and Packaging

- **CFG-08** Launch files `launch/controller.launch.py` and
  `launch/mqtt_bridge.launch.py` shall start the respective node under a
  configurable `namespace` (default `vda5050`), loading parameters from
  `parameters_config_file` (default `config/connector_example.yaml`) after
  namespacing the YAML through `RewrittenYaml`.
- **CFG-09** The build shall install: headers, the `adapter` library
  (exported as CMake target `export_vda5050_connector`), the
  `vda5050_connector_py` Python package, both executable scripts to
  `lib/vda5050_connector`, and the `config`/`launch` directories to the
  package share.
- **CFG-10 (JLG)** The build shall install the VDA5050 JSON schema directory
  from `../../VDA5050/json_schemas` (sibling checkout of the VDA5050 repo,
  two levels above the package) to
  `share/vda5050_connector/config/json_schemas`, as required by BRG-15.

---

## 7. Utility Requirements

- **UTL-01** `get_vda5050_ts()` shall produce ISO-8601 UTC timestamps of the
  form `YYYY-MM-DDTHH:mm:ss.ssZ` (hundredths of seconds).
- **UTL-02** Typed parameter helpers (`read_bool/str/int/double/str_array
  _parameter`, Python and C++) shall declare-and-read a parameter with a
  default in one call.
- **UTL-03** JSON key case conversion shall be recursive and bidirectional
  (`json_camel_to_snake_case`, `json_snake_to_camel_case`);
  `convert_ros_message_to_json` shall serialize any ROS message to a
  camelCase JSON string via `rosidl_runtime_py`.
- **UTL-04** C++ `to_snake_case` shall convert plugin action names to the
  snake_case parameter keys used in ADP-03.
- **UTL-05** Topic builders `get_vda5050_mqtt_topic` /
  `get_vda5050_ros2_topic` shall enforce the valid topic set and
  `v<digit>` version format, raising `ValueError` otherwise (BRG-08/09/10).
- **UTL-06 (JLG)** `validate_vda5050_payload`, `has_unique_ids`,
  `collect_ids`, and `is_uuid` shall implement the validation of BRG-15/16.

---

## 8. Non-Functional Requirements

- **NFR-01 Resilience:** malformed MQTT payloads, unavailable diagnostic
  services, broker disconnects, and unsupported actions shall degrade
  gracefully (log + error report) and never terminate a node.
- **NFR-02 Concurrency:** the controller relies on a multi-threaded executor
  with mutually exclusive callback groups for its service clients and
  visualization timer; the adapter executes goals in detached threads and
  guards shared state with a reader/writer lock; the multi-node result
  callback orders its teardown to avoid re-dispatch races (CTRL-32).
- **NFR-03 Boundedness:** state-message errors are capped (CTRL-09); MQTT
  in-flight/queued messages are capped (BRG-02).
- **NFR-04 Extensibility:** all vendor-specific behavior lives in pluginlib
  handlers and virtual methods of `AdapterNode`; the controller and bridge
  are robot-agnostic. Example adapters:
  <https://github.com/inorbit-ai/vda5050_adapter_examples>.
- **NFR-05 Code quality:** the package shall pass `ament_flake8`,
  `ament_pep257`, `ament_copyright`, and `ament_clang_format` linters.

---

## 9. Verification

- **TST-01** Python unit tests shall cover the controller order lifecycle
  (`test_vda5050_controller.py`), action handling
  (`test_vda5050_controller_actions.py`), invalid-order handling
  (`invalid_orders_test.py`), the MQTT bridge
  (`test_vda5050_mqtt_bridge.py`), and utilities (`test_utils.py`), executed
  via `ament_add_pytest_test` / pytest with `pytest-mock`.
- **TST-02** C++ gtest suites under `test/adapter/` shall verify adapter node
  configuration (`node_config_test`), plugin loading against the stub
  plugins declared in `plugins_test.xml` (`plugin_load_test`), and handler
  execution paths (`handler_execution_test`), using stub implementations of
  all three handler types.
- **TST-03** The full suite shall run with
  `colcon test --packages-select vda5050_connector`.

---

## Appendix A — Default Interface Map (defaults: namespace `vda5050`, manufacturer `robots`, robot/serial `robot_1`, interface `uagv`, v2)

| Kind | Name | Producer → Consumer |
|---|---|---|
| MQTT topic | `uagv/v2/robots/robot_1/order` | MC → bridge |
| MQTT topic | `uagv/v2/robots/robot_1/instantActions` | MC → bridge |
| MQTT topic | `uagv/v2/robots/robot_1/state` | bridge → MC |
| MQTT topic | `uagv/v2/robots/robot_1/connection` | bridge → MC (incl. will) |
| MQTT topic | `uagv/v2/robots/robot_1/visualization` | bridge → MC |
| ROS topic | `/uagv/v2/robots/robot_1/{order,instantActions}` | bridge → controller |
| ROS topic | `/uagv/v2/robots/robot_1/{state,connection,visualization,factsheet}` | controller → bridge |
| ROS topic | `diagnostics/dtc` **(JLG)** | bridge → diagnostics |
| ROS service | `diagnostics/{unlatch_dtc,force_latch_dtc}` **(JLG)** | bridge → diagnostics |
| ROS service | `/vda5050/robots/robot_1/adapter/get_state` | controller → adapter |
| ROS service | `/vda5050/robots/robot_1/adapter/supported_actions` | controller → adapter |
| ROS action | `/vda5050/robots/robot_1/adapter/nav_to_node` | controller → adapter |
| ROS action | `/vda5050/robots/robot_1/adapter/nav_through_nodes` **(JLG)** | controller → adapter |
| ROS action | `/vda5050/robots/robot_1/adapter/vda_action` | controller → adapter |
