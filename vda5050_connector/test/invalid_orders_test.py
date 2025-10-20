import json
import jsonschema
from jsonschema import Draft202012Validator, FormatChecker
import paho.mqtt.client as mqtt

from pathlib import Path
import os

script_dir = Path(__file__).parent
file_path = os.path.join(script_dir, "../../../VDA5050/json_schemas", "order.schema")

# load schema.json and payload.json however you like
with open(file_path) as f:
    schema = json.load(f)

payload = {
    "orderId": "81d0b3cb-661f-4dde-8536-3e6a1a62cd54",
    "orderUpdateId": 0,
    "nodes": [
        {
            "nodeId": "S-22",
            "sequenceId": 0,
            "released": True,
            "nodePosition": {
                "x": 2.629,
                "y": 14.278,
                "theta": 3.0956475734710693,
                "allowedDeviationXY": 0.594625354,
                "allowedDeviationTheta": 3.141592654,
                "mapId": "0",
            },
            "actions": [
                {
                    "actionType": "initPosition",
                    "actionId": "1ea42704-e3c9-481b-bc60-6281901956ef",  # "81d0b3cb-661f-4dde-8536-3e6a1a62cd54",  # "1ea42704-e3c9-481b-bc60-6281901956ef",
                    "blockingType": "HARD",
                    "actionParameters": [
                        {"key": "x", "value": 0},
                        {"key": "y", "value": 0},
                        {"key": "theta", "value": 0},
                    ],
                }
            ],
        },
        {
            "nodeId": "22-T",
            "sequenceId": 2,
            "released": True,
            "nodePosition": {
                "x": -1.7327,
                "y": 14.4786,
                "theta": 3.0956475734710693,
                "mapId": "0",
            },
            "actions": [],
        },
    ],
    "edges": [
        {
            "edgeId": "22",
            "sequenceId": 1,
            "released": True,
            "startNodeId": "S-22",
            "endNodeId": "22-T",
            "maxSpeed": 1.0,
            "orientation": 3.14159,
            "orientationType": "TANGENTIAL",
            # "trajectory": {
            #     "degree": 1,
            #     "knotVector": [0.0, 0.0, 1.0, 1.0],
            #     "controlPoints": [
            #         {
            #             "x": 2.629,
            #             "y": 14.278,
            #             "orientation": 3.0956475734710693,
            #             "weight": 1.0,
            #         },
            #         {
            #             "x": -1.7327,
            #             "y": 14.4786,
            #             "orientation": 3.0956475734710693,
            #             "weight": 1.0,
            #         },
            #     ],
            # },
            "actions": [],
        }
    ],
    "headerId": 394,
    "timestamp": "2025-08-13T12:42:38.3107016Z",
    "version": "2.0.0",
    "manufacturer": "jlg",
    "serialNumber": "AMR_001",
}

# (optional) validate the schema itself
Draft202012Validator.check_schema(schema)

validator = Draft202012Validator(schema, format_checker=FormatChecker())
errors = sorted(validator.iter_errors(payload), key=lambda e: e.path)
print(errors)
if not errors:
    print("✅ Payload is valid.")
else:
    for e in errors:
        loc = list(e.path)
        print(f"❌ At {loc if loc else '<root>'}: {e.message}")

# --------------------- MQTT PUBLISH ---------------------
mqtt_broker = "localhost"  # change to your broker
mqtt_port = 1883
mqtt_topic = "uagv/v2/jlg/amr_1/order"

client = mqtt.Client()
client.connect(mqtt_broker, mqtt_port, 60)

# Convert Python dict → JSON string
json_payload = json.dumps(payload)

# Publish
result = client.publish(mqtt_topic, json_payload, qos=1)
client.disconnect()

print(f"📡 Sent MQTT message to topic `{mqtt_topic}`")
