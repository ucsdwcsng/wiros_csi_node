import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "src" / "pythonNoRos" / "nexcsiserver.py"
)


def load_module():
    fake_mqtt = types.SimpleNamespace(
        MQTT_ERR_SUCCESS=0,
        MQTT_ERR_QUEUE_SIZE=15,
        Client=object,
        CallbackAPIVersion=types.SimpleNamespace(VERSION2=2),
    )
    sys.modules.setdefault("paho", types.ModuleType("paho"))
    sys.modules.setdefault("paho.mqtt", types.ModuleType("paho.mqtt"))
    sys.modules["paho.mqtt"].client = fake_mqtt
    sys.modules["paho.mqtt.client"] = fake_mqtt
    sys.modules.setdefault("csi", types.SimpleNamespace(parse_frame=Mock()))
    spec = importlib.util.spec_from_file_location("nexcsiserver_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PublishMatrixTest(unittest.TestCase):
    def test_qos_publish_does_not_wait_for_ack_in_receive_loop(self):
        module = load_module()
        result = Mock(rc=module.mqtt.MQTT_ERR_SUCCESS)
        client = Mock()
        client.publish.return_value = result

        self.assertTrue(module.publish_matrix(client, "/csi", {"sequence": 4}, 1))

        result.wait_for_publish.assert_not_called()

    def test_queue_full_drops_matrix_without_blocking(self):
        module = load_module()
        client = Mock()
        client.publish.return_value = Mock(rc=module.mqtt.MQTT_ERR_QUEUE_SIZE)

        self.assertFalse(module.publish_matrix(client, "/csi", {}, 1))


class MatrixShapeTest(unittest.TestCase):
    def test_matrix_preserves_each_stream_timestamp(self):
        module = load_module()
        streams = {}
        for tx in range(2):
            for rx in range(2):
                streams[(tx, rx)] = {
                    "n_subcarriers": 4,
                    "timestamp_ns": 1_000_000_000 + tx * 10 + rx,
                    "csi_r": [float(tx * 10 + rx)] * 4,
                    "csi_i": [float(-(tx * 10 + rx))] * 4,
                }

        matrix = module.assemble_csi_matrix(streams, n_tx=2, n_rx=2)

        self.assertEqual(
            [
                {"tx": 0, "rx": 0, "timestamp_ns": 1_000_000_000},
                {"tx": 0, "rx": 1, "timestamp_ns": 1_000_000_001},
                {"tx": 1, "rx": 0, "timestamp_ns": 1_000_000_010},
                {"tx": 1, "rx": 1, "timestamp_ns": 1_000_000_011},
            ],
            matrix["stream_timestamps_ns"],
        )

    def test_incomplete_4x4_stream_set_is_rejected_instead_of_zero_padded(self):
        module = load_module()
        streams = {}
        for tx in range(2):
            for rx in range(4):
                streams[(tx, rx)] = {
                    "n_subcarriers": 4,
                    "csi_r": [float(tx * 10 + rx)] * 4,
                    "csi_i": [float(-(tx * 10 + rx))] * 4,
                }

        with self.assertRaisesRegex(ValueError, "incomplete CSI matrix"):
            module.assemble_csi_matrix(streams, n_tx=4, n_rx=4)

    def test_capture_stream_count_cannot_exceed_output_shape(self):
        module = load_module()
        config = {
            "router": {"host": "router", "username": "user"},
            "receiver": {"id": "rpi", "port": 5500},
            "capture": {
                "bandwidth_mhz": 80,
                "tx_streams": 4,
                "capture_tx_streams": 5,
                "rx_cores": 4,
            },
            "mqtt": {"host": "mqtt", "port": 1883, "topic": "/csi"},
        }

        with self.assertRaisesRegex(ValueError, "capture_tx_streams"):
            module.validate_config(config)

    def test_capture_stream_count_cannot_be_less_than_output_shape(self):
        module = load_module()
        config = {
            "router": {"host": "router", "username": "user"},
            "receiver": {"id": "rpi", "port": 5500},
            "capture": {
                "bandwidth_mhz": 80,
                "tx_streams": 4,
                "capture_tx_streams": 2,
                "rx_cores": 4,
            },
            "mqtt": {"host": "mqtt", "port": 1883, "topic": "/csi"},
        }

        with self.assertRaisesRegex(ValueError, "must equal tx_streams"):
            module.validate_config(config)


if __name__ == "__main__":
    unittest.main()
