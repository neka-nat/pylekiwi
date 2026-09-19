import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from pylekiwi.models import (
    BaseCommand,
    CommandLease,
    CommandLeaseResponse,
    LekiwiCommand,
)
from pylekiwi.nodes import ClientControllerNode
from pylekiwi.settings import Settings


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.node = ClientControllerNode.__new__(ClientControllerNode)
        self.node.settings = Settings()
        self.node.session = Mock()
        self.node.publisher = Mock()
        self.node._wait_for_matching = False
        self.node._client_id = "client"
        self.node._command_sequence = 0
        self.node._lease = None
        self.node._lease_refresh_at = 0.0
        self.node._lease_validity_s = None
        self.node._send_lock = threading.Lock()
        self.lease = CommandLease(
            host_id="host",
            client_id="client",
            lease_id="lease",
            issued_at_monotonic_ns=50,
            expires_at_monotonic_ns=500000050,
        )
        response = CommandLeaseResponse(ok=True, lease=self.lease)
        self.node.session.get.return_value = [
            SimpleNamespace(
                ok=SimpleNamespace(
                    payload=SimpleNamespace(to_string=response.model_dump_json)
                )
            )
        ]
        self.command = LekiwiCommand(
            base_command=BaseCommand(x_vel=0.1, y_vel=0, theta_deg_vel=0)
        )

    def test_fresh_envelope_and_sequence_without_mutating_input(self):
        with patch("pylekiwi.nodes.time.monotonic", return_value=10.0):
            self.node.send_command(self.command)
            self.node.send_command(self.command)
        self.node.session.get.assert_called_once()
        commands = [
            LekiwiCommand.model_validate_json(call.args[0])
            for call in self.node.publisher.put.call_args_list
        ]
        self.assertEqual([cmd.envelope.sequence for cmd in commands], [0, 1])
        self.assertEqual(commands[0].envelope.lease, self.lease)
        self.assertIsNone(self.command.envelope)

    def test_missing_lease_does_not_fallback_to_unbounded_command(self):
        self.node.session.get.return_value = []
        with self.assertRaisesRegex(RuntimeError, "No command-lease response"):
            self.node.send_command(self.command)
        self.node.publisher.put.assert_not_called()

    def test_delayed_lease_response_is_not_published(self):
        with patch("pylekiwi.nodes.time.monotonic", side_effect=[10.0, 10.0, 11.0]):
            with self.assertRaisesRegex(RuntimeError, "expired"):
                self.node.send_command(self.command)
        self.node.publisher.put.assert_not_called()

    def test_explicit_deadline_on_input_is_not_restamped(self):
        from pylekiwi.models import ControlEnvelope

        self.command.envelope = ControlEnvelope(lease=self.lease, sequence=1)
        with self.assertRaisesRegex(ValueError, "plain command"):
            self.node.send_command(self.command)
        self.node.publisher.put.assert_not_called()


if __name__ == "__main__":
    unittest.main()
