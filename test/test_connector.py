"""Unit tests for the RefringenceConnector class."""

import asyncio
import time
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

from refringence_bridge.connector import RefringenceConnector, MAX_EMISSIONS_PER_SECOND


class TestRefringenceConnector(unittest.TestCase):
    """Test the connector without a real server."""

    def setUp(self):
        self.connector = RefringenceConnector(
            token="test_token_abc123",
            server_url="https://test.refringence.com",
        )

    def test_init(self):
        self.assertEqual(self.connector._token, "test_token_abc123")
        self.assertEqual(self.connector._server_url, "https://test.refringence.com")
        self.assertFalse(self.connector.connected)

    def test_init_strips_trailing_slash(self):
        c = RefringenceConnector(token="t", server_url="https://example.com/")
        self.assertEqual(c._server_url, "https://example.com")

    def test_rate_check_allows_under_limit(self):
        """Rate check should allow emissions under the limit."""
        for _ in range(MAX_EMISSIONS_PER_SECOND):
            self.assertTrue(self.connector._rate_check())

    def test_rate_check_blocks_over_limit(self):
        """Rate check should block emissions over the limit."""
        for _ in range(MAX_EMISSIONS_PER_SECOND):
            self.connector._rate_check()
        self.assertFalse(self.connector._rate_check())

    def test_rate_check_resets_after_window(self):
        """Rate check window should reset after 1 second."""
        for _ in range(MAX_EMISSIONS_PER_SECOND):
            self.connector._rate_check()
        # Simulate time passing
        self.connector._emit_window_start = time.monotonic() - 1.1
        self.assertTrue(self.connector._rate_check())

    def test_on_command_registration(self):
        """Command callbacks should be registered."""
        handler = MagicMock()
        self.connector.on_command("command:joint", handler)
        self.assertIn("command:joint", self.connector._command_handlers)
        self.assertIs(self.connector._command_handlers["command:joint"], handler)

    def test_connected_property(self):
        """Connected property should track internal state."""
        self.assertFalse(self.connector.connected)
        self.connector._connected = True
        self.assertTrue(self.connector.connected)

    def test_emit_joint_states(self):
        """emit_joint_states should format data correctly."""
        loop = asyncio.new_event_loop()
        self.connector._sio = MagicMock()
        self.connector._sio.connected = True
        self.connector._sio.emit = AsyncMock()

        async def test():
            await self.connector.emit_joint_states(
                names=["joint_1", "joint_2"],
                positions=[0.5, -1.0],
                velocities=[0.1, 0.2],
            )

        loop.run_until_complete(test())
        loop.close()

        self.connector._sio.emit.assert_called_once()
        call_args = self.connector._sio.emit.call_args
        self.assertEqual(call_args[0][0], "telemetry:joint_states")
        data = call_args[0][1]
        self.assertEqual(data["name"], ["joint_1", "joint_2"])
        self.assertEqual(data["position"], [0.5, -1.0])
        self.assertEqual(data["velocity"], [0.1, 0.2])

    def test_emit_sim_clock(self):
        """emit_sim_clock should format simTime and realTimeFactor."""
        loop = asyncio.new_event_loop()
        self.connector._sio = MagicMock()
        self.connector._sio.connected = True
        self.connector._sio.emit = AsyncMock()

        async def test():
            await self.connector.emit_sim_clock(12.5, 0.95)

        loop.run_until_complete(test())
        loop.close()

        call_args = self.connector._sio.emit.call_args
        data = call_args[0][1]
        self.assertEqual(data["simTime"], 12.5)
        self.assertEqual(data["realTimeFactor"], 0.95)

    def test_emit_skipped_when_disconnected(self):
        """Emissions should be silently skipped when not connected."""
        loop = asyncio.new_event_loop()
        self.connector._sio = MagicMock()
        self.connector._sio.connected = False
        self.connector._sio.emit = AsyncMock()

        async def test():
            await self.connector.emit_joint_states(["j1"], [0.0])

        loop.run_until_complete(test())
        loop.close()

        self.connector._sio.emit.assert_not_called()

    def test_upload_robot_description(self):
        """upload_robot_description should emit robot:description event."""
        loop = asyncio.new_event_loop()
        self.connector._sio = MagicMock()
        self.connector._sio.connected = True
        self.connector._sio.emit = AsyncMock()

        async def test():
            await self.connector.upload_robot_description(
                "<robot/>", ["meshes/link.stl"]
            )

        loop.run_until_complete(test())
        loop.close()

        call_args = self.connector._sio.emit.call_args
        self.assertEqual(call_args[0][0], "robot:description")
        data = call_args[0][1]
        self.assertEqual(data["urdfXml"], "<robot/>")
        self.assertEqual(data["meshFileNames"], ["meshes/link.stl"])


class TestConnectorEventHandlers(unittest.TestCase):
    """Test internal Socket.io event handlers."""

    def setUp(self):
        self.connector = RefringenceConnector(token="t", server_url="https://x.com")

    def test_on_connect_sets_connected(self):
        loop = asyncio.new_event_loop()
        self.connector._sio = MagicMock()
        self.connector._sio.sid = "abc123"
        loop.run_until_complete(self.connector._on_connect())
        loop.close()
        self.assertTrue(self.connector._connected)

    def test_on_disconnect_clears_connected(self):
        loop = asyncio.new_event_loop()
        self.connector._connected = True
        loop.run_until_complete(self.connector._on_disconnect())
        loop.close()
        self.assertFalse(self.connector._connected)

    def test_command_handler_invoked(self):
        """Registered command callback should be invoked."""
        loop = asyncio.new_event_loop()
        handler = MagicMock()
        self.connector.on_command("command:joint", handler)

        internal_handler = self.connector._make_command_handler("command:joint")
        loop.run_until_complete(internal_handler({"name": "j1", "position": 0.5}))
        loop.close()

        handler.assert_called_once_with({"name": "j1", "position": 0.5})

    def test_async_command_handler_invoked(self):
        """Async command callback should be awaited."""
        loop = asyncio.new_event_loop()
        handler = AsyncMock()
        self.connector.on_command("command:reset", handler)

        internal_handler = self.connector._make_command_handler("command:reset")
        loop.run_until_complete(internal_handler({"action": "reset"}))
        loop.close()

        handler.assert_called_once_with({"action": "reset"})


if __name__ == "__main__":
    unittest.main()
