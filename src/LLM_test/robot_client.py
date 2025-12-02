import socket
import json
from typing import Any, Dict, Optional

from config import SERVER_HOST, SERVER_PORT, SIMULATION_DT
from logger import get_server_logger


class IsaacRobotClient:
    def __init__(self, host: str = SERVER_HOST, port: int = SERVER_PORT, timeout: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.logger = get_server_logger()

    def _send(self, action: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload = {"action": action, "params": params or {}}
        self.logger.debug(f"[RobotClient] -> {payload}")
        data = json.dumps(payload).encode("utf-8")

        with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
            sock.sendall(data)
            sock.shutdown(socket.SHUT_WR)
            resp_data = sock.recv(4096)

        if not resp_data:
            raise RuntimeError("Empty response from IsaacSim server")

        resp = json.loads(resp_data.decode("utf-8"))
        self.logger.debug(f"[RobotClient] <- {resp}")

        if not resp.get("success", False):
            raise RuntimeError(f"IsaacSim command failed: {resp}")

        return resp


    def move(
        self,
        forward: float = 0.0,
        sideways: float = 0.0,
        rotation: float = 0.0,
        duration: float = SIMULATION_DT,
    ) -> None:

        self._send(
            "move",
            {
                "forward": float(forward),
                "sideways": float(sideways),
                "rotation": float(rotation),
                "duration": float(duration),
            },
        )

    def stop(self) -> None:
        """Stop motion if your CommandHandler supports it."""
        try:
            self._send("stop", {})
        except Exception as e:
            self.logger.warning(f"[RobotClient] stop() failed: {e}")

    def get_pose(self) -> Dict[str, Any]:
        resp = self._send("get_pose", {})
        return {
            "position": resp.get("position", {}),
            "orientation": resp.get("orientation", {}),
        }

    # Convenience helpers often useful in skills
    def get_position(self) -> Dict[str, float]:
        return self.get_pose()["position"]

    def get_orientation(self) -> Dict[str, float]:
        return self.get_pose()["orientation"]
