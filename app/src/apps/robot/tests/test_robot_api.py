# -*- coding: utf-8 -*-
import unittest
from unittest.mock import MagicMock, patch
from fastapi import FastAPI
from fastapi.testclient import TestClient

from apps.robot.api import robot_router
from apps.robot.services.robot_service import RobotService


class TestRobotAPI(unittest.TestCase):
    def setUp(self):
        self.app = FastAPI()
        self.app.include_router(robot_router)
        self.client = TestClient(self.app)

    @patch("apps.robot.api.robot_service")
    def test_connect_success(self, mock_service):
        mock_service.connect.return_value = (True, "Connected")
        resp = self.client.post("/api/robot/connect", json={"robot_type": "dobot"})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.json(), {"status": "connected"})

    @patch("apps.robot.api.robot_service")
    def test_connect_failure(self, mock_service):
        mock_service.connect.return_value = (False, "Connection failed")
        resp = self.client.post("/api/robot/connect", json={"robot_type": "dobot"})
        self.assertEqual(resp.status_code, 400)
        self.assertIn("Connection failed", resp.json()["detail"])

    @patch("apps.robot.api.robot_service")
    def test_disconnect(self, mock_service):
        mock_service.disconnect.return_value = (True, "Disconnected")
        resp = self.client.post("/api/robot/disconnect")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.json(), {"status": "disconnected"})

    @patch("apps.robot.api.robot_service")
    def test_jog(self, mock_service):
        mock_service.jog_step.return_value = (True, "OK")
        resp = self.client.post("/api/robot/jog", json={
            "axis": "X",
            "direction": 1,
            "step": 5.0,
            "speed_l": 10.0,
            "acc_l": 10.0,
            "speed_j": 10.0,
            "acc_j": 10.0
        })
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.json(), {"status": "ok"})
        mock_service.jog_step.assert_called_once_with("X", 1, 5.0, speed=10.0, acc=10.0)

    @patch("apps.robot.api.robot_service")
    def test_jog_continuous(self, mock_service):
        mock_service.jog_continuous.return_value = (True, "OK")
        resp = self.client.post("/api/robot/jog_continuous", json={"axis": "Y", "direction": -1})
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.json(), {"status": "ok"})
        mock_service.jog_continuous.assert_called_once_with("Y", -1)

    @patch("apps.robot.api.robot_service")
    def test_motions_zero_fold_home(self, mock_service):
        mock_service.go_zero.return_value = (True, "OK")
        resp = self.client.post("/api/robot/zero", json={"speed": 10.0, "acc": 10.0})
        self.assertEqual(resp.status_code, 200)

        mock_service.go_fold.return_value = (True, "OK")
        resp = self.client.post("/api/robot/fold", json={"speed": 10.0, "acc": 10.0})
        self.assertEqual(resp.status_code, 200)

        mock_service.go_home.return_value = (True, "OK")
        resp = self.client.post("/api/robot/home", json={"speed": 10.0, "acc": 10.0})
        self.assertEqual(resp.status_code, 200)

    @patch("apps.robot.api.robot_service")
    def test_speed_endpoints(self, mock_service):
        mock_service.get_speed.return_value = (20.0, 30.0, 15.0, 25.0)
        mock_service.get_feedback_diagnostics.return_value = {
            "tcp_speed_actual": [0.0] * 6,
            "qd_actual": [0.0] * 6,
            "load": 1.2,
            "error_status": 0
        }
        mock_service.global_speed_factor = 100
        mock_service.max_tcp_speed_mm_s = 500.0
        mock_service.max_joint_speed_deg_s = [180.0] * 6

        resp = self.client.get("/api/robot/speed")
        self.assertEqual(resp.status_code, 200)
        data = resp.json()
        self.assertEqual(data["speed_l"], 20.0)
        self.assertEqual(data["load"], 1.2)

        mock_service.set_speed.return_value = (True, "OK")
        resp = self.client.post("/api/robot/speed", json={
            "speed_l": 50.0, "acc_l": 40.0, "speed_j": 30.0, "acc_j": 20.0
        })
        self.assertEqual(resp.status_code, 200)

        mock_service.set_global_speed_factor.return_value = (True, "OK")
        resp = self.client.post("/api/robot/global_speed", json={"factor": 80})
        self.assertEqual(resp.status_code, 200)

    @patch("apps.robot.api.robot_service")
    def test_control_endpoints(self, mock_service):
        mock_service.pause.return_value = (True, "OK")
        self.assertEqual(self.client.post("/api/robot/pause").status_code, 200)

        mock_service.resume.return_value = (True, "OK")
        self.assertEqual(self.client.post("/api/robot/resume").status_code, 200)

        mock_service.estop.return_value = (True, "OK")
        self.assertEqual(self.client.post("/api/robot/estop").status_code, 200)

        mock_service.clear_error.return_value = (True, "OK")
        self.assertEqual(self.client.post("/api/robot/clear_error").status_code, 200)

    @patch("apps.robot.api.robot_service")
    def test_state_and_joints_endpoints(self, mock_service):
        # Disconnected state
        mock_service.is_connected.return_value = False
        resp = self.client.get("/api/robot/state")
        self.assertEqual(resp.status_code, 200)
        self.assertFalse(resp.json()["connected"])

        resp = self.client.get("/api/robot/joints")
        self.assertEqual(resp.status_code, 400)
        self.assertIn("not connected", resp.json()["detail"].lower())

        # Connected state
        mock_service.is_connected.return_value = True
        mock_service.is_moving.return_value = False
        mock_service.get_running_state.return_value = 0
        mock_service.get_current_joint.return_value = ([10.0, 20.0, -30.0, 40.0, -50.0, 60.0], "")
        mock_service.get_current_pose.return_value = ([100.0, 200.0, 300.0, 0.0, 90.0, 0.0], "")

        resp = self.client.get("/api/robot/state")
        self.assertEqual(resp.status_code, 200)
        data = resp.json()
        self.assertTrue(data["connected"])
        self.assertEqual(data["joints"], [10.0, 20.0, -30.0, 40.0, -50.0, 60.0])
        self.assertEqual(data["pose"], [100.0, 200.0, 300.0, 0.0, 90.0, 0.0])

        resp = self.client.get("/api/robot/joints")
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.json()["joints"], [10.0, 20.0, -30.0, 40.0, -50.0, 60.0])


if __name__ == "__main__":
    unittest.main()
