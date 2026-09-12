# -*- coding: utf-8 -*-
import unittest
from unittest.mock import MagicMock, patch

from core.config import SprayerConfig
from core.hardware.robot.base_driver import BaseRobotDriver, RobotPose
from apps.robot.services.robot_service import RobotService


class TestRobotService(unittest.TestCase):
    def setUp(self):
        self.config = SprayerConfig()
        self.service = RobotService(config=self.config)

    def test_initial_properties(self):
        self.assertFalse(self.service.is_connected())
        self.assertEqual(self.service.global_speed_factor, self.config.global_speed_factor)
        self.assertEqual(self.service.max_tcp_speed_mm_s, self.config.max_tcp_speed_mm_s)
        self.assertEqual(self.service.max_joint_speed_deg_s, self.config.max_joint_speed_deg_s)

    def test_set_global_speed_factor(self):
        # Invalid factor
        success, msg = self.service.set_global_speed_factor(0)
        self.assertFalse(success)
        success, msg = self.service.set_global_speed_factor(101)
        self.assertFalse(success)

        # Valid factor
        success, msg = self.service.set_global_speed_factor(50)
        self.assertTrue(success)
        self.assertEqual(self.service.global_speed_factor, 50)

    def test_set_speed(self):
        # Invalid speed values (<= 0)
        success, msg = self.service.set_speed(0, 10, 10, 10)
        self.assertFalse(success)
        success, msg = self.service.set_speed(10, 0, 10, 10)
        self.assertFalse(success)
        success, msg = self.service.set_speed(10, 10, 0, 10)
        self.assertFalse(success)
        success, msg = self.service.set_speed(10, 10, 10, 0)
        self.assertFalse(success)

        # Valid speed values
        success, msg = self.service.set_speed(25.0, 30.0, 15.0, 20.0)
        self.assertTrue(success)
        sl, al, sj, aj = self.service.get_speed()
        self.assertEqual(sl, 25.0)
        self.assertEqual(al, 30.0)
        self.assertEqual(sj, 15.0)
        self.assertEqual(aj, 20.0)

    def test_operations_when_not_connected(self):
        # Without connection, motion operations should fail gracefully
        success, msg = self.service.jog_step("X", 1, 10.0)
        self.assertFalse(success)
        self.assertIn("not connected", msg.lower())

        success, msg = self.service.jog_continuous("X", 1)
        self.assertFalse(success)
        self.assertIn("not connected", msg.lower())

        success, msg = self.service.go_zero()
        self.assertFalse(success)
        self.assertIn("not connected", msg.lower())

        success, msg = self.service.go_fold()
        self.assertFalse(success)
        self.assertIn("not connected", msg.lower())

        success, msg = self.service.go_home()
        self.assertFalse(success)
        self.assertIn("not connected", msg.lower())

        success, msg = self.service.move_to_pose([100, 200, 300, 0, 0, 0])
        self.assertFalse(success)

        success, msg = self.service.move_to_joint([0, 0, 0, 0, 0, 0])
        self.assertFalse(success)

        success, msg = self.service.pause()
        self.assertFalse(success)

        success, msg = self.service.resume()
        self.assertFalse(success)

        success, msg = self.service.estop()
        self.assertFalse(success)

        success, msg = self.service.clear_error()
        self.assertFalse(success)

        pose, _ = self.service.get_current_pose()
        self.assertIsNone(pose)
        joints, _ = self.service.get_current_joint()
        self.assertIsNone(joints)

    def test_mock_driver_operations(self):
        mock_driver = MagicMock(spec=BaseRobotDriver)
        mock_driver.is_connected = True
        mock_driver.get_current_pose.return_value = RobotPose(x=100.0, y=200.0, z=300.0, a=0.0, b=0.0, c=0.0)
        mock_driver.get_current_joint.return_value = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0]
        mock_driver.pause.return_value = True
        mock_driver.resume.return_value = True
        mock_driver.estop.return_value = True
        mock_driver.clear_error.return_value = True

        self.service._driver = mock_driver
        self.service._is_connected = True

        # Test pose and joint queries
        pose, err = self.service.get_current_pose()
        self.assertIsNotNone(pose)
        self.assertEqual(pose[0], 100.0)
        self.assertEqual(pose[1], 200.0)

        joints, err = self.service.get_current_joint()
        self.assertEqual(joints, [0.0, 10.0, 20.0, 30.0, 40.0, 50.0])

        # Test pause, resume, estop, clear_error
        success, _ = self.service.pause()
        self.assertTrue(success)
        mock_driver.pause.assert_called_once()

        success, _ = self.service.resume()
        self.assertTrue(success)
        mock_driver.resume.assert_called_once()

        success, _ = self.service.estop()
        self.assertTrue(success)
        mock_driver.estop.assert_called_once()

        success, _ = self.service.clear_error()
        self.assertTrue(success)
        mock_driver.clear_error.assert_called_once()

    def test_ws_callbacks(self):
        cb1 = MagicMock()
        cb2 = MagicMock()

        self.service.register_ws_callback(cb1)
        self.service.register_ws_callback(cb2)
        self.assertEqual(len(self.service._ws_callbacks), 2)

        # Duplicate register should not add twice
        self.service.register_ws_callback(cb1)
        self.assertEqual(len(self.service._ws_callbacks), 2)

        self.service.unregister_ws_callback(cb1)
        self.assertEqual(len(self.service._ws_callbacks), 1)
        self.assertNotIn(cb1, self.service._ws_callbacks)

    def test_jog_step_axis_validation(self):
        mock_driver = MagicMock(spec=BaseRobotDriver)
        mock_driver.is_connected = True
        mock_driver.get_current_pose.return_value = RobotPose(x=100.0, y=100.0, z=100.0, a=0.0, b=0.0, c=0.0)
        mock_driver.get_current_joint.return_value = [0.0] * 6

        self.service._driver = mock_driver
        self.service._is_connected = True

        # Invalid axis
        success, msg = self.service.jog_step("INVALID", 1, 10.0)
        self.assertFalse(success)
        self.assertIn("invalid axis", msg.lower())

        # Valid axis X
        success, msg = self.service.jog_step("X", 1, 10.0)
        self.assertTrue(success)

    def test_go_home_and_fold_ensure_spray_do_off(self):
        mock_driver = MagicMock(spec=BaseRobotDriver)
        mock_driver.is_connected = True
        mock_driver.go_home.return_value = 0
        mock_driver.set_do.return_value = True
        mock_driver.move_joint.return_value = 0
        mock_driver.get_current_joint.return_value = [0.0] * 6

        self.service._driver = mock_driver
        self.service._is_connected = True

        # go_home should call set_do(spray_do_index, 0, immediate=True)
        success, msg = self.service.go_home(speed=20.0, acc=20.0)
        self.assertTrue(success)
        mock_driver.set_do.assert_called_with(self.service.spray_do_index, 0, immediate=True)

        # go_fold should also call set_do(spray_do_index, 0, immediate=True)
        mock_driver.set_do.reset_mock()
        success, msg = self.service.go_fold(speed=20.0, acc=20.0)
        self.assertTrue(success)
        mock_driver.set_do.assert_called_with(self.service.spray_do_index, 0, immediate=True)

    def test_base_driver_move_l_segments_turns_off_do_on_complete(self):
        with patch.object(BaseRobotDriver, "__abstractmethods__", set()):
            class DummyDriver(BaseRobotDriver):
                def __init__(self):
                    super().__init__()
                    self.do_calls = []
                    self.queue_calls = []

                def set_do(self, index: int, status: int, immediate: bool = False) -> bool:
                    self.do_calls.append((index, status, immediate))
                    return True

                def move_l_queue(self, poses, velocity=100.0, acc=80.0, dec=80.0, tool_num=None, wait=True, cp_ratio=50, speeds=None) -> int:
                    self.queue_calls.append((poses, wait, speeds))
                    return 0

            driver = DummyDriver()
        segments = [
            {"spraying": True, "poses": [RobotPose(x=10, y=20, z=30)]},
        ]
        ret = driver.move_l_segments(segments, spray_do_index=1)
        self.assertEqual(ret, 0)
        # First call: queue DO 1; Second call after segments complete: immediate DO 0
        self.assertEqual(driver.do_calls[0], (1, 1, False))
        self.assertEqual(driver.do_calls[-1], (1, 0, True))
        # 未提供逐航点 speeds 时透传 None (保持原有匀速行为)
        self.assertIsNone(driver.queue_calls[0][2])

    def test_base_driver_move_l_segments_threads_per_waypoint_speeds(self):
        """奇异降速: 段内 speeds 与 poses 等长时逐层透传到 move_l_queue; 不等长不应报错但驱动层会回退。"""
        with patch.object(BaseRobotDriver, "__abstractmethods__", set()):
            class DummyDriver(BaseRobotDriver):
                def __init__(self):
                    super().__init__()
                    self.do_calls = []
                    self.queue_calls = []

                def set_do(self, index: int, status: int, immediate: bool = False) -> bool:
                    self.do_calls.append((index, status, immediate))
                    return True

                def move_l_queue(self, poses, velocity=100.0, acc=80.0, dec=80.0, tool_num=None, wait=True, cp_ratio=50, speeds=None) -> int:
                    self.queue_calls.append((poses, wait, speeds))
                    return 0

            driver = DummyDriver()
        segments = [
            {"spraying": True,
             "poses": [RobotPose(x=10, y=20, z=30), RobotPose(x=11, y=20, z=30)],
             "speeds": [150.0, 30.0]},
        ]
        ret = driver.move_l_segments(segments, velocity=150.0, spray_do_index=1)
        self.assertEqual(ret, 0)
        # 逐航点速度按序透传至驱动层
        self.assertEqual(driver.queue_calls[0][2], [150.0, 30.0])

    def test_home_and_fold_position_cascading_and_driver_dispatch(self):
        from core.config import SprayerConfig
        from services.setting_service import SettingService
        from apps.system.api import validate_setting_entry
        from fastapi import HTTPException

        config = SprayerConfig()
        setting_srv = SettingService()

        # 1. Baseline from YAML or defaults
        default_home = config.home_position
        default_fold = config.fold_position
        self.assertEqual(len(default_home), 6)
        self.assertEqual(len(default_fold), 6)
        self.assertTrue(all(isinstance(x, float) for x in default_home))
        self.assertTrue(all(isinstance(x, float) for x in default_fold))

        # 2. Database override takes precedence
        custom_home = [5.0, -10.0, -85.0, 0.0, -95.0, 15.0]
        custom_fold = [0.0, 10.0, -150.0, 5.0, -165.0, 0.0]
        try:
            setting_srv.set_value("robot.home_position", custom_home)
            setting_srv.set_value("robot.fold_position", custom_fold)
            config.reload_db_overrides()
            self.service.reload_config()

            self.assertEqual(config.home_position, custom_home)
            self.assertEqual(config.fold_position, custom_fold)
            self.assertEqual(self.service.home_position, custom_home)
            self.assertEqual(self.service.fold_position, custom_fold)

            # 3. Verify driver dispatch with custom positions
            mock_driver = MagicMock(spec=BaseRobotDriver)
            mock_driver.is_connected = True
            mock_driver.go_home.return_value = 0
            mock_driver.set_do.return_value = True
            mock_driver.move_joint.return_value = 0
            mock_driver.get_current_joint.return_value = [0.0] * 6

            self.service._driver = mock_driver
            self.service._is_connected = True

            # go_home passes target_joints=custom_home
            ok, msg = self.service.go_home(speed=15.0, acc=15.0)
            self.assertTrue(ok)
            mock_driver.go_home.assert_called_with(wait=True, velocity=15.0, acc=15.0, target_joints=custom_home)

            # go_fold passes custom_fold to move_joint
            ok, msg = self.service.go_fold(speed=10.0, acc=10.0)
            self.assertTrue(ok)
            mock_driver.move_joint.assert_called_with(custom_fold, velocity=unittest.mock.ANY, acc=10.0)
        finally:
            # 4. Clean up DB overrides and verify rollback to YAML baseline
            setting_srv.delete_value("robot.home_position")
            setting_srv.delete_value("robot.fold_position")
            config.reload_db_overrides()
            self.service.reload_config()

            self.assertEqual(config.home_position, default_home)
            self.assertEqual(config.fold_position, default_fold)

        # 5. Test vector6 validation in system API
        valid_vec = validate_setting_entry("robot.home_position", [1, 2, 3, 4, 5, 6])
        self.assertEqual(valid_vec, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

        with self.assertRaises(HTTPException) as cm:
            validate_setting_entry("robot.home_position", [1, 2, 3, 4, 5])
        self.assertEqual(cm.exception.status_code, 400)
        self.assertIn("6-element list", cm.exception.detail)

        with self.assertRaises(HTTPException) as cm:
            validate_setting_entry("robot.home_position", [1, 2, "abc", 4, 5, 6])
        self.assertEqual(cm.exception.status_code, 400)
        self.assertIn("non-numeric", cm.exception.detail)


if __name__ == "__main__":
    unittest.main()
