from math import isclose, pi
from struct import pack
import unittest

from escape_robot_base.protocol import (
    ARM_CODE,
    FrameParser,
    ProtocolError,
    build_arm_command,
    build_beep_command,
    build_frame,
    build_velocity_command,
    decode_telemetry,
)


class ProtocolTest(unittest.TestCase):
    def test_non_finite_commands_are_rejected(self) -> None:
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaises(ProtocolError):
                build_velocity_command(value, 0.0, 0.0)
            with self.assertRaises(ProtocolError):
                build_arm_command([value] + [0.0] * 5)

    def test_reference_frame_checksum(self) -> None:
        frame = build_frame(0x01, bytes.fromhex("03 E8 FC 18 00 0A"))
        self.assertEqual(frame.hex(" ").upper(), "AA 55 0B 01 03 E8 FC 18 00 0A 14")

    def test_velocity_command_big_endian_and_clamped(self) -> None:
        frame = build_velocity_command(0.25, -0.125, 2.0, max_wz_rps=1.2)
        self.assertEqual(len(frame), 11)
        self.assertEqual(frame[3], 0x50)
        self.assertEqual(frame[4:10], pack(">hhh", 250, -125, 1200))
        self.assertEqual(frame[-1], sum(frame[:-1]) & 0xFF)

    def test_incremental_parser_handles_noise_split_and_concatenation(self) -> None:
        first = build_beep_command(True)
        second = build_velocity_command(0.1, 0.0, -0.2)
        parser = FrameParser()
        self.assertEqual(parser.feed(b"noise\xAA"), [])
        self.assertEqual(parser.feed(first[1:4]), [])
        frames = parser.feed(first[4:] + second)
        self.assertEqual([frame.code for frame in frames], [0x54, 0x50])
        self.assertEqual(parser.discarded_bytes, 5)

    def test_bad_checksum_resynchronizes(self) -> None:
        damaged = bytearray(build_beep_command(True))
        damaged[-1] ^= 0xFF
        parser = FrameParser()
        frames = parser.feed(bytes(damaged) + build_beep_command(False))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload, b"\x00")
        self.assertEqual(parser.bad_checksums, 1)

    def test_decode_telemetry_to_si(self) -> None:
        payload = pack(
            ">hhhhhhhhhh", 16384, 0, -16384, 3276, 0, -3276, 120, -50, 300, 1120
        )
        telemetry = decode_telemetry(build_frame(0x10, payload))
        self.assertTrue(isclose(telemetry.acceleration_mps2[0], 9.80665, rel_tol=1e-6))
        self.assertTrue(isclose(telemetry.acceleration_mps2[2], -9.80665, rel_tol=1e-6))
        self.assertTrue(
            isclose(telemetry.angular_velocity_rps[0], 49.9878 * pi / 180.0, rel_tol=1e-3)
        )
        self.assertEqual(telemetry.velocity_mps, (0.12, -0.05, 0.3))
        self.assertEqual(telemetry.battery_voltage, 11.2)

    def test_arm_command_has_six_int16_angles_and_computed_length(self) -> None:
        frame = build_arm_command([0.0, pi / 2, -pi / 2, 0.1, -0.1, 0.0])
        self.assertEqual(len(frame), 17)
        self.assertEqual(frame[2], 17)
        self.assertEqual(frame[3], ARM_CODE)
        self.assertEqual(frame[4:10], pack(">hhh", 0, 900, -900))

    def test_arm_command_rejects_wrong_joint_count(self) -> None:
        with self.assertRaises(ProtocolError):
            build_arm_command([0.0] * 5)


if __name__ == "__main__":
    unittest.main()
