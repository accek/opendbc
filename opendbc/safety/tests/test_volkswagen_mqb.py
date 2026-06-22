#!/usr/bin/env python3
import unittest
import numpy as np
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.car.volkswagen.values import VolkswagenSafetyFlags

MAX_ACCEL = 2.0
MIN_ACCEL = -3.5

MSG_ESP_19 = 0xB2       # RX from ABS, for wheel speeds
MSG_LH_EPS_03 = 0x9F    # RX from EPS, for driver steering torque
MSG_ESP_05 = 0x106      # RX from ABS, for brake light state
MSG_TSK_06 = 0x120      # RX from ECU, for ACC status from drivetrain coordinator
MSG_MOTOR_20 = 0x121    # RX from ECU, for driver throttle input
MSG_ACC_06 = 0x122      # TX by OP, ACC control instructions to the drivetrain coordinator
MSG_HCA_01 = 0x126      # TX by OP, Heading Control Assist steering torque
MSG_GRA_ACC_01 = 0x12B  # TX by OP, ACC control buttons for cancel/resume
MSG_ACC_07 = 0x12E      # TX by OP, ACC control instructions to the drivetrain coordinator
MSG_ACC_02 = 0x30C      # TX by OP, ACC HUD data to the instrument cluster
MSG_ACC_04 = 0x324      # acspilot: TX by OP, ACC HUD data to the instrument cluster
MSG_ACC_13 = 0x2A7      # acspilot: TX by OP, ACC HUD data to the instrument cluster
MSG_LDW_02 = 0x397      # TX by OP, Lane line recognition and text alerts

# acspilot: the grace period that keeps accel valid briefly after long is disabled
ACC_CHECKS_GRACE_PERIOD_US = 50000


class TestVolkswagenMqbSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01, MSG_LDW_02), 2: (MSG_LH_EPS_03,)}

  MAX_RATE_UP = 4
  MAX_RATE_DOWN = 10
  MAX_TORQUE_LOOKUP = [0], [300]
  MAX_RT_DELTA = 75

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 3

  # Wheel speeds _esp_19_msg
  def _speed_msg(self, speed):
    values = {"ESP_%s_Radgeschw_02" % s: speed for s in ["HL", "HR", "VL", "VR"]}
    return self.packer.make_can_msg_safety("ESP_19", 0, values)

  # Driver brake pressure over threshold
  def _esp_05_msg(self, brake):
    values = {"ESP_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("ESP_05", 0, values)

  # Brake pedal switch
  def _motor_14_msg(self, brake):
    values = {"MO_Fahrer_bremst": brake}
    return self.packer.make_can_msg_safety("Motor_14", 0, values)

  def _user_brake_msg(self, brake):
    return self._motor_14_msg(brake)

  # Driver throttle input
  def _user_gas_msg(self, gas):
    values = {"MO_Fahrpedalrohwert_01": gas}
    return self.packer.make_can_msg_safety("Motor_20", 0, values)

  # ACC engagement status
  def _tsk_status_msg(self, enable, main_switch=True):
    if main_switch:
      tsk_status = 3 if enable else 2
    else:
      tsk_status = 0
    values = {"TSK_Status": tsk_status}
    return self.packer.make_can_msg_safety("TSK_06", 0, values)

  def _pcm_status_msg(self, enable):
    return self._tsk_status_msg(enable)

  # Driver steering input torque
  def _torque_driver_msg(self, torque):
    values = {"EPS_Lenkmoment": abs(torque), "EPS_VZ_Lenkmoment": torque < 0}
    return self.packer.make_can_msg_safety("LH_EPS_03", 0, values)

  # openpilot steering output torque
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"HCA_01_LM_Offset": abs(torque), "HCA_01_LM_OffSign": torque < 0, "HCA_01_Sendestatus": steer_req}
    return self.packer.make_can_msg_safety("HCA_01", 0, values)

  # Cruise control buttons
  def _gra_acc_01_msg(self, cancel=0, resume=0, _set=0, bus=2):
    values = {"GRA_Abbrechen": cancel, "GRA_Tip_Setzen": _set, "GRA_Tip_Wiederaufnahme": resume}
    return self.packer.make_can_msg_safety("GRA_ACC_01", bus, values)

  # Acceleration request to drivetrain coordinator
  def _acc_06_msg(self, accel, cnt=0):
    values = {"ACC_Sollbeschleunigung_02": accel, "COUNTER": cnt}
    return self.packer.make_can_msg_safety("ACC_06", 0, values)

  # acspilot: stock ACC_06 forwarded from the camera/radar bus, with the engagement status field the
  # fwd hook reads to decide whether the stock ACC is in control (ACC_Status_ACC) plus the accel request
  def _acc_06_status_msg(self, status, accel=0.0, cnt=0, bus=2):
    values = {"ACC_Status_ACC": status, "ACC_Sollbeschleunigung_02": accel, "COUNTER": cnt}
    return self.packer.make_can_msg_safety("ACC_06", bus, values)

  # Acceleration request to drivetrain coordinator
  def _acc_07_msg(self, accel, secondary_accel=3.02, cnt=0):
    values = {"ACC_Sollbeschleunigung_02": accel, "ACC_Folgebeschl": secondary_accel, "COUNTER": cnt}
    return self.packer.make_can_msg_safety("ACC_07", 0, values)

  # acspilot: the carcontroller forwards/replaces stock messages, so nothing is blocked from
  # forwarding until openpilot has attempted to send its first message
  def test_fwd_hook(self):
    self._tx(self._torque_cmd_msg(0))
    super().test_fwd_hook()

  # Verify brake_pressed is true if either the switch or pressure threshold signals are true
  def test_redundant_brake_signals(self):
    test_combinations = [(True, True, True), (True, True, False), (True, False, True), (False, False, False)]
    for brake_pressed, motor_14_signal, esp_05_signal in test_combinations:
      self._rx(self._motor_14_msg(False))
      self._rx(self._esp_05_msg(False))
      self.assertFalse(self.safety.get_brake_pressed_prev())
      self._rx(self._motor_14_msg(motor_14_signal))
      self._rx(self._esp_05_msg(esp_05_signal))
      self.assertEqual(brake_pressed, self.safety.get_brake_pressed_prev(),
                       f"expected {brake_pressed=} with {motor_14_signal=} and {esp_05_signal=}")

  def test_torque_measurements(self):
    # TODO: make this test work with all cars
    self._rx(self._torque_driver_msg(50))
    self._rx(self._torque_driver_msg(-50))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))
    self._rx(self._torque_driver_msg(0))

    self.assertEqual(-50, self.safety.get_torque_driver_min())
    self.assertEqual(50, self.safety.get_torque_driver_max())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(-50, self.safety.get_torque_driver_min())

    self._rx(self._torque_driver_msg(0))
    self.assertEqual(0, self.safety.get_torque_driver_max())
    self.assertEqual(0, self.safety.get_torque_driver_min())


class TestVolkswagenMqbStockSafety(TestVolkswagenMqbSafetyBase):
  TX_MSGS = [[MSG_HCA_01, 0], [MSG_LDW_02, 0], [MSG_LH_EPS_03, 2], [MSG_GRA_ACC_01, 0], [MSG_GRA_ACC_01, 2]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03], 2: [MSG_HCA_01, MSG_LDW_02]}

  def setUp(self):
    self.packer = CANPackerSafety("vw_mqb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, 0)
    self.safety.init_tests()

  def test_spam_cancel_safety_check(self):
    # acspilot: cancel is only allowed when stock cruise was previously engaged
    self._rx(self._tsk_status_msg(True))   # engage stock cruise -> cruise_engaged_prev
    self.safety.set_controls_allowed(0)
    self.assertTrue(self._tx(self._gra_acc_01_msg(cancel=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(resume=1)))
    self.assertFalse(self._tx(self._gra_acc_01_msg(_set=1)))
    # cancel is blocked once stock cruise has never been engaged
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._gra_acc_01_msg(cancel=1)))
    # do not block resume if we are engaged already
    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._gra_acc_01_msg(resume=1)))


class TestVolkswagenMqbLongSafety(TestVolkswagenMqbSafetyBase):
  # acspilot: the MQB carcontroller forwards stock messages with modified fields, so it transmits and
  # selectively blocks a wider set of messages than upstream. Forwarding is decided entirely by the
  # custom fwd hook, so only HCA_01 participates in relay malfunction detection.
  TX_MSGS = [[MSG_HCA_01, 0], [MSG_GRA_ACC_01, 2], [MSG_LDW_02, 0], [MSG_LH_EPS_03, 2], [MSG_TSK_06, 2],
             [MSG_ACC_02, 0], [MSG_ACC_04, 0], [MSG_ACC_06, 0], [MSG_ACC_07, 0], [MSG_ACC_13, 0]]
  FWD_BLACKLISTED_ADDRS = {0: [MSG_LH_EPS_03, MSG_TSK_06, MSG_GRA_ACC_01],
                           2: [MSG_HCA_01, MSG_LDW_02, MSG_ACC_02, MSG_ACC_04, MSG_ACC_06, MSG_ACC_07, MSG_ACC_13]}
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_HCA_01,)}
  INACTIVE_ACCEL = 3.01

  def setUp(self):
    self.packer = CANPackerSafety("vw_mqb")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.volkswagen, VolkswagenSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  # stock cruise controls are entirely bypassed under openpilot longitudinal control
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_set_and_resume_buttons(self):
    for button in ["set", "resume"]:
      # ACC main switch must be on, engage on falling edge
      self.safety.set_controls_allowed(0)
      self._rx(self._tsk_status_msg(False, main_switch=False))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} with main switch off")
      self._rx(self._tsk_status_msg(False, main_switch=True))
      self._rx(self._gra_acc_01_msg(_set=(button == "set"), resume=(button == "resume"), bus=0))
      self.assertFalse(self.safety.get_controls_allowed(), f"controls allowed on {button} rising edge")
      self._rx(self._gra_acc_01_msg(bus=0))
      self.assertTrue(self.safety.get_controls_allowed(), f"controls not allowed on {button} falling edge")

  def test_cancel_button(self):
    # Disable on rising edge of cancel button
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._gra_acc_01_msg(cancel=True, bus=0))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after cancel")

  def test_main_switch(self):
    # Disable as soon as main switch turns off
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(1)
    self._rx(self._tsk_status_msg(False, main_switch=False))
    self.assertFalse(self.safety.get_controls_allowed(), "controls allowed after ACC main switch off")

  def test_accel_safety_check(self):
    # acspilot: ACC messages are gated by a counter-continuity check, so the counter must advance on
    # every accepted tx. Push the timer past the grace period so the base accel limits are enforced.
    self.safety.set_timer(ACC_CHECKS_GRACE_PERIOD_US + 1)
    cnt_06 = cnt_07 = 0
    for controls_allowed in [True, False]:
      # enforce we don't skip over 0 or inactive accel
      for accel in np.concatenate((np.arange(MIN_ACCEL - 2, MAX_ACCEL + 2, 0.03), [0, self.INACTIVE_ACCEL])):
        accel = round(accel, 2)  # floats might not hit exact boundary conditions without rounding
        is_inactive_accel = accel == self.INACTIVE_ACCEL
        send = (controls_allowed and MIN_ACCEL <= accel <= MAX_ACCEL) or is_inactive_accel
        self.safety.set_controls_allowed(controls_allowed)
        # primary accel request used by ECU
        self.assertEqual(send, self._tx(self._acc_06_msg(accel, cnt_06)), (controls_allowed, accel))
        if send:
          cnt_06 = (cnt_06 + 1) % 16
        # additional accel request used by ABS/ESP
        self.assertEqual(send, self._tx(self._acc_07_msg(accel, cnt=cnt_07)), (controls_allowed, accel))
        if send:
          cnt_07 = (cnt_07 + 1) % 16
        # acspilot: the secondary accel field (ACC_Folgebeschl) is repurposed to feed lead acceleration
        # and is no longer enforced inactive
        self.assertEqual(send, self._tx(self._acc_07_msg(accel, secondary_accel=accel, cnt=cnt_07)), (controls_allowed, accel))
        if send:
          cnt_07 = (cnt_07 + 1) % 16

  def test_acc_counter_check(self):
    # acspilot: ACC messages must be sent with sequential counters so they never duplicate the
    # forwarded stock messages when switching between stock and openpilot control
    self.safety.set_timer(ACC_CHECKS_GRACE_PERIOD_US + 1)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=0)))
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=1)))
    # out of sequence counter is rejected
    self.assertFalse(self._tx(self._acc_06_msg(0.0, cnt=5)))
    # resuming the sequence is accepted again
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=2)))

  def test_accel_grace_period(self):
    # acspilot: in-bounds accel stays valid within the grace period after controls are disabled, so a
    # few dropped frames don't fault the stock ACC module
    self.safety.set_timer(10 * ACC_CHECKS_GRACE_PERIOD_US)
    # establish that long was allowed (refreshed in the rx hook while controls are allowed)
    self.safety.set_controls_allowed(True)
    self._rx(self._tsk_status_msg(False, main_switch=True))
    self.safety.set_controls_allowed(False)
    # still within the grace period: in-bounds accel allowed even though controls are not allowed
    self.assertTrue(self._tx(self._acc_06_msg(MAX_ACCEL, cnt=0)))
    # out-of-bounds accel is never allowed, even within the grace period
    self.assertFalse(self._tx(self._acc_06_msg(MAX_ACCEL + 1.0, cnt=1)))
    # after the grace period expires, in-bounds accel is rejected with controls not allowed
    self.safety.set_timer(10 * ACC_CHECKS_GRACE_PERIOD_US + 2 * ACC_CHECKS_GRACE_PERIOD_US)
    self.assertFalse(self._tx(self._acc_06_msg(MAX_ACCEL, cnt=1)))

  def test_fwd_before_first_tx(self):
    # acspilot: everything is forwarded until openpilot sends its first message
    for addr in (MSG_HCA_01, MSG_LDW_02, MSG_ACC_06):
      self.assertEqual(0, self.safety.safety_fwd_hook(make_msg(2, addr, 8)), hex(addr))
    self.assertEqual(2, self.safety.safety_fwd_hook(make_msg(0, MSG_LH_EPS_03, 8)))
    # after the first tx, stock messages openpilot replaces are blocked
    self._tx(self._torque_cmd_msg(0))
    for addr in (MSG_HCA_01, MSG_LDW_02, MSG_ACC_06):
      self.assertEqual(-1, self.safety.safety_fwd_hook(make_msg(2, addr, 8)), hex(addr))
    self.assertEqual(-1, self.safety.safety_fwd_hook(make_msg(0, MSG_LH_EPS_03, 8)))

  # acspilot: the fwd hook reads forwarded ACC_06 to know whether the stock ACC is engaged. While it is,
  # the stock ACC accel/HUD messages must keep flowing to the car; once it is not, openpilot takes over
  # and those messages are blocked. These cover the ACC_Status_ACC parsing that gates that decision.
  STOCK_ACC_MSGS = (MSG_ACC_02, MSG_ACC_04, MSG_ACC_07, MSG_ACC_13)

  def test_stock_acc_engaged_forwards_acc_messages(self):
    self._tx(self._torque_cmd_msg(0))  # mark tx attempted so the fwd hook begins gating
    # status 5 (also 6, 7) means stock ACC is active regardless of the accel request
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5)))
    for addr in self.STOCK_ACC_MSGS:
      self.assertEqual(0, self.safety.safety_fwd_hook(make_msg(2, addr, 8)), hex(addr))

  def test_stock_acc_not_engaged_blocks_acc_messages(self):
    self._tx(self._torque_cmd_msg(0))
    # status 2 is not an engaged state -> openpilot controls longitudinal, stock ACC messages blocked
    self.assertEqual(-1, self.safety.safety_fwd_hook(self._acc_06_status_msg(2)))
    for addr in self.STOCK_ACC_MSGS:
      self.assertEqual(-1, self.safety.safety_fwd_hook(make_msg(2, addr, 8)), hex(addr))

  def test_stock_acc_status3_engaged_only_below_inactive_accel(self):
    # status 3/4 only counts as engaged while the stock accel request is below the inactive sentinel
    self._tx(self._torque_cmd_msg(0))
    # accel well below inactive -> engaged -> ACC_02 forwarded
    self.safety.safety_fwd_hook(self._acc_06_status_msg(3, accel=0.0, cnt=0))
    self.assertEqual(0, self.safety.safety_fwd_hook(make_msg(2, MSG_ACC_02, 8)))
    # accel at the inactive sentinel -> not engaged -> ACC_02 blocked
    self.safety.safety_fwd_hook(self._acc_06_status_msg(3, accel=self.INACTIVE_ACCEL, cnt=1))
    self.assertEqual(-1, self.safety.safety_fwd_hook(make_msg(2, MSG_ACC_02, 8)))


if __name__ == "__main__":
  unittest.main()
