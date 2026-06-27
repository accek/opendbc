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

  def test_acc_counter_reset(self):
    # acspilot: test-only reset of the counter-continuity tracking. test_models rebuilds the
    # CarController per tx scenario (restarting its counter source) while reusing one safety instance,
    # so it resets this state between scenarios to mirror a fresh control session.
    self.safety.set_timer(ACC_CHECKS_GRACE_PERIOD_US + 1)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=0)))
    # without a reset, a counter that isn't last+1 is rejected
    self.assertFalse(self._tx(self._acc_06_msg(0.0, cnt=0)))
    # after a reset, the next counter is accepted as a fresh start regardless of the prior value
    self.safety.reset_volkswagen_mqb_long_counters()
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=0)))

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

  def test_fwd_stock_acc_counter_continuity(self):
    # acspilot: forwarded stock ACC messages must keep sequential counters so they never duplicate the
    # counters openpilot sends across a stock<->OP handoff; a non-sequential counter blocks the forward
    self._tx(self._torque_cmd_msg(0))  # mark tx attempted so the fwd hook begins gating
    # stock ACC engaged (status 5) -> ACC_06 forwarded; sequential counters are accepted
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=0)))
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=1)))
    # out-of-sequence counter -> forwarding is blocked
    self.assertEqual(-1, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=5)))

  def test_long_allowed_ts_pinned_after_grace(self):
    # acspilot: once the grace period lapses with controls not allowed, the rx hook pins the
    # long-allowed timestamp just past the grace window so timer wraparound can't re-enable accel
    self.safety.set_controls_allowed(False)
    self.safety.set_timer(10 * ACC_CHECKS_GRACE_PERIOD_US)
    # rx with long enabled, controls not allowed, well past the grace period -> pins last_ts
    self._rx(self._tsk_status_msg(False, main_switch=True))
    # grace has truly expired, so in-bounds accel stays rejected with controls not allowed
    self.assertFalse(self._tx(self._acc_06_msg(MAX_ACCEL, cnt=0)))

  # ===========================================================================================
  # acspilot: stock-ACC / openpilot dynamic switch -- COUNTER continuity across the handoff.
  #
  # The safety tracks one counter slot per {addr, destination-bus} that is advanced by BOTH the
  # fwd hook (forwarded stock frames, source bus 0<->2 mapped to the dest bus) and the tx hook
  # (openpilot-injected frames, keyed on msg->bus). The slots are shared, so a forwarded stock
  # frame and an injected openpilot frame for the same message form one continuous sequence: the
  # stock ACC module faults if it ever sees a duplicate or non-sequential counter across the
  # stock<->openpilot switch. The tests below exercise that shared-slot continuity directly.
  # ===========================================================================================

  # TSK_06 carrying an explicit counter, on a selectable bus. Openpilot injects TSK_06 on bus 2
  # (the only counter slot keyed to dest bus 2); the stock module sends it on bus 0.
  def _tsk_06_msg(self, cnt=0, bus=2):
    values = {"TSK_Status": 3, "COUNTER": cnt}
    return self.packer.make_can_msg_safety("TSK_06", bus, values)

  # ACC_13 cluster-speed HUD: it has no COUNTER signal in the DBC, so it is intentionally absent
  # from the counter-continuity checks. dat[1] holds the bits the counter getter would read.
  def _acc_13_msg(self, fake_counter_bits=0):
    return make_msg(0, MSG_ACC_13, 8, dat=bytes([0, fake_counter_bits, 0, 0, 0, 0, 0, 0]))

  # arm longitudinal gating: controls allowed, past the accel grace period, and tx attempted so
  # the fwd hook starts gating / parsing stock-ACC status (all of which require tx_attempted)
  def _start_long(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(ACC_CHECKS_GRACE_PERIOD_US + 1)
    self._tx(self._torque_cmd_msg(0))

  def test_counter_continuity_stock_to_op_handoff(self):
    # stock ACC engaged (status 5): its ACC_06 is forwarded, advancing the shared {ACC_06,0} slot
    self._start_long()
    for cnt in (0, 1, 2):
      self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=cnt)), cnt)
    # re-using the last forwarded counter (2) would duplicate it on the car bus -> rejected
    self.assertFalse(self._tx(self._acc_06_msg(0.0, cnt=2)))
    # openpilot takes over and continues the sequence from where the forwarded stock frames left off
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=3)))
    # an injected duplicate is rejected
    self.assertFalse(self._tx(self._acc_06_msg(0.0, cnt=3)))
    # ...and the sequence resumes cleanly
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=4)))

  def test_counter_continuity_op_to_stock_handoff(self):
    # openpilot is in control and injecting ACC_06, advancing the shared {ACC_06,0} slot
    self._start_long()
    for cnt in (0, 1, 2):
      self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=cnt)), cnt)
    # stock ACC re-engages (status 5): the forwarded stock frame continues the same sequence
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=3)))
    # forwarding a duplicate of the just-forwarded counter is blocked
    self.assertEqual(-1, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=3)))
    # the next sequential forwarded frame is accepted
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=4)))

  def test_no_duplicate_counter_across_switch(self):
    # the exact fault the feature prevents: a forwarded stock frame and an injected openpilot
    # frame must never carry the same counter on the car bus
    self._start_long()
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=7)))
    # openpilot injecting the same counter the stock radar just forwarded -> rejected
    self.assertFalse(self._tx(self._acc_06_msg(0.0, cnt=7)))
    # and the reverse: openpilot injects, then a forwarded stock frame duplicates it -> blocked
    self.safety.reset_volkswagen_mqb_long_counters()
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=7)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=7)))

  def test_counter_slot_not_consumed_when_forward_blocked(self):
    # the frame that disengages the stock ACC is itself blocked and must NOT consume a counter
    # slot, so openpilot picks up exactly that counter value seamlessly
    self._start_long()
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=0)))
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=1)))  # slot -> 1
    # status 2 disengages: ACC_06 is blocked, so the slot stays at 1 (counter check is skipped)
    self.assertEqual(-1, self.safety.safety_fwd_hook(self._acc_06_status_msg(2, cnt=2)))
    # openpilot continues with the counter the blocked frame carried -> accepted (1 + 1 == 2)
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=2)))

  def test_rejected_accel_does_not_advance_counter(self):
    # an accel-rejected tx must not consume a counter slot (the accel check runs before the
    # counter check and short-circuits tx), so a later valid frame reuses that counter
    self._start_long()
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=0)))  # slot -> 0
    # out-of-bounds accel: rejected on the accel check, counter never updated
    self.assertFalse(self._tx(self._acc_06_msg(MAX_ACCEL + 0.5, cnt=1)))
    # the next in-bounds frame with the same counter is accepted (slot was still 0, 0 + 1 == 1)
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=1)))

  def test_counter_wraparound(self):
    # the counter check is modulo 16; continuity must hold across the 15 -> 0 rollover
    self._start_long()
    # injected (tx) path across the wrap
    for cnt in (14, 15, 0, 1):
      self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=cnt)), cnt)
    # forwarded (fwd) path across the wrap, on the same shared slot after a fresh start
    self.safety.reset_volkswagen_mqb_long_counters()
    for cnt in (14, 15, 0, 1):
      self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=cnt)), cnt)

  def test_counter_independent_per_message(self):
    # each message keeps its own counter slot; advancing or breaking one must not affect another
    self._start_long()
    for cnt in (0, 1, 2):
      self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=cnt)), cnt)
      self.assertTrue(self._tx(self._acc_07_msg(0.0, cnt=cnt)), cnt)
    # a gap on ACC_06 is rejected, but ACC_07's independent sequence keeps flowing
    self.assertFalse(self._tx(self._acc_06_msg(0.0, cnt=9)))
    self.assertTrue(self._tx(self._acc_07_msg(0.0, cnt=3)))
    # ACC_06 resumes from its own last accepted counter (2 -> 3), unaffected by the rejected frame
    self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=3)))

  def test_tsk_06_counter_continuity_dest_bus2(self):
    # the second handoff channel: TSK_06 is the only slot keyed to dest bus 2. The stock module
    # sends it on bus 0 (forwarded to bus 2); openpilot injects its replacement on bus 2.
    self._start_long()
    # stock ACC engaged so the stock TSK_06 keeps flowing (it is only blocked once disengaged)
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=0)))
    for cnt in (0, 1):
      self.assertEqual(2, self.safety.safety_fwd_hook(self._tsk_06_msg(cnt=cnt, bus=0)), cnt)
    # openpilot takes over TSK feedback on bus 2 and continues the same dest-bus-2 sequence
    self.assertTrue(self._tx(self._tsk_06_msg(cnt=2, bus=2)))
    # an injected duplicate is rejected
    self.assertFalse(self._tx(self._tsk_06_msg(cnt=2, bus=2)))

  def test_acc_13_not_counter_checked(self):
    # ACC_13 has no COUNTER signal, so it is intentionally absent from the continuity checks: it
    # stays tx-allowed and forwardable without ever being gated on counter sequence
    self._start_long()
    # repeated identical frames (counter bits unchanged) are never deduped on tx
    for _ in range(3):
      self.assertTrue(self._tx(self._acc_13_msg(fake_counter_bits=0)))
    # arbitrary "counter" bits in the same byte position are equally accepted
    for bits in (0x0, 0x5, 0xF, 0x5):
      self.assertTrue(self._tx(self._acc_13_msg(fake_counter_bits=bits)))
    # and forwarding it repeatedly while stock ACC is engaged is never counter-blocked
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=0)))
    for _ in range(3):
      self.assertEqual(0, self.safety.safety_fwd_hook(make_msg(2, MSG_ACC_13, 8)))

  def test_realistic_acc_06_handoff_stream(self):
    # a holistic stream: one monotonically increasing counter (openpilot reuses the stock radar's
    # free-running counter), with exactly one accepted action per value -- forwarded while the
    # stock ACC drives, injected while openpilot drives -- flipping control several times and
    # crossing the 15 -> 0 wrap. The safety must never block a frame in this continuous sequence.
    self._start_long()
    # engaged windows (forward) vs openpilot windows (tx), by counter value, spanning a wrap
    engaged_windows = {0, 1, 2, 7, 8, 9, 14, 15}  # everything else is openpilot-driven
    for cnt in [c % 16 for c in range(0, 20)]:
      if cnt in engaged_windows:
        self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=cnt)),
                         f"forward blocked at cnt={cnt}")
      else:
        self.assertTrue(self._tx(self._acc_06_msg(0.0, cnt=cnt)), f"tx rejected at cnt={cnt}")
    # a duplicate injected at the next step (last value was 3 from the range above) is blocked
    last = 19 % 16  # == 3, an openpilot-driven value -> slot now holds 3
    self.assertFalse(self._tx(self._acc_06_msg(0.0, cnt=last)))

  def test_stock_acc_status_codes_table(self):
    # the fwd hook reads ACC_Status_ACC (and, for 3/4, the accel request vs the inactive sentinel)
    # to decide whether the stock ACC is in control. While engaged, the stock ACC/HUD messages are
    # forwarded; otherwise openpilot owns longitudinal and they are blocked.
    self._start_long()
    # (status, accel, expected_engaged)
    cases = [
      (0, 0.0, False), (1, 0.0, False), (2, 0.0, False),
      (3, 0.0, True), (4, 0.0, True),                       # 3/4 engaged below the inactive sentinel
      (3, self.INACTIVE_ACCEL, False), (4, self.INACTIVE_ACCEL, False),  # ...not at/above it
      (5, 0.0, True), (6, 0.0, True), (7, 0.0, True),       # 5/6/7 always engaged
      (5, self.INACTIVE_ACCEL, True),                       # ...regardless of accel
    ]
    for status, accel, engaged in cases:
      self.safety.reset_volkswagen_mqb_long_counters()
      forwarded = self.safety.safety_fwd_hook(self._acc_06_status_msg(status, accel=accel, cnt=0))
      self.assertEqual(0 if engaged else -1, forwarded, (status, accel))
      for addr in self.STOCK_ACC_MSGS:
        expected = 0 if engaged else -1
        self.assertEqual(expected, self.safety.safety_fwd_hook(make_msg(2, addr, 8)), (status, accel, hex(addr)))

  def test_tsk_06_forward_gated_by_stock_engaged(self):
    # on bus 0: stock TSK_06 is forwarded while the stock ACC is engaged and blocked once it is
    # not (openpilot replaces the TSK feedback so stock ACC doesn't fault); GRA_ACC_01 buttons on
    # bus 0 are always blocked under openpilot longitudinal (openpilot owns the ACC control input)
    self._start_long()
    # stock engaged -> TSK_06 forwarded to bus 2
    self.assertEqual(0, self.safety.safety_fwd_hook(self._acc_06_status_msg(5, cnt=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(self._tsk_06_msg(cnt=0, bus=0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(make_msg(0, MSG_GRA_ACC_01, 8)))
    # stock disengaged -> TSK_06 blocked (openpilot's replacement takes over)
    self.assertEqual(-1, self.safety.safety_fwd_hook(self._acc_06_status_msg(2, cnt=1)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(self._tsk_06_msg(cnt=1, bus=0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(make_msg(0, MSG_GRA_ACC_01, 8)))


if __name__ == "__main__":
  unittest.main()
