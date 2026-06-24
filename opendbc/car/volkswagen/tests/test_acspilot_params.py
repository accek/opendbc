from opendbc.car import Bus, structs
from opendbc.car.volkswagen.carstate import CarState as VWCarState
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, CarControllerParams, VolkswagenFlags

ButtonTypeAC = structs.CarStateAC.ButtonEvent.Type

LDW_02_ADDR = 0x397
HCA_01_ADDR = 0x126


def _first_mqb():
  return next(c for c in CAR if not (c.config.flags & (VolkswagenFlags.PQ | VolkswagenFlags.MLB)))


def _mqb_buttons_ac():
  mqb = _first_mqb()
  cp = structs.CarParams(carFingerprint=str(mqb))
  cp.flags = mqb.config.flags
  return CarControllerParams(cp).BUTTONS_AC


class FakeCANParser:
  def __init__(self, gap_value):
    self.vl = {"GRA_ACC_01": {"GRA_Verstellung_Zeitluecke": gap_value}}


def _params_ac(stock_cp, alpha_long, prefer_torque_tune=False):
  ret = structs.CarParamsAC()
  return CarInterface._get_params_ac(stock_cp, ret, None, {}, [], alpha_long, prefer_torque_tune, False)


class TestVolkswagenGetParamsAC:
  # acspilot: _get_params_ac derives the CarParamsAC cruise/gap extensions. These apply to MQB only;
  # PQ and MLB keep the struct defaults.
  def test_mqb_alpha_long_enables_stock_acc_override(self):
    ret = _params_ac(structs.CarParams(), alpha_long=True)  # MQB: no PQ/MLB flag
    assert ret.stockAccOverrideAvailable
    assert ret.stockAccSeparateGapControl
    # cruise-control extensions configured for MQB
    assert ret.accelButtonResumesCruise is False
    assert ret.decelButtonLimitedToVEgoWhenOverriding is False
    assert ret.resumeButtonSetsDefaultVCruise is True
    assert ret.cruiseLongPressReverse is True
    assert ret.cruiseLargeStep == 10

  def test_mqb_without_alpha_long_keeps_stock_acc_override_off(self):
    ret = _params_ac(structs.CarParams(), alpha_long=False)
    assert ret.stockAccOverrideAvailable is False
    assert ret.stockAccSeparateGapControl is False
    # the rest of the cruise extensions still apply on MQB regardless of alpha long
    assert ret.accelButtonResumesCruise is False
    assert ret.cruiseLargeStep == 10

  def test_pq_leaves_struct_at_defaults(self):
    cp = structs.CarParams(flags=int(VolkswagenFlags.PQ))
    ret = _params_ac(cp, alpha_long=True)
    # PQ is unsupported for ACSPilot cruise extensions -> nothing overridden, capnp defaults remain
    assert ret.stockAccOverrideAvailable is False
    assert ret.accelButtonResumesCruise is True
    assert ret.cruiseLargeStep == 5

  def test_mlb_leaves_struct_at_defaults(self):
    cp = structs.CarParams(flags=int(VolkswagenFlags.MLB))
    ret = _params_ac(cp, alpha_long=True)
    assert ret.stockAccOverrideAvailable is False
    assert ret.cruiseLargeStep == 5


class TestVolkswagenGapButtons:
  # acspilot: MQB exposes separate gap up/down controls in the custom CarStateAC struct, both driven by
  # the GRA_Verstellung_Zeitluecke signal (1 = up, 2 = down).
  def test_buttons_ac_config(self):
    by_type = {b.event_type: b for b in _mqb_buttons_ac()}
    up = by_type[ButtonTypeAC.gapAdjustCruiseUp]
    down = by_type[ButtonTypeAC.gapAdjustCruiseDown]
    assert (up.can_addr, up.can_msg, up.values) == ("GRA_ACC_01", "GRA_Verstellung_Zeitluecke", [1])
    assert (down.can_addr, down.can_msg, down.values) == ("GRA_ACC_01", "GRA_Verstellung_Zeitluecke", [2])

  def test_gap_up_and_down_events(self):
    buttons = _mqb_buttons_ac()
    cs = VWCarState.__new__(VWCarState)  # create_button_events only needs the args passed below
    states = {b.event_type: False for b in buttons}

    def fired(gap_value, btn_type, pressed):
      # e.type comes back as a capnp enum, so compare with == rather than hashing into a set
      evs = cs.create_button_events(FakeCANParser(gap_value), buttons, states, event_class=structs.CarStateAC.ButtonEvent)
      return any(e.type == btn_type and e.pressed == pressed for e in evs)

    # value 1 -> gap up pressed (and not down)
    assert fired(1, ButtonTypeAC.gapAdjustCruiseUp, True)
    assert not fired(1, ButtonTypeAC.gapAdjustCruiseDown, True)  # state already at 1; no new events

    # back to 0 -> gap up released
    assert fired(0, ButtonTypeAC.gapAdjustCruiseUp, False)

    # value 2 -> gap down pressed
    assert fired(2, ButtonTypeAC.gapAdjustCruiseDown, True)


class TestVolkswagenStockLdwPresent:
  # acspilot: the LKAS HUD (LDW_02) is forwarded by the carcontroller so gateway cars don't show a
  # continuous LKAS fault. It is only tracked when the stock camera actually emits it -- detected via
  # 0x397 in the cam-bus fingerprint and stored as the STOCK_LDW_PRESENT flag. Tracking it
  # unconditionally made it a required can_valid message and broke gateway MQB cars/routes that don't
  # send it (regression: SKODA_KAROQ_MK1 test_car_interface, can_invalid_cnt != 0).

  def _get_params_flags(self, cam_addrs):
    mqb = _first_mqb()
    ret = structs.CarParams(carFingerprint=str(mqb))
    fingerprint = {0: {}, 1: {}, 2: {a: 8 for a in cam_addrs}}
    out = CarInterface._get_params(ret, str(mqb), fingerprint, [], False, False, False, False)
    return out.flags

  def test_flag_set_when_ldw_in_fingerprint(self):
    assert self._get_params_flags([HCA_01_ADDR, LDW_02_ADDR]) & VolkswagenFlags.STOCK_LDW_PRESENT

  def test_flag_clear_when_ldw_absent(self):
    # camera present (HCA_01) but no LDW_02 -- mirrors the gateway SKODA_KAROQ_MK1 route
    assert not self._get_params_flags([HCA_01_ADDR]) & VolkswagenFlags.STOCK_LDW_PRESENT

  def _cam_can_valid_without_ldw_frames(self, stock_ldw_present):
    mqb = _first_mqb()
    cp = structs.CarParams(carFingerprint=str(mqb))
    cp.flags = mqb.config.flags
    if stock_ldw_present:
      cp.flags |= VolkswagenFlags.STOCK_LDW_PRESENT.value
    cam = VWCarState.get_can_parsers(cp, None, None)[Bus.cam]
    t = 0
    for _ in range(400):  # feed empty frames; a tracked-but-never-received message keeps can_valid False
      t += 10_000_000
      cam.update([(t, [])])
    return cam.can_valid

  def test_ldw_not_required_for_can_valid_when_absent(self):
    # flag off -> LDW_02 not in cam_messages -> can_valid holds without any LDW_02 frames
    assert self._cam_can_valid_without_ldw_frames(stock_ldw_present=False) is True

  def test_ldw_tracked_for_can_valid_when_present(self):
    # flag on -> LDW_02 tracked at 1Hz -> missing frames correctly drop can_valid
    assert self._cam_can_valid_without_ldw_frames(stock_ldw_present=True) is False
