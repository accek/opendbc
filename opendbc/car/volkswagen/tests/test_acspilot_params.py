from opendbc.car import structs
from opendbc.car.volkswagen.carstate import CarState as VWCarState
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, CarControllerParams, VolkswagenFlags

ButtonTypeAC = structs.CarStateAC.ButtonEvent.Type


def _mqb_buttons_ac():
  mqb = next(c for c in CAR if not (c.config.flags & (VolkswagenFlags.PQ | VolkswagenFlags.MLB)))
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
