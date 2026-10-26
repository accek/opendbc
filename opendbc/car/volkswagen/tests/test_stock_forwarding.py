"""acspilot: the MQB carcontroller forwards stock frames with modified fields. When the card lags, two stock
frames arrive in one step; both must be forwarded so the car bus never sees a skipped counter."""
from opendbc.can import CANPacker, CANParser
from opendbc.car.volkswagen import mqbcan
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.carstate import CarState

DBC = "vw_mqb"


def _frames(packer, msg, bus, counters, **sig):
  return [packer.make_can_msg(msg, bus, {"COUNTER": c, **sig}) for c in counters]


def _carstate():
  cs = object.__new__(CarState)
  cs.stock_values = {}
  cs.stock_frames = {}
  return cs


def _controller():
  cc = object.__new__(CarController)
  cc.forwarded_counters = {}
  cc.packer_pt = CANPacker(DBC)
  return cc


def _counters(can_sends, parser_bus):
  parser = CANParser(DBC, [("TSK_06", 50)], parser_bus)
  out = []
  for addr, dat, src in can_sends:
    parser.update([(0, [(addr, dat, src)])])
    out.append(int(parser.vl["TSK_06"]["COUNTER"]))
  return out


class TestStockFrameForwarding:
  def _step(self, cs, cp, packer, counters, status=2):
    cp.update([(0, _frames(packer, "TSK_06", 0, counters, TSK_Status=status))])
    cs.update_stock_values("TSK_06", cp)

  def test_all_frames_of_a_step_are_forwarded(self):
    packer = CANPacker(DBC)
    cp = CANParser(DBC, [("TSK_06", 50)], 0)
    cs, cc = _carstate(), _controller()

    self._step(cs, cp, packer, [3])
    sends = []
    cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends)
    assert _counters(sends, 2) == [3]

    # the card lagged: frames 4 and 5 arrive in the same step -> both forwarded, in order
    self._step(cs, cp, packer, [4, 5])
    sends = []
    cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends)
    assert _counters(sends, 2) == [4, 5]
    assert cs.stock_values["TSK_06"]["COUNTER"] == 5  # latest frame is still the "current" state

    # nothing new this step -> nothing forwarded (no duplicate of 5)
    cp.update([])
    cs.update_stock_values("TSK_06", cp)
    assert cs.stock_frames["TSK_06"] == []
    sends = []
    assert not cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends)
    assert sends == []

  def test_hook_applied_per_frame_and_checksum_dropped(self):
    packer = CANPacker(DBC)
    cp = CANParser(DBC, [("TSK_06", 50)], 0)
    cs, cc = _carstate(), _controller()
    self._step(cs, cp, packer, [7, 8], status=2)
    seen = []

    def hook(values, tag):
      seen.append((int(values["COUNTER"]), tag))
      assert "CHECKSUM" not in values
      values["TSK_Status"] = 6
      return values

    sends = []
    cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends, hook, "x")
    assert seen == [(7, "x"), (8, "x")]
    assert _counters(sends, 2) == [7, 8]

  def test_ignore_counter_sends_single_copy(self):
    # frame-driven messages (HCA_01) keep the one-copy-per-step behaviour and let the packer own COUNTER
    packer = CANPacker(DBC)
    cp = CANParser(DBC, [("TSK_06", 50)], 0)
    cs, cc = _carstate(), _controller()
    self._step(cs, cp, packer, [1, 2])
    sends = []
    cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends, ignore_counter=True)
    assert len(sends) == 1

  def test_falls_back_to_latest_when_no_frame_list(self):
    # messages not tracked with stock_frames (e.g. LDW_02 on cars without the flag) behave as before
    cs, cc = _carstate(), _controller()
    cs.stock_values["TSK_06"] = {"COUNTER": 9, "CHECKSUM": 0, "TSK_Status": 2}
    sends = []
    assert cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends)
    assert _counters(sends, 2) == [9]
    sends = []
    assert not cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends)

  def test_long_backlog_is_capped_to_newest_frames(self):
    # a card stall longer than the panda's counter tolerance: only the newest MAX_FORWARD_BACKLOG frames go
    # out. With the stock ACC engaged the panda has forwarded all of them already, and a copy 8+ frames behind
    # its slot would read as "ahead" (4-bit counter) and put stale frames back on the car bus.
    packer = CANPacker(DBC)
    cp = CANParser(DBC, [("TSK_06", 50)], 0)
    cs, cc = _carstate(), _controller()
    self._step(cs, cp, packer, list(range(9)))
    sends = []
    assert cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends)
    assert _counters(sends, 2) == [5, 6, 7, 8]
    assert cc.forwarded_counters["TSK_06"] == 8
    assert CarController.MAX_FORWARD_BACKLOG <= 7  # must stay below the panda's stale window (9..15 behind)

  def test_short_backlog_unaffected_by_cap(self):
    packer = CANPacker(DBC)
    cp = CANParser(DBC, [("TSK_06", 50)], 0)
    cs, cc = _carstate(), _controller()
    self._step(cs, cp, packer, [12, 13, 14, 15])
    sends = []
    cc.forward_message(cs, mqbcan.MSG_TSK, 2, sends)
    assert _counters(sends, 2) == [12, 13, 14, 15]
