import numpy as np

from opendbc.car.volkswagen import mqbcan


def _acc06(accel, speed, actual_accel, acc_control=3, stopping=False, starting=False, esp_hold=False):
  stock = {"COUNTER": 3, "ACC_StartStopp_Info": 1}
  return mqbcan.create_acc_accel_control_1(stock, 1, accel, acc_control, stopping, starting, esp_hold, None, None, speed, actual_accel)


class TestAccJerkLimits:
  # acspilot: jerk limits and comfort bands in ACC_06 follow the stock radar's behaviour (see tools/acspilot_tuning/REVIEW.md)
  def test_disabled_sends_zero(self):
    v = _acc06(0.5, 20.0, 0.5, acc_control=2)
    assert v["ACC_pos_Sollbeschl_Grad_02"] == 0.0 and v["ACC_neg_Sollbeschl_Grad_02"] == 0.0
    assert v["ACC_zul_Regelabw_unten"] == 0.0 and v["ACC_zul_Regelabw_oben"] == 0.0
    assert v["ACC_Sollbeschleunigung_02"] == 3.01

  def test_esp_hold_and_stopping(self):
    assert mqbcan.acc_jerk_limits(0.0, 0.0, 0.0, esp_hold=True, stopping=False) == (4.0, 4.0)
    assert mqbcan.acc_jerk_limits(-0.5, 0.3, -0.5, esp_hold=False, stopping=True) == (2.0, 4.0)
    v = _acc06(-0.5, 0.3, -0.5, stopping=True)
    assert v["ACC_zul_Regelabw_unten"] == 0.0 and v["ACC_zul_Regelabw_oben"] == 0.0

  def test_launch_uses_stock_positive_jerk(self):
    pos, _ = mqbcan.acc_jerk_limits(1.0, 2.0, 0.8, False, False)
    assert pos == 1.8

  def test_cruise_small_request_is_gentle(self):
    pos, neg = mqbcan.acc_jerk_limits(0.1, 20.0, 0.1, False, False)
    assert np.isclose(pos, 0.6 + 0.1 * 0.6) and neg == 1.0

  def test_large_request_opens_positive_limit(self):
    pos, _ = mqbcan.acc_jerk_limits(1.6, 20.0, 1.6, False, False)
    assert pos == 1.8

  def test_step_request_is_not_ramped_slowly(self):
    # car still coasting, planner asks for +1.0: limit must allow ~1 s rise, not the old 0.6 m/s3
    pos, _ = mqbcan.acc_jerk_limits(1.0, 20.0, -0.2, False, False)
    assert pos >= 1.5 * 1.2
    # and symmetrically for a sudden decel request from positive accel
    _, neg = mqbcan.acc_jerk_limits(-0.8, 20.0, 0.6, False, False)
    assert neg >= 1.5 * 1.4

  def test_hard_braking_keeps_full_negative_jerk(self):
    _, neg = mqbcan.acc_jerk_limits(-2.0, 20.0, -1.5, False, False)
    assert neg == 4.0

  def test_limits_within_dbc_range(self):
    for accel in np.linspace(-3.5, 2.0, 12):
      for speed in (0.0, 3.0, 5.0, 10.0, 30.0):
        for actual in (-3.0, 0.0, 2.0):
          pos, neg = mqbcan.acc_jerk_limits(accel, speed, actual, False, False)
          assert 0.3 <= pos <= 4.0 and 0.5 <= neg <= 4.0


class TestAccComfortBands:
  def test_bands_follow_stock(self):
    assert mqbcan.acc_comfort_bands(0.5, 10.0) == (0.36, 0.125)
    assert mqbcan.acc_comfort_bands(0.0, 10.0) == (0.192, 0.125)
    assert mqbcan.acc_comfort_bands(-1.0, 10.0) == (0.0, 0.0625)
    assert mqbcan.acc_comfort_bands(-2.0, 10.0) == (0.0, 0.0)
    assert mqbcan.acc_comfort_bands(0.2, 25.0)[1] == 0.1875

  def test_bands_are_continuous(self):
    # no 50 Hz chatter: a small change in accel changes the bands by a small amount
    accels = np.linspace(-2.0, 1.0, 301)
    lower = np.array([mqbcan.acc_comfort_bands(a, 20.0)[0] for a in accels])
    upper = np.array([mqbcan.acc_comfort_bands(a, 20.0)[1] for a in accels])
    assert np.all(np.abs(np.diff(lower)) < 0.01) and np.all(np.abs(np.diff(upper)) < 0.01)
    assert np.all(np.diff(lower) >= 0) and np.all(np.diff(upper) >= 0)

  def test_bands_within_dbc_range(self):
    for accel in np.linspace(-3.5, 2.0, 12):
      for speed in (0.0, 10.0, 30.0):
        lower, upper = mqbcan.acc_comfort_bands(accel, speed)
        assert 0.0 <= lower <= 1.512 and 0.0 <= upper <= 1.9375


class TestAccFollowAccel:
  # acspilot: ACC_Folgebeschl follows the stock radar's phases (see REVIEW.md §6)
  def test_inactive_without_lead_or_acc(self):
    assert mqbcan.acc_follow_accel(True, 0.5, None, False, False) == 3.02
    assert mqbcan.acc_follow_accel(False, 0.5, 0.3, False, False) == 3.02

  def test_moving_echoes_own_request(self):
    assert mqbcan.acc_follow_accel(True, 0.8, 0.2, False, False) == 0.8
    assert mqbcan.acc_follow_accel(True, -1.2, -0.3, False, False) == -1.2
    # clipped to the signal range
    assert mqbcan.acc_follow_accel(True, -7.0, 0.0, False, False) == -4.6
    assert mqbcan.acc_follow_accel(True, 3.5, 0.0, False, False) == 2.99

  def test_held_uses_lead_accel(self):
    assert mqbcan.acc_follow_accel(True, -0.55, 0.4, True, False) == 0.4
    # starting (hold release) goes back to the own request, as stock does
    assert mqbcan.acc_follow_accel(True, 0.9, 0.4, True, True) == 0.9

  def test_acc07_message_uses_rule(self):
    stock = {"COUNTER": 5}
    v = mqbcan.create_acc_accel_control_2(stock, 1, 0.7, 3, False, False, False, 0.1, None, 10.0, 0.6)
    assert v["ACC_Folgebeschl"] == 0.7 and v["ACC_Sollbeschleunigung_02"] == 0.7
    v = mqbcan.create_acc_accel_control_2(stock, 1, -0.55, 3, True, False, True, 0.3, 2.0, 0.0, 0.0)
    assert v["ACC_Folgebeschl"] == 0.3
    v = mqbcan.create_acc_accel_control_2(stock, 1, 0.7, 3, False, False, False, None, None, 10.0, 0.6)
    assert v["ACC_Folgebeschl"] == 3.02
