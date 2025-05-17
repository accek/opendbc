import math
import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, apply_driver_steer_torque_limits, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mqbcan, pqcan
from opendbc.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert
AudibleAlert = structs.CarControl.HUDControl.AudibleAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState
ButtonType = structs.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP, CP_AC):
    super().__init__(dbc_names, CP, CP_SP, CP_AC)
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.ext_bus = CANBUS.pt if CP.networkLocation == structs.CarParams.NetworkLocation.fwdCamera else CANBUS.cam
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    self.apply_torque_last = 0
    self.forwarded_counters = {}
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.stock_acc_button = None
    self.stock_acc_button_pressed_frame = None
    self.last_lead_distance_value = 0.0

  def update(self, CC, CC_SP, CC_AC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    if CS.out.carFaultedNonCritical:
      # Simply forward messages if the car is faulted (e.g. Emergency Assist is active)

      # TODO(accek): adjust panda safety to allow this
      self.forward_message(CS, self.CCS.MSG_STEERING, CANBUS.pt, can_sends)
      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        self.forward_message(CS, self.CCS.MSG_EPS, CANBUS.cam, can_sends)
      if self.CP.openpilotLongitudinalControl:
        self.forward_message(CS, self.CCS.MSG_ACC_1, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_2, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_TSK, CANBUS.cam, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_1, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_2, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_3, CANBUS.pt, can_sends)
      self.forward_message(CS, self.CCS.MSG_LKA_HUD, CANBUS.pt, can_sends)
      self.forward_message(CS, self.CCS.MSG_ACC_BUTTONS, CANBUS.cam, can_sends)

      new_actuators = actuators.as_builder()
      new_actuators.steer = 0
      new_actuators.steerOutputCan = 0

      return new_actuators, can_sends

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # MQB racks reset the uninterrupted steering timer after a single frame
      # of HCA disabled; this is done whenever output happens to be zero.

      if CC.latActive and CS.eps_init_complete:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        if self.apply_torque_last == apply_torque:
          self.hca_frame_same_torque += self.CCP.STEER_STEP
          if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
            apply_torque -= (1, -1)[apply_torque < 0]
            self.hca_frame_same_torque = 0
        else:
          self.hca_frame_same_torque = 0
        hca_enabled = abs(apply_torque) > 0
      else:
        hca_enabled = False
        apply_torque = 0

      if not hca_enabled:
        self.hca_frame_timer_running = 0

      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_torque_last = apply_torque
      self.forward_message(CS, self.CCS.MSG_STEERING, CANBUS.pt, can_sends, self.CCS.create_steering_control,
                           apply_torque, hca_enabled, ignore_counter=True, require_stock_values=False)

    if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT and self.can_forward_message(CS, self.CCS.MSG_EPS):
      # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
      # to include small simulated inputs. See commaai/openpilot#23274 for background.
      sim_segment_frames = int(self.CCP.STEER_DRIVER_EA_SIMULATED)   # 1Nm/s
      sim_frame = self.frame % (2*sim_segment_frames)
      sign = 1 if CS.out.steeringTorque >= 0 else -1
      sim_torque = sim_frame if sim_frame < sim_segment_frames else 2*sim_segment_frames - sim_frame
      sim_torque = min(sim_torque, abs(2*self.apply_torque_last))
      if CC_AC.stockDriverMonitoring:
        sim_torque = 0
      ea_simulated_torque = np.clip(CS.out.steeringTorque - sign*sim_torque, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
      self.forward_message(CS, self.CCS.MSG_EPS, CANBUS.cam, can_sends, self.CCS.create_eps_update, ea_simulated_torque)

    # **** Acceleration Controls ******************************************** #

    accel = 0
    if self.CP.openpilotLongitudinalControl:
      cancel_pressed = any(be.type == ButtonType.cancel for be in CS.out.buttonEvents)
      # Additional conditions in acc_active and acc_override are to ensure that no messages are filtered out by panda safety,
      # otherwise ACC will fault after it detects a predefined number of missing messages.
      acc_active = CC.longActive and not CS.out.brakePressed and not cancel_pressed
      acc_override = (CC.cruiseControl.override or (acc_active and CS.out.gasPressed)) and not CS.out.brakePressed and not cancel_pressed
      acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, acc_override,
                                              CS.out.accFaulted, acc_active)
      accel = np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if acc_active and not acc_override else 0
      stopping = actuators.longControlState == LongCtrlState.stopping
      near_stop = stopping and CS.out.vEgo < self.CP.vEgoStopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
      lead_accel = CC_AC.hudControl.leadAccel if not np.isnan(CC_AC.hudControl.leadAccel) else None
      stopping_distance = max(0.0, CC_AC.hudControl.leadDistance - self.CCP.STOP_DISTANCE) if CC_AC.hudControl.leadDistance >= 0.0 else None
      self.forward_message(CS, self.CCS.MSG_ACC_1, CANBUS.pt, can_sends, self.CCS.create_acc_accel_control_1, CS.acc_type, accel,
                                                        acc_control, near_stop, starting, CS.esp_hold_confirmation, lead_accel,
                                                        stopping_distance, CS.out.vEgo, CS.out.aEgo)
      self.forward_message(CS, self.CCS.MSG_ACC_2, CANBUS.pt, can_sends, self.CCS.create_acc_accel_control_2, CS.acc_type, accel,
                                                        acc_control, near_stop, starting, CS.esp_hold_confirmation, lead_accel,
                                                        stopping_distance, CS.out.vEgo, CS.out.aEgo)
      self.forward_message(CS, self.CCS.MSG_TSK, CANBUS.cam, can_sends, self.CCS.create_tsk_update, CS.stock_values)

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0 and self.can_forward_message(CS, self.CCS.MSG_LKA_HUD):
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        if hud_control.audibleAlert in (AudibleAlert.none, AudibleAlert.prompt, AudibleAlert.promptRepeat, AudibleAlert.promptDistracted):
          hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
        elif hud_control.audibleAlert == AudibleAlert.warningImmediate:
          hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOverUrgent"]
        else:
          hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOverChime"]
      self.forward_message(CS, self.CCS.MSG_LKA_HUD, CANBUS.pt, can_sends, self.CCS.create_lka_hud_control,
                           hud_alert, hud_control, CC_SP.mads)

    if self.CP.openpilotLongitudinalControl:
      if CC_AC.stockAccOverrideArmed or CS.out_ac.stockAccOverride:
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_1, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_2, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_3, CANBUS.pt, can_sends)
      elif self.can_forward_message(CS, self.CCS.MSG_ACC_HUD_1) or \
          self.can_forward_message(CS, self.CCS.MSG_ACC_HUD_2) or \
          self.can_forward_message(CS, self.CCS.MSG_ACC_HUD_3):
        lead_distance = self.calculate_lead_distance(CS.out.vEgo, hud_control, CS.upscale_lead_car_signal, CC_AC.hudControl.leadDistance)
        acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, acc_override, CS.out.accFaulted, acc_active)
        # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
        set_speed = hud_control.setSpeed * CV.MS_TO_KPH
        current_speed = CS.out.vEgo * CV.MS_TO_KPH
        set_speed_reached = abs(set_speed - current_speed) <= 3
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_1, CANBUS.pt, can_sends, self.CCS.create_acc_hud_control_1,
                            acc_hud_status, set_speed, set_speed_reached, lead_distance, hud_control.leadDistanceBars)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_2, CANBUS.pt, can_sends, self.CCS.create_acc_hud_control_2,
                            acc_hud_status, set_speed, set_speed_reached, lead_distance, hud_control.leadDistanceBars)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_3, CANBUS.pt, can_sends, self.CCS.create_acc_hud_control_3,
                            acc_hud_status, set_speed, set_speed_reached, lead_distance, hud_control.leadDistanceBars)

    # **** Stock ACC Button Controls **************************************** #

    if self.CP.pcmCruise and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      self.forward_message(CS, self.CCS.MSG_ACC_BUTTONS, CANBUS.cam, can_sends, self.CCS.create_acc_buttons_control,
                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume)
    elif self.CP.openpilotLongitudinalControl:
      set_speed_ms = hud_control.setSpeed
      if set_speed_ms > 250 * CV.KPH_TO_MS:
        set_speed_ms = None
      can_switch_acc = not CS.out.brakePressed and not cancel_pressed
      stock_acc_requested = CC_AC.stockAccOverrideActive and can_switch_acc
      stock_acc_button = self.calculate_stock_acc_button(CS, set_speed_ms, stock_acc_requested)
      self.forward_message(CS, self.CCS.MSG_ACC_BUTTONS, CANBUS.cam, can_sends, self.CCS.create_acc_buttons_control,
                           frame='auto', buttons=stock_acc_button,
                           cancel=(CS.out_ac.stockAccOverride and not CC_AC.stockAccOverrideActive),
                           stock_gap_control=(CC_AC.stockAccOverrideArmed or CS.out_ac.stockAccOverride))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = float(accel)

    self.frame += 1
    return new_actuators, can_sends

  def calculate_lead_distance(self, v_ego, hud_control, upscale, distance):
    v_ego = max(2.5, v_ego)
    min_value = 64 if upscale else 1
    max_value = 1023 if upscale else 15
    max_relative_time = 3.1667
    min_relative_time = 0.8333

    if distance > 0:
      t_lead = distance / v_ego
      scale_fraction = (t_lead - min_relative_time) / (max_relative_time - min_relative_time)
      lead_distance_value = np.clip(scale_fraction, 0.0, 1.0) * (max_value - min_value) + min_value
      # apply some hysteresis to avoid oscillation
      if abs(lead_distance_value - self.last_lead_distance_value) > 0.5:
        self.last_lead_distance_value = lead_distance_value
      else:
        lead_distance_value = self.last_lead_distance_value
      return round(lead_distance_value)
    else:
      return max_value if hud_control.leadVisible else 0

  def can_forward_message(self, CS, msg_name):
    stock_values = CS.stock_values.get(msg_name)
    if stock_values is None:
      return False
    counter = stock_values.get("COUNTER")
    if counter is None:
      return True
    prev_counter = self.forwarded_counters.get(msg_name)
    return counter != prev_counter

  def forward_message(self, CS, msg_name, to_bus, can_sends, hook=None, *args, ignore_counter=False, require_stock_values=True, **kwargs):
    stock_values = CS.stock_values.get(msg_name)
    if stock_values is None:
      if require_stock_values:
        return False
      else:
        stock_values = {}
    counter = stock_values.get("COUNTER")
    prev_counter = self.forwarded_counters.get(msg_name)
    if counter is not None and counter == prev_counter and not ignore_counter:
      return False
    self.forwarded_counters[msg_name] = counter
    new_values = stock_values.copy()
    new_values.pop("CHECKSUM", None)
    if ignore_counter:
      new_values.pop("COUNTER", None)
    if hook is not None:
      new_values = hook(new_values, *args, **kwargs)
      if new_values is None:
        return False
    can_sends.append(self.packer_pt.make_can_msg(msg_name, to_bus, new_values))

  def calculate_stock_acc_button(self, CS, target_set_speed, stock_acc_requested):
    if not CS.out.cruiseState.available:
      return 0
    stock_set_speed = CS.stock_acc_set_speed
    step_up_speed = min(
      math.ceil((stock_set_speed + 0.1) / self.CCP.STOCK_ACC_SET_SPEED_STEP) * self.CCP.STOCK_ACC_SET_SPEED_STEP
        if stock_set_speed is not None else self.CCP.STOCK_ACC_MIN_SET_SPEED,
      self.CCP.STOCK_ACC_MAX_SET_SPEED,
    )
    step_down_speed = max(
      math.floor((stock_set_speed - 0.1) / self.CCP.STOCK_ACC_SET_SPEED_STEP) * self.CCP.STOCK_ACC_SET_SPEED_STEP
        if stock_set_speed is not None else self.CCP.STOCK_ACC_MIN_SET_SPEED,
      self.CCP.STOCK_ACC_MIN_SET_SPEED,
    )
    if self.stock_acc_button:
      # Require a few frames of non-pressing before allowing a new press to mitigate a race condition between button logic
      # and state update
      if self.frame > self.stock_acc_button_pressed_frame + 4 * self.CCP.BTN_STEP:
        self.stock_acc_button = None
      if self.frame > self.stock_acc_button_pressed_frame + self.CCP.BTN_STEP:
        return 0
      else:
        return self.stock_acc_button
    elif any(be.type == ButtonType.setCruise for be in CS.out.buttonEvents) and stock_acc_requested:
      # Forward SET directly, so that nonexact speeds are not rounded to the nearest step
      self.stock_acc_button = 4
      self.stock_acc_button_pressed_frame = self.frame
      return 4
    elif target_set_speed is None:
      return 0
    elif stock_set_speed is None or abs(step_up_speed - target_set_speed) < abs(stock_set_speed - target_set_speed):
      self.stock_acc_button = 1
      self.stock_acc_button_pressed_frame = self.frame
      return 1
    elif abs(step_down_speed - target_set_speed) < abs(stock_set_speed - target_set_speed):
      self.stock_acc_button = 2
      self.stock_acc_button_pressed_frame = self.frame
      return 2
    elif not CS.out_ac.stockAccOverride and stock_acc_requested:
      self.stock_acc_button = 3
      self.stock_acc_button_pressed_frame = self.frame
      return 3
    else:
      return 0
