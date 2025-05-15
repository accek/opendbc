from numpy import clip, interp

from opendbc.car import structs

MSG_STEERING = "HCA_01"
MSG_EPS = "LH_EPS_03"
MSG_LKA_HUD = "LDW_02"
MSG_ACC_BUTTONS = "GRA_ACC_01"
MSG_ACC_1 = "ACC_06"
MSG_ACC_2 = "ACC_07"
MSG_ACC_HUD_1 = "ACC_02"
MSG_ACC_HUD_2 = "ACC_04"
MSG_ACC_HUD_3 = "ACC_13"
MSG_TSK = "TSK_06"


def create_steering_control(values, apply_torque, lkas_enabled):
  values.setdefault("EA_ACC_Wunschgeschwindigkeit", 327.36)
  values.update({
    "HCA_01_Status_HCA": 5 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_torque),
    "HCA_01_LM_OffSign": 1 if apply_torque < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
  })
  return values


def create_eps_update(values, ea_simulated_torque):
  values.update({
    # Absolute driver torque input and sign, with EA inactivity mitigation
    "EPS_Lenkmoment": abs(ea_simulated_torque),
    "EPS_VZ_Lenkmoment": 1 if ea_simulated_torque < 0 else 0,
  })
  return values


def create_tsk_update(values, stock_values):
  # Simulate TSK confirming matching the stock ACC status.
  # Note that when stock ACC is active, panda safety will forward real TSK messages in realtime.
  acc_1 = stock_values.get(MSG_ACC_1)
  if acc_1:
    actual_tsk_status = values["TSK_Status"]
    stock_acc_status = acc_1["ACC_Status_ACC"]
    stock_acc_desired_accel = acc_1["ACC_Sollbeschleunigung_02"]
    if actual_tsk_status not in (2, 3, 4, 5):
      simulated_tsk_status = actual_tsk_status
    elif stock_acc_status in (0, 2) or (stock_acc_status in (3, 4, 5) and stock_acc_desired_accel > 3.005):  # standby, init, active with no accel request
      simulated_tsk_status = 2
    elif stock_acc_status in (1, 3, 4, 5):  # active, main disabled
      simulated_tsk_status = actual_tsk_status
    else:
      # faults
      simulated_tsk_status = stock_acc_status
    values.update({
      "TSK_Status": simulated_tsk_status,
      "TSK_zul_Regelabw": acc_1["ACC_zul_Regelabw_unten"],
    })
  return values


def lernmodus_value(available, active, lane_visible, lane_depart):
  if lane_depart:
    return 2
  elif not available:
    return 0
  elif active:
    return 3 if lane_visible else 2
  else:
    return 1

def create_lka_hud_control(values, hud_alert, hud_control, mads):
  left_lane_visible = hud_control.lanesVisible and hud_control.leftLaneVisible
  right_lane_visible = hud_control.lanesVisible and hud_control.rightLaneVisible
  values.update({
    "LDW_Status_LED_gelb": 1 if mads.enabled and not mads.active else 0,
    "LDW_Status_LED_gruen": 1 if mads.active else 0,
    "LDW_Lernmodus_links": lernmodus_value(mads.available, mads.active, left_lane_visible, hud_control.leftLaneDepart),
    "LDW_Lernmodus_rechts": lernmodus_value(mads.available, mads.active, right_lane_visible, hud_control.rightLaneDepart),
   })
  if hud_alert > 0:
    values["LDW_Texte"] = hud_alert
  return values


def create_acc_buttons_control(values, frame=None, buttons=0, cancel=False, resume=False, stock_gap_control=True):
  accel_cruise = 1 if buttons == 1 else 0
  decel_cruise = 1 if buttons == 2 else 0
  resume_cruise = 1 if buttons == 3 else 0
  set_cruise = 1 if buttons == 4 else 0

  if frame is None:
    values["COUNTER"] = (values["COUNTER"] + 1) % 16
  elif frame != 'auto':
    values["COUNTER"] = (frame + 1) % 16

  values.update({
     "GRA_Abbrechen": cancel,
     "GRA_Tip_Wiederaufnahme": resume or resume_cruise,
     "GRA_Tip_Setzen": set_cruise,
     "GRA_Tip_Runter": decel_cruise,
     "GRA_Tip_Hoch": accel_cruise,
   })

  if not stock_gap_control:
    values["GRA_Verstellung_Zeitluecke"] = 0

  return values


def acc_control_value(cruise_available, overriding, acc_faulted, long_active):
  acc_control = 0
  if acc_faulted:
    acc_control = 7
  elif overriding:
    acc_control = 4
  elif long_active:
    acc_control = 3
  elif cruise_available:
    acc_control = 2
  return acc_control


def acc_hud_status_value(cruise_available, gas_pressed, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init
  return acc_control_value(cruise_available, gas_pressed, acc_faulted, long_active)


def create_acc_accel_control_1(values, acc_type, accel, acc_control, stopping, starting, esp_hold, lead_accel, stopping_distance, actual_speed, actual_accel):
  acc_enabled = acc_control in (3, 4)

  if acc_enabled and not esp_hold:
    startstop = 2 if starting else 1
  else:
    startstop = 0

  if not acc_enabled:
    pos_jerk = 0.0
  elif esp_hold:
    pos_jerk = 4.0
  elif stopping:
    pos_jerk = 2.0
  else:
    pos_jerk = interp(actual_speed, [4.0, 6.0], [1.8, 0.6 if lead_accel is not None else 0.3])
    pos_jerk = max(pos_jerk, -actual_accel + 0.3)  # TODO: it's not ideally replicating the original logic

  # For stock comfort bands, see https://colab.research.google.com/drive/1y80X3VBACwLfMLvUE57GV4Yv6N8JjEmG?usp=sharing
  values = {
    "COUNTER": values["COUNTER"],
    "ACC_Typ": acc_type,
    "ACC_Status_ACC": acc_control,
    "ACC_StartStopp_Info": max(startstop, values["ACC_StartStopp_Info"]),  # stock radar may know better
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_zul_Regelabw_unten": clip(accel + 0.2, 0.0, 0.2) if acc_enabled and not stopping else 0,  # TODO: even better adjustment of comfort-band
    "ACC_zul_Regelabw_oben": clip((accel + 1.5) * (0.125 / 1.5), 0, 0.125) if acc_enabled and not stopping else 0,  # TODO: even better adjustment of comfort-band
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_pos_Sollbeschl_Grad_02": pos_jerk,
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
  }
  return values


def create_acc_accel_control_2(values, acc_type, accel, acc_control, stopping, starting, esp_hold, lead_accel, stopping_distance, actual_speed, actual_accel):
  acc_enabled = acc_control in (3, 4)

  if starting:
    acc_hold_type = 4  # hold release / startup
  elif esp_hold:
    acc_hold_type = 1  # hold request
  elif stopping:
    acc_hold_type = 3  # hold standby
  else:
    acc_hold_type = 0

  if stopping_distance is None:
    stopping_distance = 0.3
  elif stopping_distance > 20:
    stopping_distance = 20.0

  values = {
    "COUNTER": values["COUNTER"],
    "ACC_Anhalteweg": stopping_distance if stopping else 20.46,  # Distance to stop (stopping coordinator handles terminal roll-out)
    "ACC_Freilauf_Info": 2 if acc_enabled else 0,
    "ACC_Folgebeschl": clip(lead_accel, -4.6, 2.99) if lead_accel is not None else 3.02,
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_Anforderung_HMS": acc_hold_type,
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
  }

  return values


def create_acc_hud_control_1(values, acc_hud_status, set_speed, set_speed_reached, lead_distance, target_distance_bars):
  # TODO(accek): "ACC_Anzeige_Zeitluecke" support
  values.update({
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Wunschgeschw_erreicht": acc_hud_status == 3 and set_speed < 250 and set_speed_reached,
    "ACC_Gesetzte_Zeitluecke": target_distance_bars,
    "ACC_Display_Prio": min(2 if acc_hud_status in (3, 4) else 3, values["ACC_Display_Prio"]),
    "ACC_Abstandsindex": lead_distance,
    "ACC_Tachokranz": acc_hud_status in (3, 4),
  })
  return values


def create_acc_hud_control_2(values, acc_hud_status, set_speed, set_speed_reached, lead_distance, target_distance_bars):
  values.update({
    "ACC_Status_Zusatzanz": 2 if acc_hud_status in (3, 4) and lead_distance > 0 else 0,
  })
  return values


def create_acc_hud_control_3(values, acc_hud_status, set_speed, set_speed_reached, lead_distance, target_distance_bars):
  values.update({
    "ACC_Tachokranz": acc_hud_status in (3, 4),
  })
  return values


# AWV = Stopping Distance Reduction
# Refer to Self Study Program 890253: Volkswagen Driver Assistance Systems, Design and Function


def create_aeb_control(packer, fcw_active, aeb_active, accel):
  values = {
    "AWV_Vorstufe": 0,  # Preliminary stage
    "AWV1_Anf_Prefill": 0,  # Brake pre-fill request
    "AWV1_HBA_Param": 0,  # Brake pre-fill level
    "AWV2_Freigabe": 0,  # Stage 2 braking release
    "AWV2_Ruckprofil": 0,  # Brake jerk level
    "AWV2_Priowarnung": 0,  # Suppress lane departure warning in favor of FCW
    "ANB_Notfallblinken": 0, # Hazard flashers request
    "ANB_Teilbremsung_Freigabe": 0,  # Target braking release
    "ANB_Zielbremsung_Freigabe": 0,  # Partial braking release
    "ANB_Zielbrems_Teilbrems_Verz_Anf": 0.0,   # Acceleration requirement for target/partial braking, m/s/s
    "AWV_Halten": 0,  # Vehicle standstill request
    "PCF_Time_to_collision": 0xFF,  # Pre Crash Front, populated only with a target, might be used on Audi only
  }

  return packer.make_can_msg("ACC_10", 0, values)


def create_aeb_hud(packer, aeb_supported, fcw_active):
  values = {
    "AWV_Texte": 5 if aeb_supported else 7,  # FCW/AEB system status, display text (from menu in VAL)
    "AWV_Status_Anzeige": 1 if aeb_supported else 2,  #  FCW/AEB system status, available or disabled
  }

  return packer.make_can_msg("ACC_15", 0, values)
