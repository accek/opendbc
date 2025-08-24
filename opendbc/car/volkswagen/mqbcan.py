from numpy import clip, interp

from opendbc.car import structs
from opendbc.car.crc import CRC8H2F

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


def volkswagen_mqb_meb_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0xFF
  for i in range(1, len(d)):
    crc ^= d[i]
    crc = CRC8H2F[crc]
  counter = d[1] & 0x0F
  const = VOLKSWAGEN_MQB_MEB_CONSTANTS.get(address)
  if const:
    crc ^= const[counter]
    crc = CRC8H2F[crc]
  return crc ^ 0xFF


def xor_checksum(address: int, sig, d: bytearray) -> int:
  checksum = 0
  checksum_byte = sig.start_bit // 8
  for i in range(len(d)):
    if i != checksum_byte:
      checksum ^= d[i]
  return checksum


VOLKSWAGEN_MQB_MEB_CONSTANTS: dict[int, list[int]] = {
    0x40:  [0x40] * 16,  # Airbag_01
    0x86:  [0x86] * 16,  # LWI_01
    0x9F:  [0xF5] * 16,  # LH_EPS_03
    0xAD:  [0x3F, 0x69, 0x39, 0xDC, 0x94, 0xF9, 0x14, 0x64,
            0xD8, 0x6A, 0x34, 0xCE, 0xA2, 0x55, 0xB5, 0x2C],  # Getriebe_11
    0x0DB: [0x09, 0xFA, 0xCA, 0x8E, 0x62, 0xD5, 0xD1, 0xF0,
            0x31, 0xA0, 0xAF, 0xDA, 0x4D, 0x1A, 0x0A, 0x97],  # AWV_03
    0xFC:  [0x77, 0x5C, 0xA0, 0x89, 0x4B, 0x7C, 0xBB, 0xD6,
            0x1F, 0x6C, 0x4F, 0xF6, 0x20, 0x2B, 0x43, 0xDD],  # ESC_51
    0xFD:  [0xB4, 0xEF, 0xF8, 0x49, 0x1E, 0xE5, 0xC2, 0xC0,
            0x97, 0x19, 0x3C, 0xC9, 0xF1, 0x98, 0xD6, 0x61],  # ESP_21
    0x101: [0xAA] * 16,  # ESP_02
    0x102: [0xD7, 0x12, 0x85, 0x7E, 0x0B, 0x34, 0xFA, 0x16,
            0x7A, 0x25, 0x2D, 0x8F, 0x04, 0x8E, 0x5D, 0x35],  # ESC_50
    0x106: [0x07] * 16,  # ESP_05
    0x10B: [0x77, 0x5C, 0xA0, 0x89, 0x4B, 0x7C, 0xBB, 0xD6,
            0x1F, 0x6C, 0x4F, 0xF6, 0x20, 0x2B, 0x43, 0xDD],  # Motor_51
    0x116: [0xAC] * 16,  # ESP_10
    0x117: [0x16] * 16,  # ACC_10
    0x120: [0xC4, 0xE2, 0x4F, 0xE4, 0xF8, 0x2F, 0x56, 0x81,
            0x9F, 0xE5, 0x83, 0x44, 0x05, 0x3F, 0x97, 0xDF],  # TSK_06
    0x121: [0xE9, 0x65, 0xAE, 0x6B, 0x7B, 0x35, 0xE5, 0x5F,
            0x4E, 0xC7, 0x86, 0xA2, 0xBB, 0xDD, 0xEB, 0xB4],  # Motor_20
    0x122: [0x37, 0x7D, 0xF3, 0xA9, 0x18, 0x46, 0x6D, 0x4D,
            0x3D, 0x71, 0x92, 0x9C, 0xE5, 0x32, 0x10, 0xB9],  # ACC_06
    0x126: [0xDA] * 16,  # HCA_01
    0x12B: [0x6A, 0x38, 0xB4, 0x27, 0x22, 0xEF, 0xE1, 0xBB,
            0xF8, 0x80, 0x84, 0x49, 0xC7, 0x9E, 0x1E, 0x2B],  # GRA_ACC_01
    0x12E: [0xF8, 0xE5, 0x97, 0xC9, 0xD6, 0x07, 0x47, 0x21,
            0x66, 0xDD, 0xCF, 0x6F, 0xA1, 0x94, 0x74, 0x63],  # ACC_07
    0x139: [0xED, 0x03, 0x1C, 0x13, 0xC6, 0x23, 0x78, 0x7A,
            0x8B, 0x40, 0x14, 0x51, 0xBF, 0x68, 0x32, 0xBA],  # VMM_02
    0x13D: [0x20, 0xCA, 0x68, 0xD5, 0x1B, 0x31, 0xE2, 0xDA,
            0x08, 0x0A, 0xD4, 0xDE, 0x9C, 0xE4, 0x35, 0x5B],  # QFK_01
    0x14C: [0x16, 0x35, 0x59, 0x15, 0x9A, 0x2A, 0x97, 0xB8,
            0x0E, 0x4E, 0x30, 0xCC, 0xB3, 0x07, 0x01, 0xAD],  # Motor_54
    0x14D: [0x1A, 0x65, 0x81, 0x96, 0xC0, 0xDF, 0x11, 0x92,
            0xD3, 0x61, 0xC6, 0x95, 0x8C, 0x29, 0x21, 0xB5],  # ACC_18
    0x187: [0x7F, 0xED, 0x17, 0xC2, 0x7C, 0xEB, 0x44, 0x21,
            0x01, 0xFA, 0xDB, 0x15, 0x4A, 0x6B, 0x23, 0x05],  # Motor_EV_01
    0x1A4: [0x69, 0xBB, 0x54, 0xE6, 0x4E, 0x46, 0x8D, 0x7B,
            0xEA, 0x87, 0xE9, 0xB3, 0x63, 0xCE, 0xF8, 0xBF],  # EA_01
    0x1AB: [0x13, 0x21, 0x9B, 0x6A, 0x9A, 0x62, 0xD4, 0x65,
            0x18, 0xF1, 0xAB, 0x16, 0x32, 0x89, 0xE7, 0x26],  # ESP_33
    0x1F0: [0x2F, 0x3C, 0x22, 0x60, 0x18, 0xEB, 0x63, 0x76,
            0xC5, 0x91, 0x0F, 0x27, 0x34, 0x04, 0x7F, 0x02],  # EA_02
    0x20A: [0x9D, 0xE8, 0x36, 0xA1, 0xCA, 0x3B, 0x1D, 0x33,
            0xE0, 0xD5, 0xBB, 0x5F, 0xAE, 0x3C, 0x31, 0x9F],  # EML_06
    0x26B: [0xCE, 0xCC, 0xBD, 0x69, 0xA1, 0x3C, 0x18, 0x76,
            0x0F, 0x04, 0xF2, 0x3A, 0x93, 0x24, 0x19, 0x51],  # TA_01
    0x30C: [0x0F] * 16,  # ACC_02
    0x30F: [0x0C] * 16,  # SWA_01
    0x324: [0x27] * 16,  # ACC_04
    0x3BE: [0x1F, 0x28, 0xC6, 0x85, 0xE6, 0xF8, 0xB0, 0x19,
            0x5B, 0x64, 0x35, 0x21, 0xE4, 0xF7, 0x9C, 0x24],  # Motor_14
    0x3C0: [0xC3] * 16,  # Klemmen_Status_01
    0x3D5: [0xC5, 0x39, 0xC7, 0xF9, 0x92, 0xD8, 0x24, 0xCE,
            0xF1, 0xB5, 0x7A, 0xC4, 0xBC, 0x60, 0xE3, 0xD1],  # Licht_Anf_01
    0x65D: [0xAC, 0xB3, 0xAB, 0xEB, 0x7A, 0xE1, 0x3B, 0xF7,
            0x73, 0xBA, 0x7C, 0x9E, 0x06, 0x5F, 0x02, 0xD9],  # ESP_20
}
