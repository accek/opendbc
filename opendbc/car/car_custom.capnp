using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb523986425343df4;

struct CarParamsAC @0xcd96dafb67a082d0 {
  stockAccOverrideAvailable @0 :Bool; # whether the car can switch between stock acc and op acc while engaged
  accelButtonResumesCruise @1 :Bool = true;
  resumeButtonSetsDefaultVCruise @2 :Bool;
  decelButtonLimitedToVEgoWhenOverriding @3 :Bool = true;
  cruiseLongPressReverse @4 :Bool;
  cruiseLargeStep @5 :Int8 = 5;
  stockAccSeparateGapControl @6 :Bool;
}

struct CarStateAC @0x9ccdc8676701b412 {
  screenBrightness @0 :Float32; # screen brightness from 0.0 to 1.0
  buttonEvents @1 :List(ButtonEvent);
  accFaultedTemporary @2 :Bool;
  stockAccOverride @3 :Bool;

  struct ButtonEvent {
    pressed @0 :Bool;
    type @1 :Type;

    enum Type {
      unknown @0;
      gapAdjustCruiseUp @1;
      gapAdjustCruiseDown @2;
    }
  }
}

struct CarControlAC @0xb057204d7deadf3f {
  stockAccOverrideArmed @0 :Bool;
  stockAccOverrideActive @1 :Bool;
  stockDriverMonitoring @2 :Bool;
  hudControl @3 :HUDControl;

  struct HUDControl {
    leadDistance @0 :Float32;
    leadAccel @1 :Float32;
  }
}
