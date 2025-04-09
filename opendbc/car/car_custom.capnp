using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb523986425343df4;

struct CarParamsAC @0xb86e6369214c01c8 {
  stockAccOverrideAvailable @0 :Bool; # whether the car can switch between stock acc and op acc while engaged
}

struct CarStateAC @0xf98d843bfd7004a3 {
  screenBrightness @0 :Float32; # screen brightness from 0.0 to 1.0
}

struct CarControlAC @0xf416ec09499d9d19 {
}
