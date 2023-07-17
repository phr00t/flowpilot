@0x8e2af1e708af8b8d;

struct CanData {
  address @0 :UInt32;
  busTime @1 :UInt16;
  dat     @2 :Data;
  src     @3 :UInt8;
}

struct Event {
  logMonoTime @0 :UInt64;  # nanoseconds
  valid @1 :Bool = true;

  union {
	can @2 :List(CanData);
	sendcan @3 :List(CanData);
  }
}
