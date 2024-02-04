#include "safety_hyundai_common.h"

int OP_LKAS_live = 0;
int OP_MDPS_live = 0;
int OP_CLU_live = 0;
int OP_SCC_live = 0;
int car_SCC_live = 0;
int OP_EMS_live = 0;
int HKG_mdps_bus = -1;
int HKG_scc_bus = -1;

bool HKG_LCAN_on_bus1 = false;
bool HKG_forward_bus1 = false;
bool HKG_forward_obd = false;
bool HKG_forward_bus2 = true;
int HKG_LKAS_bus0_cnt = 0;
int HKG_Lcan_bus1_cnt = 0;

const CanMsg HYUNDAI_COMMUNITY_TX_MSGS[] = {
  {832, 0, 8}, {832, 1, 8}, // LKAS11 Bus 0, 1
  {1265, 0, 4}, {1265, 1, 4}, {1265, 2, 4}, // CLU11 Bus 0, 1, 2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {593, 2, 8},  // MDPS12, Bus 2
  {1056, 0, 8}, //   SCC11,  Bus 0
  {1057, 0, 8}, //   SCC12,  Bus 0
  {1290, 0, 8}, //   SCC13,  Bus 0
  {905, 0, 8},  //   SCC14,  Bus 0
  {1186, 0, 8},  //   4a2SCC, Bus 0
  {790, 1, 8}, // EMS11, Bus 1
  {1155, 0, 8}, //   FCA12,  Bus 0
  {909, 0, 8},  //   FCA11,  Bus 0
  {2000, 0, 8},  // SCC_DIAG, Bus 0
  {882, 0, 8}, {882, 1, 8}, // ELECT_GEAR Bus 0, 1, 2
};

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_community_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  // {.msg = {{916, 0, 8, .expected_timestep = 20000U}}}, some Santa Fe does not have this msg, need to find alternative
};

#define HYUNDAI_COMMUNITY_ADDR_CHECK_LEN (sizeof(hyundai_community_addr_checks) / sizeof(hyundai_community_addr_checks[0]))
addr_checks hyundai_community_rx_checks = {hyundai_community_addr_checks, HYUNDAI_COMMUNITY_ADDR_CHECK_LEN};

static int hyundai_community_rx_hook(CANPacket_t *to_push) {

  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  bool valid = addr_safety_check(to_push, &hyundai_community_rx_checks,
                            hyundai_get_checksum, hyundai_compute_checksum,
                            hyundai_get_counter, NULL);

  if (!valid){
    puth(addr);
  }
  if (bus == 1 && HKG_LCAN_on_bus1) {valid = false;}
  // check if we have a LCAN on Bus1
  if (bus == 1 && (addr == 1296 || addr == 524)) {
    HKG_Lcan_bus1_cnt = 500;
    if (HKG_forward_bus1 || !HKG_LCAN_on_bus1) {
      HKG_LCAN_on_bus1 = true;
      HKG_forward_bus1 = false;
    }
  }
  // check if LKAS on Bus0
  if (addr == 832) {
    if (bus == 0 && HKG_forward_bus2) {HKG_forward_bus2 = false; HKG_LKAS_bus0_cnt = 20;}
    if (bus == 2) {
      if (HKG_LKAS_bus0_cnt > 0) {HKG_LKAS_bus0_cnt--;} else if (!HKG_forward_bus2) {HKG_forward_bus2 = true;}
      if (HKG_Lcan_bus1_cnt > 0) {HKG_Lcan_bus1_cnt--;} else if (HKG_LCAN_on_bus1) {HKG_LCAN_on_bus1 = false;}
    }
  }
  // check MDPS on Bus
  if ((addr == 593 || addr == 897) && HKG_mdps_bus != bus) {
    if (bus != 1 || (!HKG_LCAN_on_bus1 || HKG_forward_obd)) {
      HKG_mdps_bus = bus;
      if (bus == 1 && !HKG_forward_obd) {if (!HKG_forward_bus1 && !HKG_LCAN_on_bus1) {HKG_forward_bus1 = true;}}
    }
  }
  // check SCC on Bus
  if ((addr == 1056 || addr == 1057) && HKG_scc_bus != bus) {
    if (bus != 1 || !HKG_LCAN_on_bus1) {
      HKG_scc_bus = bus;
      if (bus == 1) {if (!HKG_forward_bus1) {HKG_forward_bus1 = true;}}
    }
  }

  if (valid) {
    if (addr == 593 && bus == HKG_mdps_bus) {
      int torque_driver_new = ((GET_BYTES(to_push, 0, 4) & 0x7ffU) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    if (addr == 1056 && !OP_SCC_live) {
      // 1 bits: 0
      int cruise_available = GET_BIT(to_push, 0U);
      hyundai_common_cruise_state_check(cruise_available);
    }

    // cruise control for car without SCC
    if (addr == 1265 && bus == 0 && HKG_scc_bus == -1 && !OP_SCC_live) {
      int cruise_button = GET_BYTE(to_push, 0) & 0x7U;
      // enable on res+ or set- buttons press
      if (!controls_allowed && (cruise_button == 1 || cruise_button == 2)) {
        controls_allowed = 1;
      }
      // disable on cancel press
      if (cruise_button == 4) {
        controls_allowed = 0;
      }
    }

    // sample wheel speed, averaging opposite corners
    if (addr == 902 && bus == 0) {
      uint32_t front_left_speed = GET_BYTES(to_push, 0, 2) & 0x3FFFU;
      uint32_t rear_right_speed = GET_BYTES(to_push, 6, 2) & 0x3FFFU;
      vehicle_moving = (front_left_speed > HYUNDAI_STANDSTILL_THRSLD) || (rear_right_speed > HYUNDAI_STANDSTILL_THRSLD);
    }

    gas_pressed = brake_pressed = false;

    generic_rx_checks((addr == 832 && bus == 0));
  }
  return valid;
}

static int hyundai_community_tx_hook(CANPacket_t *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  tx = msg_allowed(to_send, HYUNDAI_COMMUNITY_TX_MSGS, sizeof(HYUNDAI_COMMUNITY_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY_TX_MSGS[0]));

  // LKA STEER: safety check
  if (addr == 832) {
    OP_LKAS_live = 20;
    int desired_torque = ((GET_BYTES(to_send, 0, 4) >> 16) & 0x7ffU) - 1024U;
    bool steer_req = GET_BIT(to_send, 27U) != 0U;

    const SteeringLimits limits = hyundai_alt_limits ? HYUNDAI_STEERING_LIMITS_ALT : HYUNDAI_STEERING_LIMITS;
    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  //allow clu11 to be sent to MDPS if MDPS is not on bus0
  if (addr == 1265 && !controls_allowed && (bus != HKG_mdps_bus && HKG_mdps_bus == 1)) {
    if ((GET_BYTES(to_send, 0, 4) & 0x7U) != 4U) {
      tx = 0;
    }
  }

  if (addr == 593) {OP_MDPS_live = 20;}
  if (addr == 1265 && bus == 1) {OP_CLU_live = 20;} // only count mesage created for MDPS
  if (addr == 1057) {OP_SCC_live = 20; if (car_SCC_live > 0) {car_SCC_live -= 1;}}
  if (addr == 790) {OP_EMS_live = 20;}

  // 1 allows the message through
  return tx;
}

static int hyundai_community_fwd_hook(int bus_num, int addr) {

  int bus_fwd = -1;
  int fwd_to_bus1 = -1;
  if (HKG_forward_bus1 || HKG_forward_obd){fwd_to_bus1 = 1;}

  // forward cam to ccan and viceversa, except lkas cmd
  if (HKG_forward_bus2) {
    if (bus_num == 0) {
      if (!OP_CLU_live || addr != 1265 || HKG_mdps_bus == 0) {
        if (!OP_MDPS_live || addr != 593) {
          if (!OP_EMS_live || addr != 790) {
            bus_fwd = fwd_to_bus1 == 1 ? 12 : 2;
          } else {
            bus_fwd = 2;  // EON create EMS11 for MDPS
            OP_EMS_live -= 1;
          }
        } else {
          bus_fwd = fwd_to_bus1;  // EON create MDPS for LKAS
          OP_MDPS_live -= 1;
        }
      } else {
        bus_fwd = 2; // EON create CLU12 for MDPS
        OP_CLU_live -= 1;
      }
    }
    if (bus_num == 1 && (HKG_forward_bus1 || HKG_forward_obd)) {
      if (!OP_MDPS_live || addr != 593) {
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = 20;
        } else {
          bus_fwd = 2;  // EON create SCC11 SCC12 SCC13 SCC14 for Car
          OP_SCC_live -= 1;
        }
      } else {
        bus_fwd = 0;  // EON create MDPS for LKAS
        OP_MDPS_live -= 1;
      }
    }
    if (bus_num == 2) {
      if (!OP_LKAS_live || (addr != 832 && addr != 1157)) {
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = fwd_to_bus1 == 1 ? 10 : 0;
        } else {
          bus_fwd = fwd_to_bus1;  // EON create SCC12 for Car
          OP_SCC_live -= 1;
        }
      } else if (HKG_mdps_bus == 0) {
        bus_fwd = fwd_to_bus1; // EON create LKAS and LFA for Car
        OP_LKAS_live -= 1;
      } else {
        OP_LKAS_live -= 1; // EON create LKAS and LFA for Car and MDPS
      }
    }
  } else {
    if (bus_num == 0) {
      bus_fwd = fwd_to_bus1;
    }
    if (bus_num == 1 && (HKG_forward_bus1 || HKG_forward_obd)) {
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

static const addr_checks* hyundai_community_init(uint16_t param) {
  hyundai_common_init(param);
  controls_allowed = false;
  relay_malfunction_reset();

  // if (current_board->has_obd && HKG_forward_obd) {
  //   current_board->set_can_mode(CAN_MODE_OBD_CAN2);
  // }

  hyundai_community_rx_checks = (addr_checks){hyundai_community_addr_checks, HYUNDAI_COMMUNITY_ADDR_CHECK_LEN};
  return &hyundai_community_rx_checks;
}

const safety_hooks hyundai_community_hooks = {
  .init = hyundai_community_init,
  .rx = hyundai_community_rx_hook,
  .tx = hyundai_community_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_fwd_hook,
};
