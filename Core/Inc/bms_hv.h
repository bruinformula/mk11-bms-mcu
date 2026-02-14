#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  HVSM_IDLE = 0,
  HVSM_CLOSE_AIR_MINUS,
  HVSM_PRECHARGE,
  HVSM_CLOSE_AIR_PLUS_WAIT,
  HVSM_ON,
  HVSM_FAULT
} hvsm_state_t;

typedef struct {
  // Inputs
  bool start_request;   // "we want HV on" (e.g. RTDS done / enable msg)
  bool sdc_closed;
  bool torque_zero;
  bool bms_fault_active;

  float VA_pack;        // accumulator/pack voltage target (V)
  float Vi_inv;         // inverter DC link voltage (V)
} hvsm_in_t;

typedef struct {
  // Outputs (desired relay states)
  bool air_minus_cmd;
  bool precharge_cmd;
  bool air_plus_cmd;

  // Status
  hvsm_state_t state;
  uint32_t fault_flags;   // optional bitfield for debug
} hvsm_out_t;

typedef struct {
  hvsm_state_t st;
  uint32_t t_enter_ms;

  // Parameters (tune these)
  float tau_s;               // time constant (seconds)
  float stop_margin_V;       // VA - margin (e.g. 10V)
  float curve_tol_V;         // allowed curve error (e.g. 15V)
  float max_time_mult;       // e.g. 5.0 => timeout at 5*tau
  float prech_open_delay_s;  // e.g. 0.5s after AIR+ closes

  // Internal
  float Vi_start;
  uint8_t curve_bad_ctr;
  uint8_t curve_bad_ctr_limit;
} hv_sm_t;

void HVSM_Init(hv_sm_t *sm, uint32_t now_ms);
hvsm_out_t HVSM_Tick(hv_sm_t *sm, uint32_t now_ms, hvsm_in_t in);
