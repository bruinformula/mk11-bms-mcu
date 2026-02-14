#include "bms_hv.h"
#include <math.h>

#define HVSM_FAULT_SDC_OPEN        (1u<<0)
#define HVSM_FAULT_TORQUE_NONZERO  (1u<<1)
#define HVSM_FAULT_BMS_FAULT       (1u<<2)
#define HVSM_FAULT_TIMEOUT         (1u<<3)
#define HVSM_FAULT_BAD_CURVE       (1u<<4)

static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void HVSM_Init(hv_sm_t *sm, uint32_t now_ms) {
  sm->st = HVSM_IDLE;
  sm->t_enter_ms = now_ms;

  sm->tau_s = 0.5f;              // TODO: compute/measure RC
  sm->stop_margin_V = 10.0f;
  sm->curve_tol_V = 15.0f;
  sm->max_time_mult = 5.0f;
  sm->prech_open_delay_s = 0.5f;

  sm->Vi_start = 0.0f;
  sm->curve_bad_ctr = 0;
  sm->curve_bad_ctr_limit = 5;   // 5 consecutive bad ticks => fault
}

hvsm_out_t HVSM_Tick(hv_sm_t *sm, uint32_t now_ms, hvsm_in_t in) {
  hvsm_out_t out = {0};
  out.state = sm->st;
  out.fault_flags = 0;

  // Default outputs depend on state; we fill them per-state below.
  // But first: global "hard stop" checks.
  if (!in.sdc_closed) {
    sm->st = HVSM_FAULT;
    out.fault_flags |= HVSM_FAULT_SDC_OPEN;
  }
  if (!in.torque_zero) {
    // You might only require torque_zero *to start* precharge.
    // If you want it always true, keep this as hard stop.
    sm->st = HVSM_FAULT;
    out.fault_flags |= HVSM_FAULT_TORQUE_NONZERO;
  }
  if (in.bms_fault_active) {
    sm->st = HVSM_FAULT;
    out.fault_flags |= HVSM_FAULT_BMS_FAULT;
  }

  float t_s = (now_ms - sm->t_enter_ms) / 1000.0f;

  switch (sm->st) {
    case HVSM_IDLE:
      out.air_minus_cmd = false;
      out.precharge_cmd = false;
      out.air_plus_cmd  = false;

      if (in.start_request && in.sdc_closed && in.torque_zero && !in.bms_fault_active) {
        sm->st = HVSM_CLOSE_AIR_MINUS;
        sm->t_enter_ms = now_ms;
      }
      break;

    case HVSM_CLOSE_AIR_MINUS:
        // Step 1: Close AIR-
        out.air_minus_cmd = true;
        out.precharge_cmd = true;   // Step 2: close precharge "almost simultaneously"
        out.air_plus_cmd  = false;

        // Capture initial Vi at the moment we begin precharge
        sm->Vi_start = in.Vi_inv;
        sm->curve_bad_ctr = 0;

        sm->st = HVSM_PRECHARGE;
        sm->t_enter_ms = now_ms;
        break;

    case HVSM_PRECHARGE: {
        out.air_minus_cmd = true;
        out.precharge_cmd = true;
        out.air_plus_cmd  = false;

        float VA = in.VA_pack;
        float Vi = in.Vi_inv;

        // Expected curve:
        // Vi_expected = Vi_start + (VA - Vi_start) * (1 - exp(-t/tau))
        float alpha = 1.0f - expf(-t_s / sm->tau_s);
        alpha = clampf(alpha, 0.0f, 1.0f);
        float Vi_expected = sm->Vi_start + (VA - sm->Vi_start) * alpha;

        // Curve check with debounce
        if (fabsf(Vi - Vi_expected) > sm->curve_tol_V) {
            if (sm->curve_bad_ctr < 255) sm->curve_bad_ctr++;
        } else {
            sm->curve_bad_ctr = 0;
        }

        if (sm->curve_bad_ctr >= sm->curve_bad_ctr_limit) {
            sm->st = HVSM_FAULT;
            out.fault_flags |= HVSM_FAULT_BAD_CURVE;
            break;
        }

        // Stop condition: Vi > VA - margin
        if (Vi > (VA - sm->stop_margin_V)) {
            // also require "not too long"
            if (t_s < (sm->max_time_mult * sm->tau_s)) {
            sm->st = HVSM_CLOSE_AIR_PLUS_WAIT;
            sm->t_enter_ms = now_ms;
            } else {
            sm->st = HVSM_FAULT;
            out.fault_flags |= HVSM_FAULT_TIMEOUT;
            }
            break;
        }

        // Timeout condition: t >= 5*tau
        if (t_s >= (sm->max_time_mult * sm->tau_s)) {
            sm->st = HVSM_FAULT;
            out.fault_flags |= HVSM_FAULT_TIMEOUT;
            break;
        }
    } break;

    case HVSM_CLOSE_AIR_PLUS_WAIT:
      // Step: close AIR+ immediately, keep precharge closed briefly, then open precharge after delay
      out.air_minus_cmd = true;
      out.air_plus_cmd  = true;
      out.precharge_cmd = true;

      if (t_s >= sm->prech_open_delay_s) {
        // open precharge, stay ON
        out.precharge_cmd = false;
        sm->st = HVSM_ON;
        sm->t_enter_ms = now_ms;
      }
      break;

    case HVSM_ON:
      // Normal operation: AIRs closed, precharge open
      out.air_minus_cmd = true;
      out.air_plus_cmd  = true;
      out.precharge_cmd = false;
      break;

    case HVSM_FAULT:
    default:
      // Kill: open all
      out.air_minus_cmd = false;
      out.air_plus_cmd  = false;
      out.precharge_cmd = false;
      break;
  }

  out.state = sm->st;
  return out;
}
