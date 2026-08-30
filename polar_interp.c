/*

  polar_interp.c - polar coordinate interpolation plugin (G12.1/G13.1 equivalent)

  Part of grblHAL

*/
#include "driver.h"

#if POLAR_INTERP_ENABLE

#if N_AXIS < 4 || AXIS3_LETTER != 'A'
#error Illegal axis configuration for polar interpolation - A axis required!
#endif

#include "grbl/protocol.h"
#include "grbl/planner.h"
#include "grbl/kinematics/interface.h"
#include "polar_interp.h"

static bool polar_enabled = false;

static on_report_options_ptr on_report_options;
static user_mcode_ptrs_t user_mcode;

#define ROTARY_AXIS A_AXIS

static coord_data_t start_virtual;    // current virtual (Y, Z) — persists across calls, seeded at enable
static float last_a_real = 0.0f;      // unwrapped real A, radians

// --- segment_line state ---
static coord_data_t real_target;
static float seg_start_z, seg_end_z, seg_start_y, seg_end_y;
static uint32_t seg_count, seg_index;
static float prev_z_real, prev_a_real_deg;

static inline float angle_diff (float a, float b)
{
    float d = fmodf(a - b + (float)M_PI, 2.0f * (float)M_PI);
    if(d < 0.0f) d += 2.0f * (float)M_PI;
    return d - (float)M_PI;
}

// resolves a virtual (y, z) point to real (Z_real, A_real_rad), continuity-aware
static void polar_resolve (float y, float z, float *z_real_out, float *a_real_out)
{
    float dy = y - gc_state.modal.g5x_offset.data.coord.values[Y_AXIS];
    float dz = z - gc_state.modal.g5x_offset.data.coord.values[Z_AXIS];
    float r  = sqrtf(dy * dy + dz * dz);

    // if(r < POLE_EPSILON) {
    //     *a_real_out = last_a_real;
    //     *z_real_out = pole_offset;
    //     return;
    // }

    float candidate_angle = atan2f(dy, -dz);
    float delta_direct  = angle_diff(candidate_angle, last_a_real);
    //float delta_flipped = angle_diff(candidate_angle + (float)M_PI, last_a_real);

    // if(fabsf(delta_flipped) < fabsf(delta_direct)) {
    //    last_a_real += delta_flipped;
        *z_real_out = -r + gc_state.modal.g5x_offset.data.coord.values[Z_AXIS];
    // } else {
        last_a_real += delta_direct;
    //     *z_real_out = pole_offset + r;
    // }

    *a_real_out = last_a_real;
}

static coord_data_t *polarTransformFromCartesian (coord_data_t *target, coord_data_t *position)
{
    return target;   // always identity — real transform lives in segment_line only
}

static coord_data_t *polarStepsToCartesianOff (coord_data_t *position, mpos_t *steps)
{
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        position->values[idx] = (float)steps->values[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);

    return position;
}

static coord_data_t *polarStepsToCartesianOn (coord_data_t *position, mpos_t *steps)
{
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        position->values[idx] = (float)steps->values[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);

    return position;
}

static uint32_t polar_calc_segments (float z0, float z1)
{
    float len = fabsf(z1 - z0);

    float r0 = fabsf(z0);
    float r1 = fabsf(z1);
    float min_r = fmaxf(fminf(r0, r1), 5.0f);   // TODO: settings.min_radius_for_step_calc

    float max_a_step_rad = (float)(2.0 * M_PI / 180.0);   // TODO: settings.max_a_step_deg
    uint32_t n_by_angle = (uint32_t)ceilf((len / min_r) / max_a_step_rad);

    float max_segment_len = 0.5f;   // TODO: settings.max_segment_len
    uint32_t n_by_len = (uint32_t)ceilf(len / max_segment_len);

    uint32_t n = n_by_angle > n_by_len ? n_by_angle : n_by_len;
    return n < 1 ? 1 : n;
}

static coord_data_t *polarSegmentLineOff (coord_data_t *target, coord_data_t *position, plan_line_data_t *pl_data, bool init)
{
    static coord_data_t passthru;
    static uint_fast8_t iterations;

    if(init) {
        iterations = 2;
        memcpy(&passthru, target, sizeof(coord_data_t));
    }

    return iterations-- == 0 ? NULL : &passthru;
}

static coord_data_t *polarSegmentLineOn (coord_data_t *target, coord_data_t *position, plan_line_data_t *pl_data, bool init)
{
    if(init) {

        // tolerance
        const float eps = 1e-6f;

        seg_index = 0;
        seg_start_z = start_virtual.values[Z_AXIS];
        seg_start_y = start_virtual.values[Y_AXIS];
        seg_end_z   = target->values[Z_AXIS];
        seg_end_y   = target->values[Y_AXIS];

        memcpy(&real_target, target, sizeof(coord_data_t));
        real_target.values[Y_AXIS] = position->values[Y_AXIS];  // Y is virtual, not real
        real_target.values[ROTARY_AXIS] = position->values[ROTARY_AXIS];  // No commanded A-axis movement while polar interpolation is active

        if((fabsf(seg_end_z - seg_start_z) < eps) && (fabsf(seg_end_y - seg_start_y) < eps)) {
            seg_count = 0;
            return &real_target;   // no radial change — Z/Y untouched, one pass-through result
        }

        seg_count = 5;//polar_calc_segments(seg_start_z, seg_end_z); //temporary hard code

        prev_z_real = position->values[Z_AXIS];
        prev_a_real_deg = position->values[ROTARY_AXIS];
    }

    if(seg_index >= seg_count) {
        start_virtual.values[Z_AXIS] = seg_end_z;
        start_virtual.values[Y_AXIS] = seg_end_y;
        return NULL;
    }

    float t0 = (float)seg_index / (float)seg_count;
    float z0 = seg_start_z + t0 * (seg_end_z - seg_start_z);
    float y0 = seg_start_y + t0 * (seg_end_y - seg_start_y);

    seg_index++;
    float t1 = (float)seg_index / (float)seg_count;
    float z1 = seg_start_z + t1 * (seg_end_z - seg_start_z);
    float y1 = seg_start_y + t1 * (seg_end_y - seg_start_y);

    float z_real, a_real_rad;
    polar_resolve(y1, z1, &z_real, &a_real_rad);  
    float a_real_deg = a_real_rad * (float)(180.0 / M_PI);  

    real_target.values[Z_AXIS] = z_real;  
    real_target.values[ROTARY_AXIS] = a_real_deg;

    float virtual_step = fabsf(z1 - z0);
    float dz_real = z_real - prev_z_real;
    float da_real_rad = (a_real_deg - prev_a_real_deg) * (float)(M_PI / 180.0);
    float real_step = sqrtf(dz_real * dz_real + (da_real_rad * (z_real)) * (da_real_rad * (z_real)));

    pl_data->rate_multiplier = virtual_step > 0.0f ? real_step / virtual_step : 1.0f;

    prev_z_real = z_real;
    prev_a_real_deg = a_real_deg;

    return &real_target;
}

static uint_fast8_t polarGetAxisMask (uint_fast8_t idx)
{
    return bit(idx);
}

static void polarSetTargetPos (uint_fast8_t idx)
{
    sys.position[idx] = 0;
}

static void polarSetMachinePositions (axes_signals_t cycle)
{
    limits_set_machine_positions(cycle, true);
}

static bool polarHomingCycleValidate (axes_signals_t cycle)
{
    return true;
}

static float polarHomingCycleGetFeedrate (axes_signals_t cycle, float feedrate, homing_mode_t mode)
{
    return feedrate;
}

static void polar_enable (bool enable)
{
    if(enable) {
        float z_real = gc_state.position[Z_AXIS];

        //last_a_real = z_real > 0 ? (float)M_PI/2 : -(float)M_PI/2;   // y=0 at enable ⇒ virtual Z == real Z directly
        last_a_real = 0; // ASSUME A_AXIS IS ALWAYS 0 AT ENABLE, NEED TO FIX THIS OR IT MIGHT NOT WORK IF A HAS BEEN MOVED SINCE STARTUP

        start_virtual.values[Z_AXIS] = z_real;   // y=0 at enable ⇒ virtual Z == real Z directly
        start_virtual.values[Y_AXIS] = 0;        // y=0 at enable ⇒ virtual Z == real Z directly

        kinematics.transform_steps_to_cartesian = polarStepsToCartesianOn;
        kinematics.segment_line = polarSegmentLineOn;
    } else {
        kinematics.transform_steps_to_cartesian = polarStepsToCartesianOff;
        kinematics.segment_line = polarSegmentLineOff;
    }

    polar_enabled = enable;
    sync_position();
}

static user_mcode_type_t userMCodeCheck (user_mcode_t mcode)
{
    return mcode == POLAR_INTERP_ENABLE_MCODE || mcode == POLAR_INTERP_DISABLE_MCODE
            ? UserMCode_Normal
            : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t userMCodeValidate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case POLAR_INTERP_ENABLE_MCODE:
        case POLAR_INTERP_DISABLE_MCODE:
            gc_block->user_mcode_sync = true;   // drain planner before mode change takes effect
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    switch((uint32_t)gc_block->user_mcode) {

        case POLAR_INTERP_ENABLE_MCODE:
            polar_enable(true);
            report_message("Polar interpolation ON", Message_Info);
            break;

        case POLAR_INTERP_DISABLE_MCODE:
            polar_enable(false);
            report_message("Polar interpolation OFF", Message_Info);
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
    user_mcode.execute(state, gc_block);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Polar Interpolation", "0.01");
}


void polar_interp_init(void)
{
    kinematics.transform_from_cartesian = polarTransformFromCartesian;
    kinematics.transform_steps_to_cartesian = polarStepsToCartesianOff;
    kinematics.segment_line = polarSegmentLineOff;

    kinematics.limits_set_target_pos = polarSetTargetPos;
    kinematics.limits_get_axis_mask = polarGetAxisMask;
    kinematics.limits_set_machine_positions = polarSetMachinePositions;
    kinematics.homing_cycle_validate = polarHomingCycleValidate;
    kinematics.homing_cycle_get_feedrate = polarHomingCycleGetFeedrate;

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = userMCodeCheck;
    grbl.user_mcode.validate = userMCodeValidate;
    grbl.user_mcode.execute = userMCodeExecute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}

#endif // POLAR_INTERP_ENABLE
