/*

  polar_interp.c - polar coordinate interpolation plugin (G12.1/G13.1 equivalent)

  Part of grblHAL

  What it does
  ------------
  Lets a machine with a linear Z axis and a rotary A axis (rotating about X) cut
  shapes programmed in a flat Y/Z plane on the part, e.g. flats, slots or
  polygons around a cylinder, without CAM output in Z/A.

  While polar mode is on, Y and Z in the program are "virtual" cartesian
  coordinates in the plane perpendicular to the A axis. Each move is split into
  short segments (MAX_SEG_LENGTH_MM) along the straight virtual line, and every
  segment end point is converted to real machine coordinates:

      r      = distance from the pole to the virtual (Y, Z) point
      Z_real = pole Z - r   if the tool works below the pole (-Z side)
               pole Z + r   if the tool works above the pole (+Z side)
      A_real = angle of the point about the pole, unwrapped to the nearest
               equivalent of the current A (no unnecessary full turns)

  The working side is fixed when polar mode is turned on, from where real Z is
  relative to the pole, and Z never crosses to the other side while polar mode is
  on. A move that passes through the pole rotates A by 180 degrees and comes back
  out on the same side.

  Real Y does not move while polar mode is on. X and any other axes pass
  through unchanged.

  Usage
  -----
    M112    polar interpolation ON
    M113    polar interpolation OFF

  Both wait for all queued motion to finish before switching.

  1. Set up offsets so that work Y0 Z0 is on the A axis centreline. That point is
     the pole. All offsets count (G5x + G92 + tool length), so tool offsets (G43)
     work as normal: with the tool's length offset active, Z0 is the tool tip on
     the centreline. The pole follows offset changes, e.g. a tool change with a
     new G43 mid-program.
  2. Position real Y at the work Y0 (on the centreline), and Z on the side of the
     pole the tool will work from (above or below, not on the centreline).
     A can be at any angle. M112 is rejected with an error if real Y is not on
     the centreline (within POLAR_Y_TOLERANCE).
  3. M112, then program Y/Z moves (G0/G1/G2/G3, G90 or G91) as if the face of the
     part were a flat Y/Z plane. Leave A out of the program; A words are ignored.
  4. M113 to return to normal kinematics.

  Y and Z must be homed: M112 is rejected with error 46 (homing required)
  otherwise. A does not need to be homed.

  Polar mode is also switched off at program end (M2/M30), on reset, and after
  any homing cycle.

  While polar mode is on, reported Y/Z positions are virtual coordinates, and A is
  the real A position.

  Notes
  -----
  - If Z is on the centreline when polar mode is turned on, the side can't be
    determined: a warning is reported and the -Z side is assumed.
  - Near the pole, a short virtual segment can mean a large A rotation.
  - F applies along the virtual path (G94 and G93). Near the pole the required A
    speed can exceed A's max rate/acceleration and the planner slows down to suit.
  - G20 with the revert_metric_conversion rotary setting is not compensated:
    polar moves would run 25.4x slow. Use G21 in polar mode.

  Requires: KINEMATICS_API, POLAR_INTERP_ENABLE, N_AXIS >= 4 with an A axis.

*/
#include "driver.h"

#if POLAR_INTERP_ENABLE

#if N_AXIS < 4 || AXIS3_LETTER != 'A'
#error Illegal axis configuration for polar interpolation - A axis required!
#endif

#include "grbl/protocol.h"
#include "grbl/planner.h"
#include "grbl/state_machine.h"
#include "grbl/kinematics/interface.h"
#include "polar_interp.h"

static bool polar_enabled = false;
static volatile bool jog_cancel = false;    // set from interrupt context (onJogCancel)

static on_report_options_ptr on_report_options;
static on_program_completed_ptr on_program_completed;
static on_reset_ptr on_reset;
static on_homing_completed_ptr on_homing_completed;
static travel_limits_ptr check_travel_limits;
static on_jog_cancel_ptr on_jog_cancel;
static user_mcode_ptrs_t user_mcode;

#define ROTARY_AXIS A_AXIS

#define MAX_SEG_LENGTH_MM 0.5f
#define POLE_EPSILON 1E-2f      // radius below which the angle is treated as undefined
#define POLAR_Y_TOLERANCE 1E-2f // max real Y distance from the pole accepted by M112 (mm)

static coord_data_t start_virtual;    // virtual (Y, Z) at start of the current move, derived from planner position in init
static float last_a_real = 0.0f;      // unwrapped real A, radians, derived from planner position in init
static float polar_side = 1.0f;       // side of the pole the tool works on, latched at M112:
                                      // +1 = Z_real below pole (-Z side), -1 = above (+Z side)

// --- segment_line state ---
static coord_data_t real_target;
static float seg_start_z, seg_end_z, seg_start_y, seg_end_y;
static uint32_t seg_count, seg_index;
static coord_data_t prev_real;        // real end point of the previous segment (move start for the first)
static float seg_virtual_len;         // virtual path length of each segment (all axes except A)
static coord_data_t seg_start_pos, seg_end_pos;  // real start / target, for interpolating non-polar axes (X...)

static inline float angle_diff (float a, float b)
{
    float d = fmodf(a - b + (float)M_PI, 2.0f * (float)M_PI);
    if(d < 0.0f) d += 2.0f * (float)M_PI;
    return d - (float)M_PI;
}

// Pole (A axis centreline) in machine coordinates = machine position of work Y0/Z0 with
// all offsets applied: G5x + G92 + tool length offset. Program targets carry the same
// offsets, so target - pole is always the programmed Y/Z regardless of how they are set,
// and a tool length change mid-program moves the pole with the tool.
// real_time = true uses the offsets of the block currently executing (for reports while
// motion with older offsets is still queued), false the offsets currently being parsed.
static inline float polar_pole (uint_fast8_t idx, bool real_time)
{
    return gc_get_offset(idx, real_time);
}

// resolves a virtual (y, z) point to real (Z_real, A_real_rad), continuity-aware
static void polar_resolve (float y, float z, float *z_real_out, float *a_real_out)
{
    float pole_z = polar_pole(Z_AXIS, false);
    float dy = y - polar_pole(Y_AXIS, false);
    float dz = z - pole_z;
    float r  = sqrtf(dy * dy + dz * dz);

    if(r < POLE_EPSILON) {
        *a_real_out = last_a_real;
        *z_real_out = pole_z;   // TODO: settings.min_radius_for_step_calc
        return;
    }

    // Stay on the side of the pole latched at M112 - never drive Z through the centreline.
    // polar_side = +1: Z_real = pole - r, -1: Z_real = pole + r. The angle is mirrored to
    // match, which keeps this the exact inverse of polar_real_to_virtual() on both sides.
    float candidate_angle = atan2f(polar_side * dy, -polar_side * dz);

    *z_real_out = pole_z - polar_side * r;
    last_a_real += angle_diff(candidate_angle, last_a_real);

    *a_real_out = last_a_real;
}

// Inverse of polar_resolve: real Z and A (degrees) -> virtual Y and Z, in place.
// Other axes are left untouched.
static void polar_real_to_virtual (coord_data_t *pos, bool real_time)
{
    float pole_z = polar_pole(Z_AXIS, real_time);
    float r = pole_z - pos->values[Z_AXIS];
    float a = pos->values[ROTARY_AXIS] * RADDEG;

    pos->values[Y_AXIS] = polar_pole(Y_AXIS, real_time) + r * sinf(a);
    pos->values[Z_AXIS] = pole_z - r * cosf(a);
}

// Used by core to plan homing moves: transform_from_cartesian(&k_target, &target).
// Homing is real (machine) space, so copy the input to the output. NOTE: target is
// an uninitialised buffer on entry - it must be filled, not just returned.
static coord_data_t *polarTransformFromCartesian (coord_data_t *target, coord_data_t *position)
{
    memcpy(target, position, sizeof(coord_data_t));

    return target;
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

// Reports virtual Y/Z (A is reported as real A). Also what gc_sync_position() /
// sync_position() write into gc_state.position, so G91 moves start from virtual coords.
static coord_data_t *polarStepsToCartesianOn (coord_data_t *position, mpos_t *steps)
{
    polarStepsToCartesianOff(position, steps);
    polar_real_to_virtual(position, true);    // status reports: offsets of the executing block

    return position;
}

static uint32_t polar_calc_segments (float dz, float dy)
{
    float len = sqrtf(dz * dz + dy * dy);

    return (uint32_t)ceilf(len / MAX_SEG_LENGTH_MM);
}

// Distance between two real points as plan_buffer_line() will measure it, from step-rounded
// deltas (rotary axes in degrees counted as mm).
// Returns the distance the planner times the move over: with rotary fix enabled and both
// linear and rotary axes moving, the planner switches to inverse time over the linear
// distance only. *full is the full distance (block->millimeters), which the reported
// speed is measured along.
static float planner_distance (coord_data_t *p0, coord_data_t *p1, float *full)
{
    bool rotary = false;
    float all = 0.0f, linear = 0.0f;
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        float spm = settings.axis[idx].steps_per_mm;
        int32_t steps = lroundf(p1->values[idx] * spm) - lroundf(p0->values[idx] * spm);
        if(steps) {
            float d = (float)steps / spm;
            all += d * d;
            if(bit_istrue(settings.steppers.is_rotary.mask, bit(idx)))
                rotary = true;
            else
                linear += d * d;
        }
    } while(idx);

    *full = sqrtf(all);

    return rotary && linear > 0.0f && settings.flags.rotary_fix_enable ? sqrtf(linear) : *full;
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

        jog_cancel = false;     // a new move clears any earlier cancel

        // Derive the start point from the planner position (raw machine Z/A), not from
        // remembered state, so any resync (jog cancel, probe, feed hold...) is picked up.
        last_a_real = position->values[ROTARY_AXIS] * RADDEG;
        memcpy(&start_virtual, position, sizeof(coord_data_t));
        polar_real_to_virtual(&start_virtual, false);

        seg_index = 0;
        seg_start_z = start_virtual.values[Z_AXIS];
        seg_start_y = start_virtual.values[Y_AXIS];
        seg_end_z   = target->values[Z_AXIS];
        seg_end_y   = target->values[Y_AXIS];

        memcpy(&seg_start_pos, position, sizeof(coord_data_t));
        memcpy(&seg_end_pos, target, sizeof(coord_data_t));

        memcpy(&real_target, target, sizeof(coord_data_t));
        real_target.values[Y_AXIS] = position->values[Y_AXIS];  // Y is virtual, not real
        real_target.values[ROTARY_AXIS] = position->values[ROTARY_AXIS];  // No commanded A-axis movement while polar interpolation is active

        memcpy(&prev_real, position, sizeof(coord_data_t));

        if((fabsf(seg_end_z - seg_start_z) < eps) && (fabsf(seg_end_y - seg_start_y) < eps)) {
            // No Y/Z change (e.g. X only): one segment, which resolves to the current real Z/A.
            seg_end_z = seg_start_z;
            seg_end_y = seg_start_y;
            seg_count = 1;
        } else
            seg_count = polar_calc_segments(seg_end_z-seg_start_z, seg_end_y-seg_start_y);

        if(seg_count == 0)
            seg_count = 1;

        // Virtual length of the whole move: virtual Y/Z plus the non-polar axes (X...),
        // which are the same in virtual and real space. A is not programmed in polar mode.
        float len = (seg_end_y - seg_start_y) * (seg_end_y - seg_start_y) +
                    (seg_end_z - seg_start_z) * (seg_end_z - seg_start_z);
        uint_fast8_t idx = N_AXIS;
        do {
            idx--;
            if(idx != Y_AXIS && idx != Z_AXIS && idx != ROTARY_AXIS)
                len += (seg_end_pos.values[idx] - seg_start_pos.values[idx]) * (seg_end_pos.values[idx] - seg_start_pos.values[idx]);
        } while(idx);

        seg_virtual_len = sqrtf(len) / (float)seg_count;   // segments are equal length

        // mc_line() soft limit checks the init return value, so it must be the real end
        // point, not the virtual target. Resolve it without disturbing last_a_real, which
        // the segment calls unwrap from step by step. (A straight line sweeps < 180 deg
        // about the pole, so the end A is the same either way; Z does not depend on it.)
        float z_end, a_end_rad, a_start = last_a_real;
        polar_resolve(seg_end_y, seg_end_z, &z_end, &a_end_rad);
        last_a_real = a_start;

        real_target.values[Z_AXIS] = z_end;
        real_target.values[ROTARY_AXIS] = a_end_rad * DEGRAD;
    } else {

        // Stop feeding segments once a jog is cancelled. Only checked here, never on the
        // init call: mc_line() dereferences the init return value unconditionally.
        if(jog_cancel)
            return NULL;

        if(seg_index >= seg_count)
            return NULL;

        seg_index++;
        float t1 = (float)seg_index / (float)seg_count;
        float z1 = seg_start_z + t1 * (seg_end_z - seg_start_z);
        float y1 = seg_start_y + t1 * (seg_end_y - seg_start_y);

        float z_real, a_real_rad;
        polar_resolve(y1, z1, &z_real, &a_real_rad);  
        float a_real_deg = a_real_rad * (float)(180.0 / M_PI);  

        real_target.values[Z_AXIS] = z_real;
        real_target.values[ROTARY_AXIS] = a_real_deg;

        // Non-polar axes (X and any others) move linearly with the virtual Y/Z path,
        // landing exactly on the target at the last segment. Y (held) and A are untouched.
        uint_fast8_t idx = N_AXIS;
        do {
            idx--;
            if(idx != Y_AXIS && idx != Z_AXIS && idx != ROTARY_AXIS)
                real_target.values[idx] = seg_index == seg_count
                                           ? seg_end_pos.values[idx]
                                           : seg_start_pos.values[idx] + t1 * (seg_end_pos.values[idx] - seg_start_pos.values[idx]);
        } while(idx);

        // Feed rate compensation. The planner runs each segment at feed_rate over its own
        // (real, joint space) distance; the programmed F applies along the virtual path.
        // Scale feed_rate so the segment takes virtual_len / F, and set rate_multiplier
        // so the reported speed is the virtual path speed.
        // NOTE: mc_line() restores feed_rate after every segment, so *= is per segment.
        if(!pl_data->condition.rapid_motion) {     // rapids run at axis max rates, F not used
            if(pl_data->condition.inverse_time)
                pl_data->feed_rate *= (float)seg_count; // G93: F is for the whole move, each segment gets 1/seg_count of the time
            else {
                float full_len, timed_len = planner_distance(&prev_real, &real_target, &full_len);

                if(timed_len > 0.0f && seg_virtual_len > 0.0f) {
                    pl_data->feed_rate *= timed_len / seg_virtual_len;
                    pl_data->rate_multiplier = seg_virtual_len / full_len;
                } else
                    pl_data->rate_multiplier = 1.0f;  // no steps: planner drops the segment
            }
        }

        memcpy(&prev_real, &real_target, sizeof(coord_data_t));
    }

    //TODO: add case for no movement? to return NULL
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

// Without KINEMATICS_API core calls limits_set_machine_positions(cycle, true) after a
// switch homing cycle, and (cycle, false) for manually homed axes (no homing cycle,
// current position accepted as home). The kinematics hook has no flag, so split the
// mask the same way: axes with a homing cycle add the pulloff, manual ones don't.
static void polarSetMachinePositions (axes_signals_t cycle)
{
    axes_signals_t switched = { .mask = cycle.mask & sys.homing.mask },
                   manual   = { .mask = cycle.mask & ~sys.homing.mask };

    if(switched.mask)
        limits_set_machine_positions(switched, true);

    if(manual.mask)
        limits_set_machine_positions(manual, false);
}

// With KINEMATICS_API, mc_line() soft limit checks with is_cartesian = false, which core's
// default check always fails. The point checked is already real machine coordinates
// (passthru when off, the resolved real end point when on), so check it as cartesian.
static bool polarCheckTravelLimits (float *target, axes_signals_t axes, bool is_cartesian, work_envelope_t *envelope)
{
    return check_travel_limits(target, axes, true, envelope);
}

static bool polarHomingCycleValidate (axes_signals_t cycle)
{
    return true;
}

static float polarHomingCycleGetFeedrate (axes_signals_t cycle, float feedrate, homing_mode_t mode)
{
    return feedrate;
}

// Swaps kinematics pointers only - safe from interrupt context (used by onReset).
static void polar_set_mode (bool enable)
{
    if(enable) {
        kinematics.transform_steps_to_cartesian = polarStepsToCartesianOn;
        kinematics.segment_line = polarSegmentLineOn;
    } else {
        kinematics.transform_steps_to_cartesian = polarStepsToCartesianOff;
        kinematics.segment_line = polarSegmentLineOff;
    }

    polar_enabled = enable;
}

// Polar mode relies on absolute Y (centreline) and Z (radius), so these axes must be
// homed to turn it on. A is not required (rotary without a home switch).
#define POLAR_HOMED_AXES (Y_AXIS_BIT|Z_AXIS_BIT)

static inline bool polar_axes_homed (void)
{
    return (sys.homed.mask & POLAR_HOMED_AXES) == POLAR_HOMED_AXES;
}

// Moves derive their start from the planner position; only the working side is latched here.
static void polar_enable (bool enable)
{
    if(enable) {
        float dz = (float)sys.position[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm
                    - polar_pole(Z_AXIS, false);

        polar_side = dz > 0.0f ? -1.0f : 1.0f;

        if(fabsf(dz) < POLE_EPSILON)
            report_message("Polar: Z is on the centreline, assuming tool below (-Z side)", Message_Warning);
    }

    polar_set_mode(enable);
    sync_position();    // gc_state.position <- virtual (on) or real (off) coordinates
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
            gc_block->user_mcode_sync = true;   // drain planner before mode change takes effect

            if(state_get() != STATE_CHECK_MODE && !polar_axes_homed()) {
                report_message("Polar: Y and Z must be homed for M112", Message_Warning);
                state = Status_HomingRequired;
                break;
            }

            // The transform assumes the tool is in the plane through the A axis, i.e. real Y
            // is on the pole. Reject M112 otherwise - Y is never moved by polar moves, so an
            // offset would distort every cut while the DRO shows the programmed path.
            // Drain first so sys.position is where Y will actually be when M112 executes.
            if(state_get() != STATE_CHECK_MODE && protocol_buffer_synchronize()) {
                float spm = settings.axis[Y_AXIS].steps_per_mm;
                float tolerance = max(POLAR_Y_TOLERANCE, 1.0f / spm);
                float y_error = (float)sys.position[Y_AXIS] / spm - polar_pole(Y_AXIS, false);

                if(fabsf(y_error) > tolerance) {
                    report_message("Polar: Y must be on the A axis centreline (work Y0) for M112", Message_Warning);
                    state = Status_GcodeInvalidTarget;
                }
            }
            break;

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

// Called from interrupt context.
static void onJogCancel (sys_state_t state)
{
    jog_cancel = true;

    if(on_jog_cancel)
        on_jog_cancel(state);
}

static void onProgramCompleted (program_flow_t program_flow, bool check_mode)
{
    if(polar_enabled) {
        polar_enable(false);
        report_message("Polar interpolation OFF", Message_Info);
    }

    if(on_program_completed)
        on_program_completed(program_flow, check_mode);
}

// A homing cycle ($H) clears and re-establishes homed status without a reset, and moves
// Y off the centreline - turn polar mode off after any homing cycle (success or not).
// Loss of homed status through reset is handled by onReset.
static void onHomingCompleted (axes_signals_t cycle, bool success)
{
    if(polar_enabled) {
        polar_enable(false);
        report_message("Polar interpolation OFF", Message_Info);
    }

    if(on_homing_completed)
        on_homing_completed(cycle, success);
}

// Called from interrupt context - only swap pointers, core resyncs position after reset.
static void onReset (void)
{
    if(polar_enabled)
        polar_set_mode(false);

    if(on_reset)
        on_reset();
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Polar Interpolation", "0.02");
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

    polar_set_mode(false);

    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = userMCodeCheck;
    grbl.user_mcode.validate = userMCodeValidate;
    grbl.user_mcode.execute = userMCodeExecute;

    on_jog_cancel = grbl.on_jog_cancel;
    grbl.on_jog_cancel = onJogCancel;

    on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = onProgramCompleted;

    on_reset = grbl.on_reset;
    grbl.on_reset = onReset;

    on_homing_completed = grbl.on_homing_completed;
    grbl.on_homing_completed = onHomingCompleted;

    check_travel_limits = grbl.check_travel_limits;
    grbl.check_travel_limits = polarCheckTravelLimits;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}

#endif // POLAR_INTERP_ENABLE
