#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Pickaxes error codes.
 *
 * Errors are reported as negatives of these codes. Functions return 0 on
 * success or a negative axes_error on failure.
 */
enum axes_error {
  /** A buffer is too small */
  AXES_ERR_SIZE = 1,

  /** An input holds a value the library does not accept */
  AXES_ERR_INVALID,

  /** A calibration session is missing a pose it needs */
  AXES_ERR_INCOMPLETE,
};

/*
 * The full-scale magnitude of a normalized output is chosen to be 2^12, giving
 * sticks a range of [-4096, 4096] per axis and triggers a range of [0, 4096].
 *
 * Most real world game controllers only use 8-bit outputs, but this library
 * supports up to 16-bit ADC input values. We want to avoid 64-bit math, so
 * 12-bits is chosen as a happy balance.
 *
 * Shaping values (deadzone widths, octagon corner positions) are denominated
 * in this scale, so changing it changes the meaning of stored configuration.
 */
#define AXES_FULL_SCALE_BITS 12
#define AXES_FULL_SCALE      (1 << AXES_FULL_SCALE_BITS)

/** Layout version of the packed records, bumped when any layout changes */
#define AXES_PACKED_VERSION 1

/** Packed record sizes, for sizing buffers and flash slots at compile time */
#define AXES_PACKED_TRIGGER_CALIBRATION_SIZE 4
#define AXES_PACKED_STICK_CALIBRATION_SIZE   13
#define AXES_PACKED_TRIGGER_SHAPING_SIZE     7
#define AXES_PACKED_STICK_SHAPING_SIZE       10

/**
 * What happens to input outside the deadzone.
 */
enum axes_deadzone_mode {
  /** Stretches the remaining travel to cover the full range, so output rises from zero */
  AXES_DEADZONE_MODE_SCALED,

  /** Passes input through unchanged (position-true), so output jumps as the input leaves the zone */
  AXES_DEADZONE_MODE_UNSCALED,

  /** Number of deadzone modes, not a mode itself */
  AXES_DEADZONE_MODE_COUNT,
};

/**
 * How the deadzone region is measured.
 */
enum axes_deadzone_shape {
  /** A circle applied to the stick's distance from center */
  AXES_DEADZONE_SHAPE_RADIAL,

  /** A separate band applied to each axis */
  AXES_DEADZONE_SHAPE_AXIAL,

  /** Number of deadzone shapes, not a shape itself */
  AXES_DEADZONE_SHAPE_COUNT,
};

/**
 * A response curve exponent of exactly 1.0 (linear), in Q8.8 fixed point.
 * A gamma of 1.5 is `1.5 * AXES_GAMMA_LINEAR`. Zero is treated as linear, so
 * a zeroed shaping struct stays passthrough.
 */
#define AXES_GAMMA_LINEAR 256

/**
 * Stick gate, the target shape the stick's range is mapped to.
 *
 * All but NONE are *virtual* gates, simulating a shell opening the stick does
 * not physically have. The stick's full range of travel is taken to be a
 * circle, and how the input meets the boundary is a separate choice
 * (see @ref axes_gate_mode).
 */
enum axes_gate_shape {
  /**
   * No reshaping. Each axis is clamped to full scale independently, so the
   * edge is whatever the hardware produces
   */
  AXES_GATE_SHAPE_NONE,

  /** Full scale at every angle, a plain magnitude clamp onto a round edge */
  AXES_GATE_SHAPE_CIRCLE,

  /**
   * Full scale at the cardinals, with the corners at the shaping's corner
   * position (see AXES_OCTAGON_*). Covers everything from a square down to a
   * diamond, and below that a concave four-pointed star
   */
  AXES_GATE_SHAPE_OCTAGON,

  /** Number of gate shapes, not a shape itself */
  AXES_GATE_SHAPE_COUNT,
};

/** Corners at full scale on both axes, a square */
#define AXES_OCTAGON_SQUARE   AXES_FULL_SCALE

/** Corners on the full-scale circle, AXES_FULL_SCALE / sqrt(2) */
#define AXES_OCTAGON_REGULAR  2896

/** Corners at 70/85 of full scale, matching an OEM N64 gate */
#define AXES_OCTAGON_N64      (AXES_FULL_SCALE * 70 / 85)

/**
 * How a gate is reconciled with the stick's range of travel.
 */
enum axes_gate_mode {
  /**
   * Treats the virtual gate as a physical restriction, so output tracks position
   * exactly until the stick meets the boundary, then holds there. A gate too wide
   * for the travel is shrunk to fit first, keeping its shape but costing cardinal
   * range.
   */
  AXES_GATE_MODE_CLAMP,

  /**
   * Rescales the whole travel onto the gate instead, so the cardinals always reach
   * full scale. The only mode that reaches a wide gate at its stated size
   */
  AXES_GATE_MODE_SCALE,

  /** Number of gate modes, not a mode itself */
  AXES_GATE_MODE_COUNT,
};

/** Number of points in a derived response curve lookup table */
#define AXES_CURVE_POINTS 33

/**
 * Number of directions a stick sweep tracks coverage over, each 22.5 degrees
 * wide. Fixed, since the sector math splits each octant in two.
 */
#define AXES_SWEEP_SECTORS 16

/**
 * Readings a sweep needs in every direction before it reports complete.
 *
 * A rushed sweep lands only a couple of readings near each cardinal and misses
 * the furthest point, so a sweep is judged on how much it saw rather than on
 * how far the stick travelled. A user sweeping quickly is asked for more
 * rotations, which costs the same in time as one slow one.
 */
#define AXES_SWEEP_MIN_SAMPLES 8

/**
 * Complete rotations a sweep needs before it reports complete.
 *
 * One is not enough, since the readings taken while the stick is pushed out
 * from rest can leave the direction it started in measured only by that push.
 */
#define AXES_SWEEP_MIN_ROTATIONS 3

/**
 * Named trigger positions, for prompting through a calibration routine.
 *
 * Calibration sessions take readings tagged with these, so an application and
 * its host protocol can share one vocabulary for the readings that fill in
 * @ref axes_trigger_calibration.
 */
enum axes_trigger_pose {
  /** Trigger at rest, giving the `rest` reading */
  AXES_TRIGGER_POSE_RELEASED,

  /** Trigger held at its stop, giving the `pressed` reading */
  AXES_TRIGGER_POSE_PRESSED,

  /** Number of trigger poses, not a pose itself */
  AXES_TRIGGER_POSE_COUNT,
};

/**
 * Named stick positions, for prompting through a calibration routine.
 *
 * Directions are logical rather than physical, so UP is a full deflection
 * toward logical +Y however the stick is mounted. The numbering matches the HID
 * hat switch, running clockwise from zero at up with the centered null state
 * last.
 *
 * Calibration sessions take readings tagged with these, so an application and
 * its host protocol can share one vocabulary for the readings that fill in
 * @ref axes_stick_calibration.
 */
enum axes_stick_pose {
  /** Full deflection toward logical +Y */
  AXES_STICK_POSE_UP,

  /** Full deflection up and right, resting in the gate's corner if there is one */
  AXES_STICK_POSE_UP_RIGHT,

  /** Full deflection toward logical +X */
  AXES_STICK_POSE_RIGHT,

  /** Full deflection down and right, resting in the gate's corner if there is one */
  AXES_STICK_POSE_DOWN_RIGHT,

  /** Full deflection toward logical -Y */
  AXES_STICK_POSE_DOWN,

  /** Full deflection down and left, resting in the gate's corner if there is one */
  AXES_STICK_POSE_DOWN_LEFT,

  /** Full deflection toward logical -X */
  AXES_STICK_POSE_LEFT,

  /** Full deflection up and left, resting in the gate's corner if there is one */
  AXES_STICK_POSE_UP_LEFT,

  /** Stick released, giving the `rest_x` and `rest_y` readings */
  AXES_STICK_POSE_CENTERED,

  /** Number of stick poses, not a pose itself */
  AXES_STICK_POSE_COUNT,
};

/**
 * Calibration data for a trigger, or any single-axis input
 */
struct axes_trigger_calibration {
  /** Raw ADC reading at the resting (released) position */
  uint16_t rest;

  /** Raw ADC reading at the fully-pressed position */
  uint16_t pressed;
};

/**
 * Calibration data for a stick, or any two-axis input.
 *
 * @see axes_stick_calibration_orient for deriving invert_x, invert_y and swap_xy.
 */
struct axes_stick_calibration {
  /** Raw ADC readings at the resting (centered) position */
  uint16_t rest_x, rest_y;

  /** Minimum observed ADC readings across the full range of motion */
  uint16_t min_x, min_y;

  /** Maximum observed ADC readings across the full range of motion */
  uint16_t max_x, max_y;

  /** X axis is inverted (logical X opposite to physical source) */
  bool invert_x;

  /** Y axis is inverted (logical Y opposite to physical source) */
  bool invert_y;

  /** Stick mounted at 90-degrees, physical X feeds logical Y and vice versa */
  bool swap_xy;
};

/**
 * Working state for a trigger calibration session.
 *
 * Accumulates readings while an application walks the user through the
 * trigger poses, then resolves them into an @ref axes_trigger_calibration.
 * Populate using axes_trigger_calibration_session_begin(). The fields are
 * working state, so treat them as opaque.
 */
struct axes_trigger_calibration_session {
  // Smallest travel a usable trigger must show, from the ADC range handed to begin
  uint16_t min_travel;

  // Released readings are averaged, with the count saturating rather than wrapping
  uint32_t rest_sum;
  uint16_t rest_count;

  // The pressed reading farthest from rest so far
  bool pressed_seen;
  uint16_t pressed;
};

/**
 * Working state for a stick calibration session.
 *
 * Accumulates readings while an application walks the user through the stick
 * poses, and optionally a sweep around the rim, then resolves them into an
 * @ref axes_stick_calibration. Populate using
 * axes_stick_calibration_session_begin(). The fields are working state, so
 * treat them as opaque.
 */
struct axes_stick_calibration_session {
  // Smallest deflection that counts as leaving rest, from the ADC range handed to begin
  uint16_t min_reach;

  // Centered readings are averaged, with the count saturating rather than wrapping
  uint32_t rest_sum_x, rest_sum_y;
  uint16_t rest_count;

  // Extents, widened by every deflected pose and every sweep sample
  uint16_t min_x, min_y, max_x, max_y;

  // The UP and RIGHT readings farthest from rest so far, which the orientation comes from
  bool up_seen, right_seen;
  uint16_t up_x, up_y, right_x, right_y;

  // Sweep state, with rest snapshotted at sweep_begin so samples need no divide
  bool sweep_active;
  uint16_t sweep_rest_x, sweep_rest_y;
  uint16_t sweep_reach[AXES_SWEEP_SECTORS];

  // Readings taken in each direction, saturating, so a rushed sweep is asked for more
  uint8_t sweep_samples[AXES_SWEEP_SECTORS];

  // Directions reached since the current rotation began, and rotations closed so far
  uint16_t sweep_rotation_mask;
  uint8_t sweep_rotations;
};

/**
 * Output shaping for a trigger, or any single-axis input.
 */
struct axes_trigger_shaping {
  /** How far from rest the trigger still reads as released, [0, AXES_FULL_SCALE) */
  uint16_t deadzone_inner;

  /** How far in from full press the trigger reads as fully pressed, [0, AXES_FULL_SCALE) */
  uint16_t deadzone_outer;

  /** What happens to input outside the zones (see @ref axes_deadzone_mode) */
  enum axes_deadzone_mode deadzone_mode;

  /** Response curve exponent in Q8.8, applied to the press after deadzones, zero or AXES_GAMMA_LINEAR is linear */
  uint16_t response_gamma;
};

/**
 * Output shaping for a stick, or any two-axis input.
 */
struct axes_stick_shaping {
  /** How far from rest input still reads as zero, [0, AXES_FULL_SCALE) */
  uint16_t deadzone_inner;

  /** How far in from full deflection input reads as fully deflected, [0, AXES_FULL_SCALE) */
  uint16_t deadzone_outer;

  /** How the deadzone region is measured (see @ref axes_deadzone_shape) */
  enum axes_deadzone_shape deadzone_shape;

  /** What happens to input outside the zones (see @ref axes_deadzone_mode) */
  enum axes_deadzone_mode deadzone_mode;

  /** Response curve exponent in Q8.8, applied after deadzones, zero or AXES_GAMMA_LINEAR is linear */
  uint16_t response_gamma;

  /** Gate shape (see @ref axes_gate_shape) */
  enum axes_gate_shape gate_shape;

  /** How the gate is reconciled with the travel (see @ref axes_gate_mode) */
  enum axes_gate_mode gate_mode;

  /** Corner position for octagon gates, zero means AXES_OCTAGON_REGULAR (see AXES_OCTAGON_*) */
  uint16_t gate_corner;
};

/**
 * A single-axis mapping, shared by both transforms.
 *
 * Clamps a raw reading into its calibrated travel, then scales the reading's
 * distance from `zero` onto the normalized output range.
 */
struct axes_axis {
  // The raw reading that maps to zero output
  uint16_t zero;

  // Calibrated travel, which also keeps the multiply inside 32 bits
  uint16_t clamp_lo, clamp_hi;

  // Scale factor, negative when the axis is inverted
  int32_t scale;
};

/**
 * Runtime transform for a trigger, or any single-axis input.
 *
 * Populate using axes_trigger_derive() at boot and whenever calibration or
 * shaping changes, then pass to axes_trigger_apply(). The fields are derived
 * state, so treat them as opaque.
 */
struct axes_trigger_transform {
  // Raw reading mapping, clamped to the travel left between the deadzones
  struct axes_axis axis;

  // Output at or below this reads released
  int16_t snap_zero;

  // Output at or beyond this reads fully pressed
  int16_t snap_full;

  // Skip the curve entirely, for a linear gamma
  bool curve_linear;

  // Response curve, baked to a lookup table over [0, AXES_FULL_SCALE]
  uint16_t curve[AXES_CURVE_POINTS];
};

/**
 * Runtime transform for a stick, or any two-axis input.
 *
 * Populate using axes_stick_derive() at boot and whenever calibration
 * or shaping changes, then pass to axes_stick_apply(). The fields are derived
 * state, so treat them as opaque.
 */
struct axes_stick_transform {
  // Raw reading mappings, [0] for logical X and [1] for logical Y
  struct axes_axis axis[2];

  // Stick mounted at 90 degrees, so physical X feeds logical Y and vice versa
  bool swap_xy;

  // How the deadzone region is measured
  enum axes_deadzone_shape deadzone_shape;

  // What happens to input outside the deadzone
  enum axes_deadzone_mode deadzone_mode;

  // Inner deadzone width
  int32_t inner;

  // Inner width squared, so a resting stick skips the sqrt entirely
  int32_t inner_sq;

  // Output at or beyond this reads fully deflected, INT32_MAX when there is no outer deadzone
  int32_t snap_full;

  // Stretches the usable travel onto full scale
  int32_t usable_scale;

  // Skip the curve entirely, for a linear gamma
  bool curve_linear;

  // Response curve, baked to a lookup table over [0, AXES_FULL_SCALE]
  uint16_t curve[AXES_CURVE_POINTS];

  // Gate shape baked to one number, the tilt of the first-octant boundary edge
  int32_t gate_k;

  // Gate shape, which decides how the boundary is measured
  enum axes_gate_shape gate_shape;

  // How the gate is reconciled with the travel
  enum axes_gate_mode gate_mode;

  // Cardinal reach of the gate, below full scale when clamping had to shrink it
  int32_t gate_limit;
};

/**
 * Fill in a neutral trigger calibration covering the whole ADC range.
 *
 * Maps a reading of zero to released and `adc_max` to fully pressed, which is
 * a reasonable starting point for a trigger that has not been calibrated yet.
 *
 * @param calibration destination for the calibration
 * @param adc_max largest reading the ADC produces, 4095 for a 12-bit ADC
 */
void axes_trigger_calibration_default(struct axes_trigger_calibration *calibration, uint16_t adc_max);

/**
 * Derive a trigger transform from calibration and shaping settings.
 *
 * @param transform destination for the derived transform
 * @param calibration calibration data
 * @param shaping output shaping settings (may be NULL for defaults)
 */
void axes_trigger_derive(struct axes_trigger_transform *transform, const struct axes_trigger_calibration *calibration,
                         const struct axes_trigger_shaping *shaping);

/**
 * Apply a trigger transform to a raw reading.
 *
 * @param transform the runtime transform
 * @param raw raw ADC reading
 * @return normalized output in [0, AXES_FULL_SCALE]
 */
int16_t axes_trigger_apply(const struct axes_trigger_transform *transform, uint16_t raw);

/**
 * Start a trigger calibration session.
 *
 * Clears any readings from a previous session. The ADC range sets how much
 * travel a trigger must show before its calibration is accepted.
 *
 * @param session session to start
 * @param adc_max largest reading the ADC produces, 4095 for a 12-bit ADC
 */
void axes_trigger_calibration_session_begin(struct axes_trigger_calibration_session *session, uint16_t adc_max);

/**
 * Record a raw reading taken while the trigger is held in a pose.
 *
 * Poses may repeat, and capturing the same pose many times is encouraged. Rest
 * is a zero point, so the released pose is averaged over every reading, while
 * fully pressed is an extent, so it keeps whichever reading sits farthest from
 * rest. Released must be captured before pressed, since rest decides which way
 * the trigger reads.
 *
 * @param session session in progress
 * @param pose the pose the trigger is held in (see @ref axes_trigger_pose)
 * @param raw raw ADC reading
 * @return 0 on success, negative axes_error otherwise
 */
int axes_trigger_calibration_session_capture(struct axes_trigger_calibration_session *session,
                                             enum axes_trigger_pose pose, uint16_t raw);

/**
 * Resolve a trigger calibration session into calibration data.
 *
 * The session is left untouched, so an application can preview the result and
 * keep capturing. Fails with AXES_ERR_INCOMPLETE if a pose is missing, and
 * AXES_ERR_INVALID if the readings show too little travel to be usable.
 *
 * @param session session to resolve
 * @param calibration destination for the calibration, untouched on failure
 * @return 0 on success, negative axes_error otherwise
 */
int axes_trigger_calibration_session_end(const struct axes_trigger_calibration_session *session,
                                         struct axes_trigger_calibration *calibration);

/**
 * Fill in a neutral stick calibration covering the whole ADC range.
 *
 * Rests at the midpoint of the ADC range with travel reaching both ends on
 * each axis, and no orientation fixups, which is a reasonable starting point
 * for a stick that has not been calibrated yet.
 *
 * @param calibration destination for the calibration
 * @param adc_max largest reading the ADC produces, 4095 for a 12-bit ADC
 */
void axes_stick_calibration_default(struct axes_stick_calibration *calibration, uint16_t adc_max);

/**
 * Helper function to derive stick orientation from raw readings, updating the calibration
 *
 * @param calibration data to update (rest_x/rest_y must already be set)
 * @param up_x,up_y raw reading while pushed toward logical UP (see @ref axes_stick_pose)
 * @param right_x,right_y raw reading while pushed toward logical RIGHT
 */
void axes_stick_calibration_orient(struct axes_stick_calibration *calibration, uint16_t up_x, uint16_t up_y,
                                   uint16_t right_x, uint16_t right_y);

/**
 * Derive a stick transform from calibration and shaping settings.
 *
 * @param transform destination for the derived transform
 * @param calibration center/orientation/extent calibration record
 * @param shaping output shaping settings (may be NULL for defaults)
 */
void axes_stick_derive(struct axes_stick_transform *transform, const struct axes_stick_calibration *calibration,
                       const struct axes_stick_shaping *shaping);

/**
 * Apply a stick transform to a raw reading.
 *
 * @param transform the runtime transform
 * @param raw_x raw ADC reading for physical X
 * @param raw_y raw ADC reading for physical Y
 * @param out_x destination for normalized X in [-AXES_FULL_SCALE, AXES_FULL_SCALE]
 * @param out_y destination for normalized Y in [-AXES_FULL_SCALE, AXES_FULL_SCALE]
 */
void axes_stick_apply(const struct axes_stick_transform *transform, uint16_t raw_x, uint16_t raw_y, int16_t *out_x,
                      int16_t *out_y);

/**
 * Start a stick calibration session.
 *
 * Clears any readings from a previous session, including a sweep. The ADC
 * range sets how far the stick must deflect before a reading counts as leaving
 * rest, both for sweep coverage and for accepting the calibration at the end.
 *
 * @param session session to start
 * @param adc_max largest reading the ADC produces, 4095 for a 12-bit ADC
 */
void axes_stick_calibration_session_begin(struct axes_stick_calibration_session *session, uint16_t adc_max);

/**
 * Record a raw reading taken while the stick is held in a pose.
 *
 * Poses may repeat, and capturing the same pose many times is encouraged. Rest
 * is a zero point, so the centered pose is averaged over every reading, while
 * every other pose is an extent, so it only ever widens the range of travel.
 * The UP and RIGHT poses also keep whichever reading sits farthest from rest,
 * which the orientation is worked out from. Centered must be captured before
 * any other pose, since deflections are judged against rest. After that poses
 * may arrive in any order.
 *
 * @param session session in progress
 * @param pose the pose the stick is held in (see @ref axes_stick_pose)
 * @param raw_x raw ADC reading for physical X
 * @param raw_y raw ADC reading for physical Y
 * @return 0 on success, negative axes_error otherwise
 */
int axes_stick_calibration_session_capture(struct axes_stick_calibration_session *session, enum axes_stick_pose pose,
                                           uint16_t raw_x, uint16_t raw_y);

/**
 * Start sweeping the stick around its rim.
 *
 * A sweep widens the range of travel from a stream of readings rather than
 * from individual poses, and tracks which directions the stick has reached so
 * the application can tell when it has been all the way round. Starting a
 * sweep clears coverage from any previous sweep but keeps the range. Centered
 * must have been captured first.
 *
 * @param session session in progress
 * @return 0 on success, negative axes_error otherwise
 */
int axes_stick_calibration_session_sweep_begin(struct axes_stick_calibration_session *session);

/**
 * Feed a raw reading into a sweep in progress.
 *
 * Call on every poll while the user rotates the stick around its rim. Readings
 * are ignored when no sweep is in progress.
 *
 * @param session session in progress
 * @param raw_x raw ADC reading for physical X
 * @param raw_y raw ADC reading for physical Y
 */
void axes_stick_calibration_session_sweep_sample(struct axes_stick_calibration_session *session, uint16_t raw_x,
                                                 uint16_t raw_y);

/**
 * Whether a sweep has reached every direction around the stick.
 *
 * Directions are physical, since orientation is not known until the session
 * ends. Each direction has to be reached at close to the range seen on that
 * side of rest, so a small circuit near rest does not count, and a sweep that
 * was one can read incomplete again once the stick reaches the real rim. Each
 * direction also has to have been sampled AXES_SWEEP_MIN_SAMPLES times, so a
 * stick swept too quickly to be measured properly asks for more rotations, and
 * AXES_SWEEP_MIN_ROTATIONS circles have to have been closed.
 *
 * @param session session in progress
 * @return true once every one of the AXES_SWEEP_SECTORS directions has been reached
 */
bool axes_stick_calibration_session_sweep_complete(const struct axes_stick_calibration_session *session);

/**
 * Resolve a stick calibration session into calibration data.
 *
 * Fills in the resting position and range of travel, and derives the
 * orientation flags from the UP and RIGHT poses. The session is left
 * untouched, so an application can preview the result and keep capturing.
 * Fails with AXES_ERR_INCOMPLETE if the centered, UP or RIGHT poses are
 * missing, and AXES_ERR_INVALID if some side of rest has too little travel to
 * be usable or the UP and RIGHT readings sit too close to rest to orient from.
 *
 * @param session session to resolve
 * @param calibration destination for the calibration, untouched on failure
 * @return 0 on success, negative axes_error otherwise
 */
int axes_stick_calibration_session_end(const struct axes_stick_calibration_session *session,
                                       struct axes_stick_calibration *calibration);

/**
 * Pack a trigger calibration into its portable byte layout.
 *
 * @param dest destination buffer
 * @param size bytes available in the destination buffer
 * @param src calibration data to pack
 * @return 0 on success, negative axes_error otherwise
 */
int axes_trigger_calibration_pack(uint8_t *dest, size_t size, const struct axes_trigger_calibration *src);

/**
 * Unpack a trigger calibration written by axes_trigger_calibration_pack().
 *
 * @param dest destination for the unpacked calibration, untouched on failure
 * @param src source bytes
 * @param size bytes available in the source buffer
 * @return 0 on success, negative axes_error otherwise
 */
int axes_trigger_calibration_unpack(struct axes_trigger_calibration *dest, const uint8_t *src, size_t size);

/**
 * Pack a stick calibration into its portable byte layout.
 *
 * @param dest destination buffer
 * @param size bytes available in the destination buffer
 * @param src calibration data to pack
 * @return 0 on success, negative axes_error otherwise
 */
int axes_stick_calibration_pack(uint8_t *dest, size_t size, const struct axes_stick_calibration *src);

/**
 * Unpack a stick calibration written by axes_stick_calibration_pack().
 *
 * @param dest destination for the unpacked calibration, untouched on failure
 * @param src source bytes
 * @param size bytes available in the source buffer
 * @return 0 on success, negative axes_error otherwise
 */
int axes_stick_calibration_unpack(struct axes_stick_calibration *dest, const uint8_t *src, size_t size);

/**
 * Pack trigger shaping settings into their portable byte layout.
 *
 * @param dest destination buffer
 * @param size bytes available in the destination buffer
 * @param src shaping settings to pack
 * @return 0 on success, negative axes_error otherwise
 */
int axes_trigger_shaping_pack(uint8_t *dest, size_t size, const struct axes_trigger_shaping *src);

/**
 * Unpack trigger shaping settings written by axes_trigger_shaping_pack().
 *
 * Deadzone widths and gammas are passed through as stored, since
 * axes_trigger_derive() already clamps them.
 *
 * @param dest destination for the unpacked settings, untouched on failure
 * @param src source bytes
 * @param size bytes available in the source buffer
 * @return 0 on success, negative axes_error otherwise
 */
int axes_trigger_shaping_unpack(struct axes_trigger_shaping *dest, const uint8_t *src, size_t size);

/**
 * Pack stick shaping settings into their portable byte layout.
 *
 * @param dest destination buffer
 * @param size bytes available in the destination buffer
 * @param src shaping settings to pack
 * @return 0 on success, negative axes_error otherwise
 */
int axes_stick_shaping_pack(uint8_t *dest, size_t size, const struct axes_stick_shaping *src);

/**
 * Unpack stick shaping settings written by axes_stick_shaping_pack().
 *
 * Deadzone widths, gammas and gate corners are passed through as stored, since
 * axes_stick_derive() already clamps them.
 *
 * @param dest destination for the unpacked settings, untouched on failure
 * @param src source bytes
 * @param size bytes available in the source buffer
 * @return 0 on success, negative axes_error otherwise
 */
int axes_stick_shaping_unpack(struct axes_stick_shaping *dest, const uint8_t *src, size_t size);
