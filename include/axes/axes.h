#pragma once

#include <stdbool.h>
#include <stdint.h>

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

/**
 * What happens to input outside the deadzone.
 */
enum axes_deadzone_mode {
  /** Stretches the remaining travel to cover the full range, so output rises from zero */
  AXES_DEADZONE_SCALED,

  /** Passes input through unchanged (position-true), so output jumps as the input leaves the zone */
  AXES_DEADZONE_UNSCALED,
};

/**
 * How the deadzone region is measured.
 */
enum axes_deadzone_shape {
  /** A circle applied to the stick's distance from center */
  AXES_DEADZONE_RADIAL,

  /** A separate band applied to each axis */
  AXES_DEADZONE_AXIAL,
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
  AXES_GATE_NONE,

  /** Full scale at every angle, a plain magnitude clamp onto a round edge */
  AXES_GATE_CIRCLE,

  /**
   * Full scale at the cardinals, with the corners at the shaping's corner
   * position (see AXES_OCTAGON_*). Covers everything from a square down to a
   * diamond, and below that a concave four-pointed star
   */
  AXES_GATE_OCTAGON,
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
  AXES_GATE_CLAMP,

  /**
   * Rescales the whole travel onto the gate instead, so the cardinals always reach
   * full scale. The only mode that reaches a wide gate at its stated size
   */
  AXES_GATE_SCALE,
};

/** Number of points in a derived response curve lookup table */
#define AXES_CURVE_POINTS 33

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
 * Helper function to derive stick orientation from raw readings, updating the calibration
 *
 * @param calibration data to update (rest_x/rest_y must already be set)
 * @param up_x,up_y raw reading while pushed toward logical UP
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
