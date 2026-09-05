#include <axes/axes.h>

// Derived scales carry this many fractional bits, so apply multiplies by a scale
// then shifts back down. 15 is the most that keeps the products inside int32.
#define AXES_SCALE_SHIFT 15

// The curve LUT lays AXES_CURVE_POINTS samples evenly over [0, AXES_FULL_SCALE].
// A magnitude's top AXES_CURVE_SEG_BITS bits pick the segment, the rest interpolate.
#define AXES_CURVE_SEG_BITS  5
#define AXES_CURVE_SEGMENTS  (1 << AXES_CURVE_SEG_BITS) // == AXES_CURVE_POINTS - 1
#define AXES_CURVE_STEP_BITS (AXES_FULL_SCALE_BITS - AXES_CURVE_SEG_BITS)
#define AXES_CURVE_STEP      (1 << AXES_CURVE_STEP_BITS)

// sqrt(2) in Q15, for measuring how far an octagon's corners reach
#define AXES_SQRT2_Q15 46341

// tan(22.5 degrees) in Q15, for splitting a sweep octant into two sectors
#define AXES_TAN_22_5_Q15 13573

// Least movement from rest a session accepts, as a shift of the ADC range, so a sixty-fourth of it.
// Only there to sit above ADC noise, since a stick may well use a small slice of the range
#define AXES_SESSION_MIN_TRAVEL_SHIFT 6

// Smallest octagon corner a gate is baked from, since a tighter one makes gate_k
// huge and the multiply in apply would leave int32 behind
#define AXES_GATE_CORNER_MIN (AXES_FULL_SCALE / 16)

// Absolute value of an int, kept local to avoid stdlib dependency
static inline int axes_iabs(int v)
{
  return v < 0 ? -v : v;
}

// Clamp an int to the given range
static inline int axes_iclamp(int x, int lo, int hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}

// Integer square root (floored), by Newton from an estimate not below the true root
static uint32_t axes_isqrt(uint32_t x, uint32_t estimate)
{
  if (x == 0)
    return 0;

  uint32_t root = estimate;
  uint32_t next = (root + x / root) >> 1;
  while (next < root) {
    root = next;
    next = (root + x / root) >> 1;
  }

  return root;
}

// x^(gamma/256) for x in Q15 and gamma in Q8.8
static uint32_t axes_pow_q15(uint32_t x, uint16_t gamma)
{
  if (x == 0)
    return 0;

  uint32_t result = 1u << 15;

  // Integer part of the exponent, multiplying in x (gamma >> 8) times
  for (int i = gamma >> 8; i > 0; i--)
    result = (result * x) >> 15;

  // Fractional part, where each set bit worth 1/2^k multiplies in x^(1/2^k), ending once none are left
  uint32_t root = x;
  for (uint32_t bits = gamma & 0xff; bits; bits = (bits << 1) & 0xff) {
    // The arithmetic mean of the two operands never falls below their geometric mean, so it seeds safely
    uint32_t product = root << 15;
    root             = axes_isqrt(product, (root + (1u << 15)) >> 1);
    if (bits & 0x80)
      result = (result * root) >> 15;
  }

  return result;
}

// Bake a gamma exponent into an evenly spaced curve LUT over [0, AXES_FULL_SCALE]
static void axes_curve_build(uint16_t lut[AXES_CURVE_POINTS], uint16_t gamma)
{
  for (int i = 0; i <= AXES_CURVE_SEGMENTS; i++) {
    // Sample position across the travel, in the Q15 that pow expects
    uint32_t position = ((uint32_t)i << 15) / AXES_CURVE_SEGMENTS;

    // Bend it, then rescale the Q15 result into normalized units
    uint32_t curved = axes_pow_q15(position, gamma);
    lut[i]          = (uint16_t)((curved << AXES_FULL_SCALE_BITS) >> 15);
  }
}

// Vector magnitude, reusing the square the caller already computed for its early-out
static inline int axes_magnitude(int x, int y, uint32_t squared)
{
  int ax = axes_iabs(x);
  int ay = axes_iabs(y);

  // Alpha-max-plus-beta-min with a half weight, which never lands below the true magnitude
  int major    = ax > ay ? ax : ay;
  int minor    = ax > ay ? ay : ax;
  int estimate = major + (minor >> 1);

  return (int)axes_isqrt(squared, (uint32_t)estimate);
}

// Map a raw reading onto the output range, shared by triggers and both stick axes
static inline int axes_axis_map(const struct axes_axis *axis, int raw)
{
  int clamped  = axes_iclamp(raw, axis->clamp_lo, axis->clamp_hi);
  int distance = clamped - axis->zero;
  return (int)(((int32_t)distance * axis->scale) >> AXES_SCALE_SHIFT);
}

// Evaluate a curve LUT with linear interpolation
static inline int axes_curve_eval(const uint16_t lut[AXES_CURVE_POINTS], int magnitude)
{
  // Full scale maps to itself, so GATE_NONE's corner overshoot survives a curve
  if (magnitude >= AXES_FULL_SCALE)
    return magnitude;

  int segment  = magnitude >> AXES_CURVE_STEP_BITS;
  int fraction = magnitude & (AXES_CURVE_STEP - 1);
  return lut[segment] + (((lut[segment + 1] - lut[segment]) * fraction) >> AXES_CURVE_STEP_BITS);
}

// Resolve one logical axis into a raw reading mapping, folding inversion into the scale sign
static void stick_derive_axis(struct axes_axis *out, int center, int min, int max, bool invert)
{
  // Reach only the smaller half, so travel stays symmetric about center
  int below  = center - min;
  int above  = max - center;
  int extent = below < above ? below : above;

  // A rest position outside the measured range leaves no reach at all
  if (extent < 0)
    extent = 0;

  out->zero     = (uint16_t)center;
  out->clamp_lo = (uint16_t)(center - extent);
  out->clamp_hi = (uint16_t)(center + extent);

  // A dead axis has no measurable travel, so it contributes nothing
  if (extent == 0) {
    out->scale = 0;
    return;
  }

  // Round up, so full deflection always reaches full scale
  int32_t scale = (((int32_t)AXES_FULL_SCALE << AXES_SCALE_SHIFT) + extent - 1) / extent;
  out->scale    = invert ? -scale : scale;
}

// Apply an axial deadzone band to one logical axis value
static inline int stick_deadzone_axis(const struct axes_stick_transform *t, int value)
{
  int deflection = axes_iabs(value);
  if (deflection <= t->inner)
    return 0;

  if (t->deadzone_mode == AXES_DEADZONE_MODE_SCALED) {
    // Rescale the usable travel onto full scale, clamped per axis
    int32_t scaled = ((int32_t)(deflection - t->inner) * t->usable_scale) >> AXES_SCALE_SHIFT;
    deflection     = scaled > AXES_FULL_SCALE ? AXES_FULL_SCALE : (int)scaled;
  } else if (deflection >= t->snap_full) {
    // Position-true, but the outer band still snaps to full deflection
    deflection = AXES_FULL_SCALE;
  }

  return value < 0 ? -deflection : deflection;
}

void axes_trigger_calibration_default(struct axes_trigger_calibration *c, uint16_t adc_max)
{
  // Released at zero, fully pressed at the top of the ADC range
  c->rest    = 0;
  c->pressed = adc_max;
}

void axes_trigger_derive(struct axes_trigger_transform *t, const struct axes_trigger_calibration *c,
                         const struct axes_trigger_shaping *s)
{
  // Missing shaping means no deadzones and a linear curve
  static const struct axes_trigger_shaping defaults = {0};
  if (!s)
    s = &defaults;

  bool scaled = s->deadzone_mode == AXES_DEADZONE_MODE_SCALED;

  // Travel of the axis, in raw ADC units
  int span = c->pressed - c->rest;

  // Protect against out of range deadzones
  int inner = axes_iclamp(s->deadzone_inner, 0, AXES_FULL_SCALE);
  int outer = axes_iclamp(s->deadzone_outer, 0, AXES_FULL_SCALE);

  // Overlapping zones leave no travel, so give it all to inner and read released
  // Firmware should validate that the zones don't overlap, this is a safety fallback
  if (inner + outer >= AXES_FULL_SCALE) {
    inner = AXES_FULL_SCALE;
    outer = 0;
  }

  // Move each endpoint inward by its deadzone
  int released = c->rest + span * inner / AXES_FULL_SCALE;
  int pressed  = c->pressed - span * outer / AXES_FULL_SCALE;

  // A trigger can read downward as it is pressed
  bool inverted = pressed < released;

  // Clamp readings into the usable travel between the zones
  t->axis.clamp_lo = (uint16_t)(inverted ? pressed : released);
  t->axis.clamp_hi = (uint16_t)(inverted ? released : pressed);

  // Scaled reads zero at the zone edge, unscaled keeps rest so mid-travel stays position true
  int zero = scaled ? released : c->rest;

  // Scaled stretches the usable travel, unscaled keeps the whole calibrated span
  int travel = axes_iabs(scaled ? pressed - released : span);

  // Round up, and let a zero-travel calibration read zero everywhere rather than divide
  int32_t scale = travel ? (((int32_t)AXES_FULL_SCALE << AXES_SCALE_SHIFT) + travel - 1) / travel : 0;

  t->axis.zero  = (uint16_t)zero;
  t->axis.scale = inverted ? -scale : scale;

  if (scaled) {
    // Scaled already spans the whole output range, so the ends are the bounds
    t->snap_zero = 0;
    t->snap_full = AXES_FULL_SCALE;
  } else {
    // Unscaled snaps at the zone edges, rounded exactly as apply will round them
    t->snap_zero = (int16_t)(((int32_t)(released - zero) * t->axis.scale) >> AXES_SCALE_SHIFT);
    t->snap_full = (int16_t)(((int32_t)(pressed - zero) * t->axis.scale) >> AXES_SCALE_SHIFT);
  }

  // Bake the response curve, skipped entirely when linear
  t->curve_linear = s->response_gamma == 0 || s->response_gamma == AXES_GAMMA_LINEAR;
  if (!t->curve_linear)
    axes_curve_build(t->curve, s->response_gamma);
}

int16_t axes_trigger_apply(const struct axes_trigger_transform *t, uint16_t raw)
{
  // Clamp into the usable travel and scale onto the output range
  int32_t position = axes_axis_map(&t->axis, raw);

  // Readings inside either deadzone snap to the ends
  if (position <= t->snap_zero)
    position = 0;
  else if (position >= t->snap_full)
    position = AXES_FULL_SCALE;

  // Apply the response curve baked at derive time
  if (!t->curve_linear)
    position = axes_curve_eval(t->curve, (int)position);

  return (int16_t)position;
}

// Least movement from rest a session accepts, never zero so noise alone cannot pass
static inline uint16_t session_min_travel(uint16_t adc_max)
{
  uint16_t travel = adc_max >> AXES_SESSION_MIN_TRAVEL_SHIFT;
  return travel ? travel : 1;
}

// Average a session's accumulated rest readings, rounded to nearest
static inline int session_rest_average(uint32_t sum, uint16_t count)
{
  return (int)((sum + count / 2) / count);
}

void axes_trigger_calibration_session_begin(struct axes_trigger_calibration_session *s, uint16_t adc_max)
{
  s->min_travel   = session_min_travel(adc_max);
  s->rest_sum     = 0;
  s->rest_count   = 0;
  s->pressed_seen = false;
  s->pressed      = 0;
}

int axes_trigger_calibration_session_capture(struct axes_trigger_calibration_session *s, enum axes_trigger_pose pose,
                                             uint16_t raw)
{
  if ((unsigned)pose >= AXES_TRIGGER_POSE_COUNT)
    return -AXES_ERR_INVALID;

  if (pose == AXES_TRIGGER_POSE_RELEASED) {
    // Rest is a zero point, so it is averaged, with the count held short of wrapping
    if (s->rest_count < UINT16_MAX) {
      s->rest_sum += raw;
      s->rest_count++;
    }
    return 0;
  }

  // Rest decides which way the trigger reads, so it has to come first
  if (s->rest_count == 0)
    return -AXES_ERR_INCOMPLETE;

  // Pressed is an extent, so the reading farthest from rest wins
  int rest = session_rest_average(s->rest_sum, s->rest_count);
  if (!s->pressed_seen || axes_iabs((int)raw - rest) > axes_iabs((int)s->pressed - rest)) {
    s->pressed      = raw;
    s->pressed_seen = true;
  }

  return 0;
}

int axes_trigger_calibration_session_end(const struct axes_trigger_calibration_session *s,
                                         struct axes_trigger_calibration *c)
{
  // Both poses are needed before there is anything to resolve
  if (s->rest_count == 0 || !s->pressed_seen)
    return -AXES_ERR_INCOMPLETE;

  // Rest is the average of every released reading
  int rest = session_rest_average(s->rest_sum, s->rest_count);

  // A trigger that barely moved would derive to a hair trigger, or nothing at all
  if (axes_iabs((int)s->pressed - rest) < s->min_travel)
    return -AXES_ERR_INVALID;

  // Pressed is the reading that landed farthest from rest
  c->rest    = (uint16_t)rest;
  c->pressed = s->pressed;

  return 0;
}

void axes_stick_calibration_default(struct axes_stick_calibration *c, uint16_t adc_max)
{
  // Rest at the middle of the ADC range, with travel reaching both ends
  c->rest_x = c->rest_y = (uint16_t)(adc_max / 2);
  c->min_x = c->min_y = 0;
  c->max_x = c->max_y = adc_max;

  // Assume the stick is mounted upright, with each axis reading the way round it should
  c->invert_x = false;
  c->invert_y = false;
  c->swap_xy  = false;
}

void axes_stick_calibration_orient(struct axes_stick_calibration *c, uint16_t up_x, uint16_t up_y, uint16_t right_x,
                                   uint16_t right_y)
{
  // Where right and up landed relative to rest, on each raw axis
  int rx = (int)right_x - (int)c->rest_x;
  int ry = (int)right_y - (int)c->rest_y;
  int ux = (int)up_x - (int)c->rest_x;
  int uy = (int)up_y - (int)c->rest_y;

  // A right push that moved raw Y more than raw X means the stick is mounted sideways
  c->swap_xy = axes_iabs(ry) > axes_iabs(rx);
  if (c->swap_xy) {
    // Sideways, so logical X reads from raw Y and logical Y from raw X, each inverted if it fell below rest
    c->invert_x = ry < 0;
    c->invert_y = ux < 0;
  } else {
    // Upright, so each logical axis reads from its own raw axis, inverted if it fell below rest
    c->invert_x = rx < 0;
    c->invert_y = uy < 0;
  }
}

void axes_stick_derive(struct axes_stick_transform *t, const struct axes_stick_calibration *c,
                       const struct axes_stick_shaping *s)
{
  // Missing shaping means no deadzones, a linear curve, and no gate
  static const struct axes_stick_shaping defaults = {0};
  if (!s)
    s = &defaults;

  // A 90-degree mount swaps which raw reading feeds each logical axis
  t->swap_xy = c->swap_xy;
  if (c->swap_xy) {
    stick_derive_axis(&t->axis[0], c->rest_y, c->min_y, c->max_y, c->invert_x);
    stick_derive_axis(&t->axis[1], c->rest_x, c->min_x, c->max_x, c->invert_y);
  } else {
    stick_derive_axis(&t->axis[0], c->rest_x, c->min_x, c->max_x, c->invert_x);
    stick_derive_axis(&t->axis[1], c->rest_y, c->min_y, c->max_y, c->invert_y);
  }

  // Shapes and modes carry through unchanged, since apply branches on them directly
  t->deadzone_shape = s->deadzone_shape;
  t->deadzone_mode  = s->deadzone_mode;
  t->gate_shape     = s->gate_shape;
  t->gate_mode      = s->gate_mode;

  // Protect against out of range deadzones, which would overflow the square below
  int inner = axes_iclamp(s->deadzone_inner, 0, AXES_FULL_SCALE);
  int outer = axes_iclamp(s->deadzone_outer, 0, AXES_FULL_SCALE);

  // The usable travel is whatever the two deadzones leave behind
  int usable = AXES_FULL_SCALE - inner - outer;

  // Floor it, so the rescale slope stays bounded and we never divide by zero
  if (usable < AXES_FULL_SCALE / 8)
    usable = AXES_FULL_SCALE / 8;

  // Inner width, and its square so apply's resting test needs no sqrt
  t->inner    = inner;
  t->inner_sq = inner * inner;

  // INT32_MAX when there is no outer deadzone, so nothing ever snaps
  t->snap_full = outer ? AXES_FULL_SCALE - outer : INT32_MAX;

  // Round up, so full deflection always reaches full scale
  t->usable_scale = (((int32_t)AXES_FULL_SCALE << AXES_SCALE_SHIFT) + usable - 1) / usable;

  // Bake the response curve, skipped entirely when linear
  t->curve_linear = s->response_gamma == 0 || s->response_gamma == AXES_GAMMA_LINEAR;
  if (!t->curve_linear)
    axes_curve_build(t->curve, s->response_gamma);

  // Only an octagon reads these, so they stay neutral for the other gate shapes
  t->gate_k     = 0;
  t->gate_limit = AXES_FULL_SCALE;

  // Bake an octagon down to its edge tilt and cardinal reach
  if (s->gate_shape == AXES_GATE_SHAPE_OCTAGON) {
    // A zero corner means the regular octagon
    int corner = s->gate_corner ? s->gate_corner : AXES_OCTAGON_REGULAR;

    // A tiny corner makes gate_k huge, so clamp it to keep apply's multiply in range
    corner = axes_iclamp(corner, AXES_GATE_CORNER_MIN, AXES_OCTAGON_SQUARE);

    // Determine the tilt of the edge running from the cardinal at full scale to the corner
    t->gate_k = ((int32_t)(AXES_FULL_SCALE - corner) << AXES_SCALE_SHIFT) / corner;

    // Adjust the gate limit down for octagons whose corners reach past full deflection
    if (s->gate_mode == AXES_GATE_MODE_CLAMP) {
      int32_t peak = ((int32_t)corner * AXES_SQRT2_Q15) >> AXES_SCALE_SHIFT;
      if (peak < AXES_FULL_SCALE)
        peak = AXES_FULL_SCALE;
      t->gate_limit = (int32_t)AXES_FULL_SCALE * AXES_FULL_SCALE / peak;
    }
  }
}

void axes_stick_apply(const struct axes_stick_transform *t, uint16_t raw_x, uint16_t raw_y, int16_t *out_x,
                      int16_t *out_y)
{
  bool axial  = t->deadzone_shape == AXES_DEADZONE_SHAPE_AXIAL;
  bool scaled = t->deadzone_mode == AXES_DEADZONE_MODE_SCALED;

  // Normalized logical vector, full scale per axis, swapping the raw readings for a 90-degree mount
  int vx = axes_axis_map(&t->axis[0], t->swap_xy ? raw_y : raw_x);
  int vy = axes_axis_map(&t->axis[1], t->swap_xy ? raw_x : raw_y);

  // Axial bands reshape each axis before any radial math
  if (axial) {
    vx = stick_deadzone_axis(t, vx);
    vy = stick_deadzone_axis(t, vy);
  }

  // Work in the squared domain first, so the test below needs no sqrt
  uint32_t magnitude_sq = (uint32_t)(vx * vx + vy * vy);

  // Axial applies its deadzone per axis, so only radial needs the inner circle
  uint32_t resting_sq = axial ? 0 : (uint32_t)t->inner_sq;

  // Inside the deadzone the answer is zero, so bail out before the sqrt
  if (magnitude_sq <= resting_sq) {
    *out_x = *out_y = 0;
    return;
  }

  int magnitude = axes_magnitude(vx, vy, magnitude_sq);

  // Full scale, in the same fixed-point units as the radius below
  const int32_t max_radius = (int32_t)AXES_FULL_SCALE << AXES_SCALE_SHIFT;

  // Target radius for this direction, kept in fixed point so the divide below keeps its fractional bits
  int32_t radius;
  if (axial || !scaled) {
    // Position-true, or already banded per axis, so the magnitude stands
    int shaped = magnitude;

    // A radial outer zone still snaps to full deflection
    if (!axial && magnitude >= t->snap_full)
      shaped = AXES_FULL_SCALE;

    radius = (int32_t)shaped << AXES_SCALE_SHIFT;
  } else {
    // Rescale the usable travel between the radial deadzones onto full scale
    radius = (magnitude - t->inner) * t->usable_scale;
  }

  // Every gate but NONE holds the radius at full scale
  if (t->gate_shape != AXES_GATE_SHAPE_NONE && radius > max_radius)
    radius = max_radius;

  // Apply the response curve baked at derive time
  if (!t->curve_linear) {
    // The table is indexed by whole magnitudes, so drop out of fixed point and back
    int curved = axes_curve_eval(t->curve, (int)(radius >> AXES_SCALE_SHIFT));
    radius     = (int32_t)curved << AXES_SCALE_SHIFT;
  }

  // How far the gate boundary reaches along this direction
  int gate_reach;
  if (t->gate_shape == AXES_GATE_SHAPE_OCTAGON) {
    // Fold the direction into the first octant, so one baked edge covers all eight
    int major = axes_iabs(vx), minor = axes_iabs(vy);
    if (minor > major) {
      int swap = major;
      major    = minor;
      minor    = swap;
    }

    // Reach to the edge, from how directly the direction points at it
    gate_reach = major + (int)(((int32_t)t->gate_k * minor) >> AXES_SCALE_SHIFT);
  } else {
    // Circle and none have no straight edge, so the boundary is the magnitude itself
    gate_reach = magnitude;
  }

  // Point the shaped radius back along (vx, vy), the magnitude cancelling out.
  // Round each step to nearest, so it stays monotonic along a ray.
  const int32_t half = 1 << (AXES_SCALE_SHIFT - 1);

  int32_t radius_scale;
  if (t->gate_mode == AXES_GATE_MODE_SCALE || t->gate_shape != AXES_GATE_SHAPE_OCTAGON) {
    // Stretch the travel onto the boundary, so the cardinals reach full scale.
    // Only an octagon can differ between the fits, since a circle already is the
    // travel boundary and NONE does no reshaping at all.
    radius_scale = (radius + gate_reach / 2) / gate_reach;
  } else if ((int32_t)(radius >> AXES_SCALE_SHIFT) * gate_reach <= t->gate_limit * magnitude) {
    // Still inside the gate, so the position stands
    radius_scale = (radius + magnitude / 2) / magnitude;
  } else {
    // Met the gate, so hold at its boundary
    radius_scale = ((t->gate_limit << AXES_SCALE_SHIFT) + gate_reach / 2) / gate_reach;
  }

  int shaped_x = (int)(((int32_t)vx * radius_scale + half) >> AXES_SCALE_SHIFT);
  int shaped_y = (int)(((int32_t)vy * radius_scale + half) >> AXES_SCALE_SHIFT);

  // Clamp per axis and write
  *out_x = (int16_t)axes_iclamp(shaped_x, -AXES_FULL_SCALE, AXES_FULL_SCALE);
  *out_y = (int16_t)axes_iclamp(shaped_y, -AXES_FULL_SCALE, AXES_FULL_SCALE);
}

// Widen a range to take in a value
static inline void axes_widen(uint16_t *lo, uint16_t *hi, uint16_t value)
{
  if (value < *lo)
    *lo = value;
  if (value > *hi)
    *hi = value;
}

// Widen a session's range of travel to take in a reading
static inline void stick_session_widen(struct axes_stick_calibration_session *s, uint16_t x, uint16_t y)
{
  axes_widen(&s->min_x, &s->max_x, x);
  axes_widen(&s->min_y, &s->max_y, y);
}

// How far a reading sits from the rest seen so far, along whichever axis moved more
static int stick_session_deflection(const struct axes_stick_calibration_session *s, uint16_t x, uint16_t y)
{
  int dx = axes_iabs((int)x - session_rest_average(s->rest_sum_x, s->rest_count));
  int dy = axes_iabs((int)y - session_rest_average(s->rest_sum_y, s->rest_count));
  return dx > dy ? dx : dy;
}

// Keep whichever of a pose's stored and new readings sits farther from rest
static void stick_session_keep_farthest(const struct axes_stick_calibration_session *s, bool *seen, uint16_t *kept_x,
                                        uint16_t *kept_y, uint16_t x, uint16_t y)
{
  if (*seen) {
    // Later readings only replace the kept one when they sit farther from rest
    int kept = stick_session_deflection(s, *kept_x, *kept_y);
    if (stick_session_deflection(s, x, y) <= kept)
      return;
  }

  *kept_x = x;
  *kept_y = y;
  *seen   = true;
}

// Which of the AXES_SWEEP_SECTORS directions a deflection from rest points in, counter-clockwise from physical +X
static int stick_session_sector(int dx, int dy)
{
  // Fold into the first octant, where the major axis is whichever moved more
  int ax = axes_iabs(dx), ay = axes_iabs(dy);
  int major = ax > ay ? ax : ay;
  int minor = ax > ay ? ay : ax;

  // Quadrants run counter-clockwise from +X, with zero counted as positive
  int quadrant = dx < 0 ? (dy < 0 ? 2 : 1) : (dy < 0 ? 3 : 0);

  // Octants likewise, with the half nearer +Y coming second in even quadrants and first in odd ones
  int octant = 2 * quadrant + ((ay > ax) ^ (quadrant & 1));

  // Split each octant at 22.5 degrees, the same flip keeping the sectors in angular order
  bool near_diagonal = ((uint32_t)minor << 15) > (uint32_t)major * AXES_TAN_22_5_Q15;
  return 2 * octant + (near_diagonal ^ (octant & 1));
}

void axes_stick_calibration_session_begin(struct axes_stick_calibration_session *s, uint16_t adc_max)
{
  // Noise floor, scaled to the ADC so it means the same on any resolution
  s->min_reach = session_min_travel(adc_max);

  // No rest yet, which also blocks every other pose until one arrives
  s->rest_sum_x = s->rest_sum_y = 0;
  s->rest_count                 = 0;

  // The range starts inside out, so the first reading sets both ends
  s->min_x = s->min_y = UINT16_MAX;
  s->max_x = s->max_y = 0;

  // No orientation poses yet, so end has nothing to orient from
  s->up_seen = s->right_seen = false;
  s->up_x = s->up_y = s->right_x = s->right_y = 0;

  // No sweep in progress, so samples are ignored until one begins
  s->sweep_active = false;
  s->sweep_rest_x = s->sweep_rest_y = 0;
  for (int i = 0; i < AXES_SWEEP_SECTORS; i++) {
    s->sweep_reach[i]   = 0;
    s->sweep_samples[i] = 0;
  }
  s->sweep_rotation_mask = 0;
  s->sweep_rotations     = 0;
}

int axes_stick_calibration_session_capture(struct axes_stick_calibration_session *s, enum axes_stick_pose pose,
                                           uint16_t raw_x, uint16_t raw_y)
{
  if ((unsigned)pose >= AXES_STICK_POSE_COUNT)
    return -AXES_ERR_INVALID;

  if (pose == AXES_STICK_POSE_CENTERED) {
    // Rest is a zero point, so it is averaged, with the count held short of wrapping
    if (s->rest_count < UINT16_MAX) {
      s->rest_sum_x += raw_x;
      s->rest_sum_y += raw_y;
      s->rest_count++;
    }
    return 0;
  }

  // Deflections are judged against rest, so it has to come first
  if (s->rest_count == 0)
    return -AXES_ERR_INCOMPLETE;

  // Every deflected pose is an extent, so it only ever widens the range
  stick_session_widen(s, raw_x, raw_y);

  // Orientation is worked out from up and right, so keep their farthest readings
  if (pose == AXES_STICK_POSE_UP) {
    stick_session_keep_farthest(s, &s->up_seen, &s->up_x, &s->up_y, raw_x, raw_y);
  } else if (pose == AXES_STICK_POSE_RIGHT) {
    stick_session_keep_farthest(s, &s->right_seen, &s->right_x, &s->right_y, raw_x, raw_y);
  }

  return 0;
}

int axes_stick_calibration_session_sweep_begin(struct axes_stick_calibration_session *s)
{
  // Directions are measured from rest, so it has to come first
  if (s->rest_count == 0)
    return -AXES_ERR_INCOMPLETE;

  // Snapshot rest, so samples need no divide and a later centered capture cannot shift the sectors
  s->sweep_rest_x = (uint16_t)session_rest_average(s->rest_sum_x, s->rest_count);
  s->sweep_rest_y = (uint16_t)session_rest_average(s->rest_sum_y, s->rest_count);

  // Fresh coverage, though the range carries over since a full restart is begin
  for (int i = 0; i < AXES_SWEEP_SECTORS; i++) {
    s->sweep_reach[i]   = 0;
    s->sweep_samples[i] = 0;
  }
  s->sweep_rotation_mask = 0;
  s->sweep_rotations     = 0;
  s->sweep_active        = true;

  return 0;
}

void axes_stick_calibration_session_sweep_sample(struct axes_stick_calibration_session *s, uint16_t raw_x,
                                                 uint16_t raw_y)
{
  // Samples outside a sweep have no rest to be measured from
  if (!s->sweep_active)
    return;

  // Every sample is an extent, so it only ever widens the range
  stick_session_widen(s, raw_x, raw_y);

  // Deflection from the rest snapshotted when the sweep began
  int dx = (int)raw_x - s->sweep_rest_x;
  int dy = (int)raw_y - s->sweep_rest_y;

  // Reach is measured along whichever axis moved more, matching how complete judges it
  int ax = axes_iabs(dx), ay = axes_iabs(dy);
  int reach = ax > ay ? ax : ay;

  // Readings that never left rest say nothing about where the rim is
  if (reach < s->min_reach)
    return;

  // Coverage only ever grows, so each sector keeps the furthest it has seen
  int sector = stick_session_sector(dx, dy);
  if (reach > s->sweep_reach[sector])
    s->sweep_reach[sector] = (uint16_t)reach;

  // Count how well each direction was measured, against the fixed noise floor so it cannot go stale
  if (s->sweep_samples[sector] < UINT8_MAX)
    s->sweep_samples[sector]++;

  // Rotations are counted by watching every direction come round again. Only this mask
  // resets, so no measurement is lost, and readings banked while the stick was pushed out
  // from rest can only ever count toward the first rotation rather than closing a later one
  s->sweep_rotation_mask |= (uint16_t)(1u << sector);
  if (s->sweep_rotation_mask == (uint16_t)((1u << AXES_SWEEP_SECTORS) - 1)) {
    s->sweep_rotation_mask = 0;
    if (s->sweep_rotations < UINT8_MAX)
      s->sweep_rotations++;
  }
}

bool axes_stick_calibration_session_sweep_complete(const struct axes_stick_calibration_session *s)
{
  if (!s->sweep_active)
    return false;

  // Enough circles closed, so no slice is left measured only by the push out from rest
  if (s->sweep_rotations < AXES_SWEEP_MIN_ROTATIONS)
    return false;

  // How far the range reaches from rest on each side, +X, +Y, -X, -Y
  int side[4] = {
    (int)s->max_x - s->sweep_rest_x,
    (int)s->max_y - s->sweep_rest_y,
    (int)s->sweep_rest_x - s->min_x,
    (int)s->sweep_rest_y - s->min_y,
  };

  // Every sector has to be covered, judged against the range as it stands now
  for (int i = 0; i < AXES_SWEEP_SECTORS; i++) {
    int reach = s->sweep_reach[i];

    // Each sector leans on the side it is nearest, two sectors either side of each cardinal
    int nearest = ((i + 2) / 4) & 3;

    // Covered once its best reading is past the noise floor and at least half way out on that side
    if (reach < s->min_reach || reach * 2 < side[nearest])
      return false;

    // And once the direction was sampled enough to trust that the best reading found the rim
    if (s->sweep_samples[i] < AXES_SWEEP_MIN_SAMPLES)
      return false;
  }

  return true;
}

int axes_stick_calibration_session_end(const struct axes_stick_calibration_session *s, struct axes_stick_calibration *c)
{
  // Rest and both orientation poses are needed before there is anything to resolve
  if (s->rest_count == 0 || !s->up_seen || !s->right_seen)
    return -AXES_ERR_INCOMPLETE;

  // Rest is the average of every centered reading
  int rest_x = session_rest_average(s->rest_sum_x, s->rest_count);
  int rest_y = session_rest_average(s->rest_sum_y, s->rest_count);

  // Derive only uses the shorter side of each axis, so too little travel on any side would leave that axis dead
  int reach = s->min_reach;
  if (rest_x - (int)s->min_x < reach || (int)s->max_x - rest_x < reach)
    return -AXES_ERR_INVALID;
  if (rest_y - (int)s->min_y < reach || (int)s->max_y - rest_y < reach)
    return -AXES_ERR_INVALID;

  // Up and right have to have actually left rest, or the orientation would be read from noise
  if (stick_session_deflection(s, s->up_x, s->up_y) < reach ||
      stick_session_deflection(s, s->right_x, s->right_y) < reach)
    return -AXES_ERR_INVALID;

  // Hand over rest and the range the poses and sweep reached
  c->rest_x = (uint16_t)rest_x;
  c->rest_y = (uint16_t)rest_y;
  c->min_x  = s->min_x;
  c->min_y  = s->min_y;
  c->max_x  = s->max_x;
  c->max_y  = s->max_y;

  // Orientation comes from where up and right landed relative to rest
  axes_stick_calibration_orient(c, s->up_x, s->up_y, s->right_x, s->right_y);

  return 0;
}

// Little endian byte order, chosen so a blob moves between targets unchanged
static inline void axes_put_u16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)(value & 0xFF);
  p[1] = (uint8_t)(value >> 8);
}

static inline uint16_t axes_get_u16(const uint8_t *p)
{
  return (uint16_t)(p[0] | (p[1] << 8));
}

int axes_trigger_calibration_pack(uint8_t *dest, size_t size, const struct axes_trigger_calibration *src)
{
  if (size < AXES_PACKED_TRIGGER_CALIBRATION_SIZE)
    return -AXES_ERR_SIZE;

  axes_put_u16(&dest[0], src->rest);
  axes_put_u16(&dest[2], src->pressed);

  return 0;
}

int axes_trigger_calibration_unpack(struct axes_trigger_calibration *dest, const uint8_t *src, size_t size)
{
  if (size < AXES_PACKED_TRIGGER_CALIBRATION_SIZE)
    return -AXES_ERR_SIZE;

  dest->rest    = axes_get_u16(&src[0]);
  dest->pressed = axes_get_u16(&src[2]);

  return 0;
}

int axes_stick_calibration_pack(uint8_t *dest, size_t size, const struct axes_stick_calibration *src)
{
  if (size < AXES_PACKED_STICK_CALIBRATION_SIZE)
    return -AXES_ERR_SIZE;

  uint8_t orientation_flags = 0;
  if (src->invert_x)
    orientation_flags |= 1u << 0;
  if (src->invert_y)
    orientation_flags |= 1u << 1;
  if (src->swap_xy)
    orientation_flags |= 1u << 2;

  axes_put_u16(&dest[0], src->rest_x);
  axes_put_u16(&dest[2], src->rest_y);
  axes_put_u16(&dest[4], src->min_x);
  axes_put_u16(&dest[6], src->min_y);
  axes_put_u16(&dest[8], src->max_x);
  axes_put_u16(&dest[10], src->max_y);
  dest[12] = orientation_flags;

  return 0;
}

int axes_stick_calibration_unpack(struct axes_stick_calibration *dest, const uint8_t *src, size_t size)
{
  if (size < AXES_PACKED_STICK_CALIBRATION_SIZE)
    return -AXES_ERR_SIZE;

  uint8_t orientation_flags = src[12];

  dest->rest_x   = axes_get_u16(&src[0]);
  dest->rest_y   = axes_get_u16(&src[2]);
  dest->min_x    = axes_get_u16(&src[4]);
  dest->min_y    = axes_get_u16(&src[6]);
  dest->max_x    = axes_get_u16(&src[8]);
  dest->max_y    = axes_get_u16(&src[10]);
  dest->invert_x = (orientation_flags & (1u << 0)) != 0;
  dest->invert_y = (orientation_flags & (1u << 1)) != 0;
  dest->swap_xy  = (orientation_flags & (1u << 2)) != 0;

  return 0;
}

int axes_trigger_shaping_pack(uint8_t *dest, size_t size, const struct axes_trigger_shaping *src)
{
  if (size < AXES_PACKED_TRIGGER_SHAPING_SIZE)
    return -AXES_ERR_SIZE;

  // Reject out of range settings
  if (src->deadzone_mode >= AXES_DEADZONE_MODE_COUNT)
    return -AXES_ERR_INVALID;

  uint8_t deadzone_flags = (uint8_t)(src->deadzone_mode << 2);

  axes_put_u16(&dest[0], src->deadzone_inner);
  axes_put_u16(&dest[2], src->deadzone_outer);
  dest[4] = deadzone_flags;
  axes_put_u16(&dest[5], src->response_gamma);

  return 0;
}

int axes_trigger_shaping_unpack(struct axes_trigger_shaping *dest, const uint8_t *src, size_t size)
{
  if (size < AXES_PACKED_TRIGGER_SHAPING_SIZE)
    return -AXES_ERR_SIZE;

  dest->deadzone_inner = axes_get_u16(&src[0]);
  dest->deadzone_outer = axes_get_u16(&src[2]);
  dest->deadzone_mode  = (enum axes_deadzone_mode)((src[4] >> 2) & 1u);
  dest->response_gamma = axes_get_u16(&src[5]);

  return 0;
}

int axes_stick_shaping_pack(uint8_t *dest, size_t size, const struct axes_stick_shaping *src)
{
  if (size < AXES_PACKED_STICK_SHAPING_SIZE)
    return -AXES_ERR_SIZE;

  // Reject out of range settings
  if (src->deadzone_shape >= AXES_DEADZONE_SHAPE_COUNT || src->deadzone_mode >= AXES_DEADZONE_MODE_COUNT)
    return -AXES_ERR_INVALID;
  if (src->gate_shape >= AXES_GATE_SHAPE_COUNT || src->gate_mode >= AXES_GATE_MODE_COUNT)
    return -AXES_ERR_INVALID;

  uint8_t deadzone_flags = (uint8_t)(src->deadzone_shape | (src->deadzone_mode << 2));
  uint8_t gate_flags     = (uint8_t)(src->gate_shape | (src->gate_mode << 2));

  axes_put_u16(&dest[0], src->deadzone_inner);
  axes_put_u16(&dest[2], src->deadzone_outer);
  dest[4] = deadzone_flags;
  axes_put_u16(&dest[5], src->response_gamma);
  dest[7] = gate_flags;
  axes_put_u16(&dest[8], src->gate_corner);

  return 0;
}

int axes_stick_shaping_unpack(struct axes_stick_shaping *dest, const uint8_t *src, size_t size)
{
  if (size < AXES_PACKED_STICK_SHAPING_SIZE)
    return -AXES_ERR_SIZE;

  unsigned deadzone_shape = src[4] & 3u;
  unsigned gate_shape     = src[7] & 3u;

  // Reject out of range shape values
  if (deadzone_shape >= AXES_DEADZONE_SHAPE_COUNT || gate_shape >= AXES_GATE_SHAPE_COUNT)
    return -AXES_ERR_INVALID;

  dest->deadzone_inner = axes_get_u16(&src[0]);
  dest->deadzone_outer = axes_get_u16(&src[2]);
  dest->deadzone_shape = (enum axes_deadzone_shape)deadzone_shape;
  dest->deadzone_mode  = (enum axes_deadzone_mode)((src[4] >> 2) & 1u);
  dest->response_gamma = axes_get_u16(&src[5]);
  dest->gate_shape     = (enum axes_gate_shape)gate_shape;
  dest->gate_mode      = (enum axes_gate_mode)((src[7] >> 2) & 1u);
  dest->gate_corner    = axes_get_u16(&src[8]);

  return 0;
}
