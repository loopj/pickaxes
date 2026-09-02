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

void axes_stick_calibration_orient(struct axes_stick_calibration *c, uint16_t up_x, uint16_t up_y, uint16_t right_x,
                                   uint16_t right_y)
{
  int rx = (int)right_x - (int)c->rest_x;
  int ry = (int)right_y - (int)c->rest_y;
  int ux = (int)up_x - (int)c->rest_x;
  int uy = (int)up_y - (int)c->rest_y;

  c->swap_xy = axes_iabs(ry) > axes_iabs(rx);
  if (c->swap_xy) {
    c->invert_x = ry < 0;
    c->invert_y = ux < 0;
  } else {
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

size_t axes_trigger_calibration_pack(uint8_t *out, size_t size, const struct axes_trigger_calibration *c)
{
  if (size < AXES_PACKED_TRIGGER_CALIBRATION_SIZE)
    return 0;

  axes_put_u16(&out[0], c->rest);
  axes_put_u16(&out[2], c->pressed);

  return AXES_PACKED_TRIGGER_CALIBRATION_SIZE;
}

size_t axes_trigger_calibration_unpack(struct axes_trigger_calibration *c, const uint8_t *in, size_t size)
{
  if (size < AXES_PACKED_TRIGGER_CALIBRATION_SIZE)
    return 0;

  c->rest    = axes_get_u16(&in[0]);
  c->pressed = axes_get_u16(&in[2]);

  return AXES_PACKED_TRIGGER_CALIBRATION_SIZE;
}

size_t axes_stick_calibration_pack(uint8_t *out, size_t size, const struct axes_stick_calibration *c)
{
  if (size < AXES_PACKED_STICK_CALIBRATION_SIZE)
    return 0;

  uint8_t orientation_flags = 0;
  if (c->invert_x)
    orientation_flags |= 1u << 0;
  if (c->invert_y)
    orientation_flags |= 1u << 1;
  if (c->swap_xy)
    orientation_flags |= 1u << 2;

  axes_put_u16(&out[0], c->rest_x);
  axes_put_u16(&out[2], c->rest_y);
  axes_put_u16(&out[4], c->min_x);
  axes_put_u16(&out[6], c->min_y);
  axes_put_u16(&out[8], c->max_x);
  axes_put_u16(&out[10], c->max_y);
  out[12] = orientation_flags;

  return AXES_PACKED_STICK_CALIBRATION_SIZE;
}

size_t axes_stick_calibration_unpack(struct axes_stick_calibration *c, const uint8_t *in, size_t size)
{
  if (size < AXES_PACKED_STICK_CALIBRATION_SIZE)
    return 0;

  uint8_t orientation_flags = in[12];

  c->rest_x   = axes_get_u16(&in[0]);
  c->rest_y   = axes_get_u16(&in[2]);
  c->min_x    = axes_get_u16(&in[4]);
  c->min_y    = axes_get_u16(&in[6]);
  c->max_x    = axes_get_u16(&in[8]);
  c->max_y    = axes_get_u16(&in[10]);
  c->invert_x = (orientation_flags & (1u << 0)) != 0;
  c->invert_y = (orientation_flags & (1u << 1)) != 0;
  c->swap_xy  = (orientation_flags & (1u << 2)) != 0;

  return AXES_PACKED_STICK_CALIBRATION_SIZE;
}

size_t axes_trigger_shaping_pack(uint8_t *out, size_t size, const struct axes_trigger_shaping *s)
{
  if (size < AXES_PACKED_TRIGGER_SHAPING_SIZE)
    return 0;

  uint8_t deadzone_flags = (uint8_t)(s->deadzone_mode << 2);

  axes_put_u16(&out[0], s->deadzone_inner);
  axes_put_u16(&out[2], s->deadzone_outer);
  out[4] = deadzone_flags;
  axes_put_u16(&out[5], s->response_gamma);

  return AXES_PACKED_TRIGGER_SHAPING_SIZE;
}

size_t axes_trigger_shaping_unpack(struct axes_trigger_shaping *s, const uint8_t *in, size_t size)
{
  if (size < AXES_PACKED_TRIGGER_SHAPING_SIZE)
    return 0;

  s->deadzone_inner = axes_get_u16(&in[0]);
  s->deadzone_outer = axes_get_u16(&in[2]);
  s->deadzone_mode  = (enum axes_deadzone_mode)((in[4] >> 2) & 1u);
  s->response_gamma = axes_get_u16(&in[5]);

  return AXES_PACKED_TRIGGER_SHAPING_SIZE;
}

size_t axes_stick_shaping_pack(uint8_t *out, size_t size, const struct axes_stick_shaping *s)
{
  if (size < AXES_PACKED_STICK_SHAPING_SIZE)
    return 0;

  uint8_t deadzone_flags = (uint8_t)(s->deadzone_shape | (s->deadzone_mode << 2));
  uint8_t gate_flags     = (uint8_t)(s->gate_shape | (s->gate_mode << 2));

  axes_put_u16(&out[0], s->deadzone_inner);
  axes_put_u16(&out[2], s->deadzone_outer);
  out[4] = deadzone_flags;
  axes_put_u16(&out[5], s->response_gamma);
  out[7] = gate_flags;
  axes_put_u16(&out[8], s->gate_corner);

  return AXES_PACKED_STICK_SHAPING_SIZE;
}

size_t axes_stick_shaping_unpack(struct axes_stick_shaping *s, const uint8_t *in, size_t size)
{
  if (size < AXES_PACKED_STICK_SHAPING_SIZE)
    return 0;

  unsigned deadzone_shape = in[4] & 3u;
  unsigned gate_shape     = in[7] & 3u;

  // Reject shape values that no enumerator claims
  if (deadzone_shape >= AXES_DEADZONE_SHAPE_COUNT || gate_shape >= AXES_GATE_SHAPE_COUNT)
    return 0;

  s->deadzone_inner = axes_get_u16(&in[0]);
  s->deadzone_outer = axes_get_u16(&in[2]);
  s->deadzone_shape = (enum axes_deadzone_shape)deadzone_shape;
  s->deadzone_mode  = (enum axes_deadzone_mode)((in[4] >> 2) & 1u);
  s->response_gamma = axes_get_u16(&in[5]);
  s->gate_shape     = (enum axes_gate_shape)gate_shape;
  s->gate_mode      = (enum axes_gate_mode)((in[7] >> 2) & 1u);
  s->gate_corner    = axes_get_u16(&in[8]);

  return AXES_PACKED_STICK_SHAPING_SIZE;
}
