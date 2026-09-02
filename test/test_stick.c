// Stick pipeline spec tests.
//
// Each test pins one behavior documented in the README, in README section
// order: calibration, deadzones, response curve, gates. Fixture: a 12-bit
// stick resting at raw (2048, 2048) with full-rail extents, so one raw count
// is two normalized counts.
#include <axes/axes.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static const struct axes_stick_calibration cal = {
  .rest_x = 2048,
  .rest_y = 2048,
  .min_x  = 0,
  .min_y  = 0,
  .max_x  = 4096,
  .max_y  = 4096,
};

// --- Calibration ---

static void cardinals_reach_full_scale(void)
{
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, NULL);

  axes_stick_apply(&t, 4096, 2048, &x, &y);
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, x);
  TEST_ASSERT_EQUAL_INT(0, y);

  axes_stick_apply(&t, 0, 2048, &x, &y);
  TEST_ASSERT_EQUAL_INT(-AXES_FULL_SCALE, x);
  TEST_ASSERT_EQUAL_INT(0, y);

  axes_stick_apply(&t, 2048, 4096, &x, &y);
  TEST_ASSERT_EQUAL_INT(0, x);
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, y);

  axes_stick_apply(&t, 2048, 0, &x, &y);
  TEST_ASSERT_EQUAL_INT(0, x);
  TEST_ASSERT_EQUAL_INT(-AXES_FULL_SCALE, y);
}

// Model a physical mounting: where do the raw ADC values sit when the stick is
// deflected to a logical position, for a given swap/invert combination?
static void raw_for_mount(bool swap, bool inv_x, bool inv_y, int lx, int ly, uint16_t *raw_x, uint16_t *raw_y)
{
  int px = inv_x ? -lx : lx; // physical deflection feeding logical X
  int py = inv_y ? -ly : ly; // physical deflection feeding logical Y

  if (swap) {
    *raw_x = (uint16_t)(2048 + py);
    *raw_y = (uint16_t)(2048 + px);
  } else {
    *raw_x = (uint16_t)(2048 + px);
    *raw_y = (uint16_t)(2048 + py);
  }
}

static void orient_resolves_all_eight_mountings(void)
{
  for (int mount = 0; mount < 8; mount++) {
    bool swap = mount & 1, inv_x = mount & 2, inv_y = mount & 4;
    uint16_t up_x, up_y, right_x, right_y;
    raw_for_mount(swap, inv_x, inv_y, 0, 800, &up_x, &up_y);
    raw_for_mount(swap, inv_x, inv_y, 800, 0, &right_x, &right_y);

    // The helper recovers the mounting from the up and right cardinals alone
    struct axes_stick_calibration c = cal;
    axes_stick_calibration_orient(&c, up_x, up_y, right_x, right_y);
    TEST_ASSERT_EQUAL_INT(swap, c.swap_xy);
    TEST_ASSERT_EQUAL_INT(inv_x, c.invert_x);
    TEST_ASSERT_EQUAL_INT(inv_y, c.invert_y);

    // And the full pipeline sends physical up out as logical +Y
    struct axes_stick_transform t;
    int16_t x, y;
    axes_stick_derive(&t, &c, NULL);
    axes_stick_apply(&t, up_x, up_y, &x, &y);
    TEST_ASSERT_EQUAL_INT(0, x);
    TEST_ASSERT_EQUAL_INT(1600, y); // 800 raw counts, doubled by full-rail scaling
  }
}

// --- Deadzones ---

// The discriminating geometry for deadzone shape: a diagonal point whose axis
// values are each inside a per-axis band, but whose radius is outside the
// same-sized circle. Radial sees output; axial sees rest.
static const struct axes_stick_shaping radial_dz = {
  .deadzone_inner = 400,
  .deadzone_shape = AXES_DEADZONE_RADIAL,
  .gate_shape     = AXES_GATE_CIRCLE,
};
static const struct axes_stick_shaping axial_dz = {
  .deadzone_inner = 400,
  .deadzone_shape = AXES_DEADZONE_AXIAL,
  .gate_shape     = AXES_GATE_CIRCLE,
};

static void radial_deadzone_measures_the_circle(void)
{
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &radial_dz);

  // (300, 300) normalized: radius ~424, outside the 400 circle
  axes_stick_apply(&t, 2048 + 150, 2048 + 150, &x, &y);
  TEST_ASSERT_TRUE(x > 0);
  TEST_ASSERT_TRUE(y > 0);
}

static void axial_deadzone_measures_each_axis(void)
{
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &axial_dz);

  // The same (300, 300) point: each axis inside its own 400 band
  axes_stick_apply(&t, 2048 + 150, 2048 + 150, &x, &y);
  TEST_ASSERT_EQUAL_INT(0, x);
  TEST_ASSERT_EQUAL_INT(0, y);

  // Axes are independent: x inside its band reads zero, y is unaffected
  axes_stick_apply(&t, 2048 + 150, 2048 + 1024, &x, &y);
  TEST_ASSERT_EQUAL_INT(0, x);
  TEST_ASSERT_TRUE(y > 0);
}

static void scaled_deadzone_rises_from_zero(void)
{
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &radial_dz);

  // Just past the zone edge, output is small - no jump - and full deflection
  // still reaches full scale.
  axes_stick_apply(&t, 2048 + 250, 2048, &x, &y); // normalized 500 vs inner 400
  TEST_ASSERT_TRUE(x > 0);
  TEST_ASSERT_TRUE(x <= 160);

  axes_stick_apply(&t, 4096, 2048, &x, &y);
  TEST_ASSERT_INT_WITHIN(4, AXES_FULL_SCALE, x);
}

static void unscaled_deadzone_is_position_true(void)
{
  struct axes_stick_shaping s = {
    .deadzone_inner = 400,
    .deadzone_mode  = AXES_DEADZONE_UNSCALED,
    .gate_shape     = AXES_GATE_CIRCLE,
  };
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &s);

  // Outside the zone, output matches physical position exactly
  axes_stick_apply(&t, 2048 + 1024, 2048, &x, &y);
  TEST_ASSERT_INT_WITHIN(8, 2048, x);
}

static void deadzone_overlap_stays_bounded(void)
{
  // Characterization, not documented spec: inner + outer beyond full scale is
  // nonsense config, and the current behavior (a floored live band) just needs
  // to stay crash-free and bounded. Revisit if the README ever pins semantics.
  struct axes_stick_shaping s = {
    .deadzone_inner = 3000,
    .deadzone_outer = 3000,
    .deadzone_mode  = AXES_DEADZONE_SCALED,
    .gate_shape     = AXES_GATE_CIRCLE,
  };
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &s);

  for (int raw = 0; raw <= 4096; raw += 256) {
    axes_stick_apply(&t, (uint16_t)raw, (uint16_t)(4096 - raw), &x, &y);
    TEST_ASSERT_TRUE(x >= -AXES_FULL_SCALE && x <= AXES_FULL_SCALE);
    TEST_ASSERT_TRUE(y >= -AXES_FULL_SCALE && y <= AXES_FULL_SCALE);
  }
}

// --- Response curve ---

static void gamma_two_quarters_half_deflection(void)
{
  struct axes_stick_shaping s = {
    .response_gamma = 2 * AXES_GAMMA_LINEAR,
    .gate_shape     = AXES_GATE_CIRCLE,
  };
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &s);

  axes_stick_apply(&t, 3072, 2048, &x, &y); // half deflection
  TEST_ASSERT_INT_WITHIN(8, AXES_FULL_SCALE / 4, x);

  axes_stick_apply(&t, 4096, 2048, &x, &y); // full deflection is unaffected
  TEST_ASSERT_INT_WITHIN(2, AXES_FULL_SCALE, x);
}

static void gamma_applies_after_deadzones(void)
{
  // The curve bends the deadzoned signal, so output at the zone edge is still
  // zero and just past it is still near zero, for any gamma.
  struct axes_stick_shaping s = {
    .deadzone_inner = 400,
    .deadzone_mode  = AXES_DEADZONE_SCALED,
    .response_gamma = 2 * AXES_GAMMA_LINEAR,
    .gate_shape     = AXES_GATE_CIRCLE,
  };
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &s);

  axes_stick_apply(&t, 2048 + 200, 2048, &x, &y); // at the zone edge
  TEST_ASSERT_EQUAL_INT(0, x);

  axes_stick_apply(&t, 2048 + 250, 2048, &x, &y); // just past it
  TEST_ASSERT_TRUE(x >= 0);
  TEST_ASSERT_TRUE(x <= 8);
}

// --- Gates ---

static void gate_none_preserves_corners(void)
{
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, NULL); // defaults: no gate

  axes_stick_apply(&t, 4096, 4096, &x, &y);
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, x);
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, y);
}

static void gate_circle_clamps_to_the_rim(void)
{
  struct axes_stick_shaping s = {
    .gate_shape = AXES_GATE_CIRCLE,
  };
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &s);

  // A full diagonal lands on the circle at full-scale/sqrt(2) per axis
  axes_stick_apply(&t, 4096, 4096, &x, &y);
  TEST_ASSERT_INT_WITHIN(8, 2896, x);
  TEST_ASSERT_INT_WITHIN(8, 2896, y);

  // Interior positions pass through untouched
  axes_stick_apply(&t, 3072, 2048, &x, &y);
  TEST_ASSERT_INT_WITHIN(4, AXES_FULL_SCALE / 2, x);
}

static void gate_n64_matches_oem_extents(void)
{
  // The README's N64 example, verbatim: cardinal at 80 on the wire, diagonal
  // corners at ~(65, 65) - the OEM (70, 70)-vs-85 geometry, truncated.
  struct axes_stick_calibration ncal = {
    .rest_x = 2052,
    .rest_y = 2071,
    .min_x  = 214,
    .min_y  = 189,
    .max_x  = 3888,
    .max_y  = 3907,
  };
  struct axes_stick_shaping ns = {
    .deadzone_inner = AXES_FULL_SCALE * 0.05,
    .deadzone_outer = AXES_FULL_SCALE * 0.02,
    .deadzone_shape = AXES_DEADZONE_RADIAL,
    .deadzone_mode  = AXES_DEADZONE_SCALED,
    .response_gamma = AXES_GAMMA_LINEAR,
    .gate_shape     = AXES_GATE_OCTAGON,
    .gate_corner    = AXES_OCTAGON_N64,
    .gate_mode      = AXES_GATE_SCALE,
  };
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &ncal, &ns);

  axes_stick_apply(&t, 3888, 2071, &x, &y);
  TEST_ASSERT_EQUAL_INT(80, (int32_t)x * 80 / AXES_FULL_SCALE);

  axes_stick_apply(&t, 3888, 3907, &x, &y);
  TEST_ASSERT_INT_WITHIN(1, 65, (int32_t)x * 80 / AXES_FULL_SCALE);
  TEST_ASSERT_INT_WITHIN(1, 65, (int32_t)y * 80 / AXES_FULL_SCALE);
}

static void gate_clamp_is_position_true_inside_the_gate(void)
{
  // Clamping leaves the position alone until the stick meets the boundary, where
  // scaling rescales the whole ray. Sampled between a cardinal and a corner,
  // where a regular octagon sits furthest inside the full-scale circle.
  struct axes_stick_shaping con = {
    .gate_shape  = AXES_GATE_OCTAGON,
    .gate_corner = AXES_OCTAGON_REGULAR,
    .gate_mode   = AXES_GATE_CLAMP,
  };
  struct axes_stick_shaping str = {
    .gate_shape  = AXES_GATE_OCTAGON,
    .gate_corner = AXES_OCTAGON_REGULAR,
    .gate_mode   = AXES_GATE_SCALE,
  };
  struct axes_stick_transform tc, ts;
  int16_t cx, cy, sx, sy;
  axes_stick_derive(&tc, &cal, &con);
  axes_stick_derive(&ts, &cal, &str);

  // Half travel at 22.5 degrees, well inside the gate
  axes_stick_apply(&tc, 3000, 2440, &cx, &cy);
  axes_stick_apply(&ts, 3000, 2440, &sx, &sy);
  TEST_ASSERT_TRUE(cx > sx);
  TEST_ASSERT_TRUE(cy > sy);

  // At the rim the two agree, since both land on the same boundary
  axes_stick_apply(&tc, 3939, 2832, &cx, &cy);
  axes_stick_apply(&ts, 3939, 2832, &sx, &sy);
  TEST_ASSERT_INT_WITHIN(2, sx, cx);
  TEST_ASSERT_INT_WITHIN(2, sy, cy);
}

static void gate_clamp_shrinks_gates_that_reach_past_the_circle(void)
{
  // A gate whose corners sit outside the full-scale circle cannot be reached by
  // restriction, so clamping shrinks it to fit and the cardinals give up range.
  // The corners land on the circle, keeping the requested shape.
  struct axes_stick_shaping n64 = {
    .gate_shape  = AXES_GATE_OCTAGON,
    .gate_corner = AXES_OCTAGON_N64,
    .gate_mode   = AXES_GATE_CLAMP,
  };
  struct axes_stick_shaping sq = {
    .gate_shape  = AXES_GATE_OCTAGON,
    .gate_corner = AXES_OCTAGON_SQUARE,
    .gate_mode   = AXES_GATE_CLAMP,
  };
  struct axes_stick_transform tn, tq;
  int16_t x, y;
  axes_stick_derive(&tn, &cal, &n64);
  axes_stick_derive(&tq, &cal, &sq);

  axes_stick_apply(&tn, 4096, 2048, &x, &y);
  TEST_ASSERT_INT_WITHIN(2, 3517, x);
  axes_stick_apply(&tn, 4096, 4096, &x, &y);
  TEST_ASSERT_INT_WITHIN(2, AXES_OCTAGON_REGULAR, x);
  TEST_ASSERT_INT_WITHIN(2, AXES_OCTAGON_REGULAR, y);

  axes_stick_apply(&tq, 4096, 2048, &x, &y);
  TEST_ASSERT_INT_WITHIN(2, AXES_OCTAGON_REGULAR, x);
}

static void gate_clamp_leaves_narrow_gates_alone(void)
{
  // At or below the regular octagon the gate already fits, so nothing shrinks
  // and a full cardinal deflection still reaches full scale.
  struct axes_stick_shaping reg = {
    .gate_shape  = AXES_GATE_OCTAGON,
    .gate_corner = AXES_OCTAGON_REGULAR,
    .gate_mode   = AXES_GATE_CLAMP,
  };
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &cal, &reg);
  axes_stick_apply(&t, 4096, 2048, &x, &y);
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, x);
  TEST_ASSERT_EQUAL_INT(0, y);
}

static void gate_mode_does_not_disturb_an_absent_gate(void)
{
  // NONE does no reshaping at all, so the fit setting has nothing to act on
  struct axes_stick_shaping con = {
    .gate_shape = AXES_GATE_NONE,
    .gate_mode  = AXES_GATE_CLAMP,
  };
  struct axes_stick_shaping str = {
    .gate_shape = AXES_GATE_NONE,
    .gate_mode  = AXES_GATE_SCALE,
  };
  struct axes_stick_transform tc, ts;
  int16_t cx, cy, sx, sy;
  axes_stick_derive(&tc, &cal, &con);
  axes_stick_derive(&ts, &cal, &str);
  axes_stick_apply(&tc, 4096, 4096, &cx, &cy);
  axes_stick_apply(&ts, 4096, 4096, &sx, &sy);
  TEST_ASSERT_EQUAL_INT(sx, cx);
  TEST_ASSERT_EQUAL_INT(sy, cy);
}

static void gate_corner_zero_means_regular(void)
{
  // A corner position of zero selects the regular octagon, so a config that
  // never sets it behaves identically to one that asks for REGULAR.
  struct axes_stick_shaping zeroed = {
    .gate_shape = AXES_GATE_OCTAGON,
  };
  struct axes_stick_shaping reg = {
    .gate_shape  = AXES_GATE_OCTAGON,
    .gate_corner = AXES_OCTAGON_REGULAR,
  };
  struct axes_stick_transform tz, tr;
  int16_t zx, zy, rx, ry;
  axes_stick_derive(&tz, &cal, &zeroed);
  axes_stick_derive(&tr, &cal, &reg);

  for (int i = 0; i < 8; i++) {
    uint16_t x = (uint16_t)(512 + i * 400), y = (uint16_t)(4096 - i * 350);
    axes_stick_apply(&tz, x, y, &zx, &zy);
    axes_stick_apply(&tr, x, y, &rx, &ry);
    TEST_ASSERT_EQUAL_INT(rx, zx);
    TEST_ASSERT_EQUAL_INT(ry, zy);
  }
}

int main(void)
{
  UNITY_BEGIN();
  RUN_TEST(cardinals_reach_full_scale);
  RUN_TEST(orient_resolves_all_eight_mountings);
  RUN_TEST(radial_deadzone_measures_the_circle);
  RUN_TEST(axial_deadzone_measures_each_axis);
  RUN_TEST(scaled_deadzone_rises_from_zero);
  RUN_TEST(unscaled_deadzone_is_position_true);
  RUN_TEST(deadzone_overlap_stays_bounded);
  RUN_TEST(gamma_two_quarters_half_deflection);
  RUN_TEST(gamma_applies_after_deadzones);
  RUN_TEST(gate_none_preserves_corners);
  RUN_TEST(gate_circle_clamps_to_the_rim);
  RUN_TEST(gate_n64_matches_oem_extents);
  RUN_TEST(gate_clamp_is_position_true_inside_the_gate);
  RUN_TEST(gate_clamp_shrinks_gates_that_reach_past_the_circle);
  RUN_TEST(gate_clamp_leaves_narrow_gates_alone);
  RUN_TEST(gate_mode_does_not_disturb_an_absent_gate);
  RUN_TEST(gate_corner_zero_means_regular);
  return UNITY_END();
}
