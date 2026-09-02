// Trigger pipeline spec tests.
//
// Each test pins one behavior documented in the README. Fixture: a 12-bit
// trigger resting at raw 100 and fully pressed at raw 3000.
#include <axes/axes.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static const struct axes_trigger_calibration cal = {
  .rest    = 100,
  .pressed = 3000,
};

static void rest_maps_to_zero(void)
{
  struct axes_trigger_transform t;
  axes_trigger_derive(&t, &cal, NULL);

  TEST_ASSERT_EQUAL_INT(0, axes_trigger_apply(&t, 100));
}

static void full_press_maps_to_full_scale(void)
{
  struct axes_trigger_transform t;
  axes_trigger_derive(&t, &cal, NULL);

  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, axes_trigger_apply(&t, 3000));
}

static void deadzones_rescale_remaining_travel(void)
{
  // 10% inner and outer deadzones: released inside the inner zone, fully
  // pressed inside the outer zone, and the live band between them stretched
  // back over the full output range.
  struct axes_trigger_shaping s = {
    .deadzone_inner = AXES_FULL_SCALE / 10, //
    .deadzone_outer = AXES_FULL_SCALE / 10,
  };
  struct axes_trigger_transform t;
  axes_trigger_derive(&t, &cal, &s);

  TEST_ASSERT_EQUAL_INT(0, axes_trigger_apply(&t, 300));                // inside inner zone
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, axes_trigger_apply(&t, 2800)); // inside outer zone

  // Raw 1550 is the center of the live band (390..2710), so output rises from
  // zero at the zone edge and hits half scale at the band's midpoint.
  TEST_ASSERT_INT_WITHIN(4, AXES_FULL_SCALE / 2, axes_trigger_apply(&t, 1550));
}

static void gamma_two_quarters_the_midpoint(void)
{
  // x^2 at half press is a quarter of full output; full press is unaffected.
  struct axes_trigger_shaping s = {
    .response_gamma = 2 * AXES_GAMMA_LINEAR,
  };
  struct axes_trigger_transform t;
  axes_trigger_derive(&t, &cal, &s);

  TEST_ASSERT_INT_WITHIN(8, AXES_FULL_SCALE / 4, axes_trigger_apply(&t, 1550));
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, axes_trigger_apply(&t, 3000));
}

static void unscaled_deadzones_are_position_true(void)
{
  // Position-true deadzones keep the calibrated mapping between the zones, so
  // mid-travel output matches a deadzone-free config exactly, while readings
  // inside the zones still snap to the ends.
  struct axes_trigger_shaping s = {
    .deadzone_inner = AXES_FULL_SCALE / 10,
    .deadzone_outer = AXES_FULL_SCALE / 10,
    .deadzone_mode  = AXES_DEADZONE_MODE_UNSCALED,
  };
  struct axes_trigger_transform t, plain;
  axes_trigger_derive(&t, &cal, &s);
  axes_trigger_derive(&plain, &cal, NULL);

  TEST_ASSERT_EQUAL_INT(0, axes_trigger_apply(&t, 300));                // inside inner zone
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, axes_trigger_apply(&t, 2800)); // inside outer zone

  // Mid-travel is not rescaled - it reads the same as with no deadzones at all
  TEST_ASSERT_INT_WITHIN(2, axes_trigger_apply(&plain, 1550), axes_trigger_apply(&t, 1550));
}

static void overlapping_deadzones_read_released(void)
{
  // Each width is inside its documented range, but together they cover the whole
  // travel. With nothing left between them the trigger reads released throughout,
  // rather than running backwards or chattering on a one-count threshold.
  struct axes_trigger_shaping s = {
    .deadzone_inner = 3000,
    .deadzone_outer = 3000,
  };
  struct axes_trigger_transform t;
  axes_trigger_derive(&t, &cal, &s);

  for (int raw = 0; raw <= 4095; raw += 50)
    TEST_ASSERT_EQUAL_INT(0, axes_trigger_apply(&t, (uint16_t)raw));
}

static void inverted_install_maps_correctly(void)
{
  // A trigger whose raw reading falls as it is pressed still maps rest to
  // zero and full press to full scale.
  struct axes_trigger_calibration inv = {
    .rest    = 3000,
    .pressed = 100,
  };
  struct axes_trigger_transform t;
  axes_trigger_derive(&t, &inv, NULL);

  TEST_ASSERT_EQUAL_INT(0, axes_trigger_apply(&t, 3000));
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, axes_trigger_apply(&t, 100));
  TEST_ASSERT_INT_WITHIN(4, AXES_FULL_SCALE / 2, axes_trigger_apply(&t, 1550));
}

static void zero_travel_calibration_is_safe(void)
{
  // rest == pressed describes a trigger with no measurable travel; it should
  // read released everywhere rather than dividing by zero in derive.
  struct axes_trigger_calibration dead = {
    .rest    = 2048,
    .pressed = 2048,
  };
  struct axes_trigger_transform t;
  axes_trigger_derive(&t, &dead, NULL);

  TEST_ASSERT_EQUAL_INT(0, axes_trigger_apply(&t, 0));
  TEST_ASSERT_EQUAL_INT(0, axes_trigger_apply(&t, 2048));
  TEST_ASSERT_EQUAL_INT(0, axes_trigger_apply(&t, 4095));
}

int main(void)
{
  UNITY_BEGIN();
  RUN_TEST(rest_maps_to_zero);
  RUN_TEST(full_press_maps_to_full_scale);
  RUN_TEST(deadzones_rescale_remaining_travel);
  RUN_TEST(unscaled_deadzones_are_position_true);
  RUN_TEST(overlapping_deadzones_read_released);
  RUN_TEST(gamma_two_quarters_the_midpoint);
  RUN_TEST(inverted_install_maps_correctly);
  RUN_TEST(zero_travel_calibration_is_safe);
  return UNITY_END();
}
