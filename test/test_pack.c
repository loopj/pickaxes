// Serialization spec tests.
//
// One round trip per config struct, checking that every field survives being
// packed and unpacked. Fixtures give each field a distinct value, so a swapped
// offset shows up as a mismatch.
#include <axes/axes.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static const struct axes_stick_calibration stick_cal = {
  .rest_x   = 2048,
  .rest_y   = 1900,
  .min_x    = 100,
  .min_y    = 200,
  .max_x    = 3900,
  .max_y    = 4000,
  .invert_x = true,
  .invert_y = false,
  .swap_xy  = true,
};

static const struct axes_stick_shaping stick_shape = {
  .deadzone_inner = 512,
  .deadzone_outer = 256,
  .deadzone_shape = AXES_DEADZONE_AXIAL,
  .deadzone_mode  = AXES_DEADZONE_UNSCALED,
  .response_gamma = 384,
  .gate_shape     = AXES_GATE_OCTAGON,
  .gate_mode      = AXES_GATE_SCALE,
  .gate_corner    = AXES_OCTAGON_REGULAR,
};

static const struct axes_trigger_calibration trigger_cal = {
  .rest    = 300,
  .pressed = 3800,
};

static const struct axes_trigger_shaping trigger_shape = {
  .deadzone_inner = 128,
  .deadzone_outer = 64,
  .deadzone_mode  = AXES_DEADZONE_UNSCALED,
  .response_gamma = AXES_GAMMA_LINEAR,
};

static void stick_calibration_round_trips(void)
{
  uint8_t buffer[AXES_PACKED_STICK_CALIBRATION_SIZE];
  struct axes_stick_calibration out = {0};

  TEST_ASSERT_EQUAL_size_t(sizeof(buffer), axes_stick_calibration_pack(buffer, sizeof(buffer), &stick_cal));
  TEST_ASSERT_EQUAL_size_t(sizeof(buffer), axes_stick_calibration_unpack(&out, buffer, sizeof(buffer)));

  TEST_ASSERT_EQUAL_UINT16(stick_cal.rest_x, out.rest_x);
  TEST_ASSERT_EQUAL_UINT16(stick_cal.rest_y, out.rest_y);
  TEST_ASSERT_EQUAL_UINT16(stick_cal.min_x, out.min_x);
  TEST_ASSERT_EQUAL_UINT16(stick_cal.min_y, out.min_y);
  TEST_ASSERT_EQUAL_UINT16(stick_cal.max_x, out.max_x);
  TEST_ASSERT_EQUAL_UINT16(stick_cal.max_y, out.max_y);
  TEST_ASSERT_EQUAL(stick_cal.invert_x, out.invert_x);
  TEST_ASSERT_EQUAL(stick_cal.invert_y, out.invert_y);
  TEST_ASSERT_EQUAL(stick_cal.swap_xy, out.swap_xy);
}

static void stick_shaping_round_trips(void)
{
  uint8_t buffer[AXES_PACKED_STICK_SHAPING_SIZE];
  struct axes_stick_shaping out = {0};

  TEST_ASSERT_EQUAL_size_t(sizeof(buffer), axes_stick_shaping_pack(buffer, sizeof(buffer), &stick_shape));
  TEST_ASSERT_EQUAL_size_t(sizeof(buffer), axes_stick_shaping_unpack(&out, buffer, sizeof(buffer)));

  TEST_ASSERT_EQUAL_UINT16(stick_shape.deadzone_inner, out.deadzone_inner);
  TEST_ASSERT_EQUAL_UINT16(stick_shape.deadzone_outer, out.deadzone_outer);
  TEST_ASSERT_EQUAL_INT(stick_shape.deadzone_shape, out.deadzone_shape);
  TEST_ASSERT_EQUAL_INT(stick_shape.deadzone_mode, out.deadzone_mode);
  TEST_ASSERT_EQUAL_UINT16(stick_shape.response_gamma, out.response_gamma);
  TEST_ASSERT_EQUAL_INT(stick_shape.gate_shape, out.gate_shape);
  TEST_ASSERT_EQUAL_INT(stick_shape.gate_mode, out.gate_mode);
  TEST_ASSERT_EQUAL_UINT16(stick_shape.gate_corner, out.gate_corner);
}

static void trigger_calibration_round_trips(void)
{
  uint8_t buffer[AXES_PACKED_TRIGGER_CALIBRATION_SIZE];
  struct axes_trigger_calibration out = {0};

  TEST_ASSERT_EQUAL_size_t(sizeof(buffer), axes_trigger_calibration_pack(buffer, sizeof(buffer), &trigger_cal));
  TEST_ASSERT_EQUAL_size_t(sizeof(buffer), axes_trigger_calibration_unpack(&out, buffer, sizeof(buffer)));

  TEST_ASSERT_EQUAL_UINT16(trigger_cal.rest, out.rest);
  TEST_ASSERT_EQUAL_UINT16(trigger_cal.pressed, out.pressed);
}

static void trigger_shaping_round_trips(void)
{
  uint8_t buffer[AXES_PACKED_TRIGGER_SHAPING_SIZE];
  struct axes_trigger_shaping out = {0};

  TEST_ASSERT_EQUAL_size_t(sizeof(buffer), axes_trigger_shaping_pack(buffer, sizeof(buffer), &trigger_shape));
  TEST_ASSERT_EQUAL_size_t(sizeof(buffer), axes_trigger_shaping_unpack(&out, buffer, sizeof(buffer)));

  TEST_ASSERT_EQUAL_UINT16(trigger_shape.deadzone_inner, out.deadzone_inner);
  TEST_ASSERT_EQUAL_UINT16(trigger_shape.deadzone_outer, out.deadzone_outer);
  TEST_ASSERT_EQUAL_INT(trigger_shape.deadzone_mode, out.deadzone_mode);
  TEST_ASSERT_EQUAL_UINT16(trigger_shape.response_gamma, out.response_gamma);
}

int main(void)
{
  UNITY_BEGIN();
  RUN_TEST(stick_calibration_round_trips);
  RUN_TEST(stick_shaping_round_trips);
  RUN_TEST(trigger_calibration_round_trips);
  RUN_TEST(trigger_shaping_round_trips);
  return UNITY_END();
}
