// Calibration spec tests.
//
// Each test pins one behavior of the calibration sessions documented in the
// README. Fixture: a 12-bit stick resting near raw (2048, 2048), and a 12-bit
// trigger resting near raw 100.
#include <axes/axes.h>

#include "unity.h"

void setUp(void)
{
}

void tearDown(void)
{
}

// --- Trigger ---

static void trigger_session_produces_a_calibration(void)
{
  struct axes_trigger_calibration_session s;
  struct axes_trigger_calibration c;
  axes_trigger_calibration_session_begin(&s, 4095);

  // Rest is averaged over noisy released readings
  TEST_ASSERT_EQUAL_INT(0, axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_RELEASED, 98));
  TEST_ASSERT_EQUAL_INT(0, axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_RELEASED, 100));
  TEST_ASSERT_EQUAL_INT(0, axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_RELEASED, 102));

  // Pressed keeps whichever reading sits farthest from rest
  TEST_ASSERT_EQUAL_INT(0, axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_PRESSED, 2990));
  TEST_ASSERT_EQUAL_INT(0, axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_PRESSED, 3000));
  TEST_ASSERT_EQUAL_INT(0, axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_PRESSED, 2995));

  TEST_ASSERT_EQUAL_INT(0, axes_trigger_calibration_session_end(&s, &c));
  TEST_ASSERT_EQUAL_UINT16(100, c.rest);
  TEST_ASSERT_EQUAL_UINT16(3000, c.pressed);

  // An inverted install reads downward when pressed, and resolves the same way
  axes_trigger_calibration_session_begin(&s, 4095);
  axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_RELEASED, 3000);
  axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_PRESSED, 110);
  axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_PRESSED, 100);
  axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_PRESSED, 105);

  TEST_ASSERT_EQUAL_INT(0, axes_trigger_calibration_session_end(&s, &c));
  TEST_ASSERT_EQUAL_UINT16(3000, c.rest);
  TEST_ASSERT_EQUAL_UINT16(100, c.pressed);
}

static void trigger_end_without_both_poses_fails(void)
{
  struct axes_trigger_calibration_session s;
  struct axes_trigger_calibration c = {.rest = 1, .pressed = 2};
  axes_trigger_calibration_session_begin(&s, 4095);

  // Nothing captured at all
  TEST_ASSERT_EQUAL_INT(-AXES_ERR_INCOMPLETE, axes_trigger_calibration_session_end(&s, &c));

  // Pressed cannot be captured before released, so it does not count either
  TEST_ASSERT_EQUAL_INT(-AXES_ERR_INCOMPLETE,
                        axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_PRESSED, 3000));
  TEST_ASSERT_EQUAL_INT(-AXES_ERR_INCOMPLETE, axes_trigger_calibration_session_end(&s, &c));

  // Released alone is still not enough
  axes_trigger_calibration_session_capture(&s, AXES_TRIGGER_POSE_RELEASED, 100);
  TEST_ASSERT_EQUAL_INT(-AXES_ERR_INCOMPLETE, axes_trigger_calibration_session_end(&s, &c));

  // Failure leaves the destination untouched
  TEST_ASSERT_EQUAL_UINT16(1, c.rest);
  TEST_ASSERT_EQUAL_UINT16(2, c.pressed);
}

// --- Stick poses ---

// Capture the resting position, then a pose at each of the four cardinals
static void capture_cardinals(struct axes_stick_calibration_session *s)
{
  axes_stick_calibration_session_capture(s, AXES_STICK_POSE_CENTERED, 2048, 2048);
  axes_stick_calibration_session_capture(s, AXES_STICK_POSE_UP, 2048, 3900);
  axes_stick_calibration_session_capture(s, AXES_STICK_POSE_DOWN, 2048, 200);
  axes_stick_calibration_session_capture(s, AXES_STICK_POSE_RIGHT, 3900, 2048);
  axes_stick_calibration_session_capture(s, AXES_STICK_POSE_LEFT, 200, 2048);
}

static void stick_session_produces_a_calibration(void)
{
  struct axes_stick_calibration_session s;
  struct axes_stick_calibration c;
  axes_stick_calibration_session_begin(&s, 4095);

  // Rest is averaged over noisy centered readings
  TEST_ASSERT_EQUAL_INT(0, axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2046, 2050));
  TEST_ASSERT_EQUAL_INT(0, axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2050, 2046));

  // The cardinals set the range of travel
  capture_cardinals(&s);

  TEST_ASSERT_EQUAL_INT(0, axes_stick_calibration_session_end(&s, &c));
  TEST_ASSERT_EQUAL_UINT16(2048, c.rest_x);
  TEST_ASSERT_EQUAL_UINT16(2048, c.rest_y);
  TEST_ASSERT_EQUAL_UINT16(200, c.min_x);
  TEST_ASSERT_EQUAL_UINT16(200, c.min_y);
  TEST_ASSERT_EQUAL_UINT16(3900, c.max_x);
  TEST_ASSERT_EQUAL_UINT16(3900, c.max_y);
  TEST_ASSERT_FALSE(c.invert_x);
  TEST_ASSERT_FALSE(c.invert_y);
  TEST_ASSERT_FALSE(c.swap_xy);

  // And the calibration drives the pipeline, with each cardinal reaching full scale
  struct axes_stick_transform t;
  int16_t x, y;
  axes_stick_derive(&t, &c, NULL);

  axes_stick_apply(&t, 3900, 2048, &x, &y);
  TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, x);
  TEST_ASSERT_EQUAL_INT(0, y);

  axes_stick_apply(&t, 2048, 200, &x, &y);
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

static void stick_orient_resolves_all_eight_mountings(void)
{
  static const enum axes_stick_pose poses[4] = {
    AXES_STICK_POSE_UP,
    AXES_STICK_POSE_RIGHT,
    AXES_STICK_POSE_DOWN,
    AXES_STICK_POSE_LEFT,
  };
  static const int logical[4][2] = {{0, 800}, {800, 0}, {0, -800}, {-800, 0}};

  for (int mount = 0; mount < 8; mount++) {
    bool swap = mount & 1, inv_x = mount & 2, inv_y = mount & 4;

    // Capture rest and the four cardinals as this mounting would read them
    struct axes_stick_calibration_session s;
    axes_stick_calibration_session_begin(&s, 4095);
    axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2048, 2048);
    for (int i = 0; i < 4; i++) {
      uint16_t raw_x, raw_y;
      raw_for_mount(swap, inv_x, inv_y, logical[i][0], logical[i][1], &raw_x, &raw_y);
      axes_stick_calibration_session_capture(&s, poses[i], raw_x, raw_y);
    }

    // The session recovers the mounting
    struct axes_stick_calibration c;
    TEST_ASSERT_EQUAL_INT(0, axes_stick_calibration_session_end(&s, &c));
    TEST_ASSERT_EQUAL_INT(swap, c.swap_xy);
    TEST_ASSERT_EQUAL_INT(inv_x, c.invert_x);
    TEST_ASSERT_EQUAL_INT(inv_y, c.invert_y);

    // And the full pipeline sends physical up out as logical +Y
    struct axes_stick_transform t;
    int16_t x, y;
    uint16_t up_x, up_y;
    raw_for_mount(swap, inv_x, inv_y, 0, 800, &up_x, &up_y);
    axes_stick_derive(&t, &c, NULL);
    axes_stick_apply(&t, up_x, up_y, &x, &y);
    TEST_ASSERT_EQUAL_INT(0, x);
    TEST_ASSERT_EQUAL_INT(AXES_FULL_SCALE, y);
  }
}

static void stick_end_without_up_or_right_fails(void)
{
  struct axes_stick_calibration_session s;
  struct axes_stick_calibration c;

  // Every pose but up
  axes_stick_calibration_session_begin(&s, 4095);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2048, 2048);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_DOWN, 2048, 200);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_RIGHT, 3900, 2048);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_LEFT, 200, 2048);
  TEST_ASSERT_EQUAL_INT(-AXES_ERR_INCOMPLETE, axes_stick_calibration_session_end(&s, &c));

  // Every pose but right
  axes_stick_calibration_session_begin(&s, 4095);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2048, 2048);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_UP, 2048, 3900);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_DOWN, 2048, 200);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_LEFT, 200, 2048);
  TEST_ASSERT_EQUAL_INT(-AXES_ERR_INCOMPLETE, axes_stick_calibration_session_end(&s, &c));

  // Adding the missing pose is all it takes
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_RIGHT, 3900, 2048);
  TEST_ASSERT_EQUAL_INT(0, axes_stick_calibration_session_end(&s, &c));
}

// --- Sweep ---

// Feed the four edges of a square rim into a sweep, one reading every `step` counts
static void sweep_square(struct axes_stick_calibration_session *s, int lo, int hi, int step)
{
  for (int i = lo; i <= hi; i += step) {
    axes_stick_calibration_session_sweep_sample(s, (uint16_t)i, (uint16_t)lo);
    axes_stick_calibration_session_sweep_sample(s, (uint16_t)i, (uint16_t)hi);
    axes_stick_calibration_session_sweep_sample(s, (uint16_t)lo, (uint16_t)i);
    axes_stick_calibration_session_sweep_sample(s, (uint16_t)hi, (uint16_t)i);
  }
}

static void stick_up_and_right_at_rest_fail(void)
{
  // A sweep gives a good range, but up and right captured with the stick sitting
  // at rest would set the orientation flags from noise
  struct axes_stick_calibration_session s;
  struct axes_stick_calibration c;
  axes_stick_calibration_session_begin(&s, 4095);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2048, 2048);
  axes_stick_calibration_session_sweep_begin(&s);
  sweep_square(&s, 100, 4000, 50);

  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_UP, 2050, 2046);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_RIGHT, 2046, 2050);
  TEST_ASSERT_EQUAL_INT(-AXES_ERR_INVALID, axes_stick_calibration_session_end(&s, &c));

  // Real pushes replace the noise, since the farthest reading is kept
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_UP, 2048, 4000);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_RIGHT, 4000, 2048);
  TEST_ASSERT_EQUAL_INT(0, axes_stick_calibration_session_end(&s, &c));
  TEST_ASSERT_FALSE(c.invert_x);
  TEST_ASSERT_FALSE(c.invert_y);
  TEST_ASSERT_FALSE(c.swap_xy);
}

static void sweep_square_perimeter_completes_and_sets_extents(void)
{
  struct axes_stick_calibration_session s;
  struct axes_stick_calibration c;
  axes_stick_calibration_session_begin(&s, 4095);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2048, 2048);

  // Nothing is covered before any readings arrive
  TEST_ASSERT_EQUAL_INT(0, axes_stick_calibration_session_sweep_begin(&s));
  TEST_ASSERT_FALSE(axes_stick_calibration_session_sweep_complete(&s));

  // Once around the rim covers every direction
  sweep_square(&s, 100, 4000, 50);
  TEST_ASSERT_TRUE(axes_stick_calibration_session_sweep_complete(&s));

  // And the range is the rim itself
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_UP, 2048, 4000);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_RIGHT, 4000, 2048);
  TEST_ASSERT_EQUAL_INT(0, axes_stick_calibration_session_end(&s, &c));
  TEST_ASSERT_EQUAL_UINT16(100, c.min_x);
  TEST_ASSERT_EQUAL_UINT16(100, c.min_y);
  TEST_ASSERT_EQUAL_UINT16(4000, c.max_x);
  TEST_ASSERT_EQUAL_UINT16(4000, c.max_y);
}

static void sweep_noise_at_rest_never_completes(void)
{
  // A stick nobody is touching wanders a few counts in every direction, which
  // must not look like a rim
  struct axes_stick_calibration_session s;
  axes_stick_calibration_session_begin(&s, 4095);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2048, 2048);
  axes_stick_calibration_session_sweep_begin(&s);

  for (int i = 0; i < 5000; i++) {
    uint16_t x = (uint16_t)(2048 + i % 7 - 3);
    uint16_t y = (uint16_t)(2048 + (i / 7) % 7 - 3);
    axes_stick_calibration_session_sweep_sample(&s, x, y);
  }

  TEST_ASSERT_FALSE(axes_stick_calibration_session_sweep_complete(&s));
}

static void sweep_small_rim_then_one_push_does_not_complete(void)
{
  // A small circuit looks complete on its own, but one hard push shows the real
  // rim is further out, so coverage has to be judged against it again
  struct axes_stick_calibration_session s;
  axes_stick_calibration_session_begin(&s, 4095);
  axes_stick_calibration_session_capture(&s, AXES_STICK_POSE_CENTERED, 2048, 2048);
  axes_stick_calibration_session_sweep_begin(&s);

  sweep_square(&s, 1848, 2248, 25);
  TEST_ASSERT_TRUE(axes_stick_calibration_session_sweep_complete(&s));

  axes_stick_calibration_session_sweep_sample(&s, 4000, 2048);
  TEST_ASSERT_FALSE(axes_stick_calibration_session_sweep_complete(&s));
}

int main(void)
{
  UNITY_BEGIN();
  RUN_TEST(trigger_session_produces_a_calibration);
  RUN_TEST(trigger_end_without_both_poses_fails);
  RUN_TEST(stick_session_produces_a_calibration);
  RUN_TEST(stick_orient_resolves_all_eight_mountings);
  RUN_TEST(stick_end_without_up_or_right_fails);
  RUN_TEST(stick_up_and_right_at_rest_fail);
  RUN_TEST(sweep_square_perimeter_completes_and_sets_extents);
  RUN_TEST(sweep_noise_at_rest_never_completes);
  RUN_TEST(sweep_small_rim_then_one_push_does_not_complete);
  return UNITY_END();
}
