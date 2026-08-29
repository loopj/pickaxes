# Fetch the Unity test framework
if(NOT unity_FOUND)
  include(FetchContent)
  FetchContent_Declare(Unity GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git)
  FetchContent_MakeAvailable(Unity)
endif()

# Little helper function to create a test
function(add_axes_test TEST_NAME)
  # Create the test executable
  add_executable(${TEST_NAME} ${ARGN})

  # Link against the axes library and Unity
  target_link_libraries(${TEST_NAME} axes unity::framework)

  # Add the test to CTest
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

# Spec tests: named after the README behaviors they pin
add_axes_test(test_trigger test_trigger.c)
add_axes_test(test_stick test_stick.c)
