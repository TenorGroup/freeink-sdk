# Host tests for the BLE HID host library (library-local, registered from
# test/CMakeLists.txt so `ctest` runs them with the rest of the suite).
#
# Both binaries are plain host builds - no hardware, no ESP-IDF, no NimBLE:
#   * HidKeymapTest           - the HID Report Map parser and usage -> key mapping
#                               (src/HidKeymap.cpp, NimBLE-free by design).
#   * BleKeyboardHostIngestTest - the central-role code in src/BleKeyboardHost.cpp
#                               compiled with FREEINK_CAP_BLE_HID_HOST=1 against
#                               the fakes in stubs/ and FakeBle.{h,cpp}.
#
# Run from the repository root:
#   cmake -S test -B build/test && cmake --build build/test
#   ctest --test-dir build/test --output-on-failure -R 'HidKeymapTest|BleKeyboardHostIngestTest'

set(BLE_HID_DIR ${REPO_ROOT}/freeink-sdk/libs/network/BleKeyboardHost)

add_executable(HidKeymapTest
  HidKeymapTest.cpp
  ${BLE_HID_DIR}/src/HidKeymap.cpp
)

target_include_directories(HidKeymapTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs
  ${BLE_HID_DIR}/include
  ${BLE_HID_DIR}/src
)

target_link_libraries(HidKeymapTest PRIVATE
  crosspoint_test_common
  GTest::gtest_main
)

gtest_discover_tests(HidKeymapTest)

add_executable(BleKeyboardHostIngestTest
  BleKeyboardHostIngestTest.cpp
  FakeBle.cpp
  ${BLE_HID_DIR}/src/HidKeymap.cpp
  ${BLE_HID_DIR}/src/BleKeyboardHost.cpp
)

target_include_directories(BleKeyboardHostIngestTest PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs
  ${BLE_HID_DIR}/include
  ${BLE_HID_DIR}/src
)

target_link_libraries(BleKeyboardHostIngestTest PRIVATE
  crosspoint_test_common
  GTest::gtest_main
)

gtest_discover_tests(BleKeyboardHostIngestTest)
