cmake_minimum_required(VERSION 3.20)
project(esp32_ball_physics_tests LANGUAGES CXX)

enable_testing()

add_executable(lightsaber_tests lightsaber_test.cpp)
target_include_directories(lightsaber_tests PRIVATE ../../src)
target_compile_features(lightsaber_tests PRIVATE cxx_std_17)
if(MSVC)
  target_compile_options(lightsaber_tests PRIVATE /UNDEBUG)
else()
  target_compile_options(lightsaber_tests PRIVATE -UNDEBUG)
endif()
add_test(NAME lightsaber_tests COMMAND lightsaber_tests)

add_executable(threeway_tests threeway_test.cpp)
target_include_directories(threeway_tests PRIVATE ../../src)
target_compile_features(threeway_tests PRIVATE cxx_std_17)
if(MSVC)
  target_compile_options(threeway_tests PRIVATE /UNDEBUG)
else()
  target_compile_options(threeway_tests PRIVATE -UNDEBUG)
endif()
add_test(NAME threeway_tests COMMAND threeway_tests)

add_executable(physics_tests
    physics_test.cpp
    ../../src/physics.cpp
    ../../src/marbles.cpp
)
target_include_directories(physics_tests PRIVATE ../../src)
target_compile_features(physics_tests PRIVATE cxx_std_17)
add_test(NAME physics_tests COMMAND physics_tests)

add_executable(texture_tests
    texture_test.cpp
    ../../src/rolling_texture.cpp
)
target_include_directories(texture_tests PRIVATE ../../src)
target_compile_features(texture_tests PRIVATE cxx_std_17)
add_test(NAME texture_tests COMMAND texture_tests)

add_executable(detail_tests
    detail_test.cpp
    ../../src/rolling_detail.cpp
)
target_include_directories(detail_tests PRIVATE ../../src)
target_compile_features(detail_tests PRIVATE cxx_std_17)
add_test(NAME detail_tests COMMAND detail_tests)

add_executable(battery_runtime_tests battery_runtime_test.cpp)
target_include_directories(battery_runtime_tests PRIVATE battery_stubs ../../src)
target_compile_features(battery_runtime_tests PRIVATE cxx_std_17)
target_compile_definitions(battery_runtime_tests PRIVATE A7_BOARD A7_THREEWAY)
if(MSVC)
  target_compile_options(battery_runtime_tests PRIVATE /UNDEBUG)
else()
  target_compile_options(battery_runtime_tests PRIVATE -UNDEBUG)
endif()
add_test(NAME battery_runtime_tests COMMAND battery_runtime_tests)

add_executable(contact_tests contact_test.cpp)
target_include_directories(contact_tests PRIVATE ../../src)
target_compile_features(contact_tests PRIVATE cxx_std_17)
if(MSVC)
  target_compile_options(contact_tests PRIVATE /UNDEBUG)
else()
  target_compile_options(contact_tests PRIVATE -UNDEBUG)
endif()
add_test(NAME contact_tests COMMAND contact_tests)
