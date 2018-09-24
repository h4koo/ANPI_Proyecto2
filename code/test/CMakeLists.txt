find_package (Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

file(GLOB TEST_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.hpp)

add_executable (tester ${TEST_SRCS})
target_link_libraries (tester
                       anpi
                       ${Boost_FILESYSTEM_LIBRARY}
                       ${Boost_SYSTEM_LIBRARY}
                       ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_test(NAME tester COMMAND tester)
