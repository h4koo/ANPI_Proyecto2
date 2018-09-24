find_package (Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include ${Boost_INCLUDE_DIRS})

file(GLOB BM_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.hpp)

add_executable (benchmark ${BM_SRCS})
target_link_libraries (benchmark
                       anpi
                       python2.7
                       ${Boost_FILESYSTEM_LIBRARY}
                       ${Boost_SYSTEM_LIBRARY}
                       ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_test(NAME benchmark COMMAND benchmark)
