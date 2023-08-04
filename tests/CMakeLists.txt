find_package ( Eigen3 3.3 REQUIRED NO_MODULE )
add_executable( mo-brtdp-tests tests_main.cpp ../src/parser.cpp )

target_include_directories( mo-brtdp-tests PRIVATE ../include )
target_link_libraries( mo-brtdp-tests Eigen3::Eigen )
