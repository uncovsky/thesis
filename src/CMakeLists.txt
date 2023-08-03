find_package ( Eigen3 3.3 REQUIRED NO_MODULE )
 

add_executable( mo-brtdp main.cpp parser.cpp )

target_include_directories( mo-brtdp PRIVATE ../include )
target_link_libraries( mo-brtdp Eigen3::Eigen )
