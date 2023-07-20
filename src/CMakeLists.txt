find_package ( Eigen3 3.3 REQUIRED NO_MODULE )
 

# just build from main.cpp for the time being 
add_executable( mo-brtdp main.cpp )

target_include_directories( mo-brtdp PRIVATE ../include )
target_link_libraries( mo-brtdp Eigen3::Eigen )
