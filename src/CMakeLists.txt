find_package ( Eigen3 3.3 REQUIRED NO_MODULE )
 

add_library( prism-parser parser.cpp )
add_executable( mo-brtdp main.cpp
												 ../include/sea_treasure.cpp
											 	 ../include/resource_gathering.cpp
											 	 ../include/racetrack.cpp
												 ../include/frozen_lake.cpp )

target_include_directories( mo-brtdp PRIVATE ../include )
target_include_directories( prism-parser PRIVATE ../include )

target_link_libraries( prism-parser Eigen3::Eigen )
target_link_libraries( mo-brtdp prism-parser )
