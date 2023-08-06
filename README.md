#	Code repository for multi objective VI / BRTDP.


### Dependencies: cmake 3.12, Eigen 3.3 (sparse matrices)

### Installing the eigen library on Debian/Ubuntu can be done by running:
	 $ sudo apt-get install libeigen3-dev

### Afterwards simply run:
	 $ mkdir build
	 $ cd build
	 $ cmake ..
	 $ make

###### The final binary is located at build/mo-brtdp, tests at build/mo-brtdp-tests

### Implemented so far
	Env interface, Sparse MDP model + all interaction/statistics
 	2D pareto curve representation + associated operations
  	brtdp algorithm implementing the VI updates ( + heuristics for successor state sampling, pareto criterion for action selection )
   	parser for PRISM format of explicit transition & reward files

### TODO, issues:
	test properly, fix some bugs in pareto related operations
 	add more heuristics, optimize
 	find some relevant benchmarks, neither prism/storm checkers have discounted reward sum objectives
  granularity - filter points, updates are intractable for even small mdps ( if large discount parameters are considered )
