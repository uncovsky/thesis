#	Code repository for multi objective VI / BRTDP.
## Dependencies: c++17, cmake 3.12, Eigen 3.3 (sparse matrices), gnuplot (visualization)

## Installing the dependencies on Debian/Ubuntu can be done by running:
	 $ sudo apt-get install libeigen3-dev gnuplot cmake

## Afterwards simply run:
	 $ mkdir build
	 $ cd build
	 $ cmake ..
	 $ make


## Running the benchmarks
After building, the final binary is located at build/mo-brtdp. 

The binary runs all the benchmarks 5 times for SCHVI, SBPCA-DB and SBPCA-PA,
outputting the results and statistics in out/.

The code used for evaluation is located in include/evaluation.hpp.

## Parser

The file format is described in depth here: [PRISM Format Description](https://www.prismmodelchecker.org/manual/Appendices/ExplicitModelFiles)
, see Transition ( .tra ) files and Transition Reward ( .trew ) files. 

It is also possible to supply more than one dimension of rewards in one file for any transition.

* Original PRISM format:
	+ 1 0 2 6 - transition from 1 under action 0 to state 2 has reward 6

* Additionally supported:
	+ 1 0 2 6 3 - transition has reward (6, 3)

More details on the parser ( all the things that are checked, etc. ) are given
in the source file src/parser.cpp

## Using the tool

No CLI for now, but running the solvers on an MDP of your choice can be done as
follows.

note that PRISM needs to be installed, see [github](https://github.com/prismmodelchecker/prism).


* Export the explicit transition files, this can be done using PRISM by
running the following in the root directory of PRISM

    $ prism/bin/prism MODEL_PATH --exporttrans out.tra --exporttransrewards out.trew


* Locate the ID of the starting state, this can be done by 
    $ prism/bin/prism MODEL_PATH --exportlabels labels.txt
		 , and looking at the index associated with the label "init", which is usually 0


* Run the solvers on these transition files. An example setup is given in include/eval_example.hpp


## Visualization

The results can be visualized by utilizing the gnuplot script in
out/visualization

This can be done by running $ gnuplot -p plotscript.p, which visualizes the
contents of the file curve.txt in the directory.

