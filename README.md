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
in the source file.

## Using the tool

No CLI for now, but running the solvers on an MDP of your choice can be done as
follows.

note that PRISM needs to be installed, see [github](https://github.com/prismmodelchecker/prism).

exporting the transition


1) Export the explicit transition files, this can be done using PRISM by
running

$ prism/bin/prism modelpath --exporttrans out.tra --exporttransrewards out.trew

once in the root directory of prism.

2) Locate the ID of the starting state, this can be done by 
$ prism/bin/prism modelpath --exportlabels labels.txt

and finding the index associated to the label "init".


3) Run the solvers on these transition files. An example setup is given in
../include/eval_example.hpp



