#	Code repository for multi objective VI / BRTDP.
## Dependencies: c++17, cmake 3.12, Eigen 3.3 (sparse matrices), gnuplot

## Installing the eigen library on Debian/Ubuntu can be done by running:
	 $ sudo apt-get install libeigen3-dev gnuplot

## Afterwards simply run:
	 $ mkdir build
	 $ cd build
	 $ cmake ..
	 $ make

###### The final binary is located at build/mo-brtdp, tests at build/mo-brtdp-tests

## Current features

* General MDP class implemented using sparse matrices.

* Support for maximization or minimization of 2 objectives.


* BRTDP algorithm implementation that uses both of the above, keeps lower and
* upper bounds on objective for each state/action pair - pareto curves. Accepts
discount parameters / precision on hausdorff distance of starting bound. 

* Heuristics for BRTDP for action & successor sampling

* Parser for explicit PRISM files with some additional features that can be used to load
explicit models and use them to build the MDP for solving.

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

No CLI for now, but running the solver on an MDP of your choice can be done as
follows.



