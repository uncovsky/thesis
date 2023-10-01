# include "models/environment.hpp"
# include "models/env_wrapper.hpp"
# include "solvers/brtdp.hpp"
# include "solvers/chvi.hpp"

# include <fstream>
# include <iostream>
# include <string>


struct ExplorationSettings {
    
/* global settings */
    size_t max_iterations;

    // bound distance on starting state to terminate solver
    double precision;

/* sampling settings */

    // min/max dephts of trajectory
    size_t min_trajectory, max_trajectory;

    // bound distance that is sufficient to end the trajectory earlier
    double bound_cutoff;

    // state / action heuristics
    StateSelectionHeuristic state_heuristic;
    ActionSelectionHeuristic action_heuristic; 
};



// have some environment, some settings of the solver 
