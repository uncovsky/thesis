# pragma once

# include "utils/eigen_types.hpp"
# include <vector>

/* enums and config for environment and solvers */
enum class StateSelectionHeuristic { BRTDP, 
                                     Dynamics };

enum class ActionSelectionHeuristic { Hypervolume, 
                                      Pareto, 
                                      Uniform };


template < typename value_t >
struct ExplorationSettings{

    // hausdorff distance required for termination
    double precision;

    // discount values for each objective
    std::vector< value_t > discount_params;

    // heuristics for action & successor selection
    StateSelectionHeuristic state_heuristic;
    ActionSelectionHeuristic action_heuristic;

    // max trajectories sampled before termination
    size_t max_episodes;

    // max trajectory length
    size_t max_trajectory;

    // if enabled, prints trajectories and debugging output while solving
    bool trace;

    // points to be used as initial lower/upper bounds on obj
    // if empty the bounds are initialized from min/max infinite discounted
    // reward.
    Point< value_t> lower_bound_init;
    Point< value_t> upper_bound_init;
    
    // filename to output logs in
    std::string filename;

    // basic config for testing
    ExplorationSettings() : precision( 0.1 )
                          , discount_params( { 0.9, 0.9 } )
                          , state_heuristic( StateSelectionHeuristic::BRTDP )
                          , action_heuristic( ActionSelectionHeuristic::Hypervolume )
                          , max_episodes( 2000 )
                          , max_trajectory( 100 )
                          , trace( true )
                          , lower_bound_init()
                          , upper_bound_init() 
                          , filename( "test" ){ }
};
