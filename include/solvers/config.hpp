# pragma once

# include "utils/eigen_types.hpp"
# include <vector>
# include <chrono>

enum class ActionSelectionHeuristic { Hypervolume, 
                                      Pareto, 
                                      Uniform };

enum class OptimizationDirection { MAXIMIZE, 
                                   MINIMIZE };


template < typename value_t >
struct ExplorationSettings{

    // hausdorff distance required for termination
    double precision;

    // discount value
    value_t discount_param;

    /* optimization direction for each objective
     * this is used in the env wrapper class to multiply reward
     * by minus one if the objective is set to MINIMIZE 
     * 
     * if a largeer vector than the num of objs is specified, the 
     * extra directions are ignored, if a smaller one is, the missing ones 
     * are implicitly maximize */
    std::vector< OptimizationDirection > directions;

    // heuristic for action selection
    ActionSelectionHeuristic action_heuristic;

    // max trajectories sampled before termination
    //  ( or sweeps of chvi )
    size_t max_episodes;

    // max trajectory depth
    size_t max_depth;

    // if enabled, prints trajectories and debugging output while solving
    bool trace;

    // points to be used as initial lower/upper bounds on obj
    // if empty the bounds are initialized from min/max infinite discounted
    // reward.
    Point< value_t > lower_bound_init;
    Point< value_t > upper_bound_init;

    /* In some cases it is advantageous ( or even assumed ) that terminal
     * states are initialized with a different lower & upper bounds
     *
     * for examply in all episodic problems, it is advantageous to initialize
     * the terminal state of the MDP with U_0 = L_0 = 0, consider for example
     * SSP problems used by BRTDP 
     */

    Point< value_t > lower_bound_init_term;
    Point< value_t > upper_bound_init_term;
    
    // filename to output logs in
    std::string filename;

    // basic config for testing 2 objective benchmarks
    ExplorationSettings() : precision( 0.1 )
                          , discount_param( 0.9 )
                          , directions( { OptimizationDirection::MAXIMIZE, OptimizationDirection::MAXIMIZE } )
                          , action_heuristic( ActionSelectionHeuristic::Pareto )
                          , max_episodes( 2000 )
                          , max_depth( 1000 )
                          , trace( true )
                          , lower_bound_init()
                          , upper_bound_init() 
                          , lower_bound_init_term()
                          , upper_bound_init_term() 
                          , filename( "benchmark_test" ){ }
};
