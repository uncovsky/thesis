# pragma once
#include "models/env_wrapper.hpp"
#include "utils/prng.hpp"
#include <stack> 


template < typename state_t, typename action_t, typename reward_t >
class BRTDPSolver{

    // change double to bounds object afterwards
    using Environment = EnvironmentWrapper< state_t, action_t, reward_t, double >;

    // implicitly starts from inital state s'
    using TrajectoryStack = std::stack< std::pair< action_t, state_t > >;

    Environment env;
    PRNG gen;

    std::vector< double > discount_params;

    enum class StateSelectionHeuristic { Hypervolume, Pareto, Uniform };
    enum class ActionSelectionHeuristic { BRTDP, Uniform, RoundRobin };

    double precision;


    TrajectoryStack sample_trajectory( size_t limit ) {
        
        std::stack< std::pair< state_t, action_t > > trajectory;
        auto [ state, rew, terminated ] = env.reset( 0, false );


        size_t iter = 0;
    
        while ( ( !terminated ) && ( iter < limit ) ) {

            std::vector< action_t > actions = env.get_actions( state );

            // todo implement action selection heuristics (analogous to PQL) 
            // for now just pick first avail one
            action_t action = actions[0];


            // todo implement successor heuristics, for now just env dynamics
            auto res = env.step( action );

            state = std::get< 0 > ( res );
            terminated = std::get< 2 > ( res );

            trajectory.push_back( { state, action } );

        }

        return trajectory;
        
    }

    void update_along_trajectory( TrajectoryStack& trajectory ) {

        state_t s, successor;
        action_t a;

        while ( !trajectory.empty() ) {

        }
        
    }

public:
   BRTDPSolver( Environment &&env, const std::vector< double > discount_params ) : env( std::move( env ) ) ,
                                    gen(  ) ,
                                    discount_params( discount_params ) {}

};
