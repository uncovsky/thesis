# pragma once
#include "models/env_wrapper.hpp"
#include "utils/prng.hpp"
#include <stack> 


template < typename state_t, typename action_t, typename value_t >
class BRTDPSolver{

    using Environment = EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t >;

    // implicitly starts from inital state s'
    using TrajectoryStack = std::stack< std::pair< action_t, state_t > >;

    Environment env;
    state_t starting_state;

    PRNG gen;

    std::vector< value_t > discount_params;

    enum class StateSelectionHeuristic { Hypervolume, Pareto, Uniform };
    enum class ActionSelectionHeuristic { BRTDP, Uniform, RoundRobin };

    value_t precision;

    TrajectoryStack sample_trajectory( size_t limit ) {
        
        std::stack< std::pair< state_t, action_t > > trajectory;
        auto [ state, _ , terminated ] = env.reset( 0, false );


        size_t iter = 0;
        terminated = false;
    
        while ( ( !terminated ) && ( iter < limit ) ) {

            std::vector< action_t > actions = env.get_actions( state );


            // todo implement action selection heuristics (analogous to PQL) 
            // for now just pick first avail one
            action_t action = actions[0];


            // todo implement successor heuristics, for now just env dynamics
            auto res = env.step( action );


            state = std::get< 0 > ( res );

            std::cout << iter << " : Selected action " << action << " successor state " << state << "\n";

            terminated = std::get< 2 > ( res );

            trajectory.push( { action, state } );
            iter++;

        }

        return trajectory;
        
    }

    void update_bounds( state_t s, action_t a ) {
        std::map< state_t, double > transitions = env.get_transition( s, a );

        Bounds< value_t > result;

        for ( const auto &[ succ, prob ] : transitions ) {
           Bounds< value_t > succ_bound = env.get_state_bound( succ );
           succ_bound.multiply_bounds( prob );
           result.sum_bounds( succ_bound );
        }

        result.multiply_bounds( discount_params );
        result.shift_bounds( env.get_expected_reward( s, a ) );
        env.set_bound( s, a, std::move( result ) );
    }

    void update_along_trajectory( TrajectoryStack& trajectory ) {
        while ( !trajectory.empty() ) {
            auto [ a, successor ] = trajectory.top();
            trajectory.pop();
            state_t s = trajectory.empty() ? starting_state : trajectory.top().second;
            update_bounds( s, a );
        }
        
    }


public:

    BRTDPSolver( const Environment &_env, 
                 const std::vector< value_t > discount_params,
                 value_t precision ) :  
                                        env( _env ),
                                        gen( ), 
                                        discount_params( discount_params ),
                                        precision( precision )
                {
                    starting_state = std::get< 0 > ( env.reset( 0 ) ); // assuming one starting state
                }

    void solve( size_t episode_limit, size_t trajectory_limit ) {

        size_t episode = 0;

        while ( episode < episode_limit ){
            std::cout << "episode #" << episode << "\n";
            TrajectoryStack trajectory = sample_trajectory( trajectory_limit );
            size_t i = 1;
            update_along_trajectory( trajectory );

            episode++;
        }

        auto start_bound = env.get_state_bound( starting_state );

        std::cout << "distance: " << start_bound.bound_distance( env.reward_range().first ) << std::endl;

        env.write_statistics();
        start_bound.lower().write_to_file( "starting_lower.txt" );
        start_bound.upper().write_to_file( "starting_lower.txt" );
    }

};
