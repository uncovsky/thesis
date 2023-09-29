#pragma once

#include <queue>
#include "models/env_wrapper.hpp"
#include "utils/eigen_types.hpp"
#include "utils/prng.hpp"

template < typename state_t, typename action_t, typename value_t >
class CHVIExactSolver{

    using EnvironmentHandle = EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t >;

    EnvironmentHandle env;
    std::set< state_t > reachable_states;

    std::vector< value_t > discount_params;

    void set_reachable_states() {
        std::queue< state_t > q;
        q.push( env.get_current_state() );
        reachable_states.insert( q.front() );

        while ( !q.empty() ) {

            state_t curr = q.front();
            q.pop();
        
            for ( const auto &act : env.get_actions( curr ) ) {
                for ( const auto &[ succ, _ ] : env.get_transition( curr, act ) ) {
                    if ( reachable_states.find( succ ) == reachable_states.end() ) {
                        q.push( succ );
                        reachable_states.insert( succ );
                    }
                }
            }
        }
    }

    void update_bounds( state_t s, action_t a ) {
        std::map< state_t, double > transitions = env.get_transition( s, a );

        Bounds< value_t > result;

        std::vector< Bounds< value_t > > successors;

        for ( const auto &[ succ, prob ] : transitions ) {
           successors.emplace_back( env.get_state_bound( succ ) );
           successors.back().multiply_bounds( prob );
        }

        result.sum_successors( successors );

        // result.nondominated();
        result.multiply_bounds( discount_params );
        result.shift_bounds( env.get_expected_reward( s, a ) );

        // get the lowest possible objective value and run the pareto operator
        /*
        auto [ ref_point, _ ] = env.min_max_discounted_reward();
        result.pareto( ref_point );
        */

        env.set_bound( s, a, std::move( result ) );
    }


public:
    CHVIExactSolver( EnvironmentHandle &&_env, 
                    const std::vector< value_t > discount_params ) :  
                                                                     env( std::move( _env ) )
                                                                   , reachable_states(  )
                                                                   , discount_params( discount_params ) {  }

    Bounds< value_t > solve( value_t precision ) {

        size_t sweeps = 0;
        state_t starting_state = std::get< 0 > ( env.reset( 0 ) );
        env.set_discount_params( discount_params );
        set_reachable_states();
        
        while ( env.get_state_bound( starting_state ).bound_distance() >= precision ){
            std::cout << "Sweep number: " << sweeps << ".\n";
            for ( const state_t &s : reachable_states ) {
                // initialize bound
                if ( sweeps == 0 ) { env.discover( s ); }

                // update all s,a pairs
                for ( const action_t &act : env.get_actions( s ) ) {
                    update_bounds( s, act );
                }
            }

            sweeps++;
        }

        env.write_statistics( false );
        auto start_bound = env.get_state_bound( starting_state );
        std::cout << "distance: " << start_bound.bound_distance( ) << std::endl;
        return start_bound;
    }

};
