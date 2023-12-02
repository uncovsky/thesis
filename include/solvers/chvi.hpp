#pragma once

# include <queue>
# include "models/env_wrapper.hpp"
# include "solvers/config.hpp"
# include "utils/eigen_types.hpp"
# include "utils/prng.hpp"

template < typename state_t, typename action_t, typename value_t >
class CHVIExactSolver{

    using EnvironmentHandle = EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t >;
    ExplorationSettings< value_t > config;

    EnvironmentHandle env;

    std::set< state_t > reachable_states;

    // bfs to find all reachable states
    void set_reachable_states() {
        std::queue< state_t > q;
        q.push( env.get_current_state() );
        reachable_states.insert( q.front() );

        while ( !q.empty() ) {

            state_t curr = q.front();
            // init bounds
            env.discover( curr );
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

        if ( config.trace ){
            std::cout << "CHVI - reachable states:\n";

            for ( const auto& state : reachable_states ) {
                std::cout << state << "\n";
                for ( const auto act : env.get_actions( state ) ) {
                    std::cout << "  Action " << act << ".\n";
                    for ( const auto &[ succ, prob ] : env.get_transition( state, act ) ) {
                        std::cout << "Successor: " << succ << " with p. " << prob << ".\n";
                    }

                }
            }
            std::cout << "Total reachable states: " << reachable_states.size() << ".\n";
        }
    }


public:
    CHVIExactSolver( EnvironmentHandle &&_env, 
                     const ExplorationSettings< value_t >& config) :  
                                                                     env( std::move( _env ) )
                                                                   , reachable_states() 
                                                                   , config( config )   {  }

    VerificationResult< value_t > solve() {

        auto start_time = std::chrono::steady_clock::now();
        std::ofstream logs( config.filename + "_chvi-logs.txt" );
        std::ofstream result( config.filename + "_chvi-result.txt" );

        size_t sweeps = 0;
        reachable_states.clear();
        state_t starting_state = std::get< 0 > ( env.reset( 0 ) );

        env.set_config( config );
        set_reachable_states();
        
        while ( env.get_state_bound( starting_state ).hausdorff_distance() >= config.precision ){

            if ( config.trace ) {
                logs << "Sweep number: " << sweeps << ".\n";
                std::cout << "Sweep number: " << sweeps << ".\n";
                std::cout <<  env.get_state_bound( starting_state ).hausdorff_distance() << ".\n";
                std::cout <<  env.get_state_bound( starting_state ) << ".\n";
            }

            for ( const state_t &s : reachable_states ) {
                // initialize bound
                if ( sweeps == 0 ) { env.discover( s ); }

                // update all s,a pairs
                for ( const action_t &act : env.get_actions( s ) ) {
                    env.update_bound( s, act );
                }

                env.update_bound( s );
            }

            sweeps++;
            if ( ( config.max_episodes > 0 ) && ( sweeps >= config.max_episodes ) )  { break; }

            auto finish_time = std::chrono::steady_clock::now();
            std::chrono::duration< double > exec_time = finish_time - start_time;

            if ( exec_time.count() > config.max_seconds ) { break; }
        }

        auto finish_time = std::chrono::steady_clock::now();
        auto start_bound = env.get_state_bound( starting_state );
        std::chrono::duration< double > exec_time = finish_time - start_time;

        VerificationResult< value_t > res{  env.get_update_num() // num of updates
                                       , start_bound.hausdorff_distance() < config.precision // bool converged
                                       , start_bound 
                                       , exec_time.count()
                                       , env.num_states_explored() }; // num of explored states
                                        
        return res;
    }

};
