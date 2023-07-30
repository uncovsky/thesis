# pragma once
#include "models/env_wrapper.hpp"
#include "utils/prng.hpp"
#include <stack> 


template < typename state_t, typename action_t, typename value_t >
class BRTDPSolver{

    using EnvironmentHandle = EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t >;

    // implicitly starts from inital state s'
    using TrajectoryStack = std::stack< std::pair< action_t, state_t > >;

    // handle on given mdp model
    EnvironmentHandle env;

    // prng for selecting actions/successors
    PRNG gen;


    /* TODO: move these into a specialized exploration settings class ? */

    enum class StateSelectionHeuristic { BRTDP, Uniform, Dynamics };
    enum class ActionSelectionHeuristic { Hypervolume, Pareto, Uniform };


    // selected heuristics to guide the search
    StateSelectionHeuristic state_heuristic = StateSelectionHeuristic::Uniform;
    ActionSelectionHeuristic action_heuristic = ActionSelectionHeuristic::Uniform;

    // precision that terminates the search ( difference of bounds )
    value_t precision;

    // vector of gammas for each objective ( maximum of two objectives
    // supported as of now )
    std::vector< value_t > discount_params;


    action_t uniform_action( const std::vector< action_t > avail_actions ) {
        return avail_actions[ gen.rand_int( 0, avail_actions.size() ) ];
    }

    
    /* pareto set evaluation heuristic from pareto q-learning:
     * 1) look at upper bounds of available actions
     * 2) select action uniformly from all actions that have >= 1 nondominated
     * vector in their upper bound across avail_actions 
     */
    /*
    action_t pareto_action( state_t s, const std::vector< action_t > &avail_actions ) const {

        std::vector< std::unique_ptr< std::set< value_t > > > bounds;

        // copy vertices of upper bounds
        for ( action_t a : avail_actions ) {
            std::set< value_t > vertices;

            // get vertices of respective upper bound
            vertices = env.get_state_action_bound( s, a ).upper().get_vertices();
            bounds.push_back( std::make_unique( vertices ) );
        }

        // filter out dominated vertices
        for ( size_t bound = 0; bound < bounds.size(); bound++ ) {

            auto& current_set = ( *bounds[bound] );

            // for every element in this set
            for ( auto it = current_set.begin(); it < current_set.end(); ) {

                bool dominated = false;

                // check if element is dominated, and remove vectors it
                // dominates from other sets
                for ( size_t other = bound + 1; other < bounds.size(); other++ ){
                    if ( pareto_check_remove( *it, *bounds[other] ) )
                        dominated = true;
                }

                // if current vector was dominated by vector of other action, remove
                if ( dominated ) { it = current_set.remove( it ); }
                else { it++; }
            }

        }

        std::vector< action_t > pareto_actions;

        // add those actions that have a nondominated vector result
        for ( size_t i = 0; i < avail_actions.size(); i++ ) {
            if ( !( *bounds[ i ] ).empty() ) {
                pareto_actions.push_back( avail_actions[i] );
            }

        }

        // choose uniformly from these actions
        return uniform_action( pareto_actions );
        
    }


    // right now only 2d supported, so area_action would be a more fitting name
    // hypervolume heuristic from pareto q learning paper
    action_t hypervolume_action( state_t s, const std::vector< action_t > &avail_actions ) {
        return 0;
    }
    */


    action_t action_selection( state_t s, const std::vector< action_t > &avail_actions ) {
        return uniform_action( avail_actions );
        /*
        }

        else if ( action_heuristic == ActionSelectionHeuristic::Pareto ) {
            return pareto_action( s, avail_actions );

        }

        return hypervolume_action( s, avail_actions );
        */
    }


    // so far just dynamics
    state_t state_selection( const std::map< state_t, double > &transitions ) {
        return gen.sample_distribution( transitions );
    }
                               

    TrajectoryStack sample_trajectory( size_t limit ) {
        
        std::stack< std::pair< state_t, action_t > > trajectory;
        auto [ state, _ , terminated ] = env.reset( 0, false );


        size_t iter = 0;
        terminated = false;
    
        while ( ( !terminated ) && ( iter < limit ) ) {

            env.discover( state );

            // select action in this state
            std::vector< action_t > actions = env.get_actions( state );
            action_t action = action_selection( state, actions );

            // get successor state
            auto transitions = env.get_transition( state, action );
            state = state_selection( transitions );


            std::cout << iter << " : Selected action " << action << " successor state " << state << "\n";

            terminated = env.is_terminal_state( state );

            trajectory.push( { action, state } );
            iter++;

        }

        // handle terminal state and select one final action in it ( to trigger
        // an update on the state itself ) 
        if ( terminated ) {
            std::vector< action_t > actions = env.get_actions( state );
            action_t action = action_selection( state, actions );
            trajectory.push( { action, state } );
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

    void update_along_trajectory( TrajectoryStack& trajectory, state_t starting_state ) {
        while ( !trajectory.empty() ) {
            auto [ a, successor ] = trajectory.top();
            trajectory.pop();
            state_t s = trajectory.empty() ? starting_state : trajectory.top().second;
            update_bounds( s, a );
        }
        
    }


public:

    // need to move env since it has ownership of solver resources ( bounds )
    BRTDPSolver( EnvironmentHandle &&_env, 
                 const std::vector< value_t > discount_params,
                 value_t precision ) :  
                                        env( std::move( _env ) ),
                                        gen( ), 
                                        discount_params( discount_params ),
                                        precision( precision ) { }

    void solve( size_t episode_limit, size_t trajectory_limit ) {

        size_t starting_state = std::get< 0 > ( env.reset( 0 ) );
        env.set_discount_params( discount_params );
        size_t episode = 0;

        while ( episode < episode_limit ){
            std::cout << "episode #" << episode << "\n";
            TrajectoryStack trajectory = sample_trajectory( trajectory_limit );
            size_t i = 1;
            update_along_trajectory( trajectory, starting_state );
            episode++;
        }

        std::cout << starting_state << std::endl;

        Bounds< value_t >  start_bound = env.get_state_action_bound( 0, 0 );
        Bounds< value_t > action_bound = env.get_state_action_bound( 4, 0 );

        std::cout << "distance: " << start_bound.bound_distance( env.reward_range().first ) << std::endl;

        env.write_statistics();
        start_bound.lower().write_to_file( "starting_lower.txt" );
        start_bound.upper().write_to_file( "starting_upper.txt" );
    }

};
