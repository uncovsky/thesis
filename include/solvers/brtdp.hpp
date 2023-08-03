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


    /* TODO: move these into a specialized exploration settings class, along
     * with precision etc. ? */

    enum class StateSelectionHeuristic { BRTDP, Uniform, Dynamics };
    enum class ActionSelectionHeuristic { Hypervolume, Pareto, Uniform };


    // selected heuristics to guide the search
    StateSelectionHeuristic state_heuristic = StateSelectionHeuristic::BRTDP;
    ActionSelectionHeuristic action_heuristic = ActionSelectionHeuristic::Uniform;

    // vector of gammas for each objective ( maximum of two objectives
    // supported as of now )
    std::vector< value_t > discount_params;

    // TODO: eliminate the need for uniform action, isntead utilize
    // uniform_index
    action_t uniform_index( const std::vector< size_t > &indices ) {
        return indices[ gen.rand_int( 0, indices.size() ) ];
    }

    action_t uniform_action( const std::vector< action_t > &avail_actions ) {
        return avail_actions[ gen.rand_int( 0, avail_actions.size() ) ];
    }

    
    /* pareto set evaluation heuristic from pareto q-learning:
     * 1) look at upper bounds of available actions
     * 2) select action uniformly from all actions that have >= 1 nondominated
     * vector in their upper bound across avail_actions 
     * TODO: fix segfault
     */
    action_t pareto_action( state_t s, const std::vector< action_t > &avail_actions ) {

        // TODO: probably a global typedef of Point in eigen_types 
        // would be helpful to change across all files, 
        using VertexSet = typename std::vector< std::vector< value_t > >; 
        std::vector< std::unique_ptr< VertexSet > > bounds;

        // copy vertices of upper bounds
        for ( action_t a : avail_actions ) {
            VertexSet vertices;

            // get vertices of respective upper bound and copy them into a set
            vertices = env.get_state_action_bound( s, a ).upper().get_vertices();
            bounds.push_back( std::make_unique< VertexSet > ( std::move( vertices ) ) );
        }

        // filter out dominated vertices
        for ( size_t bound = 0; bound < bounds.size(); bound++ ) {

            auto& current_set = ( *bounds[bound] );

            for ( auto it = current_set.begin(); it != current_set.end(); ) {

                bool dominated = false;

                // check if element is dominated in other, and remove vectors it
                // dominates from other
                for ( size_t other = bound + 1; other < bounds.size(); other++ ){
                    if ( pareto_check_remove( *it, *bounds[other] ) )
                        dominated = true;
                }

                // if current vector was dominated by vector of other action, remove
                if ( dominated ) { it = current_set.erase( it ); }
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

    action_t action_selection( state_t s, const std::vector< action_t > &avail_actions ) {

        if ( action_heuristic == ActionSelectionHeuristic::Uniform ) {
            return uniform_action( avail_actions );
        }
        
        // only these two supported right now
        return pareto_action( s, avail_actions );

    }

    state_t bound_difference_state_selection( const std::map< state_t, double > &transitions ) {

        auto [ min_value, _ ] = env.min_max_discounted_reward();

        value_t max_diff(0);
        std::vector< value_t > diff_values;
        for ( const auto &[ s, prob ] : transitions ) {
            diff_values.push_back( env.get_state_bound( s ).bound_distance( min_value ) * prob );
            max_diff = std::max( max_diff, diff_values.back() );
        }

        std::vector< size_t > maximising_indices;
        for ( size_t i = 0; i < transitions.size(); i++ ) {
            if ( diff_values[i] == max_diff ) { maximising_indices.push_back( i ); };
        }

        size_t chosen_index = uniform_index( maximising_indices );

        return std::next( transitions.begin(), chosen_index )->first;
    }

    state_t state_selection( const std::map< state_t, double > &transitions ) {

        if ( state_heuristic == StateSelectionHeuristic::BRTDP ) {
            return bound_difference_state_selection( transitions );
        }

        return gen.sample_distribution( transitions );
    }
                               

    TrajectoryStack sample_trajectory( const std::vector< value_t > &max_value,
                                                          value_t precision ) {
        
        std::stack< std::pair< state_t, action_t > > trajectory;

        std::vector< value_t > discount_copy( max_value );

        auto [ state, _ , terminated ] = env.reset( 0, false );
        terminated = false;

        size_t iter = 0;

        while ( !terminated ) {

            // mark current state and initialize its default bounds
            env.discover( state );

            // select action in this state
            std::vector< action_t > actions = env.get_actions( state );
            action_t action = action_selection( state, actions );

            // get successor state
            auto transitions = env.get_transition( state, action );
            state = state_selection( transitions );

            trajectory.push( { action, state } );

            std::cout << iter 
                      << " : Selected action " << action 
                      << " successor state " << state << "\n";

            /* check termination ( whether gamma^iter * max_value is < precision )
             * then we can safely stop the trajectory ( cut off the tail )
             * TODO: is this the correct stopping condition? */
            
            // discount_copy = discount_params^(iter+1) * max_value
            multiply( discount_copy, discount_params );

            terminated = true;

            // if all components are < precision, terminate
            for ( value_t val : discount_copy )  {
               terminated &= val < precision; 
            }
            
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
                 const std::vector< value_t > discount_params ) :  
                                        env( std::move( _env ) ),
                                        gen( ), 
                                        discount_params( discount_params ) {  }

    // returns objective bounds on the initial state
    Bounds< value_t > solve( value_t precision ) {

        size_t starting_state = std::get< 0 > ( env.reset( 0 ) );

        env.set_discount_params( discount_params );
        
        /* get max ( theoretically ) possible value of the objective
         * i.e infinite series W_max + \gamma * W_max + ... 
         * where W_max is a vector of max rewards and \gamma discount_param
         * vector
         */
        auto [ minimal_value , maximal_value ] = env.min_max_discounted_reward();

        size_t episode = 0;

        while ( env.get_state_bound( starting_state ).bound_distance( minimal_value ) >= precision ) {
            std::cout << "episode #" << episode << "\n";
            TrajectoryStack trajectory = sample_trajectory( maximal_value, precision );
            update_along_trajectory( trajectory, starting_state );
            std::cout << env.get_state_bound( starting_state ).bound_distance( minimal_value ) << std::endl;
            episode++;
        }

        auto start_bound = env.get_state_bound( starting_state );

        std::cout << "distance: " << start_bound.bound_distance( minimal_value ) << std::endl;
        auto vertices = start_bound.lower().get_vertices();

        env.write_statistics();
        std::sort( start_bound.lower().get_vertices().begin(), start_bound.lower().get_vertices().end() );
        std::sort( start_bound.upper().get_vertices().begin(), start_bound.upper().get_vertices().end() );
        start_bound.lower().write_to_file( "starting_lower.txt" );
        start_bound.upper().write_to_file( "starting_upper.txt" );

        for ( size_t i = 0; i < vertices.size(); i++ ) {
            for ( size_t it = i + 1; it < vertices.size(); it++ ){
                assert( vertices[i] != vertices[it] );
            }

        }


        return start_bound;
    }

};
