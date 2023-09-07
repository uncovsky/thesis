# pragma once
#include <algorithm>
#include <stack> 
#include "models/env_wrapper.hpp"
#include "utils/eigen_types.hpp"
#include "utils/prng.hpp"


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

    size_t MIN_TRAJECTORY = 10, MAX_TRAJECTORY=100;
    enum class StateSelectionHeuristic { BRTDP, Uniform, Dynamics };
    enum class ActionSelectionHeuristic { Hypervolume, Pareto, Uniform };


    // selected heuristics to guide the search
    StateSelectionHeuristic state_heuristic = StateSelectionHeuristic::BRTDP;
    ActionSelectionHeuristic action_heuristic = ActionSelectionHeuristic::Pareto;

    // vector of gammas for each objective ( maximum of two objectives
    // supported as of now )
    std::vector< value_t > discount_params;


    bool trace = false;
    size_t max_iter = 10000;

    // trajectory cutoff ( stop when difference of bounds on successor is < )
    double delta_distance = 1e-12;

    /* helper functions for uniform sampling of actions/states
     * TODO: eliminate the need for uniform action, isntead utilize
     * uniform_index
     */
    size_t uniform_index( const std::vector< size_t > &indices ) {
        return indices[ gen.rand_int( 0, indices.size() ) ];
    }

    /*
     * ACTION HEURISTICS 
     */

    action_t uniform_action( const std::vector< action_t > &avail_actions ) {
        return avail_actions[ gen.rand_int( 0, avail_actions.size() ) ];
    }

    action_t uniform_action( const std::set< action_t > &avail_actions ) {
        auto idx =  gen.rand_int( 0, avail_actions.size() );

        return *std::next( avail_actions.begin(), idx );
    }

    
    /* pareto set evaluation heuristic from pareto q-learning:
     * 1) look at upper bounds of available actions
     * 2) select action uniformly from all actions that have >= 1 nondominated
     * vector in their upper bound across avail_actions 
     */
    action_t pareto_action( state_t s, const std::vector< action_t > &avail_actions ) {

        using VertexSet = typename std::vector< Point< value_t > >; 

        VertexSet nondominated;
        std::vector< VertexSet > bounds;

        std::set< action_t > pareto_actions;

        // copy vertices of upper bounds
        for ( const action_t &a : avail_actions ) {
            VertexSet vertices;

            // get vertices of respective upper bound and copy them into a set
            vertices = env.get_state_action_bound( s, a ).upper().get_vertices();
            nondominated.insert( nondominated.end(), vertices.begin(), vertices.end() );
            bounds.emplace_back( std::move( vertices ) );

        }

        remove_dominated_alt( nondominated );

        for ( const auto &point : nondominated ) {
            size_t i = 0;
            for ( const action_t &action : avail_actions ) {
                /* if Q value of this action contains a nondominated point, add
                 * it to candidates for action selection
                 */
                if ( ( pareto_actions.find( action ) == pareto_actions.end() ) &&
                       ( std::find( bounds[i].begin(), bounds[i].end(), point ) != bounds[i].end() ) ) {
                    pareto_actions.insert( action );
                }

                i++;
            }
        }

        // choose uniformly from these actions
        return uniform_action( pareto_actions );
        
    }
    /*
     * SUCCESSOR HEURISTICS 
     *
     * BRTDP heuristic for successor selection, uniformly samples from
     * successors maximizing weighted difference of their state bounds 
     * argmax ( delta( s, a , s' ) * ( dist_H( L_i( s' ), U_i( s' ) ) ) )
     */
    state_t bound_difference_state_selection( const std::map< state_t, double > &transitions ) {

        value_t max_diff(0);
        std::vector< value_t > diff_values;
        for ( const auto &[ s, prob ] : transitions ) {
            diff_values.push_back( env.get_state_bound( s ).bound_distance() * prob );
            max_diff = std::max( max_diff, diff_values.back() );
        }

        std::vector< size_t > maximising_indices;
        for ( size_t i = 0; i < transitions.size(); i++ ) {
            if ( diff_values[i] == max_diff ) { maximising_indices.push_back( i ); };
        }

        size_t chosen_index = uniform_index( maximising_indices );

        // return the chosen_index-th key ( state ) from the map
        return std::next( transitions.begin(), chosen_index )->first;
    }

    // picks action from avail actions based on specified heuristic
    action_t action_selection( state_t s, const std::vector< action_t > &avail_actions ) {

        if ( action_heuristic == ActionSelectionHeuristic::Uniform ) {
            return uniform_action( avail_actions );
        }
        
        // only these two supported right now
        return pareto_action( s, avail_actions );

    }


    state_t state_selection( const std::map< state_t, double > &transitions ) {

        if ( state_heuristic == StateSelectionHeuristic::BRTDP ) {
            return bound_difference_state_selection( transitions );
        }

        return gen.sample_distribution( transitions );
    }
                               
    /*
     * samples an MDP trajectory using specified action/successor heuristics
     * and precision, initializing newly encountered state action bounds
     */
    TrajectoryStack sample_trajectory( const std::vector< value_t > &max_value,
                                                          value_t precision ) {
        
        std::stack< std::pair< action_t, state_t > > trajectory;

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

            // if debug output is turned on, output details of trajectory
            if ( trace )
                std::cout << iter 
                          << " : Selected action " << action 
                          << " successor state " << state << "\n";

            /* check termination ( whether gamma^iter * max_value is < precision )
             * then we can safely stop the trajectory ( cut off the tail )
             * TODO: is this the correct stopping condition? */
            
            // discount_copy = discount_params^(iter+1) * max_value
            multiply( discount_copy, discount_params );

            auto successor_bound = env.get_state_bound( state );

            if ( ( iter > MAX_TRAJECTORY ) || 
                 ( successor_bound.bound_distance() ) < delta_distance ) { 
                    break; 
            }
            // if all components are < precision, terminate, but iff reached
            // min iterations
            if ( iter > MIN_TRAJECTORY ) { terminated = true; }
            for ( value_t val : discount_copy )  {
               terminated &= val < precision; 
            }
            
            iter++;

        }

        return trajectory;
        
    }

    /* BRTDP update of L(s,a) and U(s, a) */
    void update_bounds( state_t s, action_t a ) {
        std::map< state_t, double > transitions = env.get_transition( s, a );

        Bounds< value_t > result;

        for ( const auto &[ succ, prob ] : transitions ) {
           Bounds< value_t > succ_bound = env.get_state_bound( succ );
           succ_bound.multiply_bounds( prob );
           result.sum_bounds( succ_bound );
        }

        result.nondominated();
        result.multiply_bounds( discount_params );
        result.shift_bounds( env.get_expected_reward( s, a ) );

        // get the lowest possible objective value and run the pareto operator
        auto [ ref_point, _ ] = env.min_max_discounted_reward();
        result.pareto( ref_point );

        env.set_bound( s, a, std::move( result ) );
    }

    /* executes BRTDP updates for the whole sampled trajectory */
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

    /* the main BRTDP solver function, samples trajectories and updates bounds
     * until the distance of starting state bounds is less than specified
     * precision, uses discount values specified in constructor 
     * outputs the bound ( pareto objects ) into text files, sampled
     * trajectories during execution and number of discovered states at the end
     */
    Bounds< value_t > solve( value_t precision ) {

        state_t starting_state = std::get< 0 > ( env.reset( 0 ) );

        env.set_discount_params( discount_params );
        
        /* get max ( theoretically ) possible value of the objective
         * i.e infinite series W_max + \gamma * W_max + ... 
         * where W_max is a vector of max rewards and \gamma discount_param
         * vector
         */
        auto [ minimal_value , maximal_value ] = env.min_max_discounted_reward();

        size_t episode = 0;

        while ( env.get_state_bound( starting_state ).bound_distance() >= precision ) {
            std::cout << "episode #" << episode << "\n";
            TrajectoryStack trajectory = sample_trajectory( maximal_value, precision );
            update_along_trajectory( trajectory, starting_state );
            std::cout << env.get_state_bound( starting_state );
            episode++;

            if ( episode > max_iter ) { break; }
        }

        auto start_bound = env.get_state_bound( starting_state );

        std::cout << "distance: " << start_bound.bound_distance( ) << std::endl;
        auto vertices = start_bound.lower().get_vertices();

        env.write_statistics();
        std::sort( start_bound.lower().get_vertices().begin(), start_bound.lower().get_vertices().end() );
        std::sort( start_bound.upper().get_vertices().begin(), start_bound.upper().get_vertices().end() );
        start_bound.lower().write_to_file( "starting_lower.txt" );
        start_bound.upper().write_to_file( "starting_upper.txt" );

        return start_bound;
    }

};
