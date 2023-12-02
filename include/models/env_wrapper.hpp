#pragma once

#include <algorithm>
#include <memory>
#include <sstream>
#include <set>
#include "geometry/polygon.hpp"
#include "models/environment.hpp"
#include "solvers/config.hpp"
#include "utils/eigen_types.hpp"
#include "utils/prng.hpp"

/* this class is used to interact with the underlying environment, recording
 * statistics, simulation, logging, etc.
 * reward_t to the actual reward type ( so std::vector< double > etc. )
 * while value_t will be equal to the type used to represent the reward
 * components for the underlying reward, so for example double 
 */

template < typename state_t, typename action_t, typename reward_t , typename value_t >
class EnvironmentWrapper{
    
    using bounds_ptr = std::unique_ptr< Bounds< value_t > >;

    Environment< state_t, action_t, reward_t > *env;


    /* it is possible to provide more precise initial bounds
     * before the interaction begins, see set config and solvers/config.hpp
     *
     * 
     * if these are empty, maximum/minimum reward in each dimension will be
     * fetched from the environment, using the reward_range() method and 
     * the largest ( theoretically possible ) discounted reward will be used
     *
     * every state shares the same initial bound values, this could be expanded
     * later ( initializing procedures, like the dijkstra sweep in brtdp / etc )
     */

    ExplorationSettings< value_t > config;

    /* track update count for every state */
    std::map< state_t, size_t > update_count;
    std::map< std::tuple< state_t, action_t >, bounds_ptr > state_action_bounds;
    std::map< state_t, bounds_ptr > state_bounds;

public:

    EnvironmentWrapper() : env( nullptr ), 
                           update_count(), 
                           state_action_bounds() {}
    EnvironmentWrapper( Environment< state_t, action_t, reward_t > *env ) : env( env ), 
                                                                            update_count(), 
                                                                            state_action_bounds() {}

    using Observation = typename Environment< state_t, action_t, reward_t > :: Observation;


    /* interaction with the underlying environment */
    state_t get_current_state() const {
        return env->get_current_state();
    }


    std::vector< action_t > get_actions() const {
        return env->get_actions( get_current_state() );
    }


    std::map< state_t, double > get_transition( state_t state, action_t action ) const {
        return env->get_transition( state, action );
    }


    std::vector< action_t > get_actions( state_t state ) const {
        return env->get_actions( state );
    }


    reward_t get_expected_reward( state_t s, action_t a ) {
        reward_t rew_vec = env->get_reward( s, a );
        for ( size_t i = 0; i < std::min( config.directions.size(), rew_vec.size() ) ; i++ ) {
            if ( config.directions[i] == OptimizationDirection::MINIMIZE ) {
                rew_vec[i] *= -1;
            }
        }
        return rew_vec;
    }


    reward_t get_expected_reward( state_t s, action_t a, state_t succ_s ) {
        return get_reward( s, a );
    }


    std::pair< reward_t, reward_t > reward_range() const {
        auto [ min_vec, max_vec ] = env->reward_range();

        for ( size_t i = 0; i < std::min( config.directions.size(), min_vec.size() ) ; i++ ) {
            if ( config.directions[i] == OptimizationDirection::MINIMIZE ) {

                // swap the bounds and multiply by minus one
                value_t new_max = min_vec[i] * -1;
                min_vec[i] = max_vec[i] * -1;
                max_vec[i] = new_max;
            }
        }

        return std::make_pair( min_vec, max_vec );
    }


    void clear_records(){
        state_action_bounds.clear();
        state_bounds.clear();
        update_count.clear();
    }

    std::string name() const {
        return env->name();
    }


    Observation reset( unsigned seed=0, bool reset_records=true ) {
        if (reset_records) { clear_records(); }
        return env->reset( seed );
    }


    Observation step( action_t action ) {
        state_t current_state = get_current_state();

        discover( current_state );

        auto [ next_state, reward, terminated ] = env->step( action );

        return { next_state , reward, terminated };
    }
    

    /* returns min/max possible payoff ( discounted sum ) value given the
     * min/max reward bounds 
     */
    std::pair< std::vector< value_t >, std::vector< value_t > > min_max_discounted_reward() const {

        auto [ min, max ] = reward_range();

        value_t discount_copy = 1.0 / (1 - config.discount_param);
        multiply( discount_copy, min );
        multiply( discount_copy, max );

        return std::make_pair( min, max );
    }

    std::pair< std::vector< value_t >, std::vector< value_t > > get_initial_bound() const {

        // if no predefined bounds, calculate min max payoff
        if ( config.lower_bound_init.empty() )
            return min_max_discounted_reward();

        // return supplied initial bound from configuration
        return std::make_pair( config.lower_bound_init, config.upper_bound_init );
    }



    // initializes all state_action bounds of s and the state bound
    void init_bound( const state_t &s ) {

        auto [ init_low, init_upp ] = get_initial_bound();
        
        /* if terminal state set using the enabled actions instead of 
         * initial bounds, if this is changed, setting of bounds for terminal
         * SSP states has to be handled somewhere else */
        if ( is_terminal_state( s ) ) {
            size_t act_idx = 0;
            for ( const action_t & avail_action : get_actions( s ) ) {
                auto act_reward = get_expected_reward( s, avail_action );

                // if first action (todo maybe write this like a human)
                if ( act_idx++ == 0 ) {
                    init_low = act_reward;
                    init_upp = act_reward;
                }

                else {
                    for ( size_t i = 0; i < init_low.size(); i++ ) {
                        init_low[i] = std::min( init_low[i], act_reward[i] );
                        init_upp[i] = std::max( init_upp[i], act_reward[i] );
                    }
                }
            }

            // TODO change the explicit handling of SSPs to something more
            // sensible, for now just handle it here
            if ( config.discount_param != 1 ) {
                value_t discount_copy = 1.0 / (1 - config.discount_param);
                multiply( discount_copy, init_low );
                multiply( discount_copy, init_upp );
            }
        }

        for ( const action_t & avail_action : get_actions( s ) ) {
            Bounds< value_t > bound( { init_low }, { init_upp } );
            set_bound( s, avail_action, std::move( bound ) );
        }

        update_bound( s );
    }


    /* function that discovers a given state and initializes states for all its
     * available actions, if it has not been discovered in previous simulations
     * also rewrites all already set bounds for given state
     *
     * we use discovered states to track how many states BRTDP has visited /
     * updated bound estimates over its execution vs how many total states the
     * MDP has
     */
    void discover( const state_t &s ) {
        if ( state_bounds.find( s ) == state_bounds.end() ) {
            update_count[ s ] = 0;
            init_bound( s );
        }
    }


    /* if all transitions from given state ( under every action ) result in
     * staying in given state with probability 1, then the state is terminal */
    bool is_terminal_state( const state_t &state ) const {
        std::vector< action_t > avail_actions = get_actions( state );
        for ( action_t action : avail_actions ) {

            auto transitions = get_transition( state, action );

            if ( 
                 ( transitions.size() > 1 ) || 
                 ( !approx_equal( transitions[ state ], 1.0 ) )
               )
                 {
                    return false;
                 }
        }
        
        return true;
    }

    // returns L_i(s, a), U_i(s, a)
    Bounds< value_t >& get_state_action_bound( const state_t &s, const action_t &a ) {
        auto idx = std::make_pair( s, a );
        return *state_action_bounds[ idx ];
    }


    const Bounds< value_t > &get_state_action_bound( const state_t &s, const action_t &a ) const{
        auto idx = std::make_pair( s, a );
        return *state_action_bounds[ idx ];
    }

    // returns L_i(s), U_i(s)
    Bounds< value_t >& get_state_bound( const state_t &s ) {
        return *state_bounds[ s ];
    }

    void update_bound( const state_t &s, const action_t &a ) {
        update_count[ s ]++;
        auto transition = get_transition( s, a );
        std::vector< Polygon< value_t > * > lower_curves, upper_curves;
        std::vector< double > probs;
        for ( const auto &[ succ, prob ] : transition ) {
            auto &bound = get_state_bound( succ );
            lower_curves.push_back( &( bound.lower() ) );
            upper_curves.push_back( &( bound.upper() ) );
            probs.push_back( prob );
        }

        Polygon< value_t > res_lower = weighted_minkowski_sum( lower_curves, probs );
        Polygon< value_t > res_upper = weighted_minkowski_sum( upper_curves, probs );
        Bounds< value_t > result( std::move( res_lower ), std::move( res_upper ) );

        // r + \gamma * U, r + \gamma * L..
        result.multiply_bounds( config.discount_param );
        result.shift_bounds( get_expected_reward( s, a ) );
        set_bound( s, a, std::move( result ) ) ;
    }

    void update_bound( const state_t &s ) {

        std::vector< Polygon< value_t > * > lower_curves, upper_curves;
        for ( const action_t &action : get_actions( s ) ) {
            auto &bound = get_state_action_bound( s, action );
            lower_curves.push_back( &( bound.lower() ) );
            upper_curves.push_back( &( bound.upper() ) );
        }

        Polygon< value_t > res_lower = hull_union( lower_curves, config.precision );
        Polygon< value_t > res_upper = hull_union( upper_curves, config.precision );

        set_bound( s, Bounds< value_t > ( std::move( res_lower ), std::move( res_upper ) ) );
    }



    // set state x action bound
    void set_bound( const state_t &s, const action_t &a, Bounds< value_t > &&bound ) {
       auto idx = std::make_pair( s, a );
       state_action_bounds[ idx ] = std::make_unique< Bounds< value_t > > ( bound );
    }

    void set_bound( const state_t &s, Bounds< value_t > &&bound ) {

        // the lowest possible objective value
        auto [ ref_point, _ ] = min_max_discounted_reward();

        // initialize facets and closure of lower curve for BRTDP heuristics
        bound.init_facets();
        bound.downward_closure( ref_point );

        state_bounds[ s ] = std::make_unique< Bounds< value_t > > ( bound );
    }

    void set_config( const ExplorationSettings< value_t > &_config ){
        config = _config;
    }

    size_t get_update_num() const {

        size_t total = 0;

        for ( const auto &[k, v] : update_count ) {
            total += v;
        }

        return total;
    }

    size_t num_states_explored() const {
        return state_bounds.size();
    }

    void write_exploration_logs( std::string filename, bool output_all_bounds ) const {

        std::ofstream out( filename + "-logs.txt" , std::ios_base::app );

        out << "States discovered: " << update_count.size() << "\n";
        out << "Total brtdp updates ran by state:\n";
        size_t total = 0;
        for ( const auto &[k, v] : update_count ) {
            out << "State: " << k << " updates ( state x action ): " <<  v << std::endl;
            total += v;
        }

        out << " Total " << total << " state action updates.\n";

        if ( output_all_bounds ) {
            std::ofstream bounds( filename + "-all_bounds.txt" );
            for ( const auto &[ k, v ] : state_action_bounds ){
                bounds << "State: " << std::get< 0 > ( k ) << " action: " << std::get< 1 > ( k ) << ".\n";
                bounds << *v << "\n\n\n";
            }
        }
    }
};
