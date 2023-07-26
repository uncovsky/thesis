#pragma once

#include <algorithm>
#include <memory>
#include <sstream>
#include <set>

#include "geometry/polygon.hpp"
#include "models/environment.hpp"
#include "utils/eigen_types.hpp"


/* tracks lower/upper bounds on the objective value */
template < typename value_t > 
class Bounds{

    Polygon< value_t > lower_bound;
    Polygon< value_t > upper_bound;

public:

    Bounds ( const std::vector< value_t > &lower_pt, 
             const std::vector< value_t > &upper_pt ) : lower_bound( { lower_pt } ),
                                                        upper_bound( { upper_pt } ) {}


    Polygon< value_t > &lower() {
        return lower_bound; 
    }

    Polygon< value_t > &upper() {
        return upper_bound; 
    }
};


/* this class is used to interact with the underlying environment, recording
 * statistics, simulation, logging, etc.
 * reward_t to the actual reward type ( so std::vector< double > etc. )
 * while value_t will be equal to double / the templated argument of reward_t */

template < typename state_t, typename action_t, typename reward_t , typename value_t >
class EnvironmentWrapper{
    
    using bounds_ptr = std::unique_ptr< Bounds< value_t > >;

    Environment< state_t, action_t, reward_t> *env;

    std::vector< value_t > discount_params;
    std::map < state_t, size_t > visit_count;
    std::map < std::tuple< state_t, action_t >, bounds_ptr > state_action_bounds;

public:

    EnvironmentWrapper() : env( nullptr ), 
                           visit_count(), 
                           state_action_bounds() {}
    EnvironmentWrapper( Environment< state_t, action_t, reward_t > *env ) : env( env ), 
                                                                            visit_count(), 
                                                                            state_action_bounds() {}

    using Observation = typename Environment< state_t, action_t, reward_t > :: Observation;

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
        return env->get_reward( s, a );
    }

    reward_t get_expected_reward( state_t s, action_t a, state_t succ_s ) {
        return env->get_reward( s, a );
    }

    void clear_records(){
        state_action_bounds.clear();
        visit_count.clear();
    }

    void init_bound( state_t s, action_t a ) {

        // get vectors with min values and max values
        auto [ min, max ] = reward_range();

        // ( 1-\lambda_0, 1-\lambda_1, ... )
        std::vector< value_t > denominator = add( multiply( -1, discount_params ), 1 );
        std::vector< value_t > lower_bound_pt = divide( min, denominator );
        std::vector< value_t > upper_bound_pt = divide( max, denominator );
        
        state_action_bounds[std::make_pair( s, a )] = std::make_unique( lower_bound_pt, upper_bound_pt );
    }

    std::pair< reward_t, reward_t > reward_range() const {
        return env->reward_range();
    }

    Bounds< value_t > get_state_bound( state_t s ) {
        std::vector< action_t > avail_actions = get_actions( s );

    }
    
    Observation reset( unsigned seed=0, bool reset_records=true ) {
        if (reset_records) { clear_records(); }
        return env->reset( seed );
    }

    Observation step( action_t action ) {
        state_t current_state = get_current_state();

        visit_count[ current_state ]++;

        auto [ next_state, reward, terminated ] = env->step( action );

        return { next_state , reward, terminated };
    }

};


