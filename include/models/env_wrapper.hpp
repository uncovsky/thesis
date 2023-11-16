#pragma once

#include <algorithm>
#include <memory>
#include <sstream>
#include <set>
#include "geometry/polygon.hpp"
#include "geometry/pareto.hpp"
#include "models/environment.hpp"
#include "solvers/config.hpp"
#include "utils/eigen_types.hpp"
#include "utils/prng.hpp"


/* class used to store upper and lower bounds on the objective value
 * for every state action pair. ( the over/under approximations of the pareto
 * curve )
 */
template < typename value_t > 
class Bounds{

    Polygon< value_t > lower_bound;
    Polygon< value_t > upper_bound;

    bool distance_valid = false;
    value_t distance = 0;

public:

    Bounds() : lower_bound(), upper_bound(){}

    Bounds ( const std::vector< std::vector< value_t > > &lower_pts, 
             const std::vector< std::vector< value_t > >&upper_pts ) : lower_bound( lower_pts ),
                                                                       upper_bound( upper_pts ){}
    Bounds ( const Polygon< value_t > &lower, 
             const Polygon< value_t > &upper ) : lower_bound( lower ),
                                                 upper_bound( upper ){}

    Bounds ( Polygon< value_t > &&lower, 
             Polygon< value_t > &&upper ) : lower_bound( std::move( lower ) ),
                                            upper_bound( std::move( upper ) ){}

    Polygon< value_t > &lower() {
        return lower_bound; 
    }

    Polygon< value_t > &upper() {
        return upper_bound; 
    }

    const Polygon< value_t > &lower() const {
        return lower_bound; 
    }

    const Polygon< value_t > &upper() const {
        return upper_bound; 
    }

    // helper functions for multiplying/adding scalars/vectors to all entries
    void multiply_bounds( value_t mult ) {
        lower_bound.multiply_scalar( mult );
        upper_bound.multiply_scalar( mult );
    }

    void multiply_bounds( const std::vector<value_t> &mult ) {
        lower_bound.multiply_vector( mult );
        upper_bound.multiply_vector( mult );
    }

    void shift_bounds( const std::vector< value_t > &shift ) {
        lower_bound.shift_vector( shift );
        upper_bound.shift_vector( shift );
    }

    void init_facets() {
        lower_bound.init_facets();
    }

    void downward_closure( const Point< value_t > &pt ) {
        lower_bound.downward_closure( pt );
    }

    // input conditions -> this is a state bound set by set_bound() function,
    // i.e. facets and downward closure intiialzied
    value_t bound_distance(){
        if ( !distance_valid ) {
            distance = lower_bound.hausdorff_distance( upper_bound );
            distance_valid = true;
        }

        return distance;
    }

    friend std::ostream &operator<<( std::ostream& os, const Bounds< value_t > &b ) {
        os << "lower bound:\n" << b.lower().to_string() << "\n";
        os << "upper bound:\n" << b.upper().to_string() << "\n";
        return os;
    }
};


/* this class is used to interact with the underlying environment, recording
 * statistics, simulation, logging, etc.
 * reward_t to the actual reward type ( so std::vector< double > etc. )
 * while value_t will be equal to the type used to represent the reward
 * components for the underlying reward, so for example double 
 *
 * templating the value/reward can be useful, since one possible
 * extension could be using exact rational arithmetic instead of floating point
 * arithmetic, and then the value_t templating could be helpful */

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
        if ( config.lower_bound_init.empty() )
            return min_max_discounted_reward();
        return std::make_pair( config.lower_bound_init, config.upper_bound_init );
    }


    /* initializes L_0(s, a), U_0(s, a) */
    void init_bound( const state_t &s, const action_t &a ) {
    
        auto [ init_low, init_upp ] = get_initial_bound();

        Bounds< value_t > result ( { init_low } , { init_upp } );

        if ( is_terminal_state( s ) && !config.lower_bound_init_term.empty() )
            result = Bounds< value_t >( { config.lower_bound_init_term } , { config.upper_bound_init_term } );

        set_bound( s, a, std::move( result ) );
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
        if ( update_count.find( s ) == update_count.end() ) {
            update_count[ s ] = 0;

            // set state action and state bound
            for ( const action_t & avail_action : get_actions( s ) ) {
                init_bound( s, avail_action );
            }
            update_bound( s );
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


    // returns L_i(s), U_i(s)
    Bounds< value_t >& get_state_bound( const state_t &s ) {
        discover( s );
        return *state_bounds[ s ];
    }

    void update_bound( const state_t &s, const action_t &a ) {
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
        discover( s );

        std::vector< Polygon< value_t > * > lower_curves, upper_curves;
        for ( const action_t &action : get_actions( s ) ) {
            auto &bound = get_state_action_bound( s, action );
            lower_curves.push_back( &( bound.lower() ) );
            upper_curves.push_back( &( bound.upper() ) );
        }

        Polygon< value_t > res_lower = hull_union( lower_curves, config.precision / 100 );
        Polygon< value_t > res_upper = hull_union( upper_curves, config.precision / 100 );

        set_bound( s, Bounds< value_t > ( std::move( res_lower ), std::move( res_upper ) ) );
    }

    // returns L_i(s, a), U_i(s, a)
    Bounds< value_t >& get_state_action_bound( const state_t &s, const action_t &a ) {
        discover( s );
        auto idx = std::make_pair( s, a );
        return *state_action_bounds[ idx ];
    }


    const Bounds< value_t > &get_state_action_bound( const state_t &s, const action_t &a ) const{
        discover( s );
        auto idx = std::make_pair( s, a );
        return *state_action_bounds[ idx ];
    }


    // set state x action bound
    void set_bound( const state_t &s, const action_t &a, Bounds< value_t > &&bound ) {
       auto idx = std::make_pair( s, a );
       update_count[ s ]++;
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

