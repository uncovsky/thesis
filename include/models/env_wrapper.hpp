#pragma once

#include <algorithm>
#include <memory>
#include <sstream>
#include <set>
#include "geometry/polygon.hpp"
#include "geometry/pareto.hpp"
#include "models/environment.hpp"
#include "utils/eigen_types.hpp"


/* tracks lower/upper bounds on the objective value */
template < typename value_t > 
class Bounds{

    Polygon< value_t > lower_bound;
    Polygon< value_t > upper_bound;

public:

    Bounds() : lower_bound(), upper_bound(){}

    Bounds ( const std::set< std::vector< value_t > > &lower_pts, 
             const std::set< std::vector< value_t > >&upper_pts ) : lower_bound( lower_pts ),
                                                                    upper_bound( upper_pts ) {}
    Bounds ( const Polygon< value_t > &lower, 
             const Polygon< value_t > &upper ) : lower_bound( lower ),
                                                 upper_bound( upper ) {}

    Bounds ( Polygon< value_t > &&lower, 
             Polygon< value_t > &&upper ) : lower_bound( std::move( lower ) ),
                                            upper_bound( std::move( upper ) ) {}

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

    void nondominated() {
        remove_dominated( lower_bound.get_vertices() );
        remove_dominated( upper_bound.get_vertices() );
    }
    
    // joins both bounds with other, which means taking a union of both of the
    // vertex sets and removing dominated elements
    void merge_bound( const Bounds< value_t > &other ) {
        auto low_vertices_rhs = other.lower().get_vertices();
        auto upp_vertices_rhs = other.upper().get_vertices();

        nondominated_union( lower_bound.get_vertices(), low_vertices_rhs );
        nondominated_union( upper_bound.get_vertices(), upp_vertices_rhs );
    }

    // performs a minkowski sum with each respective bound of other
    void sum_bounds( const Bounds< value_t > &other ){
        lower_bound.minkowski_sum( other.lower() );
        upper_bound.minkowski_sum( other.upper() );

        nondominated();

    }

    void multiply_bounds( value_t mult ) {
        lower_bound.multiply_scalar( mult );
        upper_bound.multiply_scalar( mult );
    }

    void multiply_bounds( const std::vector<value_t> &mult ) {
        lower_bound.multiply_vector( mult );
        upper_bound.multiply_vector( mult );
    }

    void shift_bounds( value_t shift ) {
        lower_bound.shift_scalar( shift );
        upper_bound.shift_scalar( shift );
    }

    void shift_bounds( const std::vector< value_t > &shift ) {
        lower_bound.shift_vector( shift );
        upper_bound.shift_vector( shift );
    }
    
    // reference point is the min of all objectives ( theoretical lowest
    // possible value in objective space dominated by all other points )
    value_t bound_distance( const std::vector< value_t > &ref_point ){
        lower_bound.pareto( ref_point );
        upper_bound.convex_hull();
        return lower_bound.hausdorff_distance( upper_bound );
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
 * while value_t will be equal to double / the templated argument of reward_t */

template < typename state_t, typename action_t, typename reward_t , typename value_t >
class EnvironmentWrapper{
    
    using bounds_ptr = std::unique_ptr< Bounds< value_t > >;

    Environment< state_t, action_t, reward_t > *env;

    std::vector< value_t > discount_params;

    std::set < state_t > discovered_states;
    std::map < std::tuple< state_t, action_t >, bounds_ptr > state_action_bounds;

public:

    EnvironmentWrapper() : env( nullptr ), 
                           discovered_states(), 
                           state_action_bounds() {}
    EnvironmentWrapper( Environment< state_t, action_t, reward_t > *env ) : env( env ), 
                                                                            discovered_states(), 
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
        discovered_states.clear();
    }

    std::pair< std::vector< value_t >, std::vector< value_t > > min_max_discounted_reward() {

        auto [ min, max ] = reward_range();

        // ( 1-\lambda_0, 1-\lambda_1, ... ), TODO: change this to something
        // more sensible, don't use c-style casts, but type traits?
        std::vector< value_t > denominator = add( value_t( 1 ), multiply( value_t(-1) , discount_params ) );

        std::vector< value_t > lower_bound_pt = divide( min, denominator );
        std::vector< value_t > upper_bound_pt = divide( max, denominator );
        
        return std::make_pair( lower_bound_pt, upper_bound_pt );
    }

    void init_bound( state_t s, action_t a ) {

        auto [ lower_bound_pt, upper_bound_pt ] = min_max_discounted_reward();
        Bounds< value_t > result ( { lower_bound_pt }, { upper_bound_pt } );

        auto idx = std::make_pair( s, a );
        state_action_bounds[idx] = std::make_unique< Bounds< value_t > > ( std::move( result ) );
    }

    std::pair< reward_t, reward_t > reward_range() const {
        return env->reward_range();
    }

    /* function that discovers a given state and initializes states for all its
     * available actions, if it has not been discovered in previous simulations
     * also rewrites all already set bounds for given state
     */

    void discover( state_t s ) {
        if ( discovered_states.find( s ) == discovered_states.end() ) {
            discovered_states.insert( s );
            for ( const action_t & avail_action : get_actions( s ) ) {
                init_bound( s, avail_action );
            }
        }
    }

    bool is_terminal_state( state_t state ) const {
        std::vector< action_t > avail_actions = get_actions( state );
        for ( size_t action : avail_actions ) {

            auto transitions = get_transition( state, action );

            if ( 
                 ( transitions.size() > 1 ) || 
                 ( !approx_equal( transitions[state], 1.0 ) )
               )
                 {
                    return false;
                 }
        }
        
        return true;
    }

    Bounds< value_t > get_state_bound( state_t s ) {
        discover( s );
        std::vector< action_t > avail_actions = get_actions( s );

        Bounds< value_t > result;
        for ( const action_t &action : avail_actions ) {
            Bounds<value_t> action_bound = get_state_action_bound( s, action );
            result.merge_bound( action_bound );
        }

        return result;
    }

    Bounds< value_t > get_state_action_bound( state_t s, action_t a ) {
        discover( s );
        auto idx = std::make_pair( s, a );
        return *state_action_bounds[ idx ];
    }

    const Bounds< value_t > &get_state_action_bound( state_t s, action_t a ) const{
        auto idx = std::make_pair( s, a );
        return *state_action_bounds[ idx ];
    }
   

    void set_bound( state_t s, action_t a, Bounds< value_t > &&bound ) {
       auto idx = std::make_pair( s, a );
       state_action_bounds[idx] = std::make_unique< Bounds< value_t > > ( bound );
    }

    void set_discount_params( const std::vector< value_t > gammas ) {
        discount_params = gammas;
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

    void write_statistics(){
        std::cout << "States discovered: " << discovered_states.size() << "\n";
        std::cout << "Total records, state action values: " << state_action_bounds.size() << "\n";

    }

};


