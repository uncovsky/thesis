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

    void nondominated() {
        remove_dominated_alt( lower_bound.get_vertices() );
        remove_dominated_alt( upper_bound.get_vertices() );
    }

    void remove_eps_close( value_t eps ){
        lower_bound.remove_eps_close( eps );
        upper_bound.remove_eps_close( eps );
    }

    void convex_hull() {
        lower_bound.convex_hull();
        upper_bound.convex_hull();
    }
    
    // joins both bounds with other, which means taking a union of both of the
    // vertex sets
    void merge_bound( const Bounds< value_t > &other ) {
        auto low_rhs = other.lower().get_vertices();
        auto upp_rhs = other.upper().get_vertices();

        auto& low_lhs = lower_bound.get_vertices();
        auto& upp_lhs = upper_bound.get_vertices();

        low_lhs.insert( low_lhs.end(), low_rhs.begin(), low_rhs.end() );
        upp_lhs.insert( upp_lhs.end(), upp_rhs.begin(), upp_rhs.end() );
    }

    // saves minkowski sum of args bounds to this
    void sum_successors( const std::vector< Bounds< value_t > > &args ){
        std::vector< const Polygon< value_t > * > lower_polygons, upper_polygons;

        for ( const auto &bound : args ){
            const Polygon< value_t > * ptr_upp = &( bound.upper() ), * ptr_low = &( bound.lower() );
            lower_polygons.push_back( ptr_low );
            upper_polygons.push_back( ptr_upp );
        }

        upper_bound.minkowski_sum( upper_polygons );
        lower_bound.minkowski_sum( lower_polygons );
    }

    void sum_bound( const Bounds< value_t > &other ){
        lower_bound.minkowski_sum( other.lower() );
        upper_bound.minkowski_sum( other.upper() );
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
    
    // reference point is the min of all objectives ( theoretical lowest
    // possible value in objective space dominated by all other points )
    void pareto( const std::vector< value_t > &ref_point ) {
        lower_bound.pareto( ref_point );
        upper_bound.pareto( ref_point );
    }

    // input conditions -> pareto operator has been ran on *this ( pareto call
    // preceded, omitting it here for performance reasons )
    value_t bound_distance(){
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
    Point< value_t > init_low_bound;
    Point< value_t > init_upp_bound;

    std::vector< value_t > discount_params;

    /* track update count for every state */
    std::map< state_t, size_t > update_count;
    std::map< std::tuple< state_t, action_t >, bounds_ptr > state_action_bounds;

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
        return env->get_reward( s, a );
    }


    reward_t get_expected_reward( state_t s, action_t a, state_t succ_s ) {
        return env->get_reward( s, a );
    }


    std::pair< reward_t, reward_t > reward_range() const {
        return env->reward_range();
    }


    void clear_records(){
        state_action_bounds.clear();
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
    

    /* returns min/max possible objective ( discounted sum ) value given the
     * min/max reward bounds 
     */
    std::pair< std::vector< value_t >, std::vector< value_t > > min_max_discounted_reward() const {

        auto [ min, max ] = reward_range();

        std::vector< value_t > discount_copy( discount_params );
        multiply( value_t( -1 ), discount_copy );
        add( value_t( 1 ), discount_copy );
        
        divide( min, discount_copy );
        divide( max, discount_copy );

        return std::make_pair( min, max );
    }

    std::pair< std::vector< value_t >, std::vector< value_t > > get_initial_bound() const {
        if ( init_low_bound.empty() )
            return min_max_discounted_reward();
        return std::make_pair( init_low_bound, init_upp_bound );
    }


    /* initializes L_0(s, a), U_0(s, a) */
    void init_bound( state_t s, action_t a ) {
    
        auto [ init_low, init_upp ] = get_initial_bound();

        // the lowest possible objective value
        auto [ ref_point, _ ] = min_max_discounted_reward();

        Bounds< value_t > result ( { init_low } , { init_upp } );

        // use to set facets & downward closure w.r.t ref point
        result.pareto( ref_point );

        auto idx = std::make_pair( s, a );
        state_action_bounds[idx] = std::make_unique< Bounds< value_t > > ( std::move( result ) );
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
            for ( const action_t & avail_action : get_actions( s ) ) {
                init_bound( s, avail_action );
            }
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
                 ( !approx_equal( transitions[state], 1.0 ) )
               )
                 {
                    return false;
                 }
        }
        
        return true;
    }


    // returns L_i(s), U_i(s)
    Bounds< value_t > get_state_bound( state_t s ) {
        discover( s );
        std::vector< action_t > avail_actions = get_actions( s );

        Bounds< value_t > result;
        for ( const action_t &action : avail_actions ) {
            Bounds<value_t> action_bound = get_state_action_bound( s, action );
            result.merge_bound( action_bound );
        }

        // remove dominated elements from result
        // result.nondominated();

        // set facets and eliminate convex dominated points
        auto [ ref_point, _ ] = min_max_discounted_reward();
        result.pareto( ref_point );

        return result;
    }


    // returns L_i(s, a), U_i(s, a)
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
       update_count[ s ]++;
       state_action_bounds[idx] = std::make_unique< Bounds< value_t > > ( bound );
    }


    void set_config( const ExplorationSettings< value_t > &config ){
        discount_params = config.discount_params;
        init_low_bound = config.lower_bound_init;
        init_upp_bound = config.upper_bound_init;
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
