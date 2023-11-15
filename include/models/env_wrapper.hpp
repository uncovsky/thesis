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

template< typename value_t >
ParetoCurve< value_t > minkowski_sum( const std::vector< ParetoCurve< value_t > * > &curves,
                                               const std::vector< double > &weights ){

        std::vector< Point< value_t > > resulting_vertices;

        // indices into vertex array of each polygon 
        std::vector< size_t > offsets( curves.size(), 0 );

        // helper lambdas for indexing using both arrays
        auto get_ith_vertex = [ & ] ( size_t polygon_idx, size_t vertex_idx ){
            Point< value_t > pt = curves[ polygon_idx ]->get_vertex( vertex_idx );
            multiply( weights[ polygon_idx ], pt );
            return pt;
        };

        auto polygon_done = [ & ]( size_t polygon_idx ){
            return offsets[ polygon_idx ] == curves[ polygon_idx ]->size() - 1; 
        };

        auto sum_unfinished = [ & ](){
            for ( size_t i = 0; i < offsets.size(); i++ ) {
                if ( !polygon_done( i ) ) { return true; }
            }
            return false;
        };

        bool unfinished = true;

        while ( unfinished ){

            // check here, because need to add last vertex after all offsets reach
            // final index
            unfinished = sum_unfinished();
            Point< value_t > next = { 0, 0 };
            for ( size_t i = 0; i < curves.size(); i++ ){
                // select current point in polygon i and add it to next vertex
                Point< value_t > added = get_ith_vertex( i, offsets[i] );
                next[0] += added[0];
                next[1] += added[1];
            }

            resulting_vertices.push_back( next );

            // track all edges with minimal polar angle, to remove colinear
            // points
            std::vector< size_t > incremented_indices = {};
            value_t min_dy( -1 );

            /* investigate the next edge of each polygon, select those edges
             * that correspond to the least polar angle and mark them for the
             * next shift */ 
            for ( size_t i = 0; i < curves.size(); i++ ){

                if ( polygon_done( i ) ) { continue; }

                Point< value_t > Pcurr = get_ith_vertex( i, offsets[i] );
                Point< value_t > Pnext = get_ith_vertex( i, offsets[i] + 1 );

                value_t dy = ( Pcurr[1] - Pnext[1] ) / ( Pnext[0] - Pcurr[1] );

                if ( ( min_dy == -1 ) || ( dy < min_dy ) ){
                    min_dy = dy;
                    incremented_indices = { i };
                }

                else if ( dy == min_dy ) {
                    incremented_indices.push_back( i );
                }

            }
            // move all vertices corresponding to edges with least polar angles
            for ( size_t idx : incremented_indices ){
                offsets[ idx ]++;
            }
        }

    return ParetoCurve< value_t >( resulting_vertices );
}


template < typename value_t >
std::pair< std::vector< Point< value_t > >, std::set< size_t > > 
upper_right_hull( std::vector< std::pair< size_t, Point< value_t > > > &tagged_vertices,
                  double eps ) {

    if ( tagged_vertices.empty() )
        return std::make_pair< std::vector< Point< value_t > >, std::set< size_t > >( {}, {} );

    // get dimension of points
    size_t dim = tagged_vertices[0].second.size();

    if ( dim > 2 ) {
        std::cout << "Higher dimension convex hulls are currently unsupported." << std::endl;
        throw std::runtime_error("invalid hull operation.");
    }

    auto cmp = []( const auto &x, const auto &y ) { return x.second < y.second; };

    std::vector< Point< value_t > > hull = { tagged_vertices.back().second };
    std::vector< size_t > tags = { tagged_vertices.back().first };

    // if 1d hull just return the only element and its tag
    if ( dim == 1 ) {
        return std::make_pair( hull , std::set< size_t >( tags.begin(), tags.end() ) );
    }

    std::sort( tagged_vertices.begin(), tagged_vertices.end(), cmp );

    for ( auto it = tagged_vertices.rbegin() + 1; it != tagged_vertices.rend(); it++ ){
        size_t tag = it->first;
        Point< value_t > pt = it->second;

        // need increasing y
        if ( pt[1] <= hull.back()[1] ) { continue; }
        else if ( hull.size() < 2 ) { hull.push_back( pt ); tags.push_back( tag ); }
        else { 
            size_t i = hull.size() - 1;
            /* if last vertex of the hulls lies in CW direction from
             * pt->hull[i-1], remove the last element of the hull,
             * repeat
             */
            while ( ( hull.size() >= 2 ) && 
                    ( ccw( pt, hull[i - 1], hull[i] ) <= eps ) ) {
                hull.pop_back();
                tags.pop_back();
                i--;
            }
            hull.push_back( pt );
            tags.push_back( tag );
        }
    }

    std::set< size_t > unique_tags( tags.begin(), tags.end() );
    return std::make_pair( hull, unique_tags );
}
// used for state bounds
// eps is precision of the hull, curves the state-action curves
// tags are used for action selection (pareto)
template< typename value_t >
ParetoCurve< value_t > hull_union( const std::vector< ParetoCurve< value_t > * > &curves,
                                   double eps ){
    std::vector< std::pair< size_t , Point< value_t > > > tagged_points;

    for ( size_t i = 0; i < curves.size(); i++ ) {
        auto vertices = curves[i]->get_vertices();
        for ( const auto &vertex : vertices ) {
            tagged_points.emplace_back( i, vertex );
        }
    }

    auto res = upper_right_hull( tagged_points, eps );
    return ParetoCurve< value_t >( res.first, res.second );
}


/* class used to store upper and lower bounds on the objective value
 * for every state action pair. ( the over/under approximations of the pareto
 * curve )
 */
template < typename value_t > 
class Bounds{

    ParetoCurve< value_t > lower_bound;
    ParetoCurve< value_t > upper_bound;
public:

    Bounds() : lower_bound(), upper_bound(){}

    Bounds ( const std::vector< std::vector< value_t > > &lower_pts, 
             const std::vector< std::vector< value_t > >&upper_pts ) : lower_bound( lower_pts ),
                                                                       upper_bound( upper_pts ){}
    Bounds ( const ParetoCurve< value_t > &lower, 
             const ParetoCurve< value_t > &upper ) : lower_bound( lower ),
                                                 upper_bound( upper ){}

    Bounds ( ParetoCurve< value_t > &&lower, 
             ParetoCurve< value_t > &&upper ) : lower_bound( std::move( lower ) ),
                                            upper_bound( std::move( upper ) ){}

    ParetoCurve< value_t > &lower() {
        return lower_bound; 
    }

    ParetoCurve< value_t > &upper() {
        return upper_bound; 
    }

    const ParetoCurve< value_t > &lower() const {
        return lower_bound; 
    }

    const ParetoCurve< value_t > &upper() const {
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
        upper_bound.init_facets();
    }

    void downward_closure( const Point< value_t > &ref_point ) {
        lower_bound.downward_closure( ref_point );
    }

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

    /* specific bounds for terminal state */
    Point< value_t > init_low_bound_term;
    Point< value_t > init_upp_bound_term;

    std::vector< value_t > discount_params;

    /* precision used to build the hull, the larger 
     * the more points get filtered out during the building of the hull 
     */
    double hull_precision;

    /* track update count for every state */
    std::map< state_t, size_t > update_count;
    std::map< std::tuple< state_t, action_t >, bounds_ptr > state_action_bounds;
    std::map< state_t, bounds_ptr > state_bounds;

    std::map< state_t, std::vector< size_t > > optimal_actions;

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
    void init_bound( const state_t &s, const action_t &a ) {
    
        // get initial upper & lower bounds
        auto [ init_low, init_upp ] = get_initial_bound();


        Bounds< value_t > result ( { init_low } , { init_upp } );

        // if s is terminal and diff state bound is set for terminal states
        if ( is_terminal_state( s ) && !init_low_bound_term.empty() )
            result = Bounds< value_t >( { init_low_bound_term } , { init_upp_bound_term } );

        // save bound
        set_bound( s, a , std::move( result ) );
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

            // set state bound
            update_state_bound( s );
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
    Bounds< value_t > get_state_action_bound( const state_t &s, const action_t &a ) {
        discover( s );
        auto idx = std::make_pair( s, a );
        return *state_action_bounds[ idx ];
    }


    // returns L_i(s), U_i(s)
    Bounds< value_t > get_state_bound( const state_t &s ) {
        discover( s );
        return *state_bounds[ s ];
    }

    void update_state_bound( const state_t &s ) {
        std::vector< action_t > avail_actions = get_actions( s );
        std::vector< ParetoCurve< value_t > * > lower_curves;
        std::vector< ParetoCurve< value_t > * > upper_curves;

        for ( const action_t &action : avail_actions ) {
            auto idx = std::make_pair( s, action );
            lower_curves.push_back( &( state_action_bounds[ idx ]->lower() ) );
            upper_curves.push_back( &( state_action_bounds[ idx ]->upper() ) );
        }

        ParetoCurve< value_t > low_res = hull_union( lower_curves, hull_precision );
        ParetoCurve< value_t > upp_res = hull_union( upper_curves, hull_precision );
        Bounds< value_t > result( low_res, upp_res );
        set_bound( s, std::move( result ) );
    }

    void update_state_action_bound( const state_t &s, const action_t &a ){
        discover( s );
        update_count[ s ]++;

        std::vector< ParetoCurve< value_t > * > curves;
        std::vector< double > weights;
        auto transition = get_transition( s, a );

        std::vector< ParetoCurve< value_t > * > lower_curves;
        std::vector< ParetoCurve< value_t > * > upper_curves;
        for ( const auto &[succ, prob] : transition ) {
            weights.push_back( prob );
            lower_curves.push_back( &( state_bounds[ succ ]->lower() ) );
            upper_curves.push_back( &( state_bounds[ succ ]->upper() ) );
        }


        ParetoCurve< value_t > low_res = minkowski_sum( lower_curves, weights );
        ParetoCurve< value_t > upp_res = minkowski_sum( upper_curves, weights );

        Bounds< value_t > result( low_res, upp_res );
        result.multiply_bounds( discount_params );
        result.shift_bounds( get_expected_reward( s, a ) );
        set_bound( s, a, std::move( result ) ) ;
    }

    const Bounds< value_t > &get_state_action_bound( const state_t &s, const action_t &a ) const{
        auto idx = std::make_pair( s, a );
        return *state_action_bounds[ idx ];
    }


    // set state x action bound, run downward closure on lower bound
    void set_bound( const state_t &s, const action_t &a, Bounds< value_t > &&bound ) {
       auto idx = std::make_pair( s, a );
       auto [ ref_point, _ ] = min_max_discounted_reward();
       bound.init_facets();
       bound.downward_closure( ref_point );
       state_action_bounds[ idx ] = std::make_unique< Bounds< value_t > > ( bound );
    }

    // set state bound
    void set_bound( const state_t &s, Bounds< value_t > &&bound ) {
       auto [ ref_point, _ ] = min_max_discounted_reward();
       bound.init_facets();
       bound.downward_closure( ref_point );
       state_bounds[ s ] = std::make_unique< Bounds< value_t > > ( bound );
    }

    void set_config( const ExplorationSettings< value_t > &config ){
        discount_params = config.discount_params;
        hull_precision = config.precision;
        init_low_bound = config.lower_bound_init;
        init_upp_bound = config.upper_bound_init;
        init_low_bound_term = config.lower_bound_init_term;
        init_upp_bound_term = config.upper_bound_init_term;
    }

    std::set< size_t > get_maximizing_actions( const state_t &s ){
        if ( optimal_actions.find(s) == optimal_actions.end() ) {
            return get_actions(s);
        }

        return optimal_actions(s);
    }

    size_t get_update_num() const {
        size_t total = 0;
        for ( const auto &[_, v] : update_count ) {
            total += v;
        }

        return total;
    }

    void write_exploration_logs( std::string filename, bool output_all_bounds ) const {

        size_t total = 0;
        std::ofstream out( filename + "-logs.txt" , std::ios_base::app );

        out << "States discovered: " << update_count.size() << "\n";
        out << "Total brtdp updates ran by state:\n";
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
