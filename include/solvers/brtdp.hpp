# pragma once
#include <algorithm>
#include <stack> 
#include "models/env_wrapper.hpp"
#include "solvers/config.hpp"
#include "utils/eigen_types.hpp"
#include "utils/prng.hpp"


template < typename state_t, typename action_t, typename value_t >
class BRTDPSolver{

    /* 
     * TYPEDEFS
     */
    using EnvironmentHandle = EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t >;
    using ExplorationConfig = ExplorationSettings< value_t >;

    // implicitly starts from inital state s'
    using TrajectoryStack = std::stack< std::pair< action_t, state_t > >;

    // handle on given mdp model
    EnvironmentHandle env;

    // config of the solver
    ExplorationConfig config;

    // prng for sampling from distributions, etc.
    PRNG gen;

    /*
     * ACTION HEURISTICS 
     */

    /* pareto set evaluation heuristic from pareto q-learning:
     * 1) look at upper bounds of available actions
     * 2) select action uniformly from all actions that have >= 1 nondominated
     * vector in their upper bound across avail_actions 
     */
    /*
    action_t pareto_action( const state_t &s, const std::vector< action_t > &avail_actions ) {

        using vertex_vec = typename std::vector< Point< value_t > >; 

        std::vector< vertex_vec > bounds;

        std::set< action_t > pareto_actions;

        // copy vertices of upper bounds
        for ( const action_t &a : avail_actions ) {
            vertex_vec vertices;

            // get vertices of respective upper bound and copy them into a set
            vertices = env.get_state_action_bound( s, a ).upper().get_vertices();
            bounds.push_back( vertices );

        }

        vertex_vec nondominated = env.get_state_bound( s ).upper().get_vertices();

        for ( const auto &point : nondominated ) {
            size_t i = 0;
            for ( const action_t &action : avail_actions ) {
                if ( ( pareto_actions.find( action ) == pareto_actions.end() ) &&
                       ( std::find( bounds[i].begin(), bounds[i].end(), point ) != bounds[i].end() ) ) {
                    pareto_actions.insert( action );
                }

                i++;
            }
        }

        // choose uniformly from these actions
        return gen.sample_uniformly( pareto_actions );
        
    }
    */
    action_t pareto_action( const state_t &s, const std::vector< action_t > &avail_actions ) {
        std::set< size_t > maximizing_indices = env.get_maximizing_actions( s );
        for ( size_t i : maximizing_indices ) {
            std::cout << i << " ";
        }
        std::cout << "\n";
        for ( const auto act : avail_actions ) {
            std::cout << env.get_state_action_bound( s, act );
        }
        size_t idx = gen.sample_uniformly( maximizing_indices );
        return avail_actions[idx];
    }

    action_t hypervolume_action( const state_t &s, const std::vector< action_t > &avail_actions ) {
        auto [ ref_point , _ ] = env.get_initial_bound();

        std::vector< value_t > hypervolumes;
        value_t total_hypervolume( 0 );

        for ( const action_t &act : avail_actions ) {
            auto upper_bound_vertices = env.get_state_action_bound( s, act ).upper().get_vertices();
            hypervolumes.push_back( hypervolume_indicator( upper_bound_vertices, ref_point ) );

            total_hypervolume += hypervolumes.back();
        }

        std::map< action_t, value_t > hv_distribution;

        size_t i = 0;
        for ( const action_t &act : avail_actions ) {
            hv_distribution[act] = ( total_hypervolume == 0 ) ? 1.0 / hypervolumes.size() : hypervolumes[i++] / total_hypervolume;
        }

        return gen.sample_distribution( hv_distribution );
    }
    /*
     * SUCCESSOR HEURISTICS 
     */

    /*
     * helper function to get differences of bounds for a transition map
     */
    std::vector< value_t > get_successor_diffs( const std::map< state_t , double > &transition ){
        std::vector< value_t > diff_values;
        for ( const auto &[ s, prob ] : transition ) {
            diff_values.push_back( env.get_state_bound( s ).bound_distance() * prob );
        }

        return diff_values;
    }

    /*
     * BRTDP heuristic for successor selection, uniformly samples from
     * successors maximizing weighted difference of their state bounds 
     * argmax ( delta( s, a , s' ) * ( dist_H( L_i( s' ), U_i( s' ) ) ) )
     */
    state_t bound_difference_state_selection( const std::map< state_t, double > &transitions ) {

        std::vector< value_t > diff_values = get_successor_diffs( transitions );
        value_t diff_sum = std::accumulate( diff_values.begin(), diff_values.end(), 0.0f );

        std::map< state_t, value_t > diff_distribution;
        size_t succ_count = transitions.size();

        size_t i = 0;
        for ( const auto &[ s, _ ] : transitions ) {
            diff_distribution[ s ] = ( diff_sum == 0 ) ? 1.0 / succ_count : diff_values[ i ] / diff_sum;
            i++;
        }

        return gen.sample_distribution( diff_distribution );
    }

    // picks action from avail actions based on specified heuristic
    action_t action_selection( const state_t &s, const std::vector< action_t > &avail_actions ) {

        if ( config.action_heuristic == ActionSelectionHeuristic::Pareto ) {
            return pareto_action( s, avail_actions );
        }

        if ( config.action_heuristic == ActionSelectionHeuristic::Hypervolume ){
            return hypervolume_action( s, avail_actions );
        }

        return gen.sample_uniformly( avail_actions );
    }


    state_t state_selection( const std::map< state_t, double > &transitions ) {

        if ( config.state_heuristic == StateSelectionHeuristic::BRTDP ) {
            return bound_difference_state_selection( transitions );
        }

        return gen.sample_distribution( transitions );
    }
                               
    /*
     * samples an MDP trajectory using specified action/successor heuristics
     * and precision, initializing newly encountered state action bounds
     
     * passes distance of bounds for starting state to save computation
     */
    TrajectoryStack sample_trajectory( value_t start_distance ) {
        
        std::ofstream out( config.filename + "_brtdp-logs.txt" , std::ios_base::app );

        std::stack< std::pair< action_t, state_t > > trajectory;
        std::vector< value_t > discount_pow( config.discount_params );

        // get initial state and set up sampling
        auto [ state, _ , terminated ] = env.reset( 0, false );
        terminated = false;

        size_t iter = 0;

        while ( !terminated ) {

            // mark current state and initialize its default bounds
            env.discover( state );

            // select action in this state
            std::vector< action_t > actions = env.get_actions( state );
            action_t action = action_selection( state, actions );

            auto transitions = env.get_transition( state, action );

            // get bound difference for each successor
            std::vector< value_t > diff_values = get_successor_diffs( transitions );


            // get their total sum
            value_t diff_sum = std::accumulate( diff_values.begin(), diff_values.end(), 0.0f );

            std::map< state_t, value_t > diff_distribution;
            size_t succ_count = transitions.size();

            size_t i = 0;
            for ( const auto &[ s, _ ] : transitions ) {
                diff_distribution[ s ] = ( diff_sum == 0 ) ? ( 1.0 / succ_count ) : ( diff_values[ i++ ] / diff_sum );
            }

            // get next state 
            // ( sample from distribution of weighted bound differences )
            state = gen.sample_distribution( diff_distribution );

            trajectory.push( { action, state } );

            // if debug output is turned on, output details of trajectory
            if ( config.trace ) {
                out << iter 
                    << " : Selected action " << action 
                    << " successor state " << state << "\n";

                for ( const auto &[ succ, prob ] : transitions ) {
                    out << "        " << succ << " with p. " << prob << ".\n";
                }

                out << "\n\n";
            }



            // weigh the sum of successor bounds with \gamma^depth
            std::vector< value_t > discount_copy( discount_pow );
            multiply( diff_sum, discount_copy );
            /* 
             * check conditions for terminating sampling 
             */

            // if it is sufficiently close, terminate
            bool sufficiently_close = true;
            for ( value_t val : discount_copy ) {
                if ( val >= config.precision / 100 ) { sufficiently_close = false; }
            }

            if ( sufficiently_close || ( iter >= config.max_depth ) ) {
                terminated = true;
            }

            // raise discount params to higher power
            multiply( discount_pow, config.discount_params );
            iter++;

        }

        return trajectory;
        
    }

    /* BRTDP update of L(s,a) and U(s, a) */
    /*
    void update_bounds( const state_t &s, const action_t &a ) {

        std::map< state_t, double > transitions = env.get_transition( s, a );
        Bounds< value_t > result;

        for ( const auto &[ succ, prob ] : transitions ) {
            Bounds< value_t > successor_bound = env.get_state_bound( succ );
            successor_bound.multiply_bounds( prob );
            result.sum_bound( successor_bound );
        }

        result.multiply_bounds( config.discount_params );
        result.shift_bounds( env.get_expected_reward( s, a ) );
        auto [ ref_point, _ ] = env.min_max_discounted_reward();

        // second parameter is granularity of convex hull
        result.pareto( ref_point, config.precision );

        std::vector< Bounds< value_t > > successors;

        for ( const auto &[ succ, prob ] : transitions ) {
           successors.emplace_back( env.get_state_bound( succ ) );
           successors.back().multiply_bounds( prob );
        }

        result.sum_successors( successors );

        result.multiply_bounds( config.discount_params );
        result.shift_bounds( env.get_expected_reward( s, a ) );
        env.set_bound( s, a, std::move( result ) );
        env.update_state_bound( s );
    }
    */

    /* executes BRTDP updates for the whole sampled trajectory */
    void update_along_trajectory( TrajectoryStack& trajectory, const state_t &starting_state ) {
        while ( !trajectory.empty() ) {
            auto [ a, successor ] = trajectory.top();
            trajectory.pop();
            state_t s = trajectory.empty() ? starting_state : trajectory.top().second;
            env.update_state_action_bound( s, a );
            env.update_state_bound( s );
        }
    }


public:

    // need to move env since it has ownership of solver resources ( bounds )
    BRTDPSolver( EnvironmentHandle &&_env ) :  
                                        env( std::move( _env ) ),
                                        gen( ), 
                                        config( ) {  }

    BRTDPSolver( EnvironmentHandle &&_env ,
                 const ExplorationConfig &config) :  
                                        env( std::move( _env ) ),
                                        gen( ), 
                                        config( config ) {  }

    /* the main BRTDP solver function, samples trajectories and updates bounds
     * until the distance of starting state bounds is less than specified
     * precision, uses discount values specified in constructor 
     * outputs the bound ( pareto objects ) into text files, sampled
     * trajectories during execution and number of discovered states at the end
     */

    void load_environment( Environment< state_t, action_t, std::vector< value_t > > &new_env ) {
        env = EnvironmentHandle( &new_env );
    }


    void set_config( const ExplorationConfig& _config ){
        config = _config;
    }


    /* solves the loaded MDP, utilizing currently loaded config ( discount
     * parameters, precision, etc. )
     *
     * logs are output to filename-logs.txt
     * all bounds are output to filename-all_bounds.txt
     * result pareto curve to filename-result.txt
     */
    Bounds< value_t > solve() {

        auto start_time = std::chrono::steady_clock::now();

        std::ofstream logs( config.filename + "_brtdp-logs.txt" );
        std::ofstream result( config.filename + "_brtdp-result.txt" );

        state_t starting_state = std::get< 0 > ( env.reset( 0 ) );

        // pass config to handler
        env.set_config( config );
        
        Bounds< value_t > start_bound = env.get_state_bound( starting_state );

        size_t episode = 0;

        while ( start_bound.bound_distance() >= config.precision ) {

            TrajectoryStack trajectory = sample_trajectory( start_bound.bound_distance() );
            update_along_trajectory( trajectory, starting_state );

            if ( config.trace ){
                logs << "episode #" << episode << ":\n";
                logs << "distance: " << start_bound.bound_distance() << ".\n";
                logs << start_bound;

                std::cout << "episode #" << episode << "\n.";
                std::cout << "distance" << start_bound.bound_distance() << "\n";
                std::cout << start_bound;
            }

            // get updated starting bound
            start_bound = env.get_state_bound( starting_state );

            episode++;
            if ( episode >= config.max_episodes ) { break; }
        }

        
        auto finish_time = std::chrono::steady_clock::now();
        std::chrono::duration< double > exec_time = finish_time - start_time;
        logs << "\n\n\nTime elapsed: " << exec_time.count() << ".\n";
        logs << "Total episodes: " << episode << ".\n";
        logs << "Total updates: " << env.get_update_num() << "\n";
        logs << "Converged distance: " << start_bound.bound_distance() << ".\n";
        logs << start_bound << std::endl;

        logs.close();
        result.close();
        
        env.write_exploration_logs( config.filename + "_brtdp" , true );
        result << start_bound;

        return start_bound;
    }
};
