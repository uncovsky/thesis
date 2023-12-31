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

    // prng for selecting actions/successors
    PRNG gen;

    /*
     * ACTION HEURISTICS 
     */

    /* pareto set evaluation heuristic from pareto q-learning:
     * 1) look at upper bounds of available actions
     * 2) select action uniformly from all actions that have >= 1 nondominated
     * vector in their upper bound across avail_actions 
     */
    action_t pareto_action( const state_t &s, const std::vector< action_t > &avail_actions ) {

        std::vector< Point< value_t > > &nondominated = env.get_state_bound( s ).upper().get_vertices();

        std::set< action_t > pareto_actions;

        for ( const action_t &a : avail_actions ) {
            std::vector< Point< value_t > > &sa_points = env.get_state_action_bound( s, a ).upper().get_vertices();

            bool opt = false;
            for ( const auto &pt : sa_points ) {

                // the convex hull operation sorts in reverse order, hence the
                // comparator
                if ( std::binary_search( nondominated.begin(), 
                                         nondominated.end(), 
                                         pt, 
                                         []( const auto &x, const auto &y ){ return x[0] > y[0]; }) )  
                {
                    pareto_actions.insert( a );
                    opt = true;
                }

                if ( opt ) { break; }
            }
        }

        // choose uniformly from these actions
        return gen.sample_uniformly( pareto_actions );
        
    }

    // hypervolume action selection
    action_t hypervolume_action( const state_t &s, const std::vector< action_t > &avail_actions ) {
        auto [ ref_point , _ ] = env.min_max_discounted_reward();

        std::vector< size_t > maximizing_indices;

        value_t max_hv( 0 );

        for ( size_t i = 0; i < avail_actions.size(); i++ ){
            auto &bound = env.get_state_action_bound( s, avail_actions[i] );
            
            value_t hv = bound.hypervolume( ref_point );

            if ( hv > max_hv ) {
                max_hv = hv;
                maximizing_indices = { i };
            }

            else if ( hv == max_hv ) {
                maximizing_indices.push_back( i );
            }
        }

        size_t idx = gen.sample_uniformly( maximizing_indices );
        return avail_actions[idx];
        
    }

    action_t furthest_action_selection( const state_t &s, const std::vector< action_t > &avail_actions ){

        std::vector< Point< value_t > > furthest_pts = env.get_state_bound( s ).get_furthest_points();

        std::set< action_t > maximizing_actions;

        for ( const action_t &a : avail_actions ) {
            std::vector< Point< value_t > > &sa_points = env.get_state_action_bound( s, a ).upper().get_vertices();

            bool opt = false;
            for ( const auto &pt : furthest_pts ) {

                if ( std::binary_search( sa_points.begin(), 
                                         sa_points.end(), 
                                         pt, 
                                         []( const auto &x, const auto &y ){ return x[0] > y[0]; }) )  
                {
                    maximizing_actions.insert( a );
                    opt = true;
                }

                if ( opt ) { break; }
            }
        }

        // choose uniformly from these actions
        return gen.sample_uniformly( maximizing_actions );
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
            Bounds< value_t > &bound = env.get_state_bound( s );
            diff_values.push_back( bound.hausdorff_distance() * prob );
        }

        return diff_values;
    }

    // picks action from avail actions based on specified heuristic
    action_t action_selection( const state_t &s, const std::vector< action_t > &avail_actions ) {

        if ( config.action_heuristic == ActionSelectionHeuristic::Pareto ) {
            return pareto_action( s, avail_actions );
        }

        if ( config.action_heuristic == ActionSelectionHeuristic::Hypervolume ){
            return hypervolume_action( s, avail_actions );
        }

        return furthest_action_selection( s, avail_actions );
    }

    

    /*
     * samples an MDP trajectory using specified action/successor heuristics
     * and precision, initializing newly encountered state action bounds
     */
    TrajectoryStack sample_trajectory() {
        
        std::stack< std::pair< action_t, state_t > > trajectory;

        value_t discount_pow = config.discount_param;

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

            // (potentially) discover & initialize successors
            for ( const auto&[ succ, prob ] : transitions ) {
                env.discover( succ );
            }

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

            /* 
             * check conditions for terminating sampling 
             *
             * if sum of differences of successor bounds are sufficiently close
             * or max depth has been reached 
             */
            diff_sum *= discount_pow;

            if ( ( config.depth_constant > 0 ) && ( diff_sum < config.precision / config.depth_constant ) || 
                 ( config.max_depth > 0 ) && ( iter >= config.max_depth ) ) {
                terminated = true;
            }

            discount_pow *= config.discount_param;
            iter++;

        }

        return trajectory;
        
    }

    /* executes BRTDP updates for the whole sampled trajectory */
    void update_along_trajectory( TrajectoryStack& trajectory, const state_t &starting_state ) {
        while ( !trajectory.empty() ) {

            // ignoring tail state where no action was played
            auto [ a, successor ] = trajectory.top();
            trajectory.pop();

            state_t s = trajectory.empty() ? starting_state : trajectory.top().second;

            env.update_bound( s, a );
            env.update_bound( s );
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
    VerificationResult< value_t > solve() {

        auto start_time = std::chrono::steady_clock::now();

        state_t starting_state = std::get< 0 > ( env.reset( 0 ) );
    

        // pass config to handler
        env.set_config( config );

        // initialize starting state bound
        env.discover( starting_state );
        
        Bounds< value_t > start_bound = env.get_state_bound( starting_state );

        size_t episode = 0;

        while ( start_bound.hausdorff_distance() >= config.precision ) {

            TrajectoryStack trajectory = sample_trajectory();
            update_along_trajectory( trajectory, starting_state );

            if ( config.trace ){
                std::cout << "episode #" << episode << "\n.";
                std::cout << "distance: " << start_bound.hausdorff_distance() << ".\n";
                std::cout << start_bound;
            }

            start_bound = env.get_state_bound( starting_state );

            episode++;
            // if max episodes is set to 0, no limit.
            if ( ( config.max_episodes > 0 ) && ( episode >= config.max_episodes ) )  { break; }

            auto finish_time = std::chrono::steady_clock::now();
            std::chrono::duration< double > exec_time = finish_time - start_time;

            if ( exec_time.count() > config.max_seconds ) { break; }

        }
    
        auto finish_time = std::chrono::steady_clock::now();
        std::chrono::duration< double > exec_time = finish_time - start_time;
        VerificationResult< value_t > res{ env.get_update_num() // num of updates
                                       , start_bound.hausdorff_distance() < config.precision // bool converged
                                       , start_bound 
                                       , exec_time.count()
                                       , env.num_states_explored() }; // num of explored states
                                        
        return res;
    }
};
