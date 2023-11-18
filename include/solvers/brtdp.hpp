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
        return gen.sample_uniformly( pareto_actions );
        
    }

    action_t hypervolume_action( const state_t &s, const std::vector< action_t > &avail_actions ) {
        auto [ ref_point , _ ] = env.get_initial_bound();

        std::vector< value_t > hypervolumes;
        value_t total_hypervolume( 0 );

        for ( const action_t &act : avail_actions ) {
            auto upper_bound_vertices = env.get_state_action_bound( s, act ).upper().get_vertices();
            hypervolumes.push_back( hypervolume_indicator( upper_bound_vertices, ref_point ) );

            /*

            std::cout << "  Action: " << act << "\n";
            for ( auto pt : upper_bound_vertices ) {
                for ( auto dim : pt ) {
                    std::cout << "      " << dim << ", ";
                }
                std::cout << "\n";
            }

            std::cout << "HV: " << hypervolumes.back() << "\n";

            */

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
            Bounds< value_t > &bound = env.get_state_bound( s );
            diff_values.push_back( bound.bound_distance() * prob );
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

        return gen.sample_uniformly( avail_actions );
    }

    /*
     * samples an MDP trajectory using specified action/successor heuristics
     * and precision, initializing newly encountered state action bounds
     */
    TrajectoryStack sample_trajectory() {
        
        std::ofstream out( config.filename + "_brtdp-logs.txt" , std::ios_base::app );

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



            /* 
             * check conditions for terminating sampling 
             *
             * if sum of differences of successor bounds are sufficiently close
             * or max depth has been reached 
             */
            diff_sum *= discount_pow;

            if ( ( diff_sum < config.precision / 1000 ) || ( iter >= config.max_depth ) ) {
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

        std::ofstream logs( config.filename + "_brtdp-logs.txt" );
        std::ofstream result( config.filename + "_brtdp-result.txt" );

        state_t starting_state = std::get< 0 > ( env.reset( 0 ) );
    

        // pass config to handler
        env.set_config( config );

        // initialize starting state bound
        env.discover( starting_state );
        
        Bounds< value_t > start_bound = env.get_state_bound( starting_state );

        size_t episode = 0;

        while ( start_bound.bound_distance() >= config.precision ) {

            TrajectoryStack trajectory = sample_trajectory();
            update_along_trajectory( trajectory, starting_state );

            if ( config.trace ){
                logs << "episode #" << episode << ":\n";
                logs << "distance: " << start_bound.bound_distance() << ".\n";
                logs << start_bound;

                std::cout << "episode #" << episode << "\n.";
                std::cout << start_bound;
            }

            start_bound = env.get_state_bound( starting_state );

            episode++;
            if ( episode >= config.max_episodes ) { break; }
        }

        
        auto finish_time = std::chrono::steady_clock::now();
        std::chrono::duration< double > exec_time = finish_time - start_time;
        VerificationResult< value_t > res{  env.get_update_num() // num of updates
                                       , start_bound.bound_distance() < config.precision // bool converged
                                       , start_bound 
                                       , exec_time.count()
                                       , env.num_states_explored() }; // num of explored states
                                        
        return res;
    }
};
