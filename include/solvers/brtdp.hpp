# pragma once
#include <algorithm>
#include <stack> 
#include "models/env_wrapper.hpp"
#include "solvers/config.hpp"
#include "utils/eigen_types.hpp"
#include "utils/prng.hpp"


/* templated type value_t is the underlying type used to represent reward
 * values of the model
 */

template < typename state_t, typename action_t, typename value_t >
class BRTDPSolver{

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
    action_t pareto_action( const state_t &s, const std::vector< action_t > &avail_actions ) {

        using vertex_vec = typename std::vector< Point< value_t > >; 

        std::vector< vertex_vec > bounds;

        std::set< action_t > pareto_actions;

        // copy vertices of upper bounds
        for ( const action_t &a : avail_actions ) {
            vertex_vec vertices;

            // get vertices of respective upper bound and copy them into a set
            vertices = env.get_state_action_bound( s, a ).upper().get_vertices();
            bounds.emplace_back( std::move( vertices ) );

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
        return uniform_action( pareto_actions );
        
    }

    action_t hypervolume_action( const state_t &s, const std::vector< action_t > &avail_actions ) {
        auto [ ref_point , _ ] = env.min_max_discounted_reward();

        std::vector< action_t > maximizing_actions;
        value_t max_hypervolume( 0 );

        for ( const action_t &act : avail_actions ) {
            auto upper_bound_vertices = env.get_state_action_bound( s, act ).upper().get_vertices();
            value_t hypervolume = ( hypervolume_indicator( upper_bound_vertices, ref_point ) );

            if ( hypervolume > max_hypervolume ){
                maximizing_actions = { act };
            }

            else if ( hypervolume == max_hypervolume ){
                maximizing_actions.push_back( act );
            }

            max_hypervolume = std::max( hypervolume, max_hypervolume );
        }

        return uniform_action( maximizing_actions );
    }
    /*
     * SUCCESSOR HEURISTICS 
     *
     * BRTDP heuristic for successor selection, uniformly samples from
     * successors maximizing weighted difference of their state bounds 
     * argmax ( delta( s, a , s' ) * ( dist_H( L_i( s' ), U_i( s' ) ) ) )
     */
    state_t bound_difference_state_selection( const std::map< state_t, double > &transitions ) {

        value_t diff_sum( 0 );
        std::vector< value_t > diff_values;
        for ( const auto &[ s, prob ] : transitions ) {
            diff_values.push_back( env.get_state_bound( s ).bound_distance() * prob );
            diff_sum += diff_values.back();
        }

        size_t i = 0;
        std::map< state_t, value_t > diff_distribution;
        for ( const auto &[ s, _ ] : transitions ) {
            diff_distribution[ s ] = diff_values[ i ] / diff_sum;
            i++;
        }

        return gen.sample_distribution( diff_distribution );
    }

    // picks action from avail actions based on specified heuristic
    action_t action_selection( state_t s, const std::vector< action_t > &avail_actions ) {

        if ( config.action_heuristic == ActionSelectionHeuristic::Pareto ) {
            return pareto_action( s, avail_actions );
        }

        if ( config.action_heuristic == ActionSelectionHeuristic::Hypervolume ){
            return hypervolume_action( s, avail_actions );
        }

        return uniform_action( avail_actions );
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
            /*
            if ( config.trace )
                std::cout << iter 
                          << " : Selected action " << action 
                          << " successor state " << state << "\n";
            */

            /* check termination ( whether gamma^iter * max_value is < precision )
             * then we can safely stop the trajectory ( cut off the tail )
             */
         
            // discount_copy = discount_params^(iter+1) * max_value
            multiply( discount_copy, config.discount_params );

            // if a limit on trajectory depth is set, check for termination
            if ( ( config.max_trajectory != 0 ) && ( iter >= config.max_trajectory ) ){
                break;
            }

            // if all components are < precision, terminate
            bool terminated = true;
            for ( value_t val : discount_copy )  {
               terminated &= std::abs( val ) < precision; 
            }

            if ( terminated )
               break; 
            
            iter++;

        }

        return trajectory;
        
    }

    /* BRTDP update of L(s,a) and U(s, a) */
    void update_bounds( state_t s, action_t a ) {
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
        result.pareto( ref_point );

        /*
        std::vector< Bounds< value_t > > successors;

        for ( const auto &[ succ, prob ] : transitions ) {
           successors.emplace_back( env.get_state_bound( succ ) );
           successors.back().multiply_bounds( prob );
        }

        result.sum_successors( successors );

        result.multiply_bounds( config.discount_params );
        result.shift_bounds( env.get_expected_reward( s, a ) );
        */
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

        std::ofstream logs( config.filename + "_brtdp-logs.txt" );
        std::ofstream result( config.filename + "_brtdp-result.txt" );

        state_t starting_state = std::get< 0 > ( env.reset( 0 ) );

        // pass config to handler
        env.set_config( config );
        
        // get initial upper and lower bounds on objective
        auto [ minimal_value , maximal_value ] = env.get_initial_bound();
        Bounds< value_t > start_bound = env.get_state_bound( starting_state );

        size_t episode = 0;

        while ( start_bound.bound_distance() >= config.precision ) {

            TrajectoryStack trajectory = sample_trajectory( maximal_value, config.precision );
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

        
        logs << "Total episodes: " << episode << ".\n";
        logs << "Converged distance: " << start_bound.bound_distance() << ".\n";
        logs << start_bound;
        
        env.write_exploration_logs( config.filename + "_brtdp" , true );
        result << start_bound;

        return start_bound;
    }
};
