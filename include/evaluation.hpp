# pragma once

#include "benchmarks/frozen_lake.hpp"
#include "benchmarks/sea_treasure.hpp"
#include "benchmarks/racetrack.hpp"
#include "benchmarks/resource_gathering.hpp"

#include "models/environment.hpp"
#include "models/env_wrapper.hpp"

#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"
#include "solvers/config.hpp"

#include "parser.hpp"

#include <fstream>
#include <iostream>
#include <string>

// helper struct for outputting to csv
struct LogOutput {
    double time_mean, updates_mean, explored_mean;
    double time_std, updates_std, explored_std;
    size_t didnt_converge;
};

// helper function to aggregate multiple runs
template < typename value_t >
LogOutput aggregate_results( const std::vector< VerificationResult< value_t > > &results ) {
    size_t didnt_converge = 0;
    double time_mean = 0, updates_mean = 0, explored_mean = 0;
    double time_std = 0, updates_std = 0, explored_std = 0;

    double res_size = static_cast< double > ( results.size() );

    for ( const auto& res : results ) {
        if ( !res.converged ) { didnt_converge++; }
        time_mean += static_cast< double > (res.time_to_convergence) / res_size;
        updates_mean += static_cast< double > (res.update_number) / res_size;
        explored_mean += static_cast< double > (res.states_explored) / res_size;
    }

    if ( results.size() > 1 )
    for ( const auto& res : results ) {
        time_std += std::pow( ( static_cast< double > ( res.time_to_convergence ) - time_mean ), 2 ) / ( res_size - 1 );
        updates_std += std::pow( ( static_cast< double > ( res.update_number ) - updates_mean ), 2 ) / ( res_size - 1 );
        explored_std += std::pow( ( static_cast< double > ( res.states_explored ) - explored_mean ), 2 ) / ( res_size - 1 );
    }

    time_std = std::sqrt( time_std );
    updates_std = std::sqrt( updates_std );
    explored_std = std::sqrt( explored_std );
    return LogOutput{ time_mean, updates_mean, explored_mean,
                      time_std, updates_std, explored_std,
                      didnt_converge };
}


void output_curve( const std::string &filename,
                   const ExplorationSettings< double > &config,
                   const VerificationResult< double > &res ){

    std::ofstream curve( "../out/" + filename + "_curve.txt", std::fstream::app );
    auto curve_obj = res.result_bound.lower();
    
    std::vector< double > multipliers = { 1, 1 };
    if ( config.directions[0] == OptimizationDirection::MINIMIZE ) { multipliers[0] = -1; }
    if ( config.directions[1] == OptimizationDirection::MINIMIZE ) { multipliers[1] = -1; }

    curve_obj.multiply_vector( multipliers );

    curve << curve_obj.to_string() << std::endl;
    curve.close();
}



// helper function for evaluation
template < typename state_t, typename action_t, typename value_t >
void run_benchmark( Environment< state_t, action_t, std::vector< value_t > >  *env,
                    const ExplorationSettings< value_t > &config,
                    size_t repeat=5 ){


    EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t > envw( env );
    EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t > chvi_envw( env );

    BRTDPSolver brtdp( std::move( envw ), config );
    CHVIExactSolver chvi( std::move( chvi_envw ), config );

    std::vector< VerificationResult< double > > brtdp_results;
    std::vector< VerificationResult< double > > chvi_results;

    for ( size_t i = 1; i <= repeat; i++ ) {
        auto res_brtdp = brtdp.solve();
        if ( !res_brtdp.converged ) {
            std::cout << config.filename << " : BRTDP run " << i << " did not converge, continuing.\n";
        }
        auto res_chvi = chvi.solve();
        if ( !res_chvi.converged ) {
            std::cout << config.filename << " : CHVI run " << i << " did not converge, continuing.\n";
        }

        brtdp_results.emplace_back( std::move( res_brtdp ) );
        chvi_results.emplace_back( std::move( res_chvi ) );
    }

    LogOutput brtdp_logs = aggregate_results( brtdp_results );
    LogOutput chvi_logs = aggregate_results( chvi_results );
    
    // destination csv file
    std::ofstream out( "../out/results.csv", std::fstream::app );
    std::ofstream expl( "../out/explored.csv", std::fstream::app );

    out << config.filename << ";" << chvi_logs.explored_mean << ";" << brtdp_logs.time_mean << ";" << brtdp_logs.time_std << ";";
    out << brtdp_logs.updates_mean << ";" << brtdp_logs.updates_std << ";";
    out << chvi_logs.time_mean << ";" << chvi_logs.time_std << ";" << chvi_logs.updates_mean << "\n";
    expl << config.filename << ";" << chvi_logs.explored_mean << ";";
    expl << brtdp_logs.explored_mean << ";" << brtdp_logs.explored_std << ";";
    expl << brtdp_logs.didnt_converge << ";" << chvi_logs.didnt_converge << "\n";

    for ( size_t i = 0; i < repeat; i++ ) {
        output_curve( config.filename + "_brtdp", config, brtdp_results[i] );
        output_curve( config.filename + "_chvi", config, chvi_results[i] );

    }
        

    out.close();
    expl.close();
}



void eval_uav( double tau, ActionSelectionHeuristic heuristic ){

    PrismParser parser;

    auto uav5 = parser.parse_model( "../benchmarks/uav/uav5.tra",
                      {
                      "../benchmarks/uav/uav51.trew",
                      "../benchmarks/uav/uav52.trew"
                      },
                      0 );


    auto ptaskgraph5 = parser.parse_model( "../benchmarks/taskgraph/taskgraph5.tra",
            {
                      "../benchmarks/taskgraph/taskgraph52.trew",
                      "../benchmarks/taskgraph/taskgraph51.trew"
                      },
                      0 );
    
    auto teamform3 = parser.parse_model( "../benchmarks/teamform/teamform3.tra",
                                         {
                                            "../benchmarks/teamform/teamform31.trew",
                                            "../benchmarks/teamform/teamform32.trew"
                                         }, 0 );
    
    auto taskgraph30 = parser.parse_model( "../benchmarks/taskgraph2/taskgraph30.tra",
                      {
                      "../benchmarks/taskgraph2/taskgraph301.trew",
                      "../benchmarks/taskgraph2/taskgraph302.trew"
                      },
                      0 );

    // basic config for benchmarks
    ExplorationSettings< double > config;
    config.trace = true;
    
    // no limit on depth or episodes
    config.max_depth = 0;
    config.max_episodes = 0;
    config.max_seconds = 300;

    config.precision = 0.01;
    config.depth_constant = tau;
    config.discount_param = 0.99;

    // use default init
    config.lower_bound_init = {};
    config.upper_bound_init = {};
    config.action_heuristic = heuristic;
    config.directions = { OptimizationDirection::MINIMIZE, OptimizationDirection::MINIMIZE };

    config.filename = "uav5";
    run_benchmark( &uav5, config );

    config.directions = { OptimizationDirection::MAXIMIZE, OptimizationDirection::MAXIMIZE };

    config.filename = "teamform3";
    run_benchmark( &teamform3, config );

    config.directions = { OptimizationDirection::MINIMIZE, OptimizationDirection::MINIMIZE };

    config.filename = "pareto_taskgraph5";
    run_benchmark( &ptaskgraph5, config );

    config.filename = "taskgraph30";
    run_benchmark( &taskgraph30, config );
}

void eval_racetrack( double tau, ActionSelectionHeuristic heuristic ){
    
    ExplorationSettings< double > config;

    config.action_heuristic = heuristic;
    config.max_depth = 100;
    config.max_seconds = 600;
    config.max_episodes = 0;
    config.directions = { OptimizationDirection::MINIMIZE, OptimizationDirection::MINIMIZE };
    config.discount_param = 1;
    config.precision = 0.01;
    config.depth_constant = tau;
    config.lower_bound_init = { -1000, -1000 };
    config.upper_bound_init = { 0, 0 };

    Racetrack easy;
    config.filename = "racetrack-easy";
    config.trace = true;
    easy.from_file("../benchmarks/racetracks/racetrack-easy.track");
    run_benchmark( &easy, config );

    config.filename = "racetrack-ring";
    easy.from_file("../benchmarks/racetracks/racetrack-ring.track");
    run_benchmark( &easy, config );

    config.filename = "racetrack-hard";
    easy.from_file("../benchmarks/racetracks/racetrack-hard.track");
    run_benchmark( &easy, config );
}


void eval_treasure( double tau, ActionSelectionHeuristic heuristic ){

    ExplorationSettings< double > config;
    config.action_heuristic = heuristic;
    config.directions = { OptimizationDirection::MAXIMIZE, OptimizationDirection::MINIMIZE };
    config.max_depth = 0;
    // until convergence
    config.max_episodes = 0;
    config.depth_constant = tau;
    config.discount_param = 0.95;
    config.trace = false;
    config.precision = 0.01;


    DeepSeaTreasure dst, dst_convex;
    dst.from_file( "../benchmarks/treasures/treasure-concave.txt" );
    dst_convex.from_file( "../benchmarks/treasures/treasure-convex.txt" );

    config.filename = "treasure-concave";
    run_benchmark( &dst, config );
    config.filename = "treasure-convex";
    run_benchmark( &dst_convex, config );
}

void eval_frozenlake( double tau, ActionSelectionHeuristic heuristic ){
    ExplorationSettings< double > config;
    config.action_heuristic = heuristic;
    config.max_depth = 0;
    config.max_episodes = 0;
    config.discount_param = 0.95;
    config.max_seconds = 300;
    config.trace = true;
    config.precision = 0.01;
    config.depth_constant = tau;
    config.directions = { OptimizationDirection::MAXIMIZE, OptimizationDirection::MINIMIZE };
    config.filename = "lake-easy";

    FrozenLake lake;

    run_benchmark( &lake, config );

    FrozenLake lake2( 15, 15, {
                       Coordinates(1, 5),
                       Coordinates(1, 8),
                       Coordinates(1, 12),
                       Coordinates(2, 16),
                       Coordinates(3, 21),
                       Coordinates(5, 10),
                       Coordinates(6, 4),
                       Coordinates(6, 24),
                       Coordinates(7, 4),
                       Coordinates(9, 2),
                       Coordinates(9, 15),
                       Coordinates(10, 4),
                       Coordinates(13, 7),
            }, 0.33 );
    config.filename = "lake-hard";
    run_benchmark( &lake2, config );
}

void eval_benchmarks( double tau, ActionSelectionHeuristic heuristic ) {
    eval_uav( tau , heuristic );
    eval_treasure( tau, heuristic );
    eval_frozenlake( tau, heuristic );
    eval_racetrack( tau , heuristic );
}

