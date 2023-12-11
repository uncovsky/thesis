# pragma once

#include "parser.hpp"

#include "models/environment.hpp"
#include "models/env_wrapper.hpp"

#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"
#include "solvers/config.hpp"

void eval_parser( size_t n_times ){

    std::ofstream data( "../out/parsing_time.csv" );
    PrismParser parser;
    
    std::vector< std::string > names = { "uav", "taskgraph", "teamform", "taskgraph2" };
    std::vector< std::vector< double > > results = { {}, {}, {}, {} };

    for ( size_t i = 0; i < n_times; i++ ) {

        auto start_time = std::chrono::steady_clock::now();


        auto uav5 = parser.parse_model( "../benchmarks/uav/uav5.tra",
                          {
                          "../benchmarks/uav/uav51.trew",
                          "../benchmarks/uav/uav52.trew"
                          },
                          0 );

        auto finish_time = std::chrono::steady_clock::now();
        std::chrono::duration< double > exec_time = finish_time - start_time;
        results[0].push_back( exec_time.count() );

        auto ptaskgraph5 = parser.parse_model( "../benchmarks/taskgraph/taskgraph5.tra",
                {
                          "../benchmarks/taskgraph/taskgraph52.trew",
                          "../benchmarks/taskgraph/taskgraph51.trew"
                          },
                          0 );

        finish_time = std::chrono::steady_clock::now();
        exec_time = finish_time - start_time;
        results[1].push_back( exec_time.count() );
        
        auto teamform3 = parser.parse_model( "../benchmarks/teamform/teamform3.tra",
                                             {
                                                "../benchmarks/teamform/teamform31.trew",
                                                "../benchmarks/teamform/teamform32.trew"
                                             }, 0 );
        finish_time = std::chrono::steady_clock::now();
        exec_time = finish_time - start_time;
        results[2].push_back( exec_time.count() );
        
        auto taskgraph30 = parser.parse_model( "../benchmarks/taskgraph2/taskgraph30.tra",
                          {
                          "../benchmarks/taskgraph2/taskgraph301.trew",
                          "../benchmarks/taskgraph2/taskgraph302.trew"
                          },
                          0 );
        finish_time = std::chrono::steady_clock::now();
        exec_time = finish_time - start_time;
        results[3].push_back( exec_time.count() );
    }

    for ( size_t i = 0; i < 4; i++ ) {
        data << names[i] << ";";
        double mean = 0;
        double size = static_cast< double > ( results[i].size() );
        double std = 0;

        for ( auto val : results[i] ) {
            mean += val / size;
        }

        for ( auto val : results[i] ) {
            std += std::pow( val - mean , 2) / ( size - 1 );
        }

        data << mean << ";" << std;
    }
}
