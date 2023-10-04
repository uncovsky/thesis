#include "models/environment.hpp"
#include "models/env_wrapper.hpp"
#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"
#include <fstream>
#include <iostream>
#include <string>




template < typename state_t, typename action_t, typename value_t >
void run_benchmark( Environment< state_t, action_t, std::vector< value_t >, value_t > env,
                    const ExplorationSettings< value_t > &config ){


    EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t > envw( &env );
    EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t > chvi_enw( &env );

    BRTDPSolver brtdp( std::move( envw ) );
    brtdp.set_config( config );
}

