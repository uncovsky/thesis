# include "models/environment.hpp"
# include "models/env_wrapper.hpp"
# include "solvers/brtdp.hpp"
# include "solvers/chvi.hpp"

# include <fstream>
# include <iostream>
# include <string>


template< typename state_t, typename action_t, typename reward_t >
class TestCore{
    CHVIExactSolver< state_t, action_t, reward_t > chvi;
    BRTDPSolver< state_t, action_t, reward_t > brtdp;


};



