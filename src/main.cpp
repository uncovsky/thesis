#include "geometry/pareto.hpp"
#include "geometry/polygon.hpp"

#include "models/env_wrapper.hpp"
#include "models/mdp.hpp"

#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"
#include "solvers/config.hpp"

#include "utils/prng.hpp"

#include "evaluation.hpp"
#include "parser.hpp"
#include <iostream>


int main() {
    for ( int i = 2; i <= 5; i++ ){
        evaluate_benchmarks("../out/" + std::to_string(i) + "_");
    }
}
