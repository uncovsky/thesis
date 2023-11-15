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
    // eval_uav( "../out/" );
    //eval_treasure( "../out/" );
    evaluate_benchmarks("../out/");
}
