#include "geometry/polygon.hpp"

#include "models/env_wrapper.hpp"
#include "models/mdp.hpp"

#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"
#include "solvers/config.hpp"

#include "utils/prng.hpp"

#include "evaluation.hpp"
#include "parser_evaluation.hpp"

#include "parser.hpp"
#include <iostream>


int main() {

    std::ofstream out( "../out/results.csv" );
    std::ofstream expl( "../out/explored.csv" );
    out << "Benchmark name;num of states;time mean brtdp;time std brtdp;";
    out << "updates brtdp mean;updates brtdp std;time chvi; updates chvi\n";
    expl << "Benchmark name; num of states; mean; std\n";
    out.close();
    expl.close();

    for ( auto heuristic : { ActionSelectionHeuristic::Pareto, ActionSelectionHeuristic::Hausdorff } ) {
        eval_benchmarks( 50, heuristic );
    }
}
