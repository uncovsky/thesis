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
   // Define the new lower bound points
  std::vector<Point<double>> lower = {
        {-78.4529, -39.2264},
        {-95.7356, -22.0326},
        {-96.8547, -21.084},
        {-97.8344, -20.3598},
        {-101.347, -19.4829},
        {-113.44, -17.0726},
        {-117.649, -16.3005},
        {-122.982, -15.6344},
        {-132.909, -14.6591},
        {-142.173, -14.0772},
        {-161.392, -13.2993},
        {-197.591, -12.9852},
        {-248.92, -12.8797}
    };

    // Define the new upper bound points
    std::vector<Point<double>> upper = {
        {-78.4529, -39.2264},
        {-95.7367, -22.0317},
        {-96.9437, -21.0182},
        {-97.8342, -20.3598},
        {-101.347, -19.4829},
        {-113.402, -17.0798},
        {-117.65, -16.3004},
        {-123.921, -15.5441},
        {-132.917, -14.6589},
        {-142.426, -14.0695},
        {-160.968, -13.3039},
        {-198.654, -12.9801},
        {-247.093, -12.8797}
    };

    Polygon< double > upp( upper ), low( lower );
    Bounds bound( std::move(low), std::move(upp) );
    bound.init_facets();
    bound.downward_closure( { -1000, -1000 } );
    std::cout << bound.bound_distance();
    evaluate_benchmarks("../out/");
}
