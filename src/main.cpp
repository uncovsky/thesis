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
        {-78.4531, -39.2264},
        {-93.6775, -24.0112},
        {-94.5902, -23.1126},
        {-95.6046, -22.1315},
        {-96.6134, -21.2645},
        {-97.8161, -20.3619},
        {-98.6321, -20.1433},
        {-101.636, -19.3973},
        {-103.003, -19.1213},
        {-108.089, -18.1074},
        {-112.393, -17.2534},
        {-117.081, -16.3946},
        {-117.762, -16.2878},
        {-122.675, -15.6755},
        {-130.415, -14.8816},
        {-133.787, -14.5973},
        {-140.244, -14.169},
        {-142.143, -14.0689},
        {-156.427, -13.4625},
        {-160.694, -13.2973},
        {-162.497, -13.2583},
        {-163.982, -13.2331},
        {-167.111, -13.1946},
        {-193.188, -12.9921},
        {-198.088, -12.9713},
        {-236.659, -12.8914},
        {-243.348, -12.8817},
        {-247.085, -12.8797}
    };

    // Define the new upper bound points
    std::vector<Point<double>> upper = {
        {-78.4045, -43.1042},
        {-78.4349, -39.6046},
        {-78.4494, -39.2269},
        {-93.5834, -24.102},
        {-94.6894, -23.0164},
        {-95.7088, -22.0416},
        {-96.6246, -21.2554},
        {-97.8159, -20.3619},
        {-98.6319, -20.1434},
        {-101.637, -19.3966},
        {-103.003, -19.1213},
        {-108.088, -18.1075},
        {-112.392, -17.2535},
        {-117.146, -16.3843},
        {-118.191, -16.234},
        {-122.663, -15.6766},
        {-130.473, -14.8763},
        {-133.783, -14.5975},
        {-140.248, -14.1688},
        {-142.146, -14.0686},
        {-156.416, -13.462},
        {-160.684, -13.2965},
        {-162.501, -13.257},
        {-163.841, -13.2346},
        {-166.994, -13.1959},
        {-193.22, -12.9917},
        {-198.051, -12.9714},
        {-236.971, -12.8908},
        {-243.598, -12.8815},
        {-247.091, -12.8797}
    };

    Polygon< double > upp( upper ), low( lower );
    Bounds bound( std::move(low), std::move(upp) );
    bound.init_facets();
    bound.downward_closure( { -1000, -1000 } );
    std::cout << bound.bound_distance();
    evaluate_benchmarks("../out/");
}
