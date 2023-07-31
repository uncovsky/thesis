# include "polygon_tests.hpp"
# include "brtdp_tests.hpp"
# include <iostream>

int main() {
    std::cout << "Testing geometry utilities..\n";
    test_euclidean_distance();
    std::cout << "Success!\n\n\n";

    std::cout << "Testing convex hull..\n";
    test_convex_hull();
    std::cout << "Success!\n\n\n";

    std::cout << "Testing hausdorff distance..\n";
    test_hausdorff_distance();
    std::cout << "Success!\n\n\n";

    std::cout << "Testing brtdp..\n";
    test_brtdp_on_simple_mdp();
    std::cout << "Success!\n\n\n";

    return 0;
}
