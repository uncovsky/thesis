# include "brtdp_tests.hpp"
# include "pareto_tests.hpp"
# include "parser_tests.hpp"
# include "polygon_tests.hpp"
# include <iostream>

int main() {
    std::cout << "Testing geometry utilities..\n";
    test_euclidean_distance();
    std::cout << "Testing pareto utilities..\n";
    test_nondominated();
    std::cout << "Success!\n\n\n";

    /* commented for now, since the tests are in fact useless, we don't want
     * the full convex hull, but only a subset ( we only care about points that
     * have maximal value w.r.t some weight w, not all vertices that lie on the
     * hull )
     */
    // std::cout << "Testing convex hull..\n";
    // test_convex_hull();
    // std::cout << "Success!\n\n\n";

    std::cout << "Testing hausdorff distance..\n";
    test_hausdorff_distance();
    std::cout << "Success!\n\n\n";

    std::cout << "Testing brtdp..\n";
    test_brtdp_on_simple_mdp();
    std::cout << "Success!\n\n\n";

    std::cout << "Testing parser..\n";
    test_parser();

    return 0;
}
