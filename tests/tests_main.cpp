# include "brtdp_tests.hpp"
# include "polygon_tests.hpp"
# include "pareto_tests.hpp"
# include "parser_tests.hpp"
# include <iostream>


int main() {



    std::cout << "Testing pareto utilities..\n";
    test_nondominated();
    std::cout << "Success!\n\n\n";

    std::cout << "Testing brtdp..\n";
    test_brtdp_on_simple_mdp();
    std::cout << "Success!\n\n\n";

    test_minkowski_sum();
    return 0;

    /*
    std::cout << "Testing parser..\n";
    test_parser();
    */

}
