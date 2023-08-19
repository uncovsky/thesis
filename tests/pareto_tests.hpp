# pragma once
# include <cassert>

# include "geometry/pareto.hpp"
# include "utils/eigen_types.hpp"


void test_nondominated(  ){

    
    std::vector< Point< int > > test1 ({ { 0, 0 }, {0, 0} });
    remove_dominated_alt(test1);
    assert( test1 == std::vector< Point< int > >( { { 0, 0 } } ) );


    std::vector< Point< double > > result( { { 1.3, 1.0 }, { 1.2, 1.1 }, { 1.1, 1.2 }, { 0.5, 2.0 }, { 0.0, 3.0 } } );


    std::vector< Point< double > > result_copy( result );
    std::vector< Point< double > > test;

    remove_dominated_alt( result_copy );
    assert( result == result_copy );

    for ( const auto& p : result ) {
        for ( size_t i = 0; i < 20; i++ ) {
            test.push_back(p);
        }
    }

    remove_dominated_alt( test );
    assert( test == result );

    nondominated_union( result_copy, std::vector< Point< double > > ({
                                                                       { 0.0, 0.0 },
                                                                       { 0.0, 0.0 },
                                                                       { -3.0, 0.9 }
                                                                       }) );
    assert( result_copy == result );

    nondominated_union( result_copy, std::vector< Point< double > > ({}));

    assert( result_copy == result );

    nondominated_union( result_copy, std::vector< Point< double > >({
                                                                       { 1.0, 1.0 },
                                                                       { 1.0, 1.0 },
                                                                       { -3.0, 0.9 },
                                                                       { 3.5, 6.0 }
                                                                       }) );
    result = { { 3.5, 6.0 } };
    assert( result_copy == result );

    std::vector< Point< double > > test2({ { 1.05, 1.03 },
                                    { 1.00, 1.00 },
                                    { 2.00, 1.01 },
                                    { 0.0, -5.0 },
                                    { -3.0, 7.0},
                                    { 0.0, -5.0 },
                                    { 5.1 , 3.3} });

    std::vector< Point< int > > test3;
    for ( int i = 0; i < 30 ; i++ ){
        for ( int j = 0; j < i + 1; j++ ){
            test3.push_back( { i, i } );
        }
    }

    assert ( is_dominated< int> ( { 26, 26 }, { 28, 28 } ) );
    remove_dominated_alt( test3 );

    assert( test3 == std::vector< Point< int > >( {  { 29, 29 } } ) );

    remove_dominated_alt( test2 );
    assert( test2 == std::vector< Point< double > >({
                                                       { -3.0, 7.0 },
                                                       { 5.1, 3.3 } 
                                                 }));

}
