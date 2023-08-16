# pragma once

# include "geometry/polygon.hpp"
# include "models/env_wrapper.hpp"
# include "utils/geometry_utils.hpp"
# include <cassert>


template< typename value_t >
bool check_convex_hull( const std::vector< std::vector< value_t > > &original_points,
                        const std::vector< std::vector< value_t > > &hull_points,
                        const std::vector< LineSegment< value_t > > &facet_points )
{
    Polygon< value_t > result( hull_points, facet_points );
    Polygon< value_t > input( original_points);
    input.convex_hull();

    bool check_result = input == result;
    if ( ! check_result ) {
        Polygon< value_t > copy( original_points );

        copy.write_to_file( "ch_input.txt" );
        input.write_to_file( "ch_output.txt" );
        result.write_to_file( "ch_output_expected.txt" );
    }

    return check_result;
}


/* function to test the quickhull functionality, facets are given as oriented
 * line segments beginning from the first point lexicographically, in ccw
 * direction.
 * Note that if facets are points ( either degenerate 2D hull, 1D case ), degenerate line segments are used to
 * represent them
 *  if the convex hull is a two dimensional line, then both
 * facets ( in each direction A->B, B->A ) are returned
 */


void test_convex_hull() {

    std::vector < std::vector< int > > line_vertices = { { -10 }, { -5 }, { 2 }, { 10 }, { 25 } };
    std::vector < std::vector< int > > hull_vertices = { { -10 } , { 25 } };


    assert( check_convex_hull( line_vertices, hull_vertices, { LineSegment< int >( { -10 }, { -10 } ), LineSegment< int> ( { 25 }, { 25 } ) } ) );



    std::vector < std::vector< int > > line2D = { { -10, -10 }, { 2, 2 }, { 5, 5 }, { 300, 300 } } ;

    std::vector < std::vector< int > > line2Dhull = { { -10, -10 } , { 300, 300 } } ;

    assert( check_convex_hull( line2D, 
                               line2Dhull, 
                               { LineSegment< int >( { -10, -10 }, { 300, 300 } ),
                                 LineSegment< int > ( { 300, 300 }, {-10, -10} ) }
                             ) );


    std::vector< std::vector< double > > polygon1vert ( { { 5.0, 2.0 },
                                                   { 5.0, 3.0 },
                                                   { 6.5, 3.0 },
                                                   { 5.0, 3.5 },
                                                   { 5.5, 3.5 },
                                                   { 3.0, 4.0 },
                                                   { 4.5, 4.0 },
                                                   { 6.0, 4.0 },
                                                   { 5.25, 4.5 },
                                                   { 4.5, 5.0 },
                                                   { 6.5, 5.0 } } );

    std::vector< std::vector< double > > polygon1hullvert ( { { 5.0, 2.0 },
                                                           { 6.5, 3.0 },
                                                           { 3.0, 4.0 },
                                                           { 4.5, 5.0 },
                                                           { 6.5, 5.0 } } );

    std::vector< LineSegment< double > > polygon1hullfacets;

    polygon1hullfacets.push_back( LineSegment< double >( { 6.5, 3.0 }, { 5.0, 2.0 } ) );
    polygon1hullfacets.push_back( LineSegment< double >( { 5.0, 2.0 }, { 3.0, 4.0 } ) );
    polygon1hullfacets.push_back( LineSegment< double >( { 3.0, 4.0 }, { 4.5, 5.0 } ) );
    polygon1hullfacets.push_back( LineSegment< double >( { 4.5, 5.0 }, { 6.5, 5.0 } ) );
    polygon1hullfacets.push_back( LineSegment< double >( { 6.5, 5.0 }, { 6.5, 3.0 } ) );


    assert( check_convex_hull( polygon1vert, polygon1hullvert, polygon1hullfacets ) );
}

template < typename value_t > 
bool check_hausdorff_distance( const std::vector< std::vector< value_t > > &lower_vertices,
                               const std::vector< std::vector< value_t > > &upper_vertices,
                               const std::vector< value_t > &minimal_point,
                               value_t expected_distance ){

    Bounds< value_t > res( lower_vertices, upper_vertices );
    res.pareto( minimal_point );
    return res.bound_distance() == expected_distance;
}



void test_euclidean_distance(){
    assert( euclidean_distance< double > ( { 0, 0 }, { 0, 0 } )  == 0 );
    assert( euclidean_distance< double > ( { 0, 10 }, { 0, 0 } )  == 10 );
    assert( euclidean_distance< double > ( { 12, 4 }, { 0, 0 } )  == std::sqrt( 160 ) );
    assert( euclidean_distance< double > ( { 12, 4 }, { 2, 2 } )  == std::sqrt( 104 ) );
    assert( euclidean_distance< double > ( { 1, 5 }, { -10, 14 } )  == std::sqrt( 202 ) );
}

void test_hausdorff_distance(){
    std::vector< std::vector< double > > vertices1( { { 12, 4 } } );
    std::vector< std::vector< double > > vertices2( { { 0, 0 } } );

    assert( check_hausdorff_distance( vertices2, vertices1, { 0, 0 }, std::sqrt( 160 ) ) );
}




