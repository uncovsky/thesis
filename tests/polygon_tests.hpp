# pragma once

# include "geometry/polygon.hpp"
# include <cassert>

void test_convex_hull() {

    std::set < std::vector< int > > line_vertices = { { -10 }, { -5 }, { 2 }, { 10 }, { 25 } };
    std::set < std::vector< int > > hull_vertices = { { -10 } , { 25 } };


    Polygon< int > line_test( line_vertices );
    Polygon< int > line( hull_vertices, { LineSegment< int >( { -10 }, { 25 } ) } );


    line_test.convex_hull();
    assert( line_test == line );


    std::set < std::vector< int > > line2D = { { -10, -10 }, { 2, 2 }, { 5, 5 }, { 300, 300 } } ;

    std::set < std::vector< int > > line2Dhull = { { -10, -10 } , { 300, 300 } } ;

    Polygon< int > line2Dtest( line2D );
    Polygon< int > line2Dtrue( line2Dhull,  {LineSegment< int >( { -10, -10 }, { 300, 300 }) });

    line2Dtest.convex_hull();
    assert( line2Dtest == line2Dtrue );

    std::set< std::vector< double > > polygon1vert ( { { 5.0, 2.0 },
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

    std::set< std::vector< double > > polygon1hullvert ( { { 5.0, 2.0 },
                                                           { 6.5, 3.0 },
                                                           { 3.0, 4.0 },
                                                           { 4.5, 5.0 },
                                                           { 6.5, 5.0 } } );

    std::set< LineSegment< double > > polygon1hullfacets;

    polygon1hullfacets.insert( LineSegment< double >( { 5.0, 2.0 }, { 6.5, 3.0 } ) );
    polygon1hullfacets.insert( LineSegment< double >( { 5.0, 2.0 }, { 3.0, 4.0 } ) );
    polygon1hullfacets.insert( LineSegment< double >( { 3.0, 4.0 }, { 4.5, 5.0 } ) );
    polygon1hullfacets.insert( LineSegment< double >( { 4.5, 5.0 }, { 6.5, 5.0 } ) );
    polygon1hullfacets.insert( LineSegment< double >( { 6.5, 3.0 }, { 6.5, 5.0 } ) );

    Polygon< double > polygon1( polygon1vert );
    Polygon< double > polygon1hull( polygon1hullvert, polygon1hullfacets );

    polygon1.convex_hull();

    assert( polygon1hull == polygon1 );

}





