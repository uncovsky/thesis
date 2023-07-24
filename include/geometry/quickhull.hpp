# include "geometry/geometry.hpp"
# include <iostream>


/*
 * Implementation of the Quickhull algorithm for two dimensions
 */


// get convex hull of a set of points (only works for 2D so far)
template < typename value_t >
std::vector< Point< value_t > > quickhull( const std::set< Point< value_t > > &points,
                                           double eps = 1e-10 ) {
    if ( points.empty() )
        return {};

    // get lexicographical min/max elements
    auto [ min_it, max_it ] = std::minmax_element( points.begin(), 
                                                   points.end(), 
                                                   std::less< std::vector< value_t > > () );


    Point<value_t> min_x = *min_it, max_x = *max_it;
    std::vector< Point< value_t > > result;


    // select & connect two extremal points with a line segment
    LineSegment< value_t > line( min_x, max_x );

    std::set< Point< value_t > > left_pts, right_pts;

    // sort points based on which halfspace they belong to
    for ( const auto &p : points ) {
        double ccw_res = ccw( min_x, max_x, p ) ;
        if ( ccw_res < -1 * eps ) { right_pts.insert(p); }
        else if ( ccw_res > eps )  { left_pts.insert(p); }
    }

    // the two extremal points lie on the hull, investigate others recursively
    result.push_back( min_x );
    quickhull_rec( result, left_pts, LineSegment< value_t >( { min_x, max_x } ) );

    result.push_back( max_x );
    quickhull_rec( result, right_pts, LineSegment< value_t >( { max_x, min_x } ) );

    return result;
}


template< typename value_t >
void quickhull_rec(        std::vector< Point< value_t > >& result , 
                     const std::set< Point< value_t > >& points , 
                     const LineSegment< value_t > &line,
                           double eps = 1e-10 ) {

    if ( points.empty() )
        return;

    // find furthest point away from the line segment, this point is 
    // on the convex hull as well
    Point< value_t > farthest_pt = *(points.begin());
    value_t max_dist = line.point_distance( farthest_pt );

    for ( const auto &p : points ) {
        value_t dist = line.point_distance( p );
        if ( dist > max_dist ) {
            farthest_pt = p;
            max_dist = dist;
        }
    }


    auto [l, r] = line.get_points();

    std::set< Point< value_t > > left_pts, right_pts;


    // again, sort points based on their halfspaces
    // we only care about points outside of the triangle formed by the line
    // segment and the farthest point, since others are already encompassed in
    // the convex hull
    for ( const auto &p : points ) {
        double ccw_res_left = ccw( l , farthest_pt, p ) ;
        double ccw_res_right = ccw( farthest_pt, r,  p ) ;

        if ( ccw_res_left > eps ) { left_pts.insert(p); }
        if ( ccw_res_right > eps ) { right_pts.insert(p); }
    }


    quickhull_rec( result, left_pts, LineSegment< value_t >( { l , farthest_pt } ) );
    result.push_back( farthest_pt );
    quickhull_rec( result, right_pts, LineSegment< value_t >( { farthest_pt , r } ) );
}

