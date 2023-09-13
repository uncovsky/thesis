# pragma once
# include "geometry/geometry.hpp"
# include "utils/geometry_utils.hpp"


/*
 * Implementation of the Quickhull algorithm for two dimensions
 * We will use an adjusted version, since we only care about a subset of the
 * convex hull ( i.e only the points that attain maximal value for some weight
 * vector w ) this is equivalent to simply throwing away all the points located
 * in clockwise direction from the first line segment ( right_pts in function
 * quickhull ) since these are dominated by convex combinations of max and min
 * value vectors w.r.t the first objective
 */


// get convex hull of a set of points (only works for 2D so far)

template< typename value_t >
void quickhull_rec(        std::vector< Point< value_t > >& result , 
                     const std::vector< Point< value_t > >& points , 
                     const Point< value_t > &x1,
                     const Point< value_t > &x2,
                           double eps ) {

    if ( points.empty() )
        return;

    // find furthest point away from the line segment, this point is 
    // on the convex hull as well
    Point< value_t > farthest_pt = points[0];
    value_t max_dist = line_segment_distance( x1, x2, farthest_pt );

    for ( const auto &p : points ) {
        value_t dist = line_segment_distance( x1, x2, p );
        if ( dist > max_dist ) {
            farthest_pt = p;
            max_dist = dist;
        }
    }



    std::vector< Point< value_t > > left_pts, right_pts;

    // again, sort points based on their halfspaces
    // we only care about points outside of the triangle formed by the line
    // segment and the farthest point, since others are already encompassed in
    // the convex hull
    for ( const auto &p : points ) {
        double ccw_res_left = ccw( x1, farthest_pt, p );
        double ccw_res_right = ccw( farthest_pt, x2, p );

        if ( ccw_res_left > eps ) { left_pts.push_back(p); }
        else if ( ccw_res_right > eps ) { right_pts.push_back(p); }
    }


    quickhull_rec( result, left_pts, x1, farthest_pt, eps );
    result.push_back( farthest_pt );
    quickhull_rec( result, right_pts, farthest_pt, x2, eps);
}

template < typename value_t >
std::vector< Point< value_t > > quickhull( const std::vector< Point< value_t > > &points, 
                                           double eps = 1e-4 ) {
    if ( points.empty() )
        return {};

    // need [least, most] , [most most]
    auto max_it = std::max_element( points.begin(), 
                                                   points.end(), 
                                                   std::less< std::vector< value_t > > () );

    auto min_it = std::min_element( points.begin(), points.end(),
            []( auto p1, auto p2 ) { 
                return ( p1[0] < p2[0] ) || ( ( p1[0] == p2[0] ) && ( p1[1] > p2[1] ));
            });

    Point< value_t > min_x = *min_it, max_x = *max_it;
    if ( min_x == max_x )
        return { min_x };

    std::vector< Point< value_t > > result;


    std::vector< Point< value_t > > left_pts, right_pts;

    // sort points based on which halfspace they belong to
    for ( const auto &p : points ) {
        double ccw_res = ccw( min_x, max_x, p );
        if ( ccw_res < -1 * eps ) { right_pts.push_back( p ); }
        else if ( ccw_res > eps )  { left_pts.push_back( p ); }
    }

    // the two extremal points lie on the hull, investigate others recursively
    result.push_back( min_x );
    quickhull_rec( result, left_pts, min_x, max_x, eps );

    result.push_back( max_x );

    /* here one would execute the recursive call to the right points in the
     * original quickhull algorithm to get the full hull, however, we ignore
     * these points since they are dominated by convex combinations of the
     * points min_x and max_x 
     * quickhull_rec( result, right_pts, LineSegment< value_t >( { max_x, min_x } ), eps );
     */

    return result;
}

