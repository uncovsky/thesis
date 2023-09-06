# include "geometry/geometry.hpp"
# include <iostream>


/*
 * Implementation of the Quickhull algorithm for two dimensions
 * We will use an adjusted version, since we only care about a subset of the
 * convex hull ( i.e only the points that attain maximal value for some weight
 * vector w ) this is equivalent to simply throwing away all the points located
 * in clockwise direction from the first line segment ( right_pts in function
 * quickhull ) since these are dominated by convex combinations of max and min
 * value vectors w.r.t the first objective ( they lie under the line connecting
 * them )
 * 
 */


// get convex hull of a set of points (only works for 2D so far)
template < typename value_t >
std::vector< Point< value_t > > quickhull( const std::vector< Point< value_t > > &points,
                                           double eps = 1e-4 ) {
    if ( points.empty() )
        return {};

    // get lexicographical min/max elements
    auto [ min_it, max_it ] = std::minmax_element( points.begin(), 
                                                   points.end(), 
                                                   std::less< std::vector< value_t > > () );


    Point< value_t > min_x = *min_it, max_x = *max_it;
    if ( min_x == max_x )
        return { min_x };

    std::vector< Point< value_t > > result;

    // select & connect two extremal points with a line segment
    LineSegment< value_t > line( min_x, max_x );


    std::vector< Point< value_t > > left_pts, right_pts;

    // sort points based on which halfspace they belong to
    for ( const auto &p : points ) {
        double ccw_res = line.ccw( p );
        if ( ccw_res < -1 * eps ) { right_pts.push_back(p); }
        else if ( ccw_res > eps )  { left_pts.push_back(p); }
    }

    // the two extremal points lie on the hull, investigate others recursively
    result.push_back( min_x );
    quickhull_rec( result, left_pts, LineSegment< value_t >( { min_x, max_x } ) );

    result.push_back( max_x );

    /* here one would execute the recursive call to the right points in the
     * original quickhull algorithm to get the full hull, however, we ignore
     * these points since they are dominated by convex combinations of the
     * points min_x and max_x 
     * quickhull_rec( result, right_pts, LineSegment< value_t >( { max_x, min_x } ) );
     */

    return result;
}


template< typename value_t >
void quickhull_rec(        std::vector< Point< value_t > >& result , 
                     const std::vector< Point< value_t > >& points , 
                     const LineSegment< value_t > &line,
                           double eps = 1e-4 ) {

    if ( points.empty() )
        return;

    // find furthest point away from the line segment, this point is 
    // on the convex hull as well
    Point< value_t > farthest_pt = points[0];
    value_t max_dist = line.point_distance( farthest_pt );

    for ( const auto &p : points ) {
        value_t dist = line.point_distance( p );
        if ( dist > max_dist ) {
            farthest_pt = p;
            max_dist = dist;
        }
    }


    auto [l, r] = line.get_points();

    std::vector< Point< value_t > > left_pts, right_pts;

    LineSegment< value_t > to_farthest( l, farthest_pt );
    LineSegment< value_t > from_farthest( farthest_pt, r );


    // again, sort points based on their halfspaces
    // we only care about points outside of the triangle formed by the line
    // segment and the farthest point, since others are already encompassed in
    // the convex hull
    for ( const auto &p : points ) {
        double ccw_res_left = to_farthest.ccw( p );
        double ccw_res_right = from_farthest.ccw( p );

        if ( ccw_res_left > eps ) { left_pts.push_back(p); }
        else if ( ccw_res_right > eps ) { right_pts.push_back(p); }
    }


    quickhull_rec( result, left_pts, LineSegment< value_t >( { l , farthest_pt } ) );
    result.push_back( farthest_pt );
    quickhull_rec( result, right_pts, LineSegment< value_t >( { farthest_pt , r } ) );
}

