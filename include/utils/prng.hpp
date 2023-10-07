#pragma once

#include <cassert>
#include <cmath>
#include <limits>
#include <random>

/* helper class for PRNG, and helper functions for 
 * floating point comparisons, etc. 
 */

template < typename prob_t >
bool approx_equal( prob_t x, prob_t y, prob_t eps=1e-7 ){
    return std::abs( x - y ) < eps;
}

template < typename prob_t >
bool approx_zero( prob_t x, prob_t eps=1e-7 ){
    return approx_equal( x, 0.0 );
}

class PRNG {

    std::random_device rd;
    std::mt19937 gen{ std::random_device{}() };

    std::uniform_int_distribution< int > int_dist;
    std::uniform_real_distribution< double > float_dist; 
    std::uniform_real_distribution< double > prob_dist{ 0.0, 1.0 };

public:
    PRNG() : rd(), gen() {
        gen.seed( rd() );
    }

    void seed() {
        gen.seed( rd() );
    }
    void seed( unsigned seed ) {
        gen.seed( seed );
    }

    int rand_int( int min, int max ){
        int_dist = std::uniform_int_distribution< int >( min, max );
        return int_dist( gen );
    }


    double rand_float() {
        return float_dist( gen );
    }

    double rand_float( double min, double max ) {
        return std::fmod( rand_float() , max - min ) + min;
    }

    template < typename prob_t > 
    prob_t rand_probability() {
        return static_cast< prob_t >( prob_dist( gen ) );
    }

    // sample from distribution container (no validation) //
    template< typename state_t,
              typename prob_t, 
              template < typename, typename > class prob_map_t > 
    state_t sample_distribution( const prob_map_t< state_t, prob_t > &pm ) {

        prob_t p = rand_probability< prob_t >();
        prob_t zero( 0 );

        for ( const auto &[ state, prob ] : pm ) {
            p -= prob; 
            if ( ( p < 0 ) || ( approx_zero( p ) ) ) { return state; }
        }

        // undefined
        assert( false );
        return state_t();
    }

    template<  typename value_t,  
               template < typename > class container_t >
    value_t sample_uniformly( const container_t< value_t > &cont ){ 
        int idx = rand_int( 0, cont.size() - 1 );
        return *std::next( cont.begin(), idx );
    }
};

