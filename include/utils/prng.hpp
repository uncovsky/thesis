#pragma once

#include <random>
#include <limits>

#include "utils/random_utils.hpp"

class PRNG {

    std::random_device rd;
    std::mt19937 gen;

    std::uniform_int_distribution<int> int_dist;
    std::uniform_real_distribution<double> float_dist; 
    std::uniform_real_distribution<double> prob_dist{ 0.0, 1.0 };

public:
    PRNG() {
        gen.seed( rd() );
    }

    void seed() {
        gen.seed( rd() );
    }
    void seed( unsigned seed ) {
        gen.seed( seed );
    }

    double rand_float( double min, double max ) {
        return std::fmod( float_dist( gen ), max - min ) + min;
    }

    double rand_float() {
        return float_dist(gen);
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
        prob_t zero = 0;
        
        for ( const auto &[ state, prob ] : pm ) {
            p -= prob; 
            if ( approx_zero( p ) ) { return state; }
        }

        return 0;
    }

    int rand_int( int min, int max ) {
        return int_dist( gen ) % ( max - min ) + min;
    }

    int rand_int() {
        return int_dist( gen );
    }

};





