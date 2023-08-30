#pragma once

#include <cassert>
#include <limits>
#include <random>

#include "utils/random_utils.hpp"

class PRNG {

    std::random_device rd;
    std::mt19937 gen{ std::random_device{}() };

    std::uniform_int_distribution<int> int_dist;
    std::uniform_real_distribution<double> float_dist; 
    std::uniform_real_distribution<double> prob_dist{ 0.0, 1.0 };

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


    int rand_int(){
        return int_dist( gen );
    }

    int rand_int( int min, int max ){
        return ( rand_int() % ( max - min ) ) + min;
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
        prob_t zero = 0;

        for ( const auto &[ state, prob ] : pm ) {
            p -= prob; 
            if ( ( p < 0 ) || ( approx_zero( p ) ) ) { return state; }
        }

        // undefined
        assert( false );
        return state_t();
    }
};





