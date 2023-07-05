#pragma once

#include <random>
#include <limits>



class Seeder {

    std::random_device rand_dev;
    std::uniform_int_distribution< unsigned > seed_dist{0, std::numeric_limits<unsigned>::max()};
    std::mt19937 gen{rand_dev()};

public:
    
    Seeder() = default;

    Seeder(unsigned seed){
        set_seed(seed);
    }

    unsigned get_seed(){
        return seed_dist(gen);
    }

    void set_seed(unsigned seed){
        gen.seed(seed);
    }


};

template < typename prob_t >
bool approx_zero(prob_t x, prob_t eps=1e-7){
    return std::abs(x - 0) < eps;
}


class PRNG {

    std::random_device rd;
    std::mt19937 gen;

    std::uniform_int_distribution<int> int_dist;
    std::uniform_real_distribution<double> float_dist; 
    std::uniform_real_distribution<double> prob_dist{0.0, 1.0};

public:
    PRNG(){
        gen.seed( rd() );
    }

    void seed(){
        gen.seed( rd() );
    }
    void seed(unsigned seed){
        gen.seed( seed );
    }

    double rand_float(double min, double max){
        return std::fmod( float_dist(gen), max - min ) + min;
    }

    double rand_float(){
        return float_dist(gen);
    }

    template < typename prob_t > 
    prob_t rand_probability(){
        return static_cast< prob_t >(prob_dist(gen));
    }

    // sample from distribution container (no validation) //
    template< typename state_t,
              typename prob_t, 
              template < typename, typename > class prob_map_t > 
    state_t sample_distribution( const prob_map_t< state_t, prob_t > &pm){

        prob_t p = rand_probability< prob_t >();
        prob_t zero = 0;
        
        for (const auto &[state, prob] : pm){
            p -= prob; 
            if (approx_zero(p)) { return state; }
        }

        return 0;
    }

    int rand_int(int min, int max){
        return int_dist(gen) % (max - min) + min;
    }

    int rand_int(){
        return int_dist(gen);
    }

};





