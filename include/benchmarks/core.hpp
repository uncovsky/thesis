# pragma once

# include <iostream>
# include <utility>
# include <vector>

enum class Direction {
    UP, DOWN, LEFT, RIGHT
};

struct Coordinates{

    int x, y;
    Coordinates() : x( 0 ), y( 0 ) {}
    Coordinates( int x, int y ) : x( x ), y( y ) {}
    Coordinates( std::pair< int, int > c ) : x( c.first ), y( c.second ) {}

    Coordinates& operator+=( const Coordinates& rhs ) {
        x += rhs.x;
        y += rhs.y;

        return *this;
    }

    Coordinates operator+( const Coordinates& rhs ) const {
        Coordinates copy = *this;
        copy += rhs;
        return copy;
    }

    Coordinates operator-=( const Coordinates& rhs ) {
        x -= rhs.x;
        y -= rhs.y;

        return *this;
    }

    Coordinates operator-( const Coordinates& rhs ) const {
        Coordinates copy = *this;
        copy -= rhs;
        return copy;
    }

    bool operator==( const Coordinates &rhs ) const {
        return x == rhs.x && y == rhs.y;
    }


    bool operator<( const Coordinates& rhs ) const {
        return ( x < rhs.x ) || ( ( x == rhs.x ) && ( y < rhs.y ) );
    }

    friend std::ostream &operator<<( std::ostream& os, const Coordinates &c ) {
        os << "(" << c.x << ", " << c.y << ")";
        return os;
    }

};


inline Coordinates dir_to_vec( Direction dir ) {
    int dx = 0, dy = 0;
    switch ( dir ) {
        case Direction::UP:
            dy = -1;
            break;
        case Direction::DOWN:
            dy = 1;
            break;
        case Direction::LEFT:
            dx = -1;
            break;
        case Direction::RIGHT:
            dx = 1;
            break;
    }
    return Coordinates( dx, dy );
}

inline bool collides( const Coordinates& pos, Direction dir, size_t height, size_t width ) {

    Coordinates next = pos + dir_to_vec( dir );

    if ( ( 0 > next.x ) || ( next.x >= width ) || 
         ( 0 > next.y ) || ( next.y >= height ) ) {
        return true;
    }

    return false;
}


// returns valid perpendicular moves from pos w.r.t. dir
inline std::vector< Direction > valid_perpendicular( const Coordinates& pos, 
                                            Direction dir,
                                            size_t height,
                                            size_t width ){
    std::vector< Direction > res;

    if ( ( dir == Direction::UP ) || ( dir == Direction::DOWN ) ) {
        for ( Direction perp : { Direction::LEFT, Direction::RIGHT } ){
            if ( !collides( pos, perp, height, width ) ) {
                res.push_back( perp );
            }
        }
    }

    else {
        for ( Direction perp : { Direction::DOWN, Direction::UP } ){
            if ( !collides( pos, perp, height, width ) ) {
                res.push_back( perp );
            }
        }
    }

    return res;
}
