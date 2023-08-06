# include <iostream>
# include "parser.hpp"

/* first pass
 * find the number of states and for each state the number of associated
 * actions and successors
 *
 * validate the state names ( non-negative integers )
 *  - map them to indices
 * validate the probabilities ( floats between 0.0 - 1.0 )
 * validate that SxAxS triplets don't repeat
 * accumulate the necessary triplets ? 
 *
 * second pass
 *  build the model from triplets
 */

bool PrismParser::eol() const {
    return curr == end;
}

char PrismParser::get_token() {
    return eol() ? '\0' : *curr;
}



// accepts strings of digits of length >= 1 ( initial zeroes allowed )
std::string PrismParser::load_unsigned( ) {
    std::string id;
    char token = get_token();
    require( std::isdigit );
    id.push_back( token );

    token = get_token();

    while ( check( std::isdigit ) ) {
        id.push_back( token );
        token = get_token();
    }

    return id;
}


// accepts and loads a floating point number
// TODO: fxi this
double PrismParser::load_float(){
    std::string flt;

    if ( check( '-' ) ) {
        flt += '-';
    }

    flt.push_back( get_token() );
    require( std::isdigit );
    
    if ( check( '.' ) ){
        flt += '.' + load_unsigned();
    }
    
    else {
        char tok = get_token();
        while( check( std::isdigit ) ) {
            flt.push_back( tok );
            tok = get_token();
        }
    }

    return std::stod( flt );
}

// names of states & actions in the file are mapped to first available indices
// which are then used to construct the triplets, flag signalizes whether a
// state or an action is to be translated
size_t PrismParser::translate( const std::string& name, bool state ){

    size_t id = string_to_ull( name );
    std::map< size_t, size_t > &target = state ? state_to_index : action_to_index;

    // first available index;
    size_t index = target.size();

    if ( target.find( id ) == target.end() ) {
        target[ id ] = index;
    }

    // translate
    return target[ id ];
}

// PRISM format (differs from storm by order s, succ, a rather than s, a, succ
// and also some additional hints for the file ( # states, #transitions, which
// we ignore anyway )
std::tuple< size_t, size_t, size_t > PrismParser::match_triplet(){
    remove_all( std::isspace );

    std::string s = load_unsigned();
    remove_all( std::isspace );

    std::string a = load_unsigned();
    remove_all( std::isspace );

    std::string succ = load_unsigned();
    remove_all( std::isspace );

    // convert relevant info to indices
    size_t s_id = translate( s, true );
    size_t succ_id = translate( succ, true );
    size_t a_id = translate( a, false );

    return { s_id, a_id, succ_id };
}


void PrismParser::match_transition( ){

    auto [ s_id, a_id, succ_id ] = match_triplet();
    double p = load_float();

    if ( ( p <= 0 ) || ( p > 1 ) ) {
        throw ParsingError( line_num, "Invalid transition probability\n" );
    }

    /* 
     * transition files may contain labels, so this is incorrect, could handle
     * in some other way layer
    remove_all( std::isspace );
    require( '\0' );
    */

    transition_info[ s_id ].add_triplet( a_id, succ_id, p );
}

void PrismParser::match_reward( ){
    auto [ s_id, a_id, succ_id ] = match_triplet();

    if ( ( transition_info.find( s_id ) == transition_info.end() ) ||
         ( !transition_info[s_id].contains( a_id, succ_id ) ) ) {
        throw ParsingError( line_num, "This reward transition is not present in the transition file.\n" );
    }

    double reward = load_float();
    remove_all( std::isspace );
    require( '\0' );
    
    // REDUCE TO SxA reward
    auto idx = std::make_pair( a_id, succ_id );

    // weigh reward by probability 
    //
    reward *= transition_info[ s_id ].triplets[ idx ];


    auto &reward_structure = reward_info.back();

    if ( reward_structure.contains( a_id, s_id ) ){
        idx = std::make_pair( a_id, s_id );
        reward_structure.triplets[idx] += reward;
    }

    else { reward_structure.add_triplet( a_id, s_id, reward ); }
    auto [ _, max ] = reward_structure.get_min_max_value();
}


void PrismParser::parse_transition_file( const std::string &filename ){
    std::ifstream input_str( filename );
    if ( input_str.fail() ) {
        throw ParsingError( 0, "Transition file" + filename + " does not exist." );
    }

    line_num = 0;
    transition_info.clear();

    while ( std::getline( input_str, line ) ) {
        line_num++;
        // set iterators
        if ( !ignore_line( line ) ) {
            curr = line.begin();
            end  = line.end();
            // match transition
            match_transition();
        
        }
    }

    // TODO: could maintain reverse mappings ( from indices to original state
    // names in the file ) for better error messages
    for ( const auto &[ id, data ] : transition_info ){
        if ( !data.valid_probabilities() ){
            throw ParsingError(0, "invalid transition probabilities for state mapped to index " + std::to_string( id ) + " \n");
        }
    }
}


void PrismParser::parse_reward_file( const std::string &filename ){


    reward_info.push_back({});
    std::ifstream input_str( filename );

    if ( input_str.fail() ) {
        throw ParsingError( 0, "Reward file" + filename + " does not exist." );
    }

    line_num = 0;
    while ( std::getline( input_str, line ) ) {
        line_num++;
        if ( !ignore_line( line ) ) {
            curr = line.begin();
            end  = line.end();
            match_reward();
        }
    }

    // TODO: set misssing rewards as zero
}


bool PrismParser::check( int (* callback)( int ) ){
    if ( callback( get_token() ) ) {
        curr++;
        return true;
    }

    return false;
}


bool PrismParser::check( char token ){
    if ( token == get_token() ) {
        curr++;
        return true;
    }

    return false;
}

// match all tokens
bool PrismParser::remove_all( int (* callback)( int ) ){
    bool matched_one = false;

    while ( check( callback ) ) {
        matched_one = true;
    }

    return matched_one;
}

void PrismParser::require( int (* callback)( int ) ){
    std::string str;
    str.push_back( get_token() );
    if ( !check( callback ) )
        throw ParsingError( line_num, "-> " + str + " <- " + "required token mismatch.\n" );
}

void PrismParser::require( char token ){
    std::string str;
    str.push_back( get_token() );
    if ( !check( token ) )
        throw ParsingError( line_num , "-> " + str + " <- " + "required token mismatch.\n" );
}

// initial state given here is the number present in the file, not the index
// its mapped to in parser struct
MDP< double > PrismParser::build_model( size_t initial_state ){

    Matrix3D< double > transitions;

    for ( size_t i = 0; i < transition_info.size(); i++ ) {
        transitions.emplace_back( transition_info[i].build_matrix() );
    }

    Matrix3D< double > rewards;
    std::pair< std::vector< double >, std::vector< double > > bounds;
    
    for ( size_t i = 0; i < reward_info.size(); i++ ) {
        rewards.emplace_back( reward_info[i].build_matrix() );

        auto [ min, max ] = reward_info[i].get_min_max_value();
        auto& [ min_vec, max_vec ] = bounds;

        min_vec.push_back( min );
        max_vec.push_back( max );
    }


    return MDP< double >( std::move( transitions ),
                          std::move( rewards ), 
                          bounds, 
                          initial_state );
}


MDP< double > PrismParser::parse_model( const std::string &transition_file,
                                        const std::vector< std::string > &reward_files,
                                        size_t initial_state ) {
    //new model -> new rewards, transitions get reset in parse transition file
    reward_info.clear();

    try{

        std::cout << "Parsing " << transition_file << std::endl;
        parse_transition_file( transition_file );
        for ( const auto &str : reward_files ){
            std::cout << "Parsing " << str << std::endl;
            parse_reward_file( str );
        }
        return build_model( initial_state );
    }

    catch ( const ParsingError &e ) {
        std::cout << e.what() << std::endl;
        throw e;
    }
}

// TODO: remove whitespace bfore checknig
bool PrismParser::ignore_line( const std::string &line) {
    return line.empty() || ( line[0] == '#' ) ;
}

size_t PrismParser::string_to_ull( const std::string &input ) {

    std::istringstream iss(input);
    size_t size;
    iss >> size;

    return size;
}
