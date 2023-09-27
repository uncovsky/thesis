# include <iostream>
# include "parser.hpp"

/* 
* State & action indices are given as unsigned long
* What is checked:
*  SxAxS triplets don't repeat ( in both transition, reward files )
*  for each SxA tuple the sum of probabilities 
*      across all associated triplets is 1.0
*      any one SxAxS probability is \in [0, 1.0]
*  transition file / reward files exist
*  
* otherwise ParseError is thrown
*
* The rewards in .trew files ( given as SxAxS rewards ) are translated (
* weighted ) to expected SxA rewards.
* Missing rewards ( i.e SxA tuples that appear in the transition file but not
* in the reward file ) are assigned zero when building the resulting model.
*
* rewards are assumed to be floating point numbers 
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

    // load remaining digits ( possibly zero )
    return id + load_digits();
}

// accepts [0-9]^*
std::string PrismParser::load_digits( ){

    std::string id;
    char token = get_token();
    while ( check( std::isdigit ) ) {
        id.push_back( token );
        token = get_token();
    }

    return id;
}


double PrismParser::load_float(){
    std::string flt;

    if ( check( '-' ) ) {
        flt += '-';
    }

    flt.push_back( get_token() );
    require( std::isdigit );
    
    if ( check( '.' ) ){
        // if a dot is present, load >= 1 remaining digit
        flt += '.' + load_unsigned();
    }
    
    else {
        // load remaining ( potentially zero ) digits
        flt += load_digits();
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

// matches a SxAxS triplet from transition / reward file
std::tuple< size_t, size_t, size_t > PrismParser::match_triplet(){
    remove_all( std::isspace );

    std::string s = load_unsigned();
    remove_all( std::isspace );

    std::string a = load_unsigned();
    remove_all( std::isspace );

    std::string succ = load_unsigned();
    remove_all( std::isspace );

    // convert relevant info to indices in maps / triplets
    // commented out for now
    size_t s_id = string_to_ull( s );
    size_t a_id = string_to_ull( a );
    size_t succ_id = string_to_ull( succ );

    if ( translate_indices ) {
        s_id = translate( s, true );
        succ_id = translate( succ, true );
        a_id = translate( a, false );
    }

    return { s_id, a_id, succ_id };
}


void PrismParser::match_transition( ){

    auto [ s_id, a_id, succ_id ] = match_triplet();
    double p = load_float();

    if ( ( p <= 0 ) || ( p > 1 ) ) {
        throw ParseError( line_num, "Invalid transition probability\n" );
    }

    /* 
     * transition files may contain text labels of states, 
     * so this is incorrect, could handle it in some other way later,
     * but ignoring the rest works as well
    remove_all( std::isspace );
    require( '\0' );
    */

    transition_info[ s_id ].add_triplet( a_id, succ_id, p );
}

void PrismParser::match_reward( ){
    // match triplet and translate to indices in maps
    auto [ s_id, a_id, succ_id ] = match_triplet();

    if ( ( transition_info.find( s_id ) == transition_info.end() ) ||
         ( !transition_info[s_id].contains( a_id, succ_id ) ) ) {
        throw ParseError( line_num, "This reward transition is not present in the transition file.\n" );
    }

    // get all reward dimensions ( one file may have multiple )
    std::vector< double > rewards;

    // at least one dimension must be present
    do {
        rewards.push_back( load_float() );
        remove_all( std::isspace );
    }

    while ( !check( '\0' ) );

    // get number of currently allocated reward structures for this reward file
    int avail_dimensions = reward_info.size() - reward_dimension;

    // if this row needs more dimensions that is currently allocated, create them
    if ( rewards.size() > avail_dimensions ) {
        for ( int i = 0; i < rewards.size() - avail_dimensions; i++ ) {
            reward_info.push_back(TripletList());
        }
    }

    // update rewards
    for ( size_t i = 0; i < rewards.size(); i++ ) {
        update_expected_reward( s_id, a_id, succ_id, rewards[ i ], reward_dimension + i );
    }


}

// reduce SxAxS reward to SxA
// s_id, a_id, succ_id is the associated transition
// reward the assoc reward, idx the index ( dimension )
void PrismParser::update_expected_reward( size_t s_id, size_t a_id, size_t succ_id,
                             double reward,
                             size_t dim ){

    auto idx = std::make_pair( a_id, succ_id );

    // weigh reward by probability 
    reward *= transition_info[ s_id ].triplets[ idx ];

    // update contents of the structure
    auto &reward_structure = reward_info[dim];

    if ( reward_structure.contains( a_id, s_id ) ){
        idx = std::make_pair( a_id, s_id );
        reward_structure.triplets[ idx ] += reward;
    }

    else { reward_structure.add_triplet( a_id, s_id, reward ); }
}


// loads ( and removes previous ) transition probability file 
void PrismParser::parse_transition_file( const std::string &filename ){
    std::ifstream input_str( filename );

    if ( input_str.fail() ) {
        throw ParseError( 1, "Transition file" + filename + " does not exist." );
    }

    // reset all associated data
    line_num = 1;
    reward_dimension = 0;
    transition_info.clear();
    reward_info.clear();

    while ( std::getline( input_str, line ) ) {
        // set iterators
        if ( !ignore_line( line ) ) {
            curr = line.begin();
            end  = line.end();
            // match transition
            match_transition();
        
        }
        line_num++;
    }

    for ( const auto &[ id, data ] : transition_info ){
        if ( !data.valid_probabilities() ){
            throw ParseError( 1 , "invalid transition probabilities for state mapped to index " + std::to_string( id ) + " \n");
        }
    }
}


void PrismParser::parse_reward_file( const std::string &filename ){

    std::ifstream input_str( filename );

    if ( input_str.fail() ) {
        throw ParseError( 1, "Reward file" + filename + " does not exist." );
    }

    line_num = 1;

    while ( std::getline( input_str, line ) ) {
        if ( !ignore_line( line ) ) {
            curr = line.begin();
            end  = line.end();
            match_reward();
        }
        line_num++;
    }

    // set dimension for next file
    reward_dimension = reward_info.size();
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
        throw ParseError( line_num, "-> " + str + " <- " + "required token mismatch.\n" );
}

void PrismParser::require( char token ){
    std::string str;
    str.push_back( get_token() );
    if ( !check( token ) )
        throw ParseError( line_num , "-> " + str + " <- " + "required token mismatch.\n" );
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

    // set missing rewards as zero
    for ( const auto &[ s, triplet ] : transition_info ){

        // for each SxA with nonzero probability
        for ( const auto &[ a, _ ] : triplet.prob_sums ){

            for ( size_t i = 0; i < reward_info.size(); i++ ){
                if ( !reward_info[i].contains( a, s ) ) {
                    reward_info[i].add_triplet( a, s, 0.0 );
                }
            }

        }
    }
    
    for ( size_t i = 0; i < reward_info.size(); i++ ) {
        rewards.emplace_back( reward_info[i].build_matrix() );

        auto [ min, max ] = reward_info[i].get_min_max_value();
        auto& [ min_vec, max_vec ] = bounds;

        min_vec.push_back( min );
        max_vec.push_back( max );
    }

    if ( translate_indices ) {
        initial_state = translate( std::to_string( initial_state ), true );
    }

    return MDP< double >( std::move( transitions )
                        , std::move( rewards )
                        , bounds
                        , initial_state );
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

    // output error message and rethrow ( terminate )
    catch ( const ParseError &e ) {
        std::cout << e.what() << std::endl;
        throw e;
    }
}


bool PrismParser::ignore_line( const std::string &line) {

    return line.empty() || ( line[0] == '#' ) ;
}


// this works, apparently
size_t PrismParser::string_to_ull( const std::string &input ) {

    size_t size;
    std::istringstream iss( input );

    iss >> size;

    return size;
}
