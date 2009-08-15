//
// Lexer token basic interface
//
// Boost License 1.0
// Copyright© 2003 Juan Carlos Arevalo-Baeza, All rights reserved
// email: jcab@JCABs-Rumblings.com
// Created: 8-Feb-2003
//
// This defines the basic interface with which a lexer token must comply,
// which is just a traits class.
//

#ifndef lexer_token_base_hpp_included
#define lexer_token_base_hpp_included

namespace lexer {

///////////////////////////////////////////////////////////////////////////////
// Required token traits.
//
// When creating a lexer, you need to define these traits for the token type.
// You do that by specializing the traits class.

template < typename TokenT >
struct token_traits; /*
    static TokenT const eof_token;  // Token that signifies the end of file.
    typename file_position;         // The type used to hold a file position.
*/

} // lexer

#endif
