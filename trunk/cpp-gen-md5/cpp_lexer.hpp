//
// C++ Lexer implemented with Spirit (http://spirit.sourceforge.net/)
//
// Boost License 1.0
// Copyright© 2002-2003 Juan Carlos Arevalo-Baeza, All rights reserved
// email: jcab@JCABs-Rumblings.com
// Created: 8-Nov-2002
//
// The basics:
// The lexer is an iterator that iterates over tokens generated on the fly
// from a text input.
// In order to use it, you create a "lex_iterator" object, and initialize it
// with the beginning and end iterators of the text input, and a string that
// represents the name of the input file (for inclusion into the tokens).
// This purposefully doesn't export any Spirit parser definitions, so access
// to the input sequence must be done through a generic run-time interface
// (virtual functions). The multi_pass iterator adapter helps handle this
// very well.
//

#ifndef cpp_lexer_hpp_included
#define cpp_lexer_hpp_included

#include "lexer_base.hpp"
#include "cpp_lexer_token.hpp"

namespace cpp {

///////////////////////////////////////////////////////////////////////////////
// Lexer implementation's opaque constructors.
//
// Need one constructor for each supported iterator type.

lexer::input_interface<Token>*
NewLexer(char const* first,
         char const* last,
         char const* fname = "<filename>");

///////////////////////////////////////////////////////////////////////////////
// Lexer iterator type definition.

typedef lexer::iterator<Token> lexer_iterator;

} // cpp

#endif
