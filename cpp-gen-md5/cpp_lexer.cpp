//
// C++ Lexer implemented with Spirit (http://spirit.sourceforge.net/)
//
// Boost License 1.0
// Copyright© 2002-2003 Juan Carlos Arevalo-Baeza, All rights reserved
// email: jcab@JCABs-Rumblings.com
// Created: 8-Nov-2002
//

#include "cpp_lexer.hpp"

#include <iostream>
#include <fstream>

#include <stdio.h>
#include <limits.h> // INT_MAX needed by boost spirit

#include <boost/version.hpp>

#if BOOST_VERSION < 103700
#include <boost/spirit/symbols.hpp>
#include <boost/spirit/attribute.hpp>
#include <boost/spirit/core.hpp>
#include <boost/spirit/utility/functor_parser.hpp>

#include <boost/spirit/phoenix/primitives.hpp>
#include <boost/spirit/phoenix/casts.hpp>
#include <boost/spirit/phoenix/binders.hpp>

using namespace boost::spirit;

#else
#include <boost/spirit/include/classic_symbols.hpp>
#include <boost/spirit/include/classic_attribute.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_functor_parser.hpp>

#include <boost/spirit/include/phoenix1_primitives.hpp>
#include <boost/spirit/include/phoenix1_casts.hpp>
#include <boost/spirit/include/phoenix1_binders.hpp>

using namespace boost::spirit::classic;
#endif


///////////////////////////////////////////////////////////////////////////////
// Used namespaces and identifiers.

using std::stringstream;
using std::string;
using std::cout;
using std::cerr;
using std::endl;

using phoenix::var;
using phoenix::val;
using phoenix::value;
using phoenix::actor;
using phoenix::arg1;
using phoenix::arg2;
using phoenix::construct_;
using phoenix::function_ptr;

namespace cpp {

///////////////////////////////////////////////////////////////////////////////
// Utility parsers for debugging and error handling.

namespace private_stuff { // Private stuff.

// Trace parser used for debugging.
// Just include trace_p(<arg>) parsers in the middle of a grammar,
// and it will print out a useful diagnosting whenever it's
// executed. The argument can be a string or an actor that
// evaluates to a string.
template <typename ErrorDescrT>
class trace_ {

public:
    typedef nil_t result_t;

    trace_(ErrorDescrT const& what) : info(what) {
    }

    template <typename ScannerT>
    int
    operator()(ScannerT const& scan, result_t& result) const {
        file_position lc = (*scan).filePos;
        cout << lc << ": Trace: " << info() << "\n";
        return 0;
    }

private:

    ErrorDescrT info;
};

template < class ActorT >
static
functor_parser<trace_<ActorT> >
trace_p(ActorT const& str) {
    return trace_<ActorT>(str);
}

static
functor_parser<trace_<actor<value<std::string> > > >
trace_p(std::string const& str) {
    return trace_<actor<value<std::string> > >(val(str));
}

static
functor_parser<trace_<actor<value<char const*> > > >
trace_p(char const* str) {
    return trace_<actor<value<char const*> > >(val(str));
}

// Utility closure that defines a result value.
// Used to return values from grammars.
template < typename ResultT >
struct result_closure : closure<result_closure<ResultT>, ResultT> {
    typedef closure<result_closure<ResultT>, ResultT> base_t;
    typename base_t::member1 result_;
};

template <>
struct result_closure<nil_t> {
    typedef parser_context<nil_t> context_t;
};

// Parser to extract the current file position from the scanner.
struct get_file_position_parser {
    file_position& filePos;
    get_file_position_parser(file_position& filePos_) :
        filePos(filePos_)
    {
    }
    typedef nil_t result_t;
    template < typename ScannerT >
    int operator()(ScannerT const& scan, result_t& result) const {
        filePos = scan.first.get_position();
        return 0;
    }
};

functor_parser<get_file_position_parser>
get_file_position_p(file_position& filePos)
{
    return get_file_position_parser(filePos);
}

} // Private stuff.

///////////////////////////////////////////////////////////////////////////////
// The C++ lexer grammars.
//
// These parse all the basic terminal tokens: identifiers, characters, strings,
// integers and floats. Symbols are dealt with separately: there's just a
// pre-set number of them, so they are parsed using the token DB's
// symbols<> class.
//
// All these grammars return a std::string with the extracted token's text.

namespace private_stuff { // Private stuff.

struct IDENTIFIER :
    grammar<IDENTIFIER, result_closure<std::string>::context_t>
{
    template < typename ScannerT >
    struct definition {
        typedef rule<ScannerT> rule_t;
        rule_t main;

        rule_t const& start() const {
            return main;
        }

        definition(IDENTIFIER const& self) {
            main = (
                lexeme_d[
                    ((alpha_p | '_' | '$') >> *(alnum_p | '_' | '$'))
                    [self.result_ = construct_<std::string>(arg1, arg2)]
                ]
                );
        }
    };
} IDENTIFIER_g;

struct STRING_LITERAL :
    grammar<STRING_LITERAL, result_closure<std::string>::context_t>
{
    template < typename ScannerT >
    struct definition {
        typedef rule<ScannerT> rule_t;
        rule_t main;

        rule_t const& start() const {
            return main;
        }

        definition(STRING_LITERAL const& self) {
            bool is_wchar = false;

            main = (
                lexeme_d[
                    (
                        !(as_lower_d[chlit<>('L')] [var(is_wchar) = true])
                        >> '\"'
                        >> *(str_p("\\\\") | "\\\"" | (anychar_p - '\"') )
                    )
                    [self.result_ = construct_<std::string>(arg1, arg2)]
                    >> chlit<>('\"')
                ] >> *lexeme_d[
                    !as_lower_d[chlit<>('L')] >> '\"'
                    >> ( *( str_p("\\\\") | "\\\"" | (anychar_p - '\"' )) )
                    [self.result_ += construct_<std::string>(arg1, arg2)]
                    >> chlit<>('\"')
                ]
                );
        }
    };
} STRING_LITERAL_g;

struct CHARACTER_LITERAL :
    grammar<CHARACTER_LITERAL, result_closure<std::string>::context_t>
{
    template < typename ScannerT >
    struct definition {
        typedef rule<ScannerT> rule_t;
        rule_t main;

        rule_t const& start() const {
            return main;
        }

        definition(CHARACTER_LITERAL const& self) {
            bool is_wchar = false;

            main = (
                lexeme_d[
                    (
                        !(as_lower_d[chlit<>('L')] [var(is_wchar) = true])
                        >> '\''
                        >> +(str_p("\\\\") | "\\\'" | (anychar_p - '\'') )
                    )
                    [self.result_ = construct_<std::string>(arg1, arg2)]
                    >> chlit<>('\'')
                ]
                );
        }
    };
} CHARACTER_LITERAL_g;

struct INT_CONSTANT :
    grammar<INT_CONSTANT, result_closure<std::string>::context_t>
{
    template < typename ScannerT >
    struct definition {
        typedef rule<ScannerT> rule_t;
        rule_t main;

        rule_t const& start() const {
            return main;
        }

        definition(INT_CONSTANT const& self) {
            subrule<0> submain;
            subrule<1> hex_int;
            subrule<2> oct_int;
            subrule<3> dec_int;
            subrule<4> char_int;
            subrule<5> suffix_part;

            main = (
                submain =
                    (hex_int | oct_int | dec_int | char_int) [
                        self.result_ =
                            construct_<std::string>(arg1, arg2)
                    ],

                hex_int =
                    lexeme_d[
                        '0' >> as_lower_d[chlit<>('x')]   // prefix
                        >> +xdigit_p                       // the number
                        >> suffix_part                     // suffix
                    ],

                oct_int =
                    lexeme_d[
                        '0'                             // prefix
                        >> +range<>('0', '7')              // the number
                        >> suffix_part                     // suffix
                    ],

                dec_int =
                    lexeme_d[
                        +digit_p                        // the number
                        >> suffix_part                     // suffix
                    ],

                char_int = CHARACTER_LITERAL_g,

                suffix_part = !as_lower_d[chlit<>('l') | chlit<>('u')]
                );
        }
    };
} INT_CONSTANT_g;

struct FLOAT_CONSTANT :
    grammar<FLOAT_CONSTANT, result_closure<std::string>::context_t>
{
    template < typename ScannerT >
    struct definition {
        typedef rule<ScannerT> rule_t;
        rule_t main;

        rule_t const& start() const {
            return main;
        }

        definition(FLOAT_CONSTANT const& self) {
            subrule<0> submain;
            subrule<1> exponent_part;

            main = (
                submain =
                    lexeme_d[
                        (
                            chlit<>('.') >> +digit_p >> !exponent_part
                            | +digit_p >> (
                                (chlit<>('.') >> *digit_p)
                                || exponent_part
                                )
                        ) >> !as_lower_d[chlit<>('l') | chlit<>('f')]
                    ]
                    [
                        self.result_ =
                            construct_<std::string>(arg1, arg2)
                    ],

                exponent_part =
                    as_lower_d[chlit<>('e')]
                    >> !(chlit<>('+') | chlit<>('-')) >> +digit_p
                );
        }
    };
} FLOAT_CONSTANT_g;

} // Private stuff.

///////////////////////////////////////////////////////////////////////////////
// Main lexer grammar.
//
// This is the main workhorse. It returns an array of Token's wrapped in a
// TokenLookup structure. This is done so that a single pass of the grammar
// may generate more than one token. Currently, this is only used with
// directives, so that both EOL tokens (before and after) can be generated.
// Directives can only happen after an EOL token, so both the previous EOL
// and the directive are returned.
//
// TODO: This could be obviated by adding context data to the scanner.

// Internal transfer. Defined in cpp_lexer_tokens.cpp.
extern parser<symbols<TokenID> > const& cpp_operator_p;

namespace private_stuff { // Private stuff.

enum { MaxTokenLookup = 2 };
struct TokenLookup {
    Token token[MaxTokenLookup];
    unsigned num;

    TokenLookup() : num(MaxTokenLookup) {
    }
};

struct token_lexer :
    grammar<token_lexer, result_closure<TokenLookup>::context_t>
{
    template < typename ScannerT >
    struct definition {
        typedef rule<ScannerT> rule_t;
        rule_t main;

        rule_t const& start() const {
            return main;
        }

        TokenLookup lookup;
        file_position filePos;
        file_position filePos2;

        definition(token_lexer const& self);
    };
} token_lexer_g;

template < typename ScannerT >
token_lexer::definition<ScannerT>::definition(token_lexer const& self)
{
    subrule<0> submain;
    subrule<1> skip_until_eol;
    subrule<2> singleline_comment;
    subrule<3> multiline_comment;
    subrule<4> directive;

    main = (
        submain =
            *(blank_p | ('\\' >> eol_p))
            >> get_file_position_p(filePos)
            >> epsilon_p
            [SetUnknownToken(lookup.token[1], filePos)]
            [var(lookup.num) = 1]
            >> (
                eol_p                 [SetEOLToken       (lookup.token[0], filePos)]
                >> *blank_p
                >> get_file_position_p(filePos2)
                >> !directive         [SetDirectiveToken (lookup.token[1], filePos2)]
                [var(lookup.num) = 2]
                | singleline_comment    [SetCommentToken   (lookup.token[0], filePos)]
                | multiline_comment     [SetCommentToken   (lookup.token[0], filePos)]
                | CHARACTER_LITERAL_g   [SetStringToken    (lookup.token[0], filePos)]
                | STRING_LITERAL_g      [SetStringToken    (lookup.token[0], filePos)]
                | FLOAT_CONSTANT_g      [SetFloatingToken  (lookup.token[0], filePos)]
                | INT_CONSTANT_g        [SetIntegerToken   (lookup.token[0], filePos)]
                | IDENTIFIER_g          [SetIdentifierToken(lookup.token[0], filePos)]
                | cpp_operator_p        [SetOperatorToken  (lookup.token[0], filePos)]
                )
            >> epsilon_p [self.result_ = var(lookup)],

        skip_until_eol = *(('\\' >> eol_p) | (anychar_p - eol_p)),

        singleline_comment = "//" >> skip_until_eol,

        multiline_comment = "/*" >> *(anychar_p - "*/") >> "*/",

        directive = '#' >> skip_until_eol
        );
}

} // Private stuff.

///////////////////////////////////////////////////////////////////////////////
// Opaque lexer interface's implementation.

namespace private_stuff { // Private stuff.

template < typename IteratorT >
struct cpp_lex_input_interface : lexer::input_interface<Token> {
public:
    typedef lexer::input_interface<Token> base_t;
    typedef Token result_type;
    TokenLookup lookup;
    unsigned lookupPos;
    unsigned refCount;
    IteratorT first;
    scanner<IteratorT> scan;
    file_position curFilePos;

    cpp_lex_input_interface(IteratorT const& first_,
                            IteratorT const& last_) :
        lookupPos(MaxTokenLookup),
        refCount(1),
        first(first_),
        scan (first, last_)
    {
    }

    virtual void add_ref() {
        ++refCount;
    }
    virtual void dec_ref() {
        --refCount;
        if (refCount == 0) {
            delete this;
        }
    }

    virtual Token const& get() {
        if (lookupPos < lookup.num) {
            return lookup.token[lookupPos++];
        }
        if (token_lexer_g[assign(lookup)].parse(scan)) {
            lookupPos = 1;
            return lookup.token[0];
        } else {
            return eof();
        }
    }

    virtual file_position const& get_position() {
        curFilePos = scan.first.get_position();
        return curFilePos;
    }
};

// This allocates a new lexer implementation for an arbitrary iterator.
template < typename IteratorT >
lexer::input_interface<Token>*
NewLexerImpl(IteratorT const& first,
             IteratorT const& last,
             char const* fname = "<filename>")
{
    typedef position_iterator<IteratorT> Iterator;

    Iterator pfirst(first, last, fname);
    Iterator plast;

    return new cpp_lex_input_interface<Iterator>(pfirst, plast);
}

} // Private stuff.

///////////////////////////////////////////////////////////////////////////////
// Lexer implementation's opaque constructors.

lexer::input_interface<Token>*
NewLexer(char const* first, char const* last, char const* fname)
{
    return private_stuff::NewLexerImpl(first, last, fname);
}

} // cpp

