//
// C++ Lexer token definitions
//
// Boost License 1.0
// Copyright© 2002-2003 Juan Carlos Arevalo-Baeza, All rights reserved
// email: jcab@JCABs-Rumblings.com
// Created: 8-Nov-2002
//
// The basics:
// Tokens are constructed from a string, and contain a TokenID, which is
// unique for each token, a file position and the string representation of
// the token.
//

#ifndef cpp_lexer_token_hpp_included
#define cpp_lexer_token_hpp_included

#include "lexer_token_base.hpp"

#include <boost/version.hpp>

#if BOOST_VERSION < 103700
#include <boost/spirit/iterator/position_iterator.hpp>
//namespace bs = boost::spirit;
#else
#include <boost/spirit/include/classic_position_iterator.hpp>
#include <boost/spirit/include/classic_typeof.hpp>
//namespace bs = boost::spirit::classic;
#endif

#include <string>
#include <iosfwd>

///////////////////////////////////////////////////////////////////////////////
// File position tools.
//
// TODO: Move this to the definition within Spirit?

namespace std {

#if BOOST_VERSION < 103700
std::ostream& operator<<(std::ostream& out, boost::spirit::file_position const& lc);
#else
std::ostream& operator<<(std::ostream& out, boost::spirit::classic::file_position const& lc);
#endif

} // std

namespace cpp {

///////////////////////////////////////////////////////////////////////////////
// File position structure.
#if BOOST_VERSION < 103700
typedef boost::spirit::file_position file_position;
#else
typedef boost::spirit::classic::file_position file_position;
#endif

//typedef int file_position;

///////////////////////////////////////////////////////////////////////////////
// TokenID database definition.
//
// The implementation of this class is private.
// It holds the relationship between unique IDs and their text representation.

class TokenDB;

///////////////////////////////////////////////////////////////////////////////
// TokenID type definitions.
//
// Token types are categories of tokens (identifiers, integers,
// strings, etc..). Newlines, comments and the end of file are
// valid tokens, too, which can (or not) be bypassed in a skipper.

enum TokenType {
    IdentifierTokenType = 0x00000000,
    OperatorTokenType   = 0x10000000,
    IntegerTokenType    = 0x20000000,
    FloatingTokenType   = 0x30000000,
    StringTokenType     = 0x40000000,

    UnknownTokenType    = 0x80000000,
    DirectiveTokenType  = 0xC0000000,
    EOLTokenType        = 0xD0000000,
    EOFTokenType        = 0xE0000000,
    CommentTokenType    = 0xF0000000,
};

///////////////////////////////////////////////////////////////////////////////
// Token ID definitions.
//
// Token IDs are enough to define a unique token within the database.
// For example, identifier "x" will always have the same ID.
// So, comparing IDs for equality is enough to correctly parse
// a sequence of tokens.

enum TokenID {
    Token_Force_DWORD    = 0x7FFFFFFF,
    TokenTypeMask        = 0xF0000000,
    TokenIndexMask       = 0x0FFFFFFF,

    // Predefined tokens

    Ident_unknown   = IdentifierTokenType,
    Ident_first,
    Kwd_auto        = Ident_first,
    Kwd_break,
    Kwd_case,
    Kwd_char,
    Kwd_const,
    Kwd_continue,
    Kwd_default,
    Kwd_do,
    Kwd_double,
    Kwd_else,
    Kwd_enum,
    Kwd_extern,
    Kwd_float,
    Kwd_for,
    Kwd_goto,
    Kwd_if,
    Kwd_int,
    Kwd_long,
    Kwd_register,
    Kwd_return,
    Kwd_short,
    Kwd_signed,
    Kwd_sizeof,
    Kwd_static,
    Kwd_struct,
    Kwd_switch,
    Kwd_typedef,
    Kwd_union,
    Kwd_unsigned,
    Kwd_void,
    Kwd_volatile,
    Kwd_while,

    Kwd_asm,
    Kwd_bool,
    Kwd_catch,
    Kwd_class,
    Kwd_const_cast,
    Kwd_delete,
    Kwd_dynamic_cast,
    Kwd_explicit,
    Kwd_export,
    Kwd_false,
    Kwd_friend,
    Kwd_inline,
    Kwd_mutable,
    Kwd_namespace,
    Kwd_new,
    Kwd_operator,
    Kwd_private,
    Kwd_protected,
    Kwd_public,
    Kwd_reinterpret_cast,
    Kwd_static_cast,
    Kwd_template,
    Kwd_this,
    Kwd_throw,
    Kwd_true,
    Kwd_try,
    Kwd_typeid,
    Kwd_typename,
    Kwd_using,
    Kwd_virtual,
    Kwd_wchar_t,

    Kwd_last,
    Ident__         = Kwd_last, // This is the single underscore identifier.
    Ident_next,
    Ident_last      = OperatorTokenType-1,

    Op_unknown      = OperatorTokenType,
    Op_first,
    Op_Ellipsis     = Op_first,          // ...
    Op_Right_Assign,                   // >>=
    Op_Left_Assign,                   // <<=
    Op_Add_Assign,                   // +=
    Op_Sub_Assign,                   // -=
    Op_Mul_Assign,                   // *=
    Op_Div_Assign,                   // /=
    Op_Mod_Assign,                   // %=
    Op_BitAnd_Assign,                    // &=
    Op_BitXor_Assign,                    // ^=
    Op_BitOr_Assign,                   // |=
    Op_Right,                   // >>
    Op_Left,                   // <<
    Op_Inc,                   // ++
    Op_Dec,                   // --
    Op_Ptr,                   // ->
    Op_And,                   // &&
    Op_Or,                   // ||
    Op_LE,                   // <=
    Op_GE,                   // >=
    Op_EQ,                   // ==
    Op_NE,                   // !=
    Op_Semicolon,                   // ;
    Op_Left_Brace,                   // {
    Op_Right_Brace,                   // }
    Op_Comma,                   // ,
    Op_Colon,                   // :
    Op_Assign,                   // =
    Op_Left_Paren,                   // (
    Op_Right_Paren,                   // )
    Op_Left_Bracket,                   // [
    Op_Right_Bracket,                    // ]
    Op_Dot,                   // .
    Op_Ampersand,                   // &
    Op_Bang,                   // !
    Op_Compl,                   // ~ (for complement)
    Op_Tilde,                   // ~ (for destructors)
    Op_Minus,                   // -
    Op_Plus,                   // +
    Op_Star,                   // *
    Op_Slash,                   // /
    Op_Percent,                   // %
    Op_LT,                   // <
    Op_GT,                   // >
    Op_BitXor,                   // ^
    Op_BitOr,                   // |
    Op_Question,                   // ?

    Op_Scope,                   // ::
    Op_Member_Ref,                   // .*
    Op_Member_Ptr,                   // ->*

    Op_At,                   // @
    Op_Dollar,                   // $
    Op_DoblePound,                   // ##
    Op_Pound,                   // #

    Op_next,

    Integer_unknown  = IntegerTokenType,
    Integer_first,
    Integer_zero     = Integer_first,
    Integer_next,

    Floating_unknown = FloatingTokenType,
    Floating_first,
    Floating_next    = Floating_first,

    String_unknown   = StringTokenType,
    String_first,
    String_next      = String_first,

    Unknown_token    = UnknownTokenType,
    Directive_token  = DirectiveTokenType,
    EOL_token        = EOLTokenType,
    EOF_token        = EOFTokenType,
    Comment_token    = CommentTokenType,
};

std::string GetIdentifierName(TokenID id);
std::string GetOperatorName  (TokenID id);

// Identifier construction function.
// TODO: Add others (integer, float...).
// Used to initialize the database with extra specific tokens
// that are handled differently by the parser.
TokenID MakeIdentifierTokenID(std::string const& text);

///////////////////////////////////////////////////////////////////////////////
// Token structure definitions.
//
// A token structure holds a token ID (see above) and also information about
// where and how it was found in the source, which aids in doing better
// error reporting.

struct Token {
    file_position filePos; // Where in which file the token was found.
    std::string text;   // The actual text in the file.
    TokenID id;     // The ID of this token.

    Token() {
    }
    Token(file_position const& filePos_,
          std::string const& text_,
          TokenID id_) :
        filePos(filePos_),
        text   (text_),
        id     (id_)
    {
    }

    // Automatic conversion to ID.
    operator TokenID() const { return id; }
};

// Token comparison functions.

inline bool operator==(Token const& t1, Token const& t2)
{
    return t1.id == t2.id && t1.text == t2.text;
}

inline bool operator!=(Token const& t1, Token const& t2)
{
    return t1.id != t2.id || t1.text != t2.text;
}

inline bool operator==(Token const& t1, TokenID t2)
{
    return t1.id == t2;
}

inline bool operator!=(Token const& t1, TokenID t2)
{
    return t1.id != t2;
}

inline bool operator==(TokenID t1, Token const& t2)
{
    return t1 == t2.id;
}

inline bool operator!=(TokenID t1, Token const& t2)
{
    return t1 != t2.id;
}

// Required lexer token traits.

} // cpp

namespace lexer {

template <>
struct token_traits<cpp::Token> {
    static cpp::Token const eof_token;
    typedef cpp::file_position file_position;
};

} // lexer

namespace cpp {

// Simple actions that can be used in Spirit parsers.
// For examples of usage, look into cpp_lexer.cpp

// Used for testing.
struct PrintToken_class {
    void operator()(Token const& token) const;
};
PrintToken_class const PrintToken = PrintToken_class();

// Used for testing.
struct OutToken_class {
    void operator()(Token const& token) const;
};
OutToken_class const OutToken = OutToken_class();

// Create an identifier token.
struct SetIdentifierToken {
    Token& dest;
    file_position const& filePos;
    SetIdentifierToken(Token& dest_, file_position const& filePos_) :
        dest   (dest_),
        filePos(filePos_)
    {
    }
    void operator()(std::string const& text) const;
};

// Create an operator (symbol) token.
struct SetOperatorToken {
    Token& dest;
    file_position const& filePos;
    SetOperatorToken(Token& dest_, file_position const& filePos_) :
        dest   (dest_),
        filePos(filePos_)
    {
    }
    void operator()(std::string const& text) const;
    void operator()(TokenID id) const;
};

// Create a literal token.
struct SetLiteralToken {
    Token& dest;
    file_position const& filePos;
    TokenDB& db;
    SetLiteralToken(Token& dest_,
                    file_position const& filePos_,
                    TokenDB& db_) :
        dest   (dest_),
        filePos(filePos_),
        db     (db_)
    {
    }
    void operator()(std::string const& text) const;
};

// Create a literal integer token.
SetLiteralToken
SetIntegerToken(Token& dest, file_position const& filePos);

// Create a literal float token.
SetLiteralToken
SetFloatingToken(Token& dest, file_position const& filePos);

// Create a literal string token.
SetLiteralToken
SetStringToken(Token& dest, file_position const& filePos);

// Create a special token.
struct SetSpecialToken {
    Token& dest;
    file_position const& filePos;
    TokenID id;
    SetSpecialToken(Token& dest_,
                    file_position const& filePos_,
                    TokenID id_) :
        dest   (dest_),
        filePos(filePos_),
        id     (id_)
    {
    }
    void operator()(std::string const& text) const {
        dest = Token(filePos, text, id);
    }
    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        dest = Token(filePos, std::string(first, last), id);
    }
};

// Create a special unknown token.
inline
SetSpecialToken
SetUnknownToken(Token& dest, file_position const& filePos)
{
    return SetSpecialToken(dest, filePos, Unknown_token);
}

// Create a special directive token.
inline
SetSpecialToken
SetDirectiveToken(Token& dest, file_position const& filePos)
{
    return SetSpecialToken(dest, filePos, Directive_token);
}

// Create a special newline token.
inline
SetSpecialToken
SetEOLToken(Token& dest, file_position const& filePos)
{
    return SetSpecialToken(dest, filePos, EOL_token);
}

// Create a special end-of-file token.
inline
SetSpecialToken
SetEOFToken(Token& dest, file_position const& filePos)
{
    return SetSpecialToken(dest, filePos, EOF_token);
}

// Create a special comment token.
inline
SetSpecialToken
SetCommentToken(Token& dest, file_position const& filePos)
{
    return SetSpecialToken(dest, filePos, Comment_token);
}

// clears the state of the lexer
void clearstate();


} // cpp

#endif
