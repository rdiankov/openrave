//
// C++ Lexer token definitions
//
// Boost License 1.0
// Copyright© 2002-2003 Juan Carlos Arevalo-Baeza, All rights reserved
// email: jcab@JCABs-Rumblings.com
// Created: 8-Nov-2002
//

#include "cpp_lexer_token.hpp"

#include <boost/version.hpp>

#if BOOST_VERSION < 103700
#include <boost/spirit/symbols.hpp>
namespace bs = boost::spirit;
#else
#include <boost/spirit/include/classic_symbols.hpp>
namespace bs = boost::spirit::classic;
#endif

#include <vector>
#include <iostream>

///////////////////////////////////////////////////////////////////////////////
// File position tools.
//
// TODO: Move this to the definition within Spirit?

namespace std {
    std::ostream& operator<<(std::ostream& out, bs::file_position const& lc)
    {
        return out <<
                lc.file   << ":" <<
                lc.line   << ":" <<
                lc.column;
    }

} // std

namespace cpp {

using bs::parser;
using bs::symbols;

///////////////////////////////////////////////////////////////////////////////
// TokenID database definition.
//
// The implementation of this class is private.
// It holds the relationship between unique IDs and their text representation.
// It allows TOkenIDs to be properly reused.
//
// TODO: This should properly reformat literals as needed to improve sharing.

class TokenDB: public symbols<TokenID> {
public:
    typedef symbols<TokenID> super_t;

    TokenID next, _next_start;
    std::vector<std::string> list, _list_start;
    

    TokenDB(TokenID next_): next(next_) { set(); }

    void set() { _next_start = next; _list_start = list; }
    void reset() { next = _next_start; list = _list_start; }

    TokenID add(char const* text, TokenID id)
    {
        unsigned const type  = id & TokenTypeMask;
        unsigned const index = id & TokenIndexMask;
        unsigned const dbtype = next & TokenTypeMask;
        if (type == dbtype) {
            if (list.size() <= index) {
                list.resize(index+1);
            }
            std::string& dest = list[index];
            if (dest.empty()) {
                list[index] = text;
            }
            if (id >= next) {
                next = TokenID(id + 1);
            }
        }
        super_t::add(text, id);
        return id;
    }

    TokenID add(char const* text)
    {
        TokenID id = next;
        next = TokenID(next + 1);
        return add(text, id);
    }

    TokenID add(std::string const& text, TokenID id)
    {
        return add(text.c_str(), id);
    }

    TokenID add(std::string const& text)
    {
        return add(text.c_str());
    }

    TokenID* find(char const* text) const {
        return bs::find(
            *static_cast<super_t const*>(this),
            text
        );
    }

    TokenID* find(std::string const& text) const {
        return find(text.c_str());
    }

    std::string const& find(TokenID id) const {
        unsigned const type  = id & TokenTypeMask;
        unsigned const index = id & TokenIndexMask;
        unsigned const dbtype = next & TokenTypeMask;
        assert(type == dbtype);
        assert(list.size() > index);
        return list[index];
    }
};

namespace {

    // Definition of the keywords and symbols.

    struct c_keywords_db: TokenDB {
        c_keywords_db(): TokenDB(Ident_next) {
            add("auto"     , Kwd_auto          );
            add("break"    , Kwd_break         );
            add("case"     , Kwd_case          );
            add("char"     , Kwd_char          );
            add("const"    , Kwd_const         );
            add("continue" , Kwd_continue      );
            add("default"  , Kwd_default       );
            add("do"       , Kwd_do            );
            add("double"   , Kwd_double        );
            add("else"     , Kwd_else          );
            add("enum"     , Kwd_enum          );
            add("extern"   , Kwd_extern        );
            add("float"    , Kwd_float         );
            add("for"      , Kwd_for           );
            add("goto"     , Kwd_goto          );
            add("if"       , Kwd_if            );
            add("int"      , Kwd_int           );
            add("long"     , Kwd_long          );
            add("register" , Kwd_register      );
            add("return"   , Kwd_return        );
            add("short"    , Kwd_short         );
            add("signed"   , Kwd_signed        );
            add("sizeof"   , Kwd_sizeof        );
            add("static"   , Kwd_static        );
            add("struct"   , Kwd_struct        );
            add("switch"   , Kwd_switch        );
            add("typedef"  , Kwd_typedef       );
            add("union"    , Kwd_union         );
            add("unsigned" , Kwd_unsigned      );
            add("void"     , Kwd_void          );
            add("volatile" , Kwd_volatile      );
            add("while"    , Kwd_while         );

            add("and_eq"   , Op_BitAnd_Assign );
            add("xor_eq"   , Op_BitXor_Assign );
            add("or_eq"    , Op_BitOr_Assign  );
            add("and"      , Op_And           );
            add("or"       , Op_Or            );
            add("not_eq"   , Op_NE            );
            add("bitand"   , Op_Ampersand     );
            add("not"      , Op_Bang          );
            add("compl"    , Op_Compl         );
            add("xor"      , Op_BitXor        );
            add("bitor"    , Op_BitOr         );

            add("_"        , Ident__          );
            set();
        }
    };

    struct cpp_keywords_db: c_keywords_db {
        cpp_keywords_db() {
            add("asm"              , Kwd_asm             );
            add("bool"             , Kwd_bool            );
            add("catch"            , Kwd_catch           );
            add("class"            , Kwd_class           );
            add("const_cast"       , Kwd_const_cast      );
            add("delete"           , Kwd_delete          );
            add("dynamic_cast"     , Kwd_dynamic_cast    );
            add("explicit"         , Kwd_explicit        );
            add("export"           , Kwd_export          );
            add("false"            , Kwd_false           );
            add("friend"           , Kwd_friend          );
            add("inline"           , Kwd_inline          );
            add("mutable"          , Kwd_mutable         );
            add("namespace"        , Kwd_namespace       );
            add("new"              , Kwd_new             );
            add("operator"         , Kwd_operator        );
            add("private"          , Kwd_private         );
            add("protected"        , Kwd_protected       );
            add("public"           , Kwd_public          );
            add("reinterpret_cast" , Kwd_reinterpret_cast);
            add("static_cast"      , Kwd_static_cast     );
            add("template"         , Kwd_template        );
            add("this"             , Kwd_this            );
            add("throw"            , Kwd_throw           );
            add("true"             , Kwd_true            );
            add("try"              , Kwd_try             );
            add("typeid"           , Kwd_typeid          );
            add("typename"         , Kwd_typename        );
            add("using"            , Kwd_using           );
            add("virtual"          , Kwd_virtual         );
            add("wchar_t"          , Kwd_wchar_t         );
            set();
        }
    };

    struct c_operators_db: TokenDB {
        c_operators_db(): TokenDB(Op_next) {
            add("...", Op_Ellipsis      );
            add(">>=", Op_Right_Assign  );
            add("<<=", Op_Left_Assign   );
            add("+=" , Op_Add_Assign    );
            add("-=" , Op_Sub_Assign    );
            add("*=" , Op_Mul_Assign    );
            add("/=" , Op_Div_Assign    );
            add("%=" , Op_Mod_Assign    );
            add("&=" , Op_BitAnd_Assign );
            add("^=" , Op_BitXor_Assign );
            add("|=" , Op_BitOr_Assign  );
            add(">>" , Op_Right         );
            add("<<" , Op_Left          );
            add("++" , Op_Inc           );
            add("--" , Op_Dec           );
            add("->" , Op_Ptr           );
            add("&&" , Op_And           );
            add("||" , Op_Or            );
            add("<=" , Op_LE            );
            add(">=" , Op_GE            );
            add("==" , Op_EQ            );
            add("!=" , Op_NE            );
            add(";"  , Op_Semicolon     );
            add("{"  , Op_Left_Brace    );
            add("}"  , Op_Right_Brace   );
            add("<%" , Op_Left_Brace    );
            add("%>" , Op_Right_Brace   );
            add(","  , Op_Comma         );
            add(":"  , Op_Colon         );
            add("="  , Op_Assign        );
            add("("  , Op_Left_Paren    );
            add(")"  , Op_Right_Paren   );
            add("["  , Op_Left_Bracket  );
            add("]"  , Op_Right_Bracket );
            add("<:" , Op_Left_Bracket  );
            add(":>" , Op_Right_Bracket );
            add("."  , Op_Dot           );
            add("&"  , Op_Ampersand     );
            add("!"  , Op_Bang          );
            add("~"  , Op_Tilde         );
            add("-"  , Op_Minus         );
            add("+"  , Op_Plus          );
            add("*"  , Op_Star          );
            add("/"  , Op_Slash         );
            add("%"  , Op_Percent       );
            add("<"  , Op_LT            );
            add(">"  , Op_GT            );
            add("^"  , Op_BitXor        );
            add("|"  , Op_BitOr         );
            add("?"  , Op_Question      );

            add("##" , Op_DoblePound    );
            add("#"  , Op_Pound         );

            add("??="  , Op_Pound         );
            add("??'"  , Op_BitXor        );
            add("??("  , Op_Left_Bracket  );
            add("??)"  , Op_Right_Bracket );
            add("??!"  , Op_BitOr         );
            add("??<"  , Op_Left_Brace    );
            add("??>"  , Op_Right_Brace   );
            add("??-"  , Op_Tilde         );

            add("@"  , Op_At            );
            add("$"  , Op_Dollar        );
            set();
        }
    };

    struct cpp_operators_db: c_operators_db {
        cpp_operators_db() {
            add("::" , Op_Scope     );
            add(".*" , Op_Member_Ref);
            add("->*", Op_Member_Ptr);
            set();
        }
    };

    struct integers_db: TokenDB {
        integers_db(): TokenDB(Integer_next) {
            add("0", Integer_zero);
            set();
        }
    };

    // The actual database.
    //
    // Note that this is implemented as global variables.
    // TODO: Somehow make this stack-based. That will later require to keep
    // it to be a context in the lexer grammar's scanner.

    cpp_keywords_db  identifierDB;
    cpp_operators_db operatorDB;
    integers_db      integerDB;
    TokenDB          floatingDB  (Floating_next);
    TokenDB          stringDB    (String_next);
}

// Internal transfer. Used in cpp_lexer.cpp.
extern
parser<symbols<TokenID> > const&
cpp_operator_p = operatorDB;

void clearstate() {
    //this is such a hack, but that's what you get with global state.........
    identifierDB.~cpp_keywords_db();
    new (&identifierDB) cpp_keywords_db();
    operatorDB.~cpp_operators_db();
    new (&operatorDB) cpp_operators_db();
    integerDB.~integers_db();
    new (&integerDB) integers_db();
    floatingDB.~TokenDB();
    new (&floatingDB) TokenDB(Floating_next);
    stringDB.~TokenDB();
    new (&stringDB) TokenDB(String_next);
}


///////////////////////////////////////////////////////////////////////////////
// Token ID definitions.

std::string GetIdentifierName(TokenID id)
{
    return identifierDB.find(id);
}

std::string GetOperatorName(TokenID id)
{
    return operatorDB.find(id);
}

TokenID MakeIdentifierTokenID(std::string const& text)
{
    TokenID const* kwdID;
    if (kwdID = identifierDB.find(text)) {
        return *kwdID;
    } else {
        return identifierDB.add(text);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Token structure definitions.

} // cpp

namespace lexer {

    // Required lexer token traits.
    cpp::Token const token_traits<cpp::Token>::eof_token
        = cpp::Token(cpp::file_position(), "", cpp::EOF_token);

} // lexer

namespace cpp {

// Implementation of the simple actions that can be used in Spirit parsers.

void PrintToken_class::operator()(Token const& token) const {
    struct OutText_ { // Local function.
        void operator()(char const* str, Token const& token) {
            std::cout << str << "'" << token.text << "'\n";
        }
        void operator()(char const* str, std::string const& text) {
            std::cout << str << "'" << text << "'\n";
        }
    } OutText;

    std::cout << token.filePos << ": ";
    switch(uint64_t(token.id & TokenTypeMask)) {
    case IdentifierTokenType:
            if (token.id >= Kwd_last) {
                                  OutText("Identifier: ", token); break;
            } else {
                                  OutText("Keyword:    ", token); break;
            }
        case OperatorTokenType  : OutText("Operator:   ", token); break;
        case IntegerTokenType   : OutText("Integer:    ", token); break;
        case FloatingTokenType  : OutText("Floating:   ", token); break;
        case StringTokenType    : OutText("String:     ", token); break;
        case DirectiveTokenType : OutText("Directive:  ", token); break;
        case EOLTokenType       : std::cout << "EOL\n"    ; break;
        case CommentTokenType   : std::cout << "Comment\n"; break;
    }
}

void OutToken_class::operator()(Token const& token) const {
    switch(uint64_t(token.id & TokenTypeMask)) {
        case IdentifierTokenType: std::cout << token.text; break;
        case OperatorTokenType  : std::cout << token.text; break;
        case IntegerTokenType   : std::cout << token.text; break;
        case FloatingTokenType  : std::cout << token.text; break;
        case StringTokenType    :
            switch (token.text[0]) {
                case '"': std::cout << token.text << "\""; break;
                case '\'': std::cout << token.text << "'"; break;
                default:
                    // Must be the 'L' prefix.
                    switch (token.text[1]) {
                        case '"': std::cout << token.text << "\""; break;
                        case '\'': std::cout << token.text << "'"; break;
                    }
            }
            break;
        case DirectiveTokenType : std::cout << token.text; break;
        case EOLTokenType       : std::cout << "\n"    ; break;
//        case CommentTokenType   : std::cout << "Comment\n"; break;
    }
}

void SetIdentifierToken::operator()(std::string const& text) const
{
    TokenID const id = MakeIdentifierTokenID(text);
    if ((id & TokenTypeMask) == OperatorTokenType) {
        dest = Token(filePos, GetOperatorName(id), id);
    } else {
        dest = Token(filePos, text, id);
    }
}

void SetOperatorToken::operator()(std::string const& text) const
{
    TokenID const* kwdID;
    if (kwdID = operatorDB.find(text)) {
        operator()(*kwdID);
    } else {
        dest = Token(filePos, text, Op_unknown);
    }
}

void SetOperatorToken::operator()(TokenID id) const
{
    std::string const& text = GetOperatorName(id);
    if (!text.empty()) {
        dest = Token(filePos, text, id);
    } else {
        dest = Token(filePos, "<UNKNOWN>", Op_unknown);
    }
}

// Create a literal integer token.
SetLiteralToken
SetIntegerToken(Token& dest, file_position const& filePos)
{
    return SetLiteralToken(dest, filePos, integerDB);
}

// Create a literal float token.
SetLiteralToken
SetFloatingToken(Token& dest, file_position const& filePos)
{
    return SetLiteralToken(dest, filePos, floatingDB);
}

// Create a literal string token.
SetLiteralToken
SetStringToken(Token& dest, file_position const& filePos)
{
    return SetLiteralToken(dest, filePos, stringDB);
}

void SetLiteralToken::operator()(std::string const& text) const
{
    TokenID const* kwdID;
    if (kwdID = db.find(text)) {
        dest = Token(filePos, text, *kwdID);
    } else {
        TokenID id = db.add(text);
        dest = Token(filePos, text, id);
    }
}

} // cpp
