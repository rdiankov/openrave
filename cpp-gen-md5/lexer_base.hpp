//
// Lexer basic interface
//
// Boost License 1.0
// Copyright© 2003 Juan Carlos Arevalo-Baeza, All rights reserved
// email: jcab@JCABs-Rumblings.com
// Created: 8-Feb-2003
//
// The basics:
//
// The lexer is an iterator that iterates over tokens generated on the fly
// from a text input. It's implementation is done in an opaque manner
// by means of an abstract interface "input_interface". In order to
// write a lexer using this basic framework, you'll need to create a
// descendent of "input_interface" and correctly implement its methods.
//
// In order to use it, you create a "iterator" object, and initialize it
// with the appropriate "input_interface" implementation. Create an empty
// "iterator" object in order to use it as the end of sequence.
//

#ifndef lexer_base_hpp_included
#define lexer_base_hpp_included

//#define TEMP_SPIRIT_HACK // requires changes eof to eof() in ../inc/boost/spirit/iterator/multi_pass.hpp

#include "lexer_token_base.hpp"

#include <boost/version.hpp>

#if BOOST_VERSION < 103700
#include <boost/spirit/iterator/position_iterator.hpp>
#include <boost/spirit/iterator/multi_pass.hpp>
namespace bs = boost::spirit;
#else
#include <boost/spirit/include/classic_position_iterator.hpp>
#include <boost/spirit/include/classic_multi_pass.hpp>
#include <boost/spirit/include/classic_typeof.hpp>
namespace bs = boost::spirit::classic;
#endif

namespace lexer {

///////////////////////////////////////////////////////////////////////////////
// Required token traits.
//
// In order to create a lexer, you need to define these for the token type.

template < typename TokenT >
struct token_traits; /*
    static TokenT const eof_token;
*/

///////////////////////////////////////////////////////////////////////////////
// Opaque interface.
//
// This is the lexer implementation's abstract base class.
// This interface is all that needs to be implemented
// in order to create a lexer.

template < typename TokenT >
class input_interface {
private:
    // Copy semantics not available.
    input_interface(input_interface const&);
    input_interface& operator=(input_interface const&);

protected:
    // Virtual destructor is always needed for an abstract interface.
    virtual ~input_interface() {}
    input_interface() {}

public:
    // Useful types.
    typedef TokenT token_t;
    typedef typename token_traits<TokenT>::file_position file_position;

    // The reference counting interface.
    virtual void add_ref() = 0;
    virtual void dec_ref() = 0;

    // The lexer interface proper.
    virtual token_t const& get() = 0;
    virtual file_position const& get_position() = 0;

    // This could have been made pure, too, but the eof token is still
    // needed in order to implement the input_policy (below), so
    // we can offload work from lexer implementations by doing this here.
    token_t const& eof() const {
        return token_traits<token_t>::eof_token;
    }
};

///////////////////////////////////////////////////////////////////////////////
// Lexer iterator definition.

template < typename TokenT >
class input_policy {
public:
    typedef TokenT result_type;
    typedef input_interface<TokenT> input_interface_t;

    input_interface_t* interf;

    input_policy(input_policy const& other):
        interf(other.interf)
    {
        interf->add_ref();
    }
    input_policy& operator=(input_policy const& other)
    {
        if (other.interf != interf) {
            other.interf->add_ref();
            interf->dec_ref();
            interf = other.interf;
        }
        return *this;
    }

    input_policy(input_interface_t* interf_):
        interf(interf_)
    {
    }
    ~input_policy() {
        interf->dec_ref();
    }
    result_type operator()() {
        return interf->get();
    }

#ifndef TEMP_SPIRIT_HACK
    static result_type const& eof;
#else
    result_type const& eof() { return token_traits<TokenT>::eof_token; }
#endif
};

#ifndef TEMP_SPIRIT_HACK
template < typename TokenT >
TokenT const&
input_policy<TokenT>::eof
    = token_traits<TokenT>::eof_token;
#endif

template < typename TokenT >
class my_iterator_base {
public:
    typedef bs::multi_pass<
    input_policy<TokenT>,
               bs::multi_pass_policies::functor_input
/* defaulted    ,
                multi_pass_policies::first_owner,
                multi_pass_policies::no_check,
                multi_pass_policies::std_deque
*/
            >
        type;
};

template < typename TokenT >
struct iterator: my_iterator_base<TokenT>::type {
    typedef typename my_iterator_base<TokenT>::type base_t;
    typedef iterator self_t;

    iterator() {}

    iterator(input_interface<TokenT>* interf):
        base_t(input_policy<TokenT>(interf))
    {}
};

} // lexer

#endif
