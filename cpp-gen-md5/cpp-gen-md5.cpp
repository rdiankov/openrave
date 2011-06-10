/** \file cpp-gen-md5.cpp
    \brief Generates a md5 hash from the lexical tokens of a C++ ignoring directives and whitespace.
    \author Rosen Diankov
    \anchor cpp-gen-md5
    
    Usage:
    \verbatim
    cpp-gen-md5 [filename1 define1] [filename2 define2] ...
    \endverbatim
    
    If only a filename is given, will output a 16 byte string. If both filename and define are given
    will output a file consisting of the hashes:
    \verbatim
    #define @define2@ "@md5hash@"
    \endverbatim
 */
#include "cpp_lexer.hpp"

#include <iostream>
#include <vector>
#include <sstream>
#include <cstdio>
#include <cstring>

#include "md5.h"

using namespace std;

void getTokenData(const char* fname, vector<char>& vdata);
string getmd5hash(const char* fname, const vector<char>& vbasedata);

int main(int argc, char* argv[])
{
    if( argc < 2 ) {
        cerr << "No filename given" << endl
             << "cpp-gen-md5 [common-string] [shared-filename] [filename1 define1] [filename2 define2]* ..." << endl
             << "Generates a md5 sum from the lexical tokens of a C++ ignoring directives and whitespace." << endl
             << "If only a filename is given, will output a 16 byte string" << endl
             << "If both filename and define are given will output #define @define2@ \"@md5hash@\"" << endl;
        return 2;
    }    
    
    vector<char> vbasedata;
    vbasedata.resize(strlen(argv[1]));
    if( vbasedata.size() > 0 ) {
        memcpy(&vbasedata[0],argv[1],vbasedata.size());
    }
    if( strlen(argv[2]) > 0 ) {
        getTokenData(argv[2],vbasedata);
    }
    for(int i = 3; i < argc; i += 2) {
        vector<char> vdata=vbasedata;
        string md5hash = getmd5hash(argv[i],vdata);
        if( md5hash == "" ) {
            return 1;
        }
        if( i+1 < argc ) {
            cout << "#define " << argv[i+1] << " \"" << md5hash << "\"" << endl;
        }
        else {
            cout << md5hash << endl;
        }
    }
    
    return 0;
}

void getTokenData(const char* fname,vector<char>& vdata)
{
    // Read the input file into a buffer.
    FILE* f = fopen(fname, "rb");
    if (!f) {
        cerr << "Cannot open input file: " << fname << endl;
        return;
    }
    
    fseek(f, 0, SEEK_END);
    int const size = ftell(f);
    fseek(f, 0, SEEK_SET);
    vector<char> buf(size);
    size_t read = fread(&buf[0], 1, size, f);
    fclose(f);
    if( read == 0 || read != size ) {
        return;
    }

    cpp::clearstate();
    cpp::lexer_iterator first(cpp::NewLexer(&buf[0], &buf[0]+buf.size(), fname));
    cpp::lexer_iterator last;

    while (first != last) {
        cpp::Token const& token = *first;
        switch(token.id) {
        case cpp::Unknown_token:
        case cpp::Directive_token:
        case cpp::EOL_token:
        case cpp::EOF_token:
        case cpp::Comment_token:
            break;
        default:
            vdata.push_back(token.id&0xff);
            if(  token.id&0xffff00) {
                vdata.push_back((token.id>>8)&0xff);
                vdata.push_back((token.id>>16)&0xff);
            }
            vdata.push_back((token.id>>24)&0xff);
            //cpp::PrintToken(token);
            if( (token.id&cpp::TokenTypeMask) == cpp::OperatorTokenType ) {
                break;
            }
            if( (token.id&cpp::TokenTypeMask) == cpp::IdentifierTokenType ) {
                if (token.id < cpp::Kwd_last) {
                    break;
                }
            }
            for(size_t i = 0; i < token.text.size(); ++i) {
                vdata.push_back(token.text[i]);
            }
        }
               
        ++first;
    }
}

string getmd5hash(const char* fname, const vector<char>& vbasedata)
{
    vector<char> vdata = vbasedata;
    vdata.reserve(vdata.capacity()+10000);
    getTokenData(fname,vdata);

    md5_state_t state;
	md5_byte_t digest[16];

	md5_init(&state);
	md5_append(&state, (const md5_byte_t *)&vdata[0], vdata.size());
	md5_finish(&state, digest);
    char hex_output[16*2+1]; 
    for (int di = 0; di < 16; ++di) {
	    sprintf(hex_output + di * 2, "%02x", digest[di]);
    }
    return string(hex_output);
}
