#include "cpp_lexer.hpp"

#include <iostream>
#include <vector>
#include <sstream>
#include <cstdio>

#include "md5.h"

using namespace std;

string getmd5hash(const char* fname);

int main(int argc, char* argv[])
{
    if( argc < 2 ) {
        cerr << "No filename given" << endl
             << "cpp-gen-md5 [filename1 define1] [filename2 define2] ..." << endl
             << "Generates a md5 sum from the lexical tokens of a C++ ignoring directives and whitespace." << endl
             << "If only a filename is given, will output a 16 byte string" << endl
             << "If both filename and define are given will output #define @define2@ \"@md5hash@\"" << endl;
        return 2;
    }    
    
    for(int i = 1; i < argc; i += 2) {
        string md5hash = getmd5hash(argv[i]);
        if( md5hash == "" )
            return 1;

        if( i+1 < argc )
            cout << "#define " << argv[i+1] << " \"" << md5hash << "\"" << endl;
        else
            cout << md5hash << endl;
    }
    
    return 0;
}

string getmd5hash(const char* fname)
{
    // Read the input file into a buffer.
    FILE* f = fopen(fname, "rb");
    if (!f) {
        cerr << "Cannot open input file: " << fname << endl;
        return "";
    }

    fseek(f, 0, SEEK_END);
    int const size = ftell(f);
    fseek(f, 0, SEEK_SET);
    vector<char> buf(size);
    fread(&buf[0], 1, size, f);
    fclose(f);

    cpp::lexer_iterator first(cpp::NewLexer(&buf[0], &buf[0]+buf.size(), fname));
    cpp::lexer_iterator last;

    vector<char> vdata; vdata.reserve(10000);
    while (first != last) {
        cpp::Token const& token = *first;
        //cpp::PrintToken(token);
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

            if( (token.id&cpp::TokenTypeMask) == cpp::OperatorTokenType )
                break;

            if( (token.id&cpp::TokenTypeMask) == cpp::IdentifierTokenType ) {
                if (token.id < cpp::Kwd_last)
                    break;
            }
                        
            //cpp::PrintToken(token);
            for(size_t i = 0; i < token.text.size(); ++i)
                vdata.push_back(token.text[i]);
        }
               
        ++first;
    }

    md5_state_t state;
	md5_byte_t digest[16];
	
	md5_init(&state);
	md5_append(&state, (const md5_byte_t *)&vdata[0], vdata.size());
	md5_finish(&state, digest);
    char hex_output[16*2+1]; 
    for (int di = 0; di < 16; ++di)
	    sprintf(hex_output + di * 2, "%02x", digest[di]);
    return string(hex_output);
}

//#include <math.h>
//#include <stdio.h>
//#include <string.h>
//static int do_test(void)
//{
//    static const char *const test[7*2] = {
//	"", "d41d8cd98f00b204e9800998ecf8427e",
//	"a", "0cc175b9c0f1b6a831c399e269772661",
//	"abc", "900150983cd24fb0d6963f7d28e17f72",
//	"message digest", "f96b697d7cb7938d525a2f31aaf161d0",
//	"abcdefghijklmnopqrstuvwxyz", "c3fcd3d76192e4007dfb496cca67e13b",
//	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789",
//				"d174ab98d277d9f5a5611c2c9f419d9f",
//	"12345678901234567890123456789012345678901234567890123456789012345678901234567890", "57edf4a22be3c955ac49da2e2107b67a"
//    };
//    int i;
//    int status = 0;
//
//    for (i = 0; i < 7*2; i += 2) {
//	md5_state_t state;
//	md5_byte_t digest[16];
//	char hex_output[16*2 + 1];
//	int di;
//
//	md5_init(&state);
//	md5_append(&state, (const md5_byte_t *)test[i], strlen(test[i]));
//	md5_finish(&state, digest);
//	for (di = 0; di < 16; ++di)
//	    sprintf(hex_output + di * 2, "%02x", digest[di]);
//	if (strcmp(hex_output, test[i + 1])) {
//	    printf("MD5 (\"%s\") = ", test[i]);
//	    puts(hex_output);
//	    printf("**** ERROR, should be: %s\n", test[i + 1]);
//	    status = 1;
//	}
//    }
//    if (status == 0)
//	puts("md5 self-test completed successfully.");
//    return status;
//}
