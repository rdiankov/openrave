#include <crlibm.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
using namespace std;

int main(int argc, char ** argv)
{
    if( argc < 2 ) {
        return 1;
    }
    cout << std::setprecision(20);
    ofstream of(argv[1]);
    if( !of ) {
        cerr << "failed to open file: " << argv[1] << endl;
        return 1;
    }
    of << "/// this is a generated file using the check_libm_accuracy program" << endl;
    int numtests = 1000003;
    int i;
    double d;
    int is_accurate;

    for(i = 0, d = -49.0, is_accurate = 1; i < numtests; ++i, d += 100.0/(double)numtests) {
        if( exp(d) != exp_rn(d) ) {
            cout << "bad accuracy exp(" << d << "): " << exp(d) << " != " << exp_rn(d) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_EXP_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -0.1, is_accurate = 1; i < numtests; ++i, d += 100.0/(double)numtests) {
        if( log(d+1) != log_rn(d+1) ) {
            cout << "bad accuracy log(" << d+1 << "): " << log(d+1) << " != " << log_rn(d+1) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_LOG_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -0.1, is_accurate = 1; i < numtests; ++i, d += 100.0/(double)numtests) {
        if( log2(d+1) != log2_rn(d+1) ) {
            cout << "bad accuracy log2(" << d+1 << "): " << log2(d+1) << " != " << log2_rn(d+1) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_LOG2_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -0.1, is_accurate = 1; i < numtests; ++i, d += 100.0/(double)numtests) {
        if( log10(d+1) != log10_rn(d+1) ) {
            cout << "bad accuracy log10(" << d+1 << "): " << log10(d+1) << " != " << log10_rn(d+1) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_LOG10_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -3.14, is_accurate = 1; i < numtests; ++i, d += 6.28/(double)numtests) {
        if( cos(d) != cos_rn(d) ) {
            cout << "bad accuracy cos(" << d << "): " << cos(d) << " != " << cos_rn(d) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_COS_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -3.14, is_accurate = 1; i < numtests; ++i, d += 6.28/(double)numtests) {
        if( sin(d) != sin_rn(d) ) {
            cout << "bad accuracy sin(" << d << "): " << sin(d) << " != " << sin_rn(d) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_SIN_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -3.14, is_accurate = 1; i < numtests; ++i, d += 6.28/(double)numtests) {
        if( tan(d) != tan_rn(d) ) {
            cout << "bad accuracy tan(" << d << "): " << tan(d) << " != " << tan_rn(d) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_TAN_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -0.99999999999, is_accurate = 1; i < numtests; ++i, d += 2.0/(double)numtests) {
        if( acos(d) != acos_rn(d) ) {
            cout << "bad accuracy acos(" << d << "): " << acos(d) << " != " << acos_rn(d) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_ACOS_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -0.9999999999, is_accurate = 1; i < numtests; ++i, d += 2.0/(double)numtests) {
        if( asin(d) != asin_rn(d) ) {
            cout << "bad accuracy asin(" << d << "): " << asin(d) << " != " << asin_rn(d) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_ASIN_ACCURATE " << is_accurate << endl;

    for(i = 0, d = -2.0, is_accurate = 1; i < numtests; ++i, d += 8.0/(double)numtests) {
        if( atan(d) != atan_rn(d) ) {
            cout << "bad accuracy atan(" << d << "): " << atan(d) << " != " << atan_rn(d) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_ATAN_ACCURATE " << is_accurate << endl;

    for(i = 0, d = 0.000001, is_accurate = 1; i < numtests; ++i, d += 1000.0/(double)numtests) {
        if( sqrt(d) != pow_rn(d,0.5) ) {
            cout << "sqrt(" << d << "): " << sqrt(d) << " != " << pow_rn(d,0.5) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_SQRT_ACCURATE " << is_accurate << endl;

    for(i = 0, d = 0.000001, is_accurate = 1; i < numtests; ++i, d += 1000.0/(double)numtests) {
        if( pow(d,d*4) != pow_rn(d,d*4) ) {
            cout << "bad accuracy pow(" << d << "," << d*4 << "): " << pow(d,d*4) << " != " << pow_rn(d,d*4) << endl;
            is_accurate = 0;
            break;
        }
    }
    of << "#define LIBM_POW_ACCURATE " << is_accurate << endl;

    return 0;
}
