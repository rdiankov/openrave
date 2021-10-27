// -*- coding: utf-8 -*-
// Copyright (C) 2011-2012 Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef OPENRAVE_FPARSER_EXTENSIONS
#define OPENRAVE_FPARSER_EXTENSIONS

#include <fparser/fpconfig.hh>
#include <fparser/fparser.hh>
#include <fparser/fptypes.hh>
#include <fparser/fpaux.hh>
#include <sstream>
#include <cstdio>
#include <utility>

// visual c++ 2010 has a max defined somewhere that causes namespace conflicts.
template <typename T>
const T& mymax(const T& a, const T& b)
{
    return (a<b) ? b : a;
}

// out = fn(in)
#define EVAL_MULTI_APPLY(fn,SPout,SPin) { \
        (SPout).resize((SPin).size()); \
        for(size_t ii = 0; ii < (SPin).size(); ++ii) { \
            (SPout)[ii] = std::make_pair(fn((SPin)[ii].first),(SPin)[ii].second); \
        } \
} \

#define EVAL_MULTI_COMPARE_INDICES(index0,index1) ((index0)==-1 || (index1)==-1 || (index0)==(index1))

// out = fn(in0,in1): have to take the cross product, be careful since SPout can be SPin0 or SPin1
#define EVAL_MULTI_APPLY2(fn,SPout,SPin0,SPin1) {     \
        VALUES _vtemp_; _vtemp_.reserve((SPin0).size()*(SPin1).size()); \
        for(size_t ii = 0; ii < (SPin0).size(); ++ii) { \
            for(size_t jj = 0; jj < (SPin1).size(); ++jj) { \
                if( EVAL_MULTI_COMPARE_INDICES((SPin0)[ii].second, (SPin1)[jj].second) ) { \
                    _vtemp_.emplace_back(fn((SPin0)[ii].first,(SPin1)[jj].first),mymax((SPin0)[ii].second, (SPin1)[jj].second)); \
                } \
            } \
        } \
        (SPout).swap(_vtemp_); \
} \

// fn(out0,out1,in): have to take the cross product, be careful since SPout can be SPin0 or SPin1
#define EVAL_MULTI_APPLYOUT2(fn,SPout0,SPout1,SPin) {     \
        (SPout0).resize((SPin).size()); \
        (SPout1).resize((SPin).size()); \
        for(size_t ii = 0; ii < (SPin).size(); ++ii) { \
            fn((SPout0)[ii].first,(SPout1)[ii].first,(SPin)[ii].first); \
            (SPout0)[ii].second = (SPout1)[ii].second = (SPin)[ii].second; \
        } \
} \

// out = in0 ? in1: have to take the cross product, be careful since SPout can be SPin0 or SPin1
#define EVAL_MULTI_APPLYOP(op,SPout,SPin0,SPin1) {     \
        VALUES _vtemp_; _vtemp_.reserve((SPin0).size()*(SPin1).size()); \
        for(size_t ii = 0; ii < (SPin0).size(); ++ii) { \
            for(size_t jj = 0; jj < (SPin1).size(); ++jj) { \
                if( EVAL_MULTI_COMPARE_INDICES((SPin0)[ii].second, (SPin1)[jj].second) ) { \
                    _vtemp_.emplace_back((SPin0)[ii].first op (SPin1)[jj].first,mymax((SPin0)[ii].second, (SPin1)[jj].second)); \
                } \
            } \
        } \
        (SPout).swap(_vtemp_); \
} \

#define EVAL_MULTI_CHECK(fn,SPin,reterror) { \
        typename VALUES::iterator itvalue = (SPin).begin(); \
        while(itvalue != (SPin).end()) { \
            if( fn ) { \
                itvalue = (SPin).erase(itvalue); \
            } \
            else { \
                ++itvalue; \
            } \
        } \
        if( (SPin).size() == 0 ) { \
            mData->mEvalErrorType=reterror; return;  \
        } \
} \

namespace OpenRAVE {

template <typename Value_t>
class OpenRAVEFunctionParser : public FunctionParserBase<Value_t>
{
public:
    using FunctionParserBase<Value_t>::SYNTAX_ERROR;
    using FunctionParserBase<Value_t>::MISM_PARENTH;
    using FunctionParserBase<Value_t>::MISSING_PARENTH;
    using FunctionParserBase<Value_t>::EMPTY_PARENTH;
    using FunctionParserBase<Value_t>::EXPECT_OPERATOR;
    using FunctionParserBase<Value_t>::OUT_OF_MEMORY;
    using FunctionParserBase<Value_t>::UNEXPECTED_ERROR;
    using FunctionParserBase<Value_t>::INVALID_VARS;
    using FunctionParserBase<Value_t>::ILL_PARAMS_AMOUNT;
    using FunctionParserBase<Value_t>::PREMATURE_EOS;
    using FunctionParserBase<Value_t>::EXPECT_PARENTH_FUNC;
    using FunctionParserBase<Value_t>::UNKNOWN_IDENTIFIER;
    using FunctionParserBase<Value_t>::NO_FUNCTION_PARSED_YET;
    using FunctionParserBase<Value_t>::FP_NO_ERROR;

    typedef boost::function<void (std::vector<Value_t>&, const std::vector<Value_t>&)> BoostFunction;
    class BoostFunctionWrapper : public FunctionParserBase<Value_t>::FunctionWrapper
    {
public:
        BoostFunctionWrapper() : FunctionParserBase<Value_t>::FunctionWrapper(), _paramsAmount(0) {
        }
        BoostFunctionWrapper(const BoostFunction& fn, int paramsAmount) : FunctionParserBase<Value_t>::FunctionWrapper(), _fn(fn), _paramsAmount(paramsAmount) {
        }
        BoostFunctionWrapper(const BoostFunctionWrapper& r) : _fn(r._fn), _paramsAmount(r._paramsAmount) {
        }
        virtual ~BoostFunctionWrapper() {
        }
        BoostFunctionWrapper& operator=(const BoostFunctionWrapper& r) {
            _fn = r._fn;
            return *this;
        }

        virtual Value_t callFunction(const Value_t* params) {
            std::vector<Value_t> vresult;
            if( _paramsAmount > 0 ) {
                std::vector<Value_t> vparams(params,params+_paramsAmount);
                _fn(vresult,vparams);
            }
            else {
                _fn(vresult,std::vector<Value_t>());
            }
            return vresult.at(0);
        }

        BoostFunction _fn;
        int _paramsAmount;
    };

    bool AddBoostFunction(const std::string& name, const BoostFunction& fn, unsigned paramsAmount)
    {
        return FunctionParserBase<Value_t>::AddFunctionWrapper(name, BoostFunctionWrapper(fn,paramsAmount), paramsAmount);
    }

//===========================================================================
// Function evaluation
//===========================================================================
    void EvalMulti(std::vector<Value_t>& finalret, const Value_t* Vars)
    {
        using namespace FUNCTIONPARSERTYPES;
        typename FunctionParserBase<Value_t>::Data* mData = FunctionParserBase<Value_t>::getParserData();
        finalret.resize(0);
        if(mData->mParseErrorType != FP_NO_ERROR) return;

        const unsigned* const byteCode = &(mData->mByteCode[0]);
        const Value_t* const immed = mData->mImmed.empty() ? 0 : &(mData->mImmed[0]);
        const unsigned byteCodeSize = unsigned(mData->mByteCode.size());
        unsigned IP, DP=0;
        int SP=-1;
        int uniquevaluethreadindex = 1;
        typedef std::vector<std::pair<Value_t,int> > VALUES;
#ifdef FP_USE_THREAD_SAFE_EVAL
        /* If Eval() may be called by multiple threads simultaneously,
         * then Eval() must allocate its own stack.
         */
#ifdef FP_USE_THREAD_SAFE_EVAL_WITH_ALLOCA
        /* alloca() allocates room from the hardware stack.
         * It is automatically freed when the function returns.
         */
        struct AutoDealloc
        {
            VALUES* ptr;
            unsigned mStackSize;
            ~AutoDealloc() {
                for(unsigned i = 0; i < mStackSize; ++i) { ptr[i].~VALUES(); }
            }
        } AutoDeallocStack;
        AutoDeallocStack.ptr = (VALUES*)alloca(mData->mStackSize*sizeof(VALUES));
        for(unsigned i = 0; i < mData->mStackSize; ++i) {
            new (AutoDeallocStack.ptr+i)VALUES();
        }
        AutoDeallocStack.mStackSize = (int)mData->mStackSize;
        VALUES*& Stack = AutoDeallocStack.ptr;
#else
        /* Allocate from the heap. Ensure that it is freed
         * automatically no matter which exit path is taken.
         */
        struct AutoDealloc
        {
            VALUES* ptr;
            ~AutoDealloc() {
                delete[] ptr;
            }
        } AutoDeallocStack = { new VALUES[mData->mStackSize] };
        VALUES*& Stack = AutoDeallocStack.ptr;
#endif
#else
        /* No thread safety, so use a global stack. */
        std::vector< VALUES >& Stack = mData->mStack;
#endif

        for(IP=0; IP<byteCodeSize; ++IP)
        {
            switch(byteCode[IP])
            {
// Functions:
            case   cAbs:
                EVAL_MULTI_APPLY(fp_abs,Stack[SP],Stack[SP]);
                break;

            case  cAcos:
#ifndef FP_NO_EVALUATION_CHECKS
                if( IsComplexType<Value_t>::result == false ) {
                    EVAL_MULTI_CHECK(itvalue->first < Value_t(-1) || itvalue->first > Value_t(1),Stack[SP],4);
                }
#endif
                EVAL_MULTI_APPLY(fp_acos,Stack[SP],Stack[SP]);
                break;

            case cAcosh:
#ifndef FP_NO_EVALUATION_CHECKS
                if( IsComplexType<Value_t>::result == false ) {
                    EVAL_MULTI_CHECK(itvalue->first < Value_t(1),Stack[SP],4);
                }
#endif
                EVAL_MULTI_APPLY(fp_acosh,Stack[SP],Stack[SP]);
                break;

            case  cAsin:
#ifndef FP_NO_EVALUATION_CHECKS
                if( IsComplexType<Value_t>::result == false ) {
                    EVAL_MULTI_CHECK(itvalue->first < Value_t(-1) || itvalue->first > Value_t(1),Stack[SP],4);
                }
#endif
                EVAL_MULTI_APPLY(fp_asin,Stack[SP],Stack[SP]);
                break;

            case cAsinh:
                EVAL_MULTI_APPLY(fp_asinh,Stack[SP],Stack[SP]);
                break;

            case cAtan:
                EVAL_MULTI_APPLY(fp_atan,Stack[SP],Stack[SP]);
                break;

            case cAtan2:
                EVAL_MULTI_APPLY2(fp_atan2,Stack[SP-1],Stack[SP-1], Stack[SP]);
                --SP;
                break;

            case cAtanh:
#ifndef FP_NO_EVALUATION_CHECKS
                if( IsComplexType<Value_t>::result == false ) {
                    EVAL_MULTI_CHECK(itvalue->first <= Value_t(-1) || itvalue->first >= Value_t(1),Stack[SP],4);
                }
#endif
                EVAL_MULTI_APPLY(fp_atanh,Stack[SP], Stack[SP]);
                break;

            case  cCbrt: EVAL_MULTI_APPLY(fp_cbrt, Stack[SP], Stack[SP]); break;

            case  cCeil: EVAL_MULTI_APPLY(fp_ceil, Stack[SP], Stack[SP]); break;

            case   cCos: EVAL_MULTI_APPLY(fp_cos, Stack[SP], Stack[SP]); break;

            case  cCosh: EVAL_MULTI_APPLY(fp_cosh, Stack[SP], Stack[SP]); break;

            case   cCot:
            {
                EVAL_MULTI_APPLY(fp_tan,Stack[SP],Stack[SP]);
#ifndef FP_NO_EVALUATION_CHECKS

                EVAL_MULTI_CHECK(itvalue->first == Value_t(0),Stack[SP],1);
#endif
                EVAL_MULTI_APPLY(Value_t(1)/,Stack[SP],Stack[SP]);
                break;
            }

            case   cCsc:
            {
                EVAL_MULTI_APPLY(fp_sin,Stack[SP],Stack[SP]);
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(itvalue->first==Value_t(0),Stack[SP],1);
#endif
                EVAL_MULTI_APPLY(Value_t(1)/,Stack[SP],Stack[SP]);
                break;
            }

            case   cExp: EVAL_MULTI_APPLY(fp_exp,Stack[SP],Stack[SP]); break;

            case   cExp2: EVAL_MULTI_APPLY(fp_exp2,Stack[SP],Stack[SP]); break;

            case cFloor: EVAL_MULTI_APPLY(fp_floor,Stack[SP],Stack[SP]); break;

            case cHypot:
                EVAL_MULTI_APPLY2(fp_hypot,Stack[SP-1],Stack[SP-1],Stack[SP]);
                --SP; break;

//          case    cIf:
//                  if(fp_truth(Stack[SP--]))
//                      IP += 2;
//                  else
//                  {
//                      const unsigned* buf = &byteCode[IP+1];
//                      IP = buf[0];
//                      DP = buf[1];
//                  }
//                  break;

            case   cInt: EVAL_MULTI_APPLY(fp_int,Stack[SP],Stack[SP]); break;

            case   cLog:
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(IsComplexType<Value_t>::result ? itvalue->first == Value_t(0) : !(itvalue->first > Value_t(0)),Stack[SP],3);
#endif
                EVAL_MULTI_APPLY(fp_log,Stack[SP],Stack[SP]);
                break;

            case cLog10:
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(IsComplexType<Value_t>::result ? itvalue->first == Value_t(0) : !(itvalue->first > Value_t(0)), Stack[SP],3);
#endif
                EVAL_MULTI_APPLY(fp_log10,Stack[SP],Stack[SP]);
                break;

            case  cLog2:
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(IsComplexType<Value_t>::result ? itvalue->first == Value_t(0) : !(itvalue->first > Value_t(0)), Stack[SP], 3);
#endif
                EVAL_MULTI_APPLY(fp_log2,Stack[SP],Stack[SP]);
                break;

            case   cMax: EVAL_MULTI_APPLY2(fp_max,Stack[SP-1],Stack[SP-1],Stack[SP]);
                --SP; break;

            case   cMin: EVAL_MULTI_APPLY2(fp_min,Stack[SP-1],Stack[SP-1],Stack[SP]);
                --SP; break;

            case   cPow:
#ifndef FP_NO_EVALUATION_CHECKS
                // x:Negative ^ y:NonInteger is failure,
                // except when the reciprocal of y forms an integer
                /*if(Stack[SP-1] < Value_t(0) &&
                   !isInteger(Stack[SP]) &&
                   !isInteger(1.0 / Stack[SP]))
                   { mData->mEvalErrorType=3; return; }*/
                // x:0 ^ y:negative is failure
                EVAL_MULTI_CHECK(itvalue->first == Value_t(0), Stack[SP-1],3);
                EVAL_MULTI_CHECK(itvalue->first < Value_t(0), Stack[SP],3);
#endif
                EVAL_MULTI_APPLY2(fp_pow,Stack[SP-1],Stack[SP-1],Stack[SP]);
                --SP; break;

            case  cTrunc: EVAL_MULTI_APPLY(fp_trunc,Stack[SP], Stack[SP]); break;

            case   cSec:
            {
                EVAL_MULTI_APPLY(fp_cos,Stack[SP],Stack[SP]);
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(itvalue->first == Value_t(0),Stack[SP],1);
#endif
                EVAL_MULTI_APPLY(Value_t(1)/,Stack[SP],Stack[SP]);
                break;
            }

            case   cSin: EVAL_MULTI_APPLY(fp_sin, Stack[SP], Stack[SP]); break;

            case  cSinh: EVAL_MULTI_APPLY(fp_sinh, Stack[SP], Stack[SP]); break;

            case  cSqrt:
#ifndef FP_NO_EVALUATION_CHECKS
                if(IsComplexType<Value_t>::result == false) {
                    EVAL_MULTI_CHECK(itvalue->first < Value_t(0), Stack[SP],2);
                }
#endif
                EVAL_MULTI_APPLY(fp_sqrt,Stack[SP],Stack[SP]);
                break;

            case   cTan: EVAL_MULTI_APPLY(fp_tan, Stack[SP], Stack[SP]); break;

            case  cTanh: EVAL_MULTI_APPLY(fp_tanh, Stack[SP], Stack[SP]); break;

// Misc:
            case cImmed: Stack[++SP].resize(1); Stack[SP][0] = std::make_pair(immed[DP++],int(-1)); break;

            case  cJump:
            {
                const unsigned* buf = &byteCode[IP+1];
                IP = buf[0];
                DP = buf[1];
                break;
            }

// Operators:
            case   cNeg: EVAL_MULTI_APPLY(-,Stack[SP],Stack[SP]); break;
            case   cAdd: EVAL_MULTI_APPLYOP(+,Stack[SP-1],Stack[SP-1],Stack[SP]); --SP; break;
            case   cSub: EVAL_MULTI_APPLYOP(-,Stack[SP-1],Stack[SP-1],Stack[SP]); --SP; break;
            case   cMul: EVAL_MULTI_APPLYOP(*,Stack[SP-1],Stack[SP-1],Stack[SP]); --SP; break;

            case   cDiv:
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(itvalue->first == Value_t(0),Stack[SP],1);
#           else
                if( IsIntType<Value_t>::result ) {
                    EVAL_MULTI_CHECK(IsIntType<Value_t>::result && itvalue->first == Value_t(0),Stack[SP],1);
                }
#endif
                EVAL_MULTI_APPLYOP(/,Stack[SP-1],Stack[SP-1],Stack[SP]);
                --SP; break;

            case   cMod:
                EVAL_MULTI_CHECK(itvalue->first == Value_t(0), Stack[SP],1);
                EVAL_MULTI_APPLY2(fp_mod,Stack[SP-1], Stack[SP-1], Stack[SP]);
                --SP; break;

            case cEqual:
                EVAL_MULTI_APPLY2(fp_equal, Stack[SP-1], Stack[SP-1], Stack[SP]);
                --SP; break;

            case cNEqual:
                EVAL_MULTI_APPLY2(fp_nequal, Stack[SP-1], Stack[SP-1], Stack[SP]);
                --SP; break;

            case  cLess:
                EVAL_MULTI_APPLY2(fp_less, Stack[SP-1], Stack[SP-1], Stack[SP]);
                --SP; break;

            case  cLessOrEq:
                EVAL_MULTI_APPLY2(fp_lessOrEq, Stack[SP-1], Stack[SP-1], Stack[SP]);
                --SP; break;

            case cGreater:
                EVAL_MULTI_APPLY2(fp_less,Stack[SP-1],Stack[SP], Stack[SP-1]);
                --SP; break;

            case cGreaterOrEq:
                EVAL_MULTI_APPLY2(fp_lessOrEq, Stack[SP-1], Stack[SP], Stack[SP-1]);
                --SP; break;

            case   cNot: EVAL_MULTI_APPLY(fp_not, Stack[SP], Stack[SP]); break;

            case cNotNot: EVAL_MULTI_APPLY(fp_notNot, Stack[SP],Stack[SP]); break;

            case   cAnd:
                EVAL_MULTI_APPLY2(fp_and, Stack[SP-1], Stack[SP-1], Stack[SP]);
                --SP; break;

            case    cOr:
                EVAL_MULTI_APPLY2(fp_or, Stack[SP-1], Stack[SP-1], Stack[SP]);
                --SP; break;

// Degrees-radians conversion:
            case   cDeg: EVAL_MULTI_APPLY(RadiansToDegrees, Stack[SP], Stack[SP]); break;
            case   cRad: EVAL_MULTI_APPLY(DegreesToRadians, Stack[SP], Stack[SP]); break;

// User-defined function calls:
            case cFCall:
            {
                unsigned index = byteCode[++IP];
                unsigned params = mData->mFuncPtrs[index].mParams;
                // have to use the cross product, the for loop does this without recursion
                std::vector<Value_t> vparams(params), retVal; retVal.reserve(Stack[SP-params+1].size());
                std::vector<int> vparamindices(params,0);
                std::vector<Value_t> r;
                while(1) {
                    bool docall = true;
                    // for params to be valid, inputs have to be either all -1, or one specific value
                    int maxuniqueindexvalue = -1;
                    for(unsigned int iparam = 0; iparam < params; ++iparam) {
                        std::pair<Value_t,int> param = Stack[SP-params+1+iparam].at(vparamindices[iparam]);
                        if( param.second == -1 || maxuniqueindexvalue == -1 || param.second == maxuniqueindexvalue ) {
                            vparams[iparam] = param.first;
                            maxuniqueindexvalue = mymax(maxuniqueindexvalue,param.second);
                        }
                        else {
                            docall = false;
                            break;
                        }
                    }
                    if( docall ) {
                        if( !mData->mFuncPtrs[index].mRawFuncPtr ) {
                            BoostFunctionWrapper* pboostfn = dynamic_cast<BoostFunctionWrapper*>(mData->mFuncPtrs[index].mFuncWrapperPtr);
                            if( !pboostfn ) {
                                r.resize(1);
                                r[0] = mData->mFuncPtrs[index].mFuncWrapperPtr->callFunction(&vparams.at(0));
                            }
                            else {
                                pboostfn->_fn(r,vparams);
                            }
                        }
                        else {
                            r.resize(1);
                            r[0] = mData->mFuncPtrs[index].mRawFuncPtr(&vparams.at(0));
                        }
                        retVal.insert(retVal.end(),r.begin(),r.end());
                    }
                    // go to the next parameter
                    vparamindices.front() += 1;
                    for(size_t ii = 0; ii+1 < vparamindices.size(); ++ii) {
                        if( vparamindices[ii] >= (int)Stack[SP-params+1+ii].size() ) {
                            vparamindices[ii] = 0;
                            vparamindices[ii+1]++;
                        }
                        else {
                            break;
                        }
                    }
                    if( vparamindices.back() >= (int)Stack[SP].size() ) {
                        break;
                    }
                }
                SP -= int(params)-1;
                Stack[SP].resize(retVal.size());
                for(size_t ii = 0; ii < retVal.size(); ++ii) {
                    Stack[SP][ii] = std::make_pair(retVal[ii],uniquevaluethreadindex++);
                }
                if(retVal.size() == 0 ) {
                    mData->mEvalErrorType = 10;
                    return;
                }
                break;
            }

            case cPCall:
            {
                unsigned index = byteCode[++IP];
                unsigned params = mData->mFuncParsers[index].mParams;
                // have to use the cross product, the for loop does this without recursion
                std::vector<Value_t> vparams(params), retVal; retVal.reserve(Stack[SP-params+1].size());
                std::vector<int> vparamindices(params,0);
                std::vector<Value_t> r;
                while(1) {
                    bool docall = true;
                    // for params to be valid, inputs have to be either all -1, or one specific value
                    int maxuniqueindexvalue = -1;
                    for(unsigned int iparam = 0; iparam < params; ++iparam) {
                        std::pair<Value_t,int> param = Stack[SP-params+1+iparam].at(vparamindices[iparam]);
                        if( param.second == -1 || maxuniqueindexvalue == -1 || param.second == maxuniqueindexvalue ) {
                            vparams[iparam] = param.first;
                            maxuniqueindexvalue = mymax(maxuniqueindexvalue,param.second);
                        }
                        else {
                            docall = false;
                            break;
                        }
                    }
                    if( docall ) {
                        // for some reason cannot do dynamic cast.......
                        //OpenRAVEFunctionParser<Value_t>* mParserPtr = dynamic_cast< OpenRAVEFunctionParser<Value_t>* >(mData->mFuncParsers[index].mParserPtr);
                        OpenRAVEFunctionParser<Value_t>* mParserPtr = static_cast< OpenRAVEFunctionParser<Value_t>* >(mData->mFuncParsers[index].mParserPtr);
                        if( mParserPtr == NULL ) {
                            mData->mEvalErrorType = UNEXPECTED_ERROR;
                            return;
                        }
                        mParserPtr->EvalMulti(r,&vparams.at(0));
                        const int error = mData->mFuncParsers[index].mParserPtr->EvalError();
                        if(!error) {
                            retVal.insert(retVal.end(),r.begin(),r.end());
                        }
                    }
                    // go to the next parameter
                    vparamindices.front() += 1;
                    for(size_t ii = 0; ii+1 < vparamindices.size(); ++ii) {
                        if( vparamindices[ii] >= (int)Stack[SP-params+1+ii].size() ) {
                            vparamindices[ii] = 0;
                            vparamindices[ii+1]++;
                        }
                        else {
                            break;
                        }
                    }
                    if( vparamindices.back() >= (int)Stack[SP].size() ) {
                        break;
                    }
                }
                SP -= int(params)-1;
                Stack[SP].resize(retVal.size());
                for(size_t ii = 0; ii < retVal.size(); ++ii) {
                    Stack[SP][ii] = std::make_pair(retVal[ii],uniquevaluethreadindex++);
                }
                if(retVal.size() == 0 ) {
                    mData->mEvalErrorType = 10;
                    return;
                }
                break;
            }


            case   cFetch:
            {
                unsigned stackOffs = byteCode[++IP];
                Stack[SP+1] = Stack[stackOffs]; ++SP;
                break;
            }

#ifdef FP_SUPPORT_OPTIMIZER
            case   cPopNMov:
            {
                unsigned stackOffs_target = byteCode[++IP];
                unsigned stackOffs_source = byteCode[++IP];
                Stack[stackOffs_target] = Stack[stackOffs_source];
                SP = stackOffs_target;
                break;
            }

            case  cLog2by:
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(IsComplexType<Value_t>::result ? itvalue->first == Value_t(0) : itvalue->first <= Value_t(0),Stack[SP-1],3)
#endif
                EVAL_MULTI_APPLY(fp_log2,Stack[SP-1],Stack[SP-1]);
                EVAL_MULTI_APPLYOP(*,Stack[SP-1],Stack[SP-1],Stack[SP-1]);
                --SP;
                break;

            case cNop: break;
#endif // FP_SUPPORT_OPTIMIZER

            case cSinCos:
                EVAL_MULTI_APPLYOUT2(fp_sinCos,Stack[SP], Stack[SP+1], Stack[SP]);
                ++SP;
                break;

            case cAbsNot:
                EVAL_MULTI_APPLY(fp_absNot,Stack[SP],Stack[SP]); break;
            case cAbsNotNot:
                EVAL_MULTI_APPLY(fp_absNotNot, Stack[SP], Stack[SP]); break;
            case cAbsAnd:
                EVAL_MULTI_APPLY2(fp_absAnd,Stack[SP-1],Stack[SP-1],Stack[SP]);
                --SP; break;
            case cAbsOr:
                EVAL_MULTI_APPLY2(fp_absOr,Stack[SP-1],Stack[SP-1],Stack[SP]);
                --SP; break;
//          case cAbsIf:
//              if(fp_absTruth(Stack[SP--]))
//                  IP += 2;
//              else
//              {
//                  const unsigned* buf = &byteCode[IP+1];
//                  IP = buf[0];
//                  DP = buf[1];
//              }
//              break;

            case   cDup: Stack[SP+1] = Stack[SP]; ++SP; break;

            case   cInv:
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(itvalue->first == Value_t(0),Stack[SP],1);
#           else
                if(IsIntType<Value_t>::result ) {
                    EVAL_MULTI_CHECK(itvalue->first == Value_t(0),Stack[SP],1);
                }
#endif
                EVAL_MULTI_APPLY(Value_t(1)/,Stack[SP],Stack[SP]);
                break;

            case   cSqr:
                for(size_t ii = 0; ii < Stack[SP].size(); ++ii) {
                    Stack[SP][ii].first *= Stack[SP][ii].first;
                }
                break;

            case   cRDiv:
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(itvalue->first == Value_t(0),Stack[SP-1],1);
#           else
                if(IsIntType<Value_t>::result ) {
                    EVAL_MULTI_CHECK(itvalue->first == Value_t(0),Stack[SP-1],1);
                }
#endif
                EVAL_MULTI_APPLYOP(/,Stack[SP-1],Stack[SP],Stack[SP-1]);
                --SP; break;

            case   cRSub: EVAL_MULTI_APPLYOP(-,Stack[SP-1],Stack[SP],Stack[SP-1]); --SP; break;

            case   cRSqrt:
#ifndef FP_NO_EVALUATION_CHECKS
                EVAL_MULTI_CHECK(itvalue->first == Value_t(0),Stack[SP],1);
#endif
                EVAL_MULTI_APPLY(Value_t(1) / fp_sqrt, Stack[SP], Stack[SP]); break;

#ifdef FP_SUPPORT_COMPLEX_NUMBERS
            case   cReal: EVAL_MULTI_APPLY(fp_real,Stack[SP],Stack[SP]); break;
            case   cImag: EVAL_MULTI_APPLY(fp_imag,Stack[SP],Stack[SP]); break;
            case   cArg:  EVAL_MULTI_APPLY(fp_arg,Stack[SP],Stack[SP]); break;
            case   cConj: EVAL_MULTI_APPLY(fp_conj,Stack[SP],Stack[SP]); break;
            case   cPolar:
                EVAL_MULTI_APPLY2(fp_polar,Stack[SP-1],Stack[SP-1],Stack[SP]);
                --SP;
                break;
#endif

// Variables:
            default:
                Stack[++SP].resize(0);
                Stack[SP].emplace_back(Vars[byteCode[IP]-VarBegin], int(-1));
            }
        }

        mData->mEvalErrorType=0;
        finalret.resize(Stack[SP].size());
        for(size_t ii = 0; ii < finalret.size(); ++ii) {
            finalret[ii] = Stack[SP][ii].first;
        }
        return;
    }


    static std::string mlNumberInt(int a)
    {
        std::stringstream ss;
        ss << "<cn>" << a << "</cn>" << endl;
        return ss.str();
    }

    template<typename T>
    static std::string mlNumber(T a)
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
        ss << "<cn type=\"real\">" << a << "</cn>" << endl;
        return ss.str();
    }

    template<typename T>
    static std::string mlComplexPolarNumber(T a, T b)
    {
        std::stringstream ss;
        ss << "<cn type=\"complex-polar\">" << a << "<sep/>" << b << "</cn>" << endl;
        return ss.str();
    }

    static std::string mlApplyFunction(const char* functionname, const std::string& var)
    {
        std::string s; s.reserve(var.size() + strlen(functionname) + 20);
        s += "<apply>\n<";
        s += functionname;
        s += "/>\n";
        s += var;
        s += "</apply>\n";
        return s;
    }

    static std::string mlApplyFunction(const char* functionname, const std::string& var0, const std::string& var1)
    {
        std::string s; s.reserve(var1.size() + var0.size() + strlen(functionname) + 20);
        s += "<apply>\n<";
        s += functionname;
        s += "/>\n";
        s += var0;
        s += var1;
        s += "</apply>\n";
        return s;
    }

//===========================================================================
// Function evaluation
//===========================================================================
    bool toMathML(std::string& sout, const std::vector<std::string>& Vars)
    {
        using namespace FUNCTIONPARSERTYPES;
        typename FunctionParserBase<Value_t>::Data* mData = FunctionParserBase<Value_t>::getParserData();
        if(mData->mParseErrorType != FP_NO_ERROR) {
            return false;
        }

        const unsigned* const byteCode = &(mData->mByteCode[0]);
        const Value_t* const immed = mData->mImmed.empty() ? 0 : &(mData->mImmed[0]);
        const unsigned byteCodeSize = unsigned(mData->mByteCode.size());
        unsigned IP, DP=0;
        int SP=-1;
        std::vector<std::string> Stack(mData->mStackSize);

        for(IP=0; IP<byteCodeSize; ++IP)
        {
            switch(byteCode[IP])
            {
// Functions:
            case   cAbs: Stack[SP] = mlApplyFunction("abs",Stack[SP]); break;

            case  cAcos: Stack[SP] = mlApplyFunction("acos",Stack[SP]); break;

            case cAcosh: Stack[SP] = mlApplyFunction("acosh",Stack[SP]); break;

            case  cAsin: Stack[SP] = mlApplyFunction("asin",Stack[SP]); break;

            case cAsinh: Stack[SP] = mlApplyFunction("asinh",Stack[SP]); break;

            case  cAtan: Stack[SP] = mlApplyFunction("atan",Stack[SP]); break;

            case cAtan2: Stack[SP-1] = mlApplyFunction("atan2",Stack[SP-1], Stack[SP]);
                --SP; break;

            case cAtanh: Stack[SP] = mlApplyFunction("atanh",Stack[SP]); break;

            case  cCbrt: Stack[SP] = mlApplyFunction("cbrt",Stack[SP]); break;

            case  cCeil: Stack[SP] = mlApplyFunction("ceil",Stack[SP]); break;

            case   cCos: Stack[SP] = mlApplyFunction("cos",Stack[SP]); break;

            case  cCosh: Stack[SP] = mlApplyFunction("cosh",Stack[SP]); break;

            case   cCot: Stack[SP] = mlApplyFunction("cot",Stack[SP]); break;

            case   cCsc: Stack[SP] = mlApplyFunction("csc",Stack[SP]); break;

            case   cExp: Stack[SP] = mlApplyFunction("exp",Stack[SP]); break;

            case   cExp2: Stack[SP] = mlApplyFunction("power",mlNumberInt(2),Stack[SP]); break;

            case cFloor: Stack[SP] = mlApplyFunction("floor",Stack[SP]); break;

            case cHypot:
                Stack[SP-1] = mlApplyFunction("sqrt",mlApplyFunction("times",Stack[SP-1],Stack[SP-1]),mlApplyFunction("times", Stack[SP], Stack[SP]));
                --SP; break;

            case    cIf:
                printf("do not support cIf\n");
//                  if(fp_truth(Stack[SP--]))
//                      IP += 2;
//                  else
//                  {
//                      const unsigned* buf = &byteCode[IP+1];
//                      IP = buf[0];
//                      DP = buf[1];
//                  }
                break;

            case   cInt: Stack[SP] = mlApplyFunction("floor",mlApplyFunction("plus",Stack[SP],mlNumber<double>(0.5))); break;

            case   cLog: Stack[SP] = mlApplyFunction("ln",Stack[SP]); break;

            case cLog10: Stack[SP] = mlApplyFunction("log",Stack[SP]); break;

            case  cLog2: Stack[SP] = mlApplyFunction("log","<logbase><cn>2</cn></logbase>\n",Stack[SP]); break;

            case   cMax: Stack[SP-1] = mlApplyFunction("max",Stack[SP-1], Stack[SP]);
                --SP; break;

            case   cMin: Stack[SP-1] = mlApplyFunction("min",Stack[SP-1], Stack[SP]);
                --SP; break;

            case   cPow: Stack[SP-1] = mlApplyFunction("power",Stack[SP-1], Stack[SP]);
                --SP; break;

            case  cTrunc:
                printf("trunk not supported\n");
                //Stack[SP] = fp_trunc(Stack[SP]);
                break;

            case   cSec: Stack[SP] = mlApplyFunction("sec",Stack[SP]); break;

            case   cSin: Stack[SP] = mlApplyFunction("sin",Stack[SP]); break;

            case  cSinh: Stack[SP] = mlApplyFunction("sinh",Stack[SP]); break;

            case  cSqrt: Stack[SP] = mlApplyFunction("root","<degree><cn>2</cn></degree>",Stack[SP]); break;

            case   cTan: Stack[SP] = mlApplyFunction("tan",Stack[SP]); break;

            case  cTanh: Stack[SP] = mlApplyFunction("tanh",Stack[SP]); break;

// Misc:
            case cImmed: Stack[++SP] = mlNumber<Value_t>(immed[DP++]); break;

            case  cJump:
            {
                printf("jump?\n");
                const unsigned* buf = &byteCode[IP+1];
                IP = buf[0];
                DP = buf[1];
                break;
            }

// Operators:
            case   cNeg: Stack[SP] = mlApplyFunction("minus",Stack[SP]); break;
            case   cAdd: Stack[SP-1] = mlApplyFunction("plus",Stack[SP-1],Stack[SP]); --SP; break;
            case   cSub: Stack[SP-1] = mlApplyFunction("minus",Stack[SP-1],Stack[SP]); --SP; break;
            case   cMul: Stack[SP-1] = mlApplyFunction("times",Stack[SP-1],Stack[SP]); --SP; break;

            case   cDiv: Stack[SP-1] = mlApplyFunction("divide",Stack[SP-1],Stack[SP]); --SP; break;

            case   cMod: Stack[SP-1] = mlApplyFunction("rem",Stack[SP-1], Stack[SP]);
                --SP; break;

            case cEqual:
                Stack[SP-1] = mlApplyFunction("eq", Stack[SP-1], Stack[SP]);
                --SP; break;

            case cNEqual:
                Stack[SP-1] = mlApplyFunction("neq",Stack[SP-1], Stack[SP]);
                --SP; break;

            case  cLess:
                Stack[SP-1] = mlApplyFunction("lt",Stack[SP-1], Stack[SP]);
                --SP; break;

            case  cLessOrEq:
                Stack[SP-1] = mlApplyFunction("leq",Stack[SP-1], Stack[SP]);
                --SP; break;

            case cGreater:
                Stack[SP-1] = mlApplyFunction("gt",Stack[SP-1], Stack[SP]);
                --SP; break;

            case cGreaterOrEq:
                Stack[SP-1] = mlApplyFunction("geq",Stack[SP-1], Stack[SP]);
                --SP; break;

            case   cNot: Stack[SP] = mlApplyFunction("not",Stack[SP]); break;

            case cNotNot: Stack[SP] = mlApplyFunction("not",mlApplyFunction("not",Stack[SP])); break;

            case   cAnd:
                Stack[SP-1] = mlApplyFunction("and", Stack[SP-1], Stack[SP]);
                --SP; break;

            case    cOr:
                Stack[SP-1] = mlApplyFunction("or",Stack[SP-1], Stack[SP]);
                --SP; break;

// Degrees-radians conversion:
            case   cDeg: Stack[SP] = mlApplyFunction("times",Stack[SP],mlNumber<Value_t>(RadiansToDegrees(Value_t(1)))); break;
            case   cRad: Stack[SP] = mlApplyFunction("times",Stack[SP],mlNumber<Value_t>(DegreesToRadians(Value_t(1)))); break;

// User-defined function calls:
            case cFCall:
            {

                unsigned index = byteCode[++IP];
                unsigned params = mData->mFuncPtrs[index].mParams;
                // go through the existing names to find the function name
                string functionname;
                for(typename NamePtrsMap<Value_t>::iterator i = mData->mNamePtrs.begin(); i != mData->mNamePtrs.end(); ++i) {
                    if(i->second.type == NameData<Value_t>::FUNC_PTR && i->second.index == index ) {
                        functionname = std::string(i->first.name,i->first.nameLength);
                        break;
                    }
                }
                if( functionname.size() == 0 ) {
                    mData->mEvalErrorType = SYNTAX_ERROR;
                    return false;
                }

                // custom function, so use csymbol
                std::string s; s.reserve(100);
                s += "<apply>\n";
                s += "<csymbol encoding=\"text/xml\" definitionURL=\"\" type=\"function\">";
                s += functionname;
                s += "</csymbol>\n";
                for(unsigned i = 0; i < params; ++i) {
                    s += Stack[SP-params+1+i];
                }
                s += "</apply>\n";
                SP -= int(params)-1;
                Stack[SP] = s;
                break;
            }

            case cPCall:
            {
                printf("parser calls not supported\n");
                unsigned index = byteCode[++IP];
                unsigned params = mData->mFuncParsers[index].mParams;
                //Value_t retVal = mData->mFuncParsers[index].mParserPtr->Eval(&Stack[SP-params+1]);
                SP -= int(params)-1;
                Stack[SP] = "";
                const int error = SYNTAX_ERROR; //mData->mFuncParsers[index].mParserPtr->EvalError();
                if(error)
                {
                    mData->mEvalErrorType = error;
                    return false;
                }
                break;
            }


            case   cFetch:
            {
                unsigned stackOffs = byteCode[++IP];
                Stack[SP+1] = Stack[stackOffs]; ++SP;
                break;
            }

#ifdef FP_SUPPORT_OPTIMIZER
            case   cPopNMov:
            {
                unsigned stackOffs_target = byteCode[++IP];
                unsigned stackOffs_source = byteCode[++IP];
                Stack[stackOffs_target] = Stack[stackOffs_source];
                SP = stackOffs_target;
                break;
            }

            case  cLog2by:
                Stack[SP] = mlApplyFunction("times",mlApplyFunction("log","<logbase><cn>2</cn></logbase>\n",Stack[SP-1]),Stack[SP]);
                --SP;
                break;

            case cNop: break;
#endif // FP_SUPPORT_OPTIMIZER

            case cSinCos:
                Stack[SP+1] = mlApplyFunction("cos",Stack[SP]); break;
                Stack[SP] = mlApplyFunction("sin",Stack[SP]); break;
                ++SP;
                break;

            case cAbsNot:
                if( IsIntType<Value_t>::result ) {
                    Stack[SP] = mlApplyFunction("leq",Stack[SP],mlNumberInt(0));
                }
                else {
                    Stack[SP] = mlApplyFunction("leq",Stack[SP],mlNumber<double>(0.5));
                }
                break;
            case cAbsNotNot:
                if( IsIntType<Value_t>::result ) {
                    Stack[SP] = mlApplyFunction("gt",Stack[SP],mlNumberInt(0));
                }
                else {
                    Stack[SP] = mlApplyFunction("gt",Stack[SP],mlNumber<double>(0.5));
                }
                break;
            case cAbsAnd:
                if( IsIntType<Value_t>::result ) {
                    Stack[SP] = mlApplyFunction("and",mlApplyFunction("gt",Stack[SP-1],mlNumberInt(0)),mlApplyFunction("gt",Stack[SP],mlNumberInt(0)));
                }
                else {
                    Stack[SP] = mlApplyFunction("and",mlApplyFunction("gt",Stack[SP-1],mlNumber<double>(0.5)),mlApplyFunction("gt",Stack[SP],mlNumber<double>(0.5)));
                }
                --SP; break;
            case cAbsOr:
                if( IsIntType<Value_t>::result ) {
                    Stack[SP] = mlApplyFunction("or",mlApplyFunction("gt",Stack[SP-1],mlNumberInt(0)),mlApplyFunction("gt",Stack[SP],mlNumberInt(0)));
                }
                else {
                    Stack[SP] = mlApplyFunction("or",mlApplyFunction("gt",Stack[SP-1],mlNumber<double>(0.5)),mlApplyFunction("gt",Stack[SP],mlNumber<double>(0.5)));
                }
                --SP; break;
            case cAbsIf:
                printf("do not support if statement\n");
                IP += 2;
//              if(fp_absTruth(Stack[SP--]))
//                  IP += 2;
//              else
//              {
//                  const unsigned* buf = &byteCode[IP+1];
//                  IP = buf[0];
//                  DP = buf[1];
//              }
                break;

            case   cDup: Stack[SP+1] = Stack[SP]; ++SP; break;

            case   cInv: Stack[SP] = mlApplyFunction("divide",mlNumber<Value_t>(Value_t(1)),Stack[SP]); break;

            case   cSqr: Stack[SP] = mlApplyFunction("power",Stack[SP], mlNumberInt(2)); break;

            case   cRDiv: Stack[SP-1] = mlApplyFunction("divide",Stack[SP],Stack[SP-1]); --SP; break;

            case   cRSub: Stack[SP-1] = mlApplyFunction("minus",Stack[SP], Stack[SP-1]); --SP; break;

            case   cRSqrt: Stack[SP] = mlApplyFunction("power",Stack[SP],mlNumber<double>(-0.5)); break;

#ifdef FP_SUPPORT_COMPLEX_NUMBERS
            case   cReal: Stack[SP] = mlApplyFunction("real",Stack[SP]); break;
            case   cImag: Stack[SP] = mlApplyFunction("imaginary",Stack[SP]); break;
            case   cArg:  Stack[SP] = mlApplyFunction("arg",Stack[SP]); break;
            case   cConj: Stack[SP] = mlApplyFunction("conjugate",Stack[SP]); break;
            case   cPolar:
                Stack[SP-1] = mlComplexPolarNumber(Stack[SP-1], Stack[SP]);
                --SP;
                break;
#endif
// Variables:
            default:
                Stack[++SP] = Vars[byteCode[IP]-VarBegin];
            }
        }

        mData->mEvalErrorType=0;
        sout = Stack[SP];
        return true;
    }
};

class OpenRAVEFunctionParserReal : public OpenRAVEFunctionParser<dReal>
{
};

}

#endif
