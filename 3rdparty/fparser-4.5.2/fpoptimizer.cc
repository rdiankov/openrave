/***************************************************************************\
|* Function Parser for C++ v4.5.2                                          *|
|*-------------------------------------------------------------------------*|
|* Function optimizer                                                      *|
|*-------------------------------------------------------------------------*|
|* Copyright: Joel Yliluoma                                                *|
|*                                                                         *|
|* This library is distributed under the terms of the                      *|
|* GNU Lesser General Public License version 3.                            *|
|* (See lgpl.txt and gpl.txt for the license text.)                        *|
\***************************************************************************/

/* NOTE:
 This file contains generated code (from the optimizer sources) and is
 not intended to be modified by hand. If you want to modify the optimizer,
 download the development version of the library.
*/

#include "fpconfig.hh"
#ifdef FP_SUPPORT_OPTIMIZER
#include "fparser.hh"
#include "extrasrc/fptypes.hh"
#include "extrasrc/fpaux.hh"
#pragma GCC diagnostic ignored "-Wunused-variable"
#define tO3 ;typedef
#define tN3 (xP1 a)
#define tM3 :sim i61
#define tL3 },{{1,
#define tK3 :if(tO2
#define tJ3 xP1 a),
#define tI3 {data->
#define tH3 nW 0,
#define tG3 {tree.x6
#define tF3 "Found "
#define tE3 ;for lR1
#define tD3 stackpos
#define tC3 .GetOpcode
#define tB3 "dup(%u) "
#define tA3 "%d, cost "
#define t93 "PUSH " yR2
#define t83 ::cout<<eZ3
#define t73 "immed "<<
#define t63 mFuncParsers
#define t53 cV2{assert
#define t43 stderr
#define t33 sep2=" "
#define t23 FPHASH_CONST
#define t13 cache_needed[
#define t03 fprintf
#define eZ3 "Applying "
#define eY3 FUNCTIONPARSER_INSTANTIATE_OPTIMIZE
#define eX3 FUNCTIONPARSER_INSTANTIATE_EMPTY_OPTIMIZE
#define eW3 HANDLE_UNARY_CONST_FUNC
#define eV3 ,l1 0x7
#define eU3 within,
#define eT3 c_count
#define eS3 s_count
#define eR3 tI2 xT1
#define eQ3 2)lT 2*
#define eP3 );break
#define eO3 ].swap(
#define eN3 else{if(
#define eM3 b.Value)
#define eL3 b.Opcode
#define eK3 FP_GetOpcodeName
#define eJ3 =synth.
#define eI3 codes[b
#define eH3 whydump
#define eG3 nparams
#define eF3 450998,
#define eE3 cHypot,
#define eD3 cExp nW
#define eC3 cAbs nW
#define eB3 )&&p nV
#define eA3 eO;case
#define e93 :tree
#define e83 false;}
#define e73 l41 y6.
#define e63 cAbsIf)
#define e53 ){case
#define e43 tQ nF==
#define e33 =true;yH1
#define e23 =false;
#define e13 params
#define e03 ].first
#define cZ3 Ne_Mask
#define cY3 Gt_Mask
#define cX3 Lt_Mask
#define cW3 opcode,
#define cV3 public:
#define cU3 *xB lQ1
#define cT3 pclone
#define cS3 info.
#define cR3 --cU1.
#define cQ3 eK2 2,
#define cP3 eK2 1,
#define cO3 cOr,l6
#define cN3 switch nR1
#define cM3 xG p2;p2
#define cL3 if(op1==
#define cK3 (tree)!=
#define cJ3 TreeCountItem
#define cI3 ){if(
#define cH3 newpow
#define cG3 tX2 lU1
#define cF3 &&p e32
#define cE3 )))eO lI
#define cD3 if(xW)&&
#define cC3 xP1 2)
#define cB3 change
#define cA3 133,2,
#define c93 Needs
#define c83 byteCode
#define c73 nT2 a eF
#define c63 lP1 nF==
#define c53 factor_t
#define c43 y7 tmp2)
#define c33 value1
#define c23 a));if(!
#define c13 tM nD lD
#define c03 cAbsNot
#define yZ3 ::ByteCodeSynth yB
#define yY3 break;}
#define yX3 }switch
#define yW3 stackptr
#define yV3 cPow);lD
#define yU3 .empty()
#define yT3 cMul);xN
#define yS3 opcodes
#define yR3 did_muli
#define yQ3 c4 data.
#define yP3 &Value){
#define yO3 yK const
#define yN3 yY3}
#define yM3 used[b]
#define yL3 if(a>0){
#define yK3 :if((i42
#define yJ3 :{lW1 r=
#define yI3 sizeof(
#define yH3 cLess c9
#define yG3 ,cExp2 nW
#define yF3 lK 2},0,
#define yE3 ::string
#define yD3 return p
#define yC3 ;}return
#define yB3 Others
#define yA3 param.
#define y93 .first.
#define y83 (yA3
#define y73 break;yX3(
#define y63 &param=*
#define y53 );tmp2.n7
#define y43 nF==cLog2&&
#define y33 nF==cPow&&tU
#define y23 if(xW nR
#define y13 default:
#define y03 x83 size()
#define xZ3 ].data);
#define xY3 tL));x0
#define xX3 Ge0Lt1
#define xW3 Gt0Le1
#define xV3 =fp_pow(
#define xU3 ==cOr)l9
#define xT3 cAdd l02
#define xS3 x62 0;b<
#define xR3 iterator
#define xQ3 begin();
#define xP3 TreeSet
#define xO3 parent
#define xN3 insert(i
#define xM3 newrel
#define xL3 IsNever
#define xK3 b_needed
#define xJ3 cachepos
#define xI3 half&64)
#define xH3 half=
#define xG3 1 y8 lX1
#define xF3 FindPos(
#define xE3 src_pos
#define xD3 reserve(
#define xC3 tree.GetHash()
#define xB3 iT1 tree
#define xA3 ){pow.l41
#define x93 Params[a]
#define x83 Params.
#define x73 yD void
#define x63 treeptr
#define x53 .resize(
#define x43 yL1 xG>&
#define x33 t81 void
#define x23 ImmedTag
#define x13 yD class
#define x03 ),child);
#define nZ3 tmp.n7 0))
#define nY3 ));tmp c4
#define nX3 a,const
#define nW3 RefCount
#define nV3 Birth();
#define nU3 exponent
#define nT3 result
#define nS3 cost_t
#define nR3 fpdata
#define nQ3 middle
#define nP3 ifdata
#define nO3 };enum
#define nN3 );eE t71
#define nM3 cLog2by);
#define nL3 sqrt_cost
#define nK3 const int
#define nJ3 mul_count
#define nI3 maxValue1
#define nH3 minValue1
#define nG3 maxValue0
#define nF3 minValue0
#define nE3 ValueType
#define nD3 );}yD bool
#define nC3 xC lU 2,
#define nB3 const std::eQ
#define nA3 const char*
#define n93 x31 a-->0;)if(
#define n83 ContainsOtherCandidates
#define n73 ;if(half
#define n63 ;}void
#define n53 ,l62(1))){
#define n43 ,const e3&
#define n33 )n63
#define n23 1),l62(1));
#define n13 nT3 t6
#define n03 nT3 eY
#define lZ3 nT3 cT1
#define lY3 n03 e23 if(
#define lX3 nT3 nV
#define lW3 nT3 e32
#define lV3 yB nT3
#define lU3 yI n2 0),
#define lT3 abs_mul
#define lS3 l8 a));
#define lR3 pos_set
#define lQ3 e61);}if(
#define lP3 Rehash(iB
#define lO3 cM ifp2);
#define lN3 sim.x3 1,
#define lM3 [funcno].
#define lL3 eE1[++IP]
#define lK3 eE1[IP]==
#define lJ3 subtree
#define lI3 invtree
#define lH3 MakeHash(
#define lG3 rulenumit
#define lF3 (cond yY
#define lE3 ))break;l62
#define lD3 ;}static yE1
#define lC3 {std::cout<<
#define lB3 a<tree.xD
#define lA3 cAnd,l6
#define l93 if y83
#define l83 (tree nF)
#define l73 MakeEqual
#define l63 n61,l4::
#define l53 n61,{l4::
#define l43 newbase
#define l33 branch1op
#define l23 branch2op
#define l13 overlap
#define l03 truth_b
#define iZ2 truth_a
#define iY2 found_dup
#define iX2 cY1 xG&
#define iW2 nQ r;r c4
#define iV2 rangeutil
#define iU2 Plan_Has(
#define iT2 StackMax)
#define iS2 const nV2
#define iR2 namespace
#define iQ2 ::res,b8<
#define iP2 inverted
#define iO2 xL3:
#define iN2 .known&&
#define iM2 iftree
#define iL2 depcodes
#define iK2 explicit
#define iJ2 cPow,l2 0,2,
#define iI2 cPow,xK1
#define iH2 ,{ReplaceParams,
#define iG2 ,eB2 7168
#define iF2 cCosh nW
#define iE2 VarBegin
#define iD2 .Become(
#define iC2 begin(),
#define iB2 cond_add
#define iA2 cond_mul
#define i92 cond_and
#define i82 mulgroup
#define i72 leaf1
#define i62 );synth
#define i52 ,cEqual
#define i42 cU3.get()){
#define i32 )lT 3*3*
#define i22 Optimize()
#define i12 costree
#define i02 sintree
#define tZ2 leaf_count
#define tY2 sub_params
#define tX2 nT3.
#define tW2 printf(
#define tV2 swap(tmp);
#define tU2 cbrt_count
#define tT2 sqrt_count
#define tS2 PlusInf
#define tR2 Finite
#define tQ2 (lW3
#define tP2 tI2 n2 0),
#define tO2 xW)){l62 tmp=
#define tN2 l62(0.0 nR
#define tM2 p1 cM ifp1
#define tL2 pcall_tree
#define tK2 after_powi
#define tJ2 ))return
#define tI2 );m.max.
#define tH2 ;a<t7;++a)
#define tG2 else{xB=new
#define tF2 yS e83
#define tE2 e13)
#define tD2 grammar
#define tC2 ,cGreater
#define tB2 ,lB 0x12 nM
#define tA2 cLog nW
#define t92 0x12},{{3,
#define t82 cNeg,lU 1,
#define t72 ),0},{
#define t62 .data.get(
#define t52 data;data.
#define t42 MakeNEqual
#define t32 xP1 1)
#define t22 Dump(std::
#define t12 isInteger(
#define t02 Comparison
#define eZ2 ,bool abs)
#define eY2 needs_flip
#define eX2 (half&63)-1;
#define eW2 value]
#define eV2 x71 xR+1);
#define eU2 Rule&rule,
#define eT2 cM tree);
#define eS2 mul_item
#define eR2 innersub
#define eQ2 cbrt_cost
#define eP2 best_cost
#define eO2 condition
#define eN2 per_item
#define eM2 item_type
#define eL2 first2
#define eK2 ,l7 0,
#define eJ2 yI known
#define eI2 ,o);o<<"\n";
#define eH2 info=(*xB)[
#define eG2 cU3=r.specs;if(r.found){
#define eF2 cU3,info
#define eE2 *)start_at.get();
#define eD2 l3 18,1,
#define eC2 cIf,eV 3,
#define eB2 l0 2,
#define eA2 lK 1},0,
#define e92 .what nL1
#define e82 tree))cN
#define e72 ;std::vector<
#define e62 )x02.second
#define e52 );range.x22
#define e42 (lS,tM)nD lD
#define e32 .min.val
#define e22 Decision
#define e12 ;tree y7
#define e02 not_tree
#define cZ2 group_by
#define cY2 nU3=
#define cX2 (std::move(
#define cW2 ->second
#define cV2 xG&tree)
#define cU2 targetpos
#define cT2 eO true;}
#define cS2 ParamSpec
#define cR2 rhs.hash2;}
#define cQ2 rhs.hash1
#define cP2 struct
#define cO2 Forget()
#define cN2 &&cond eH))
#define cM2 source_tree
#define cL2 <tH,nS3>
#define cK2 p1_evenness
#define cJ2 isNegative(
#define cI2 (yI val
#define cH2 ,std::cout)
#define cG2 neg_set
#define cF2 cNop,cNop}}
#define cE2 cTanh,cNop,
#define cD2 >cP2 cG<
#define cC2 matches
#define cB2 .match_tree
#define cA2 (rule,tree,info
#define c92 cY1 void*)&
#define c82 cGreater c9
#define c72 l5 0,1,
#define c62 ,cTan nW
#define c52 .xU 2)xX cPow)
#define c42 cCos nW
#define c32 {data xE lO
#define c22 +=1 eO n21;
#define c12 negated
#define c02 iC,1,iZ+1);
#define yZ2 Specializer
#define yY2 coshtree
#define yX2 sinhtree
#define yW2 best_score
#define yV2 mulvalue
#define yU2 pow_item
#define yT2 );t9=!t9;}
#define yS2 .push_back(
#define yR2 ;DumpTree(
#define yQ2 eB[c i1
#define yP2 .IsImmed(
#define yO2 1)yP2)
#define yN2 PowiResult
#define yM2 maxValue
#define yL2 minValue
#define yK2 fp_min(yH,
#define yJ2 yI set(fp_floor
#define yI2 fp_max(yH)
#define yH2 div_tree
#define yG2 pow_tree
#define yF2 preserve
#define yE2 xP1 a).xE1
#define yD2 cLog);sim.xU
#define yC2 PullResult()
#define yB2 dup_or_fetch
#define yA2 nominator]
#define y92 test_order
#define y82 TopLevel)
#define y72 ].info=info;
#define y62 ):start_at()
#define y52 .param_count
#define y42 minimum_need
#define y32 shift(index)
#define y22 ,tree xA
#define y12 rulenumber
#define y02 cLessOrEq c9
#define xZ2 cTan l3 2,1,
#define xY2 cLog l3 2,1,
#define xX2 cTanh nW
#define xW2 ,cSinh nW
#define xV2 cInv,lU 1,
#define xU2 constraints=
#define xT2 GetDepth()
#define xS2 factor_immed
#define xR2 changes
#define xQ2 n11 cM y6 l8
#define xP2 cM cond l8
#define xO2 ,cGreaterOrEq
#define xN2 l8 0));
#define xM2 y7 mul);
#define xL2 for(typename
#define xK2 exp_diff
#define xJ2 ExponentInfo
#define xI2 lower_bound(
#define xH2 factor
#define xG2 is_logical
#define xF2 newrel_and
#define xE2 ;iC.Remember(
#define xD2 eO Unknown;}
#define xC2 res_stackpos
#define xB2 half_pos
#define xA2 ){half&=127;
#define x92 {e3 start_at;
#define x82 i52,eB2
#define x72 (size_t
#define x62 for x72 b=
#define x52 >>1)):(
#define x42 CodeTreeData
#define x32 nU3)
#define x22 multiply(
#define x12 var_trees
#define x02 parampair
#define nZ2 ,cPow,l2 2,2,
#define nY2 ,nM2 2,1,
#define nX2 std::vector<xG>
#define nW2 nE OPCODE
#define nV2 CodeTree&
#define nU2 parent_opcode
#define nT2 =GetParam(
#define nS2 changed=true;
#define nR2 log2_exponent
#define nQ2 tmp y7 tree);
#define nP2 nQ tmp;tmp c4
#define nO2 dup_fetch_pos
#define nN2 xL3 c5 lD
#define nM2 cPow l3
#define nL2 cSin nW
#define nK2 Value_EvenInt
#define nJ2 MakeFalse,{l4
#define nI2 if(y2 l8 a)xI
#define nH2 AddCollection
#define nG2 ConditionType
#define nF2 DUP_ONE(apos)
#define nE2 ,0,0x4 tL3
#define nD2 (unsigned
#define nC2 cY|nD2)
#define nB2 SpecialOpcode
#define nA2 sim.Eat(1,
#define n92 ;sim.Push(
#define n82 c4 tree nF);
#define n72 for x72 a=
#define n62 .UseGetNeeded(
#define n52 ));TriTruthValue
#define n42 synth.AddOperation(
#define n32 ,eR,synth);
#define n22 =i cW2.
#define n12 IsDefined()
#define n02 .GetHash().
#define lZ2 assimilated
#define lY2 );sim.x3 2,
#define lX2 denominator
#define lW2 fraction
#define lV2 DUP_BOTH();
#define lU2 template lM
#define lT2 -1-offset].
#define lS2 if(synth.Find(
#define lR2 IsDescendantOf
#define lQ2 TreeCounts
#define lP2 ;tree.DelParam(
#define lO2 IsImmed()cI3
#define lN2 bool t9 e23
#define lM2 SetOpcode(
#define lL2 found_log2
#define lK2 div_params
#define lJ2 immed_sum
#define lI2 OPCODE(opcode)
#define lH2 break;nT3*=
#define lG2 FactorStack yB
#define lF2 Rehash(false);
#define lE2 IsAlways c5 lD
#define lD2 282870 x9
#define lC2 cNotNot nW
#define lB2 replacing_slot
#define lA2 RefParams
#define l92 if_always[
#define l82 WhatDoWhenCase
#define l72 exponent_immed
#define l62 Value_t
#define l52 );x0 l4::
#define l42 GetOpcode())
#define l32 );goto do_return;
#define l22 {if(GetOpcode()
#define l12 CollectionSet yB
#define l02 ||op1==
#define iZ1 yF DelParams()
#define iY1 .SetParamsMove(
#define iX1 data[a].second
#define iW1 if(newrel_or==
#define iV1 eA 2,131,
#define iU1 Immed.size());
#define iT1 const xG&
#define iS1 OptimizedUsing
#define iR1 Var_or_Funcno
#define iQ1 iR1;
#define iP1 GetParams(
#define iO1 crc32_t
#define iN1 signed_chain
#define iM1 MinusInf
#define iL1 n_immeds
#define iK1 stack.size()
#define iJ1 std::cout<<"POP "
#define iI1 FindClone(xN
#define iH1 needs_rehash
#define iG1 AnyWhere_Rec
#define iF1 ~unsigned(0)
#define iE1 41,42,43,44,
#define iD1 p1_logical_b
#define iC1 p0_logical_b
#define iB1 p1_logical_a
#define iA1 p0_logical_a
#define i91 divgroup
#define i81 else if(
#define i71 {case IsAlways:
#define i61 .xU l62(
#define i51 ,l5 2,1,
#define i41 ,1,2,1,4,1,2,
#define i31 }break cT1
#define i21 tree nF==
#define i11 func(val);nJ1
#define i01 *const func)
#define tZ1 synth.DoDup(
#define tY1 cache_needed
#define tX1 eA 2,1,eA 2,
#define tW1 [nP3.ofs+
#define tV1 treelist
#define tU1 has_bad_balance
#define tT1 c53 xH2
#define tS1 fp_abs(max.val))
#define tR1 )eO m cT1
#define tQ1 fp_abs(min.val)
#define tP1 cNEqual
#define tO1 tP 2},0,0x0},{{
#define tN1 Oneness_NotOne|
#define tM1 Value_IsInteger
#define tL1 Constness_Const
#define tK1 DumpHashesFrom(
#define tJ1 iS1(
#define tI1 reltype
#define tH1 SequenceOpcodes
#define tG1 sep_list[
#define tF1 ,l1 0x0},{{3,
#define tE1 fpExponentIsTooLarge(
#define tD1 l62(0
#define tC1 goto fail;}
#define tB1 ,cNot nW
#define tA1 l1 0x4 nM
#define t91 template<
#define t81 lA2);
#define t71 subgroup
#define t61 lQ2.erase(cs_it);
#define t51 yL1 unsigned>&eE1,size_t&IP,size_t limit,size_t y4
#define t41 tJ2 true
#define t31 TreeCountType yB
#define t21 xG tmp;tmp c4
#define t11 ;nU3
#define t01 Value(Value::
#define eZ1 >(l62(1),
#define eY1 0.5))xX yV3
#define eX1 stack[iK1-
#define eW1 stack yS2
#define eV1 synth.PushImmed(
#define eU1 MaxChildDepth
#define eT1 std::pair<It,It>
#define eS1 cPow,lB
#define eR1 Sign_Negative
#define eQ1 Value_Logical
#define eP1 new_factor_immed
#define eO1 base_immed
#define eN1 public e9,public std::vector<
#define eM1 .Rehash();
#define eL1 );y6 eM1 xG
#define eK1 occurance_pos
#define eJ1 exponent_hash
#define eI1 exponent_list
#define eH1 CollectMulGroup(
#define eG1 source_set
#define eF1 nU3,xP3
#define eE1 ByteCode
#define eD1 operator
#define eC1 FindAndDup(tree);
#define eB1 .yJ l62(2)));
#define eA1 back().thenbranch
#define e91 grammar_rules[*r]
#define e81 ;flipped=!flipped;}
#define e71 DelParam(a);}
#define e61 tree.DelParam(a
#define e51 synth.xH 1
#define e41 ,iC n32
#define e31 =comp.AddItem(atree
#define e21 (long double)
#define e11 yB())xX cMul);lD
#define e01 tree yP2)c5
#define cZ1 ;synth.StackTopIs(
#define cY1 (const
#define cX1 cY1 l62&
#define cW1 >=tD1)
#define cV1 iR2 FPoptimizer_Optimize
#define cU1 NeedList
#define cT1 ;}case
#define cS1 ,lI2);
#define cR1 ParamSpec_Extract
#define cQ1 retry_anyparams_3
#define cP1 retry_anyparams_2
#define cO1 needlist_cached_t
#define cN1 lK 2}nE2
#define cM1 lK 1}nE2
#define cL1 CodeTreeImmed yB(
#define cK1 by_float_exponent
#define cJ1 fp_equal(nU3
#define cI1 new_exp
#define cH1 end()&&i->first==
#define cG1 return BecomeZero;
#define cF1 return BecomeOne;
#define cE1 if(lR.size()<=n0)
#define cD1 addgroup
#define cC1 ,PowiCache&iC,
#define cB1 NewHash.hash1
#define cA1 ;NewHash.hash2+=
#define c91 ;eE i82);
#define c81 found_log2by
#define c71 nF==c03)
#define c61 ParsePowiMuli(
#define c51 iR1)
#define c41 MakeNotP1,l4::
#define c31 MakeNotP0,l4::
#define c21 new_base_immed
#define c11 branch1_backup
#define c01 branch2_backup
#define yZ1 exponent_map
#define yY1 plain_set
#define yX1 LightWeight(
#define yW1 if(value
#define yV1 goto do_return;}lD
#define yU1 .GetParamCount()==
#define yT1 should_regenerate=true;
#define yS1 should_regenerate,
#define yR1 Collection
#define yQ1 RelationshipResult
#define yP1 Subdivide_Combine(
#define yO1 long value
#define yN1 ):e9(),std::vector<
#define yM1 ByteCodeSynth yB&synth)
#define yL1 const std::vector<
#define yK1 )const yS
#define yJ1 rhs yK1 hash1
#define yI1 best_sep_factor
#define yH1 i81!nT3
#define yG1 &&p nV<l62(
#define yF1 needlist_cached
#define yE1 inline unsigned
#define yD1 cW3 bool pad
#define yC1 MakesInteger(
#define yB1 const l62&value
#define yA1 best_sep_cost
#define y91 MultiplicationRange
#define y81 pihalf_limits
#define y71 n_stacked
#define y61 AnyParams_Rec
#define y51 ;tree.SetParam(
#define y41 continue;
#define y31 Become(value l8 0))
#define y21 PositionalParams,0}
#define y11 always_sincostan
#define y01 Recheck_RefCount_Div
#define xZ1 Recheck_RefCount_Mul
#define xY1 i82.
#define xX1 i82;i82 c4
#define xW1 MultiplyAndMakeLong(
#define xV1 cMul);nZ3;tmp
#define xU1 covers_plus1
#define xT1 template set_if<
#define xS1 if(synth.FindAndDup(
#define xR1 SynthesizeParam(
#define xQ1 xP1 a)yP2))
#define xP1 tree l8
#define xO1 ;std::cout<<
#define xN1 grammar_func
#define xM1 252421 x9 24830,
#define xL1 cA 529654 x9
#define xK1 l2 0,2,165888 x9
#define xJ1 1)?(poly^(
#define xI1 cCos l3 2,1,
#define xH1 l1 0x12 nM
#define xG1 Modulo_Radians},
#define xF1 c4 cLog);tree c4 cMul);
#define xE1 GetImmed()
#define xD1 PositionType
#define xC1 CollectionResult
#define xB1 const_offset
#define xA1 inline TriTruthValue
#define x91 stacktop_desired
#define x81 int mStackPtr=0;
#define x71 SetStackTop(
#define x61 }inline
#define x51 FPoptimizer_ByteCode
#define x41 );n41 0,x32;DelParam(1);
#define x31 GetParamCount();
#define x21 xI leaf2 l8
#define x11 cond_type
#define x01 fphash_value_t
#define nZ1 Recheck_RefCount_RDiv
#define nY1 fpEstimatePrecision(
#define nX1 SwapLastTwoInStack();
#define nW1 CollectMulGroup_Item(
#define nV1 pair<l62,xP3>
#define nU1 Rehash()e12 r);}
#define nT1 nN x71 xR-1);
#define nS1 covers_full_cycle
#define nR1 (GetLogicalValue(
#define nQ1 AssembleSequence(
#define nP1 252180 x9 281854,
#define nO1 {DataP slot_holder(y3[
#define nN1 <<std::dec<<")";}
#define nM1 :yD3 e32
#define nL1 !=xK)if(TestCase(
#define nK1 &&IsLogicalValue(
#define nJ1 else*this=model;}
#define nI1 std::pair<T1,T2>&
#define nH1 t91 typename
#define nG1 has_good_balance_found
#define nF1 n_occurrences
#define nE1 found_log2_on_exponent
#define nD1 covers_minus1
#define nC1 needs_resynth
#define nB1 immed_product
#define nA1 y73 bitmask&
#define n91 Sign_Positive
#define n81 {l4::MakeNotNotP1,l4::
#define n71 {l4::MakeNotNotP0,l4::
#define n61 ::MakeTrue
#define n51 matched_params
#define n41 SetParamMove(
#define n31 CodeTreeImmed(l62(
#define n21 Suboptimal
#define n11 changed_if
#define n01 n_as_tanh_param
#define lZ1 opposite=
#define lY1 x01(
#define lX1 eE1.size()
#define lW1 MatchResultType
#define lV1 needs_sincos
#define lU1 resulting_exponent
#define lT1 ;p1 eM1 tree y7 p1);
#define lS1 Unknown:y13;}
#define lR1 nD2 a=0;a<c2;++a)
#define lQ1 )[a].start_at
#define lP1 GetParam(a)
#define lO1 inverse_nominator]
#define lN1 cSin l3 2,1,
#define lM1 (xP1 yO2&&
#define lL1 ,typename xG::
#define lK1 IsImmed()){l62
#define lJ1 AddFunctionOpcode(
#define lI1 void FunctionParserBase
#define lH1 SetParams(iP1));
#define lG1 o<<"("<<std::hex<<data.
#define lF1 IfBalanceGood(
#define lE1 n_as_tan_param
#define lD1 changed_exponent
#define lC1 inverse_denominator
#define lB1 unsigned index
#define lA1 7168 x9 401798
#define l91 yB(rule.repl_param_list,
#define l81 retry_positionalparams_2
#define l71 situation_flags&
#define l61 518 x9 400412,
#define l51 data.subfunc_opcode
#define l41 CopyOnWrite();
#define l31 PlanNtimesCache(
#define l21 FPoptimizer_Grammar
#define l11 static inline xG
#define l01 GetPositivityInfo cK3
#define iZ recursioncount
#define iY ParamSpec_SubFunctionData
#define iX ,cPow xZ
#define iW xO1 std::endl;DumpHashes(
#define iV ,2,1 i62.xT if(found[data.
#define iU AddOperation(cInv,1,1 i62.xT}
#define iT ]);n42
#define iS ~size_t(0)){synth.yV
#define iR PositionalParams_Rec
#define iQ DumpTreeWithIndent(*this);
#define iP switch(type e53 cond_or:
#define iO CalculateResultBoundaries(
#define iN t91 unsigned Compare>
#define iM yF3 0x0 tL3
#define iL edited_powgroup
#define iK has_unknown_max
#define iJ has_unknown_min
#define iI static const range yB
#define iH if(keep_powi
#define iG synthed_tree
#define iF SelectedParams,0},0,0x0},{{
#define iE by_exponent
#define iD collections
#define iC cache
#define iB )e12 p2);tree c4 iM2 nF);cN}
#define iA goto ReplaceTreeWithOne;case
#define i9 xB3,std::ostream&o
#define i8 y7 comp.yY1[a].value);
#define i7 !=xK)return l92
#define i6 cK1.data
#define i5 iK2 x42(
#define i4 needs_sinhcosh
#define i3 t72 l62(
#define i2 MakeFalse,l4::
#define i1 ].relationship
#define i0 ,eE1,IP,limit,y4,stack);
#define tZ 408964 x9 24963,
#define tY 528504 x9 24713,
#define tX AnyParams,0}}iH2
#define tW [n0 e03=true;lR[n0].second
#define tV l21::Grammar*
#define tU powgroup l8
#define tT }},{ProduceNewTree,2,1,
#define tS ,l7 2,1,
#define tR ~size_t(0)&&found[data.
#define tQ xP1 0)
#define tP xC AnyParams,
#define tO iO xP1
#define tN =tO 0));range yB
#define tM t32.GetImmed(
#define tL (tM))
#define tK n31(
#define tJ has_mulgroups_remaining
#define tI Rehash();tY2 yS2
#define tH int_exponent_t
#define tG best_factor
#define tF RootPowerTable yB::RootPowers[
#define tE :goto ReplaceTreeWithZero;case
#define tD MatchPositionSpec_AnyParams yB
#define tC iR2 FPoptimizer_CodeTree
#define tB n_as_sinh_param
#define tA n_as_cosh_param
#define t9 is_signed
#define t8 nS l62(-n23
#define t7 tree.GetParamCount()
#define t6 .max.known
#define t5 eM1 tree c4 i72 nF);tree.
#define t4 iP1));xY1 Rehash();
#define t3 result_positivity
#define t2 biggest_minimum
#define t1 124024 x9 139399,
#define t0 142456 x9 141449,
#define eZ valueType
#define eY .min.known
#define eX x72 a=0 tH2 if(remaining[a])
#define eW ,cIf,l0 3,
#define eV lB 0x4},{{
#define eU cond_tree
#define eT else_tree
#define eS then_tree
#define eR sequencing
#define eQ string eK3(
#define eP const iY
#define eO ;return
#define eN if_stack
#define eM .max.set(fp_ceil tR1
#define eL n_as_sin_param
#define eK n_as_cos_param
#define eJ PowiResolver::
#define eI cIf l3 0,1,
#define eH .BalanceGood
#define eG {if(needs_cow){l41 goto
#define eF );bool needs_cow=GetRefCount()>1;
#define eE AddParamMove(
#define eD back().endif_location
#define eC x01 key
#define eB relationships
#define eA 130,1,
#define e9 MatchPositionSpecBase
#define e8 iK2 CodeTree(
#define e7 smallest_maximum
#define e6 }PACKED_GRAMMAR_ATTRIBUTE;
#define e5 ReplaceTreeWithParam0;
#define e4 factor_needs_rehashing
#define e3 MatchPositionSpecBaseP
#define e2 typename t31::xR3
#define e1 fp_cosh cI2);m nV=fp_cosh(m nV);
#define e0 {AdoptChildrenWithSameOpcode(tree);
#define cZ cR1 yB(nT.param_list,
#define cY );eE1 yS2 0x80000000u
#define cX for(lG3 r=range.first;r!=range.second;++r){
#define cW 243,244,245,246,249,250,251,253,255,256,257,258,259}};}
#define cV ];};extern"C"{
#define cU i9=std::cout
#define cT 79,122,123,160,161,163,164,165,166,167,168,169,178,179,180,200,204,212,216,224,236,237,239,240,
#define cS 27,28,29,30,31,32,33,35,36,
#define cR const ParamSpec_SubFunction
#define cQ const ParamSpec_ParamHolder
#define cP }if(list y93 xE1==l62(
#define cO otherhalf
#define cN goto redo;
#define cM .AddParam(
#define cL StackState
#define cK min iN2 p0 e32>=l62(0.0))
#define cJ max.known e23
#define cI l62(1.5)*fp_const_pi yB()
#define cH CalculatePowiFactorCost(
#define cG ImmedHashGenerator
#define cF ::map<fphash_t,std::set<std yE3> >
#define cE T1,typename T2>inline bool eD1()(
#define cD has_nonlogical_values
#define cC from_logical_context)
#define cB AnyParams,0}},{ProduceNewTree,
#define cA ,l2 18,2,
#define c9 ,l2 16,2,
#define c8 max iN2 p0 nV<=fp_const_negativezero yB())
#define c7 x72 a=t7;a-->0;)
#define c6 x72 a=0 tH2{
#define c5 )return false;
#define c4 .lM2
#define c3 n72 y2.x31 a-->0;)
#define c2 nT y52
#define c1 POWI_CACHE_SIZE
#define c0 ++IP;y41}if(lK3 yS3.
#define yZ },{l4::xK,l4::Never},{l4::xK,l4::Never}}
#define yY .FoundChild
#define yX BalanceResultType
#define yW n42 GetOpcode(),
#define yV DoDup(found[data.
#define yU nW3(0),Opcode(
#define yT );void lJ1 unsigned cW3 yZ2<
#define yS {return
#define yR const yS data->
#define yQ +=fp_const_twopi yB();
#define yP n72 0;a<x31++a cI3
#define yO MatchPositionSpec_AnyWhere
#define yN l93 data.match_type==
#define yM void OutFloatHex(std::ostream&o,
#define yL paramholder_matches
#define yK {static void lH3 nE fphash_t&NewHash,
#define yJ AddParam(CodeTreeImmed(
#define yI m.min.
#define yH fp_sin(min),fp_sin(max))
#define yG fp_const_twopi yB());if(
#define yF ;n11 eM1 tree c4 op1);tree.
#define yE n72 0;a<xO3.x31++a)if(
#define yD template lY
#define yC ComparisonSetBase::
#define yB <l62>
#define yA MatchPositionSpec_PositionalParams yB
#define y9 AssembleSequence_Subdivide(
#define y8 ]=0x80000000u|unsigned(
#define y7 .eE
#define y6 branch2
#define y5 unsigned c;unsigned short l[
#define y4 factor_stack_base
#define y3 data->Params
#define y2 branch1
#define y1 MatchInfo yB&
#define y0 const SequenceOpCode yB
#define xZ ,lB 0x4 nM
#define xY yR2 tree)xO1"\n";
#define xX ;sim.Eat(2,
#define xW tQ yP2
#define xV ,l62(-1)))eG
#define xU AddConst(
#define xT StackTopIs(*this)eO;}
#define xS {lQ2.erase(i);y41}
#define xR StackTop
#define xQ FPOPT_autoptr
#define xP +=nT3 eO nT3;}yD inline l62
#define xO int_exponent
#define xN newnode
#define xM eA2 0x0},{{
#define xL has_highlevel_opcodes
#define xK Unchanged
#define xJ best_selected_sep
#define xI .IsIdenticalTo(
#define xH GetStackTop()-
#define xG CodeTree yB
#define xF cAnd,tX
#define xE ->Recalculate_Hash_NoRecursion();}
#define xD x31++a)if(ApplyGrammar(tD2,tJ3
#define xC ,cAdd,
#define xB position
#define xA )){tree.FixIncompleteHashes();}
#define x9 ,{2,
#define x8 for c6 range yB
#define x7 std::vector<CodeTree>
#define x6 SetParam(0,iM2 xN2 xG p1;p1 c4
#define x5 TestImmedConstraints y83 constraints,tree)c5
#define x4 y53 0 nY3 cInv);tmp c43 eO
#define x3 nX1 sim.Eat(
#define x2 {nA2 cInv);yY3 sim.xU-1)xX yV3
#define x1 paramholder_index
#define x0 return true;case
#define nZ occurance_counts
#define nY >p=tO a));if(p.
#define nX -->0;){iT1 powgroup=lP1;if(powgroup
#define nW ,l0 1,
#define nV .max.val
#define nU const FPoptimizer_CodeTree::xG&tree
#define nT model_tree
#define nS return range yB(
#define nR )){tree.ReplaceWithImmed(
#define nQ ){xG
#define nP ),rangehalf yB model=rangehalf yB()cI3 known
#define nO x42 yB::x42(
#define nN ){using iR2 FUNCTIONPARSERTYPES;
#define nM },{{2,
#define nL AnyParams,1},0,0x0},{{
#define nK nX2&lA2
#define nJ ConstantFolding_LogicCommon(tree,yC
#define nI nH1 Ref>inline void xQ<Ref>::
#define nH cOr,tX 16,1,
#define nG ):data(new x42 yB(
#define nF tC3()
#define nE FUNCTIONPARSERTYPES::
#define nD )l32}
#define nC b;}};t91>cP2 Comp<nE
#define nB t21 cPow);nZ3;tmp.yJ l62(
#define nA xG tmp,tmp2;tmp2 c4
#define n9 iR1(),Params(),Hash(),Depth(1),tJ1 0){}
#define n8 SynthesizeByteCode(synth);
#define n7 AddParam(xP1
#define n6 while(ApplyGrammar(c92
#define n5 GetIntegerInfo(tQ)==IsAlways)goto e5
#define n4 e12 n11)cT2
#define n3 lS);if(fp_nequal(tmp,tD1)nR l62(1)/tmp l32}}lD
#define n2 xT1 cGreater>(l62(
#define n1 DumpParams yB y83 data.param_list,yA3 data y52,o);
#define n0 restholder_index
#define lZ :if(ParamComparer yB()(Params[1],Params[0])){std::swap(Params[0],Params[1]);Opcode=
#define lY <typename l62>
#define lX xG nU3 t11 c4 cMul)t11 cM
#define lW tL1,0x0},
#define lV eE pow l8 1));pow.DelParam(1);pow eM1 tree.n41 0,pow);goto NowWeAreMulGroup;}
#define lU GroupFunction,0},lW{{
#define lT ,l62(1)/l62(
#define lS tQ.xE1
#define lR restholder_matches
#define lQ cB1|=key;x01 crc=(key>>10)|(key<<(64-10))cA1((~lY1 crc))*3)^1234567;}};
#define lP n11;n11 n82 n11 y7 tQ);n11 cM y2 l8
#define lO yD xG::CodeTree(
#define lN tree.SetParam(0,tQ l8 0))y51 1,CodeTreeImmed(
#define lM lY void ByteCodeSynth yB::lJ1 unsigned cW3 yZ2<
#define lL cMul,lU 2,
#define lK cMul,AnyParams,
#define lJ (xW)&&t32 yP2 nR
#define lI iO tmp)cT1
#define lH :cB3=comp.AddRelationship(atree l8 0),atree l8 1),yC
#define lG cPow,l0 2
#define lF typename l62>inline bool eD1()cX1 nX3 l62&b)yS a
#define lE {range yB m=tO 0));
#define lD break;case
#define lC x73 xG::
#define lB y21,0,
#define lA l1 0x0 nM
#define l9 ?0:1));xG n11;n11 n82 n11 iY1 tree.iP1));n11 eM1 tree c4
#define l8 .GetParam(
#define l7 cAdd,tX
#define l6 SelectedParams,0},0,0x0 nM
#define l5 lK 0}}iH2
#define l4 RangeComparisonData
#define l3 ,y21},{ProduceNewTree,
#define l2 y21}iH2
#define l1 cMul,SelectedParams,0},0,
#define l0 lB 0x0},{{
#ifdef _MSC_VER
typedef
unsigned
int
iO1;
#else
#include <stdint.h>
typedef
uint_least32_t
iO1;
#endif
iR2
crc32{enum{startvalue=0xFFFFFFFFUL,poly=0xEDB88320UL}
;t91
iO1
crc>cP2
b8{enum{b1=(crc&xJ1
crc
x52
crc>>1),b2=(b1&xJ1
b1
x52
b1>>1),b3=(b2&xJ1
b2
x52
b2>>1),b4=(b3&xJ1
b3
x52
b3>>1),b5=(b4&xJ1
b4
x52
b4>>1),b6=(b5&xJ1
b5
x52
b5>>1),b7=(b6&xJ1
b6
x52
b6>>1),res=(b7&xJ1
b7
x52
b7>>1)}
;}
;inline
iO1
update(iO1
crc,unsigned
b){
#define B4(n) b8<n>iQ2 n+1>iQ2 n+2>iQ2 n+3>::res
#define R(n) B4(n),B4(n+4),B4(n+8),B4(n+12)
static
const
iO1
table[256]={R(0x00),R(0x10),R(0x20),R(0x30),R(0x40),R(0x50),R(0x60),R(0x70),R(0x80),R(0x90),R(0xA0),R(0xB0),R(0xC0),R(0xD0),R(0xE0),R(0xF0)}
;
#undef R
#undef B4
return((crc>>8))^table[(crc^b)&0xFF];x61
iO1
calc_upd(iO1
c,const
unsigned
char*buf,size_t
size){iO1
value=c;for
x72
p=0;p<size;++p)value=update(value,buf[p])eO
value;x61
iO1
calc
cY1
unsigned
char*buf,size_t
size)yS
calc_upd(startvalue,buf,size);}
}
#ifndef FPOptimizerAutoPtrHH
#define FPOptimizerAutoPtrHH
nH1
Ref>class
xQ{cV3
xQ():p(0){}
xQ(Ref*b):p(b){nV3}
xQ
cY1
xQ&b):p(b.p){nV3
x61
Ref&eD1*(yK1*p;x61
Ref*eD1->(yK1
p;}
bool
isnull(yK1!p;}
Ref*get(yK1
p;}
xQ&eD1=(Ref*b){Set(b)eO*this;}
xQ&eD1=cY1
xQ&b){Set(b.p)eO*this;}
#ifdef __GXX_EXPERIMENTAL_CXX0X__
xQ(xQ&&b):p(b.p){b.p=0;}
xQ&eD1=(xQ&&b
cI3
p!=b.p){cO2;p=b.p;b.p=0
yC3*this;}
#endif
~xQ(){cO2
n63
UnsafeSetP(Ref*newp){p=newp
n63
swap(xQ<Ref>&b){Ref*tmp=p;p=b.p;b.p=tmp;}
private:inline
static
void
Have(Ref*p2);inline
void
cO2;inline
void
nV3
inline
void
Set(Ref*p2);private:Ref*p;}
;nI
cO2{if(!p)return;p->nW3-=1;if(!p->nW3)delete
p;}
nI
Have(Ref*p2
cI3
p2)++(p2->nW3);}
nI
Birth(){Have(p);}
nI
Set(Ref*p2){Have(p2);cO2;p=p2;}
#endif
#include <utility>
cP2
Compare2ndRev{nH1
T>inline
bool
eD1()cY1
T&nX3
T&b
yK1
a.second>b.second;}
}
;cP2
Compare1st{nH1
cE
const
nI1
nX3
nI1
b
yK1
a.first<b.first;}
nH1
cE
const
nI1
a,T1
b
yK1
a.first<b;}
nH1
cE
T1
nX3
nI1
b
yK1
a<b.first;}
}
;
#ifndef FPoptimizerHashHH
#define FPoptimizerHashHH
#ifdef _MSC_VER
typedef
unsigned
long
long
x01;
#define FPHASH_CONST(x) x##ULL
#else
#include <stdint.h>
typedef
uint_fast64_t
x01;
#define FPHASH_CONST(x) x##ULL
#endif
iR2
FUNCTIONPARSERTYPES{cP2
fphash_t{x01
hash1,hash2;fphash_t():hash1(0),hash2(0){}
fphash_t
cY1
x01&nX3
x01&b):hash1(a),hash2(b){}
bool
eD1==cY1
fphash_t&yJ1==cQ2&&hash2==cR2
bool
eD1!=cY1
fphash_t&yJ1!=cQ2||hash2!=cR2
bool
eD1<cY1
fphash_t&yJ1!=cQ2?hash1<cQ2:hash2<cR2}
;}
#endif
#ifndef FPOptimizer_CodeTreeHH
#define FPOptimizer_CodeTreeHH
#ifdef FP_SUPPORT_OPTIMIZER
#include <vector>
#include <utility>
iR2
l21{cP2
Grammar;}
iR2
x51{x13
ByteCodeSynth;}
tC{x13
CodeTree;yD
cP2
x42;x13
CodeTree{typedef
xQ<x42
yB>DataP;DataP
data;cV3
CodeTree();~CodeTree();cP2
OpcodeTag{}
;e8
nW2
o,OpcodeTag);cP2
FuncOpcodeTag{}
;e8
nW2
o,unsigned
f,FuncOpcodeTag);cP2
x23{}
;e8
const
l62&v,x23);
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
e8
l62&&v,x23);
#endif
cP2
VarTag{}
;e8
unsigned
varno,VarTag);cP2
CloneTag{}
;e8
iS2
b,CloneTag);void
GenerateFrom
cY1
typename
FunctionParserBase
yB::Data&data,bool
keep_powi=false);void
GenerateFrom
cY1
typename
FunctionParserBase
yB::Data&data,const
x7&x12,bool
keep_powi=false);void
SynthesizeByteCode(std::vector<unsigned>&c83,std::vector
yB&immed,size_t&stacktop_max);void
SynthesizeByteCode(x51
yZ3&synth,bool
MustPopTemps=true)const;size_t
SynthCommonSubExpressions(x51::yM1
const;void
SetParams
cY1
x7&x33
SetParamsMove(x7&t81
CodeTree
GetUniqueRef();
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
void
SetParams(x7&&t81
#endif
void
SetParam
x72
which,iS2
b);void
n41
size_t
which,nV2
b);void
AddParam
cY1
nV2
param);void
eE
nV2
param);void
AddParams
cY1
x7&x33
AddParamsMove(x7&x33
AddParamsMove(x7&lA2,size_t
lB2);void
DelParam
x72
index);void
DelParams();void
Become
cY1
nV2
b);inline
size_t
GetParamCount(yK1
iP1).size();x61
nV2
GetParam
x72
n)yS
iP1)[n];x61
iS2
GetParam
x72
n
yK1
iP1)[n];x61
void
lM2
nW2
o)tI3
Opcode=o;x61
nW2
GetOpcode()yR
Opcode;x61
nE
fphash_t
GetHash()yR
Hash;x61
const
x7&iP1
yK1
y3;x61
x7&iP1)yS
y3;x61
size_t
xT2
yR
Depth;x61
const
l62&xE1
yR
Value;x61
unsigned
GetVar()yR
iQ1
x61
unsigned
GetFuncNo()yR
iQ1
x61
bool
IsDefined(yK1
GetOpcode()!=nE
cNop;x61
bool
IsImmed(yK1
GetOpcode()==nE
cImmed;x61
bool
IsVar(yK1
GetOpcode()==nE
iE2;x61
unsigned
GetRefCount()yR
nW3
n63
ReplaceWithImmed
cX1
i);void
Rehash(bool
constantfolding=true);void
Sort();inline
void
Mark_Incompletely_Hashed()tI3
Depth=0;x61
bool
Is_Incompletely_Hashed()yR
Depth==0;x61
const
tV
GetOptimizedUsing()yR
iS1;x61
void
SetOptimizedUsing
cY1
tV
g)tI3
iS1=g;}
bool
RecreateInversionsAndNegations(bool
prefer_base2=false);void
FixIncompleteHashes();void
swap(nV2
b){data.swap(b.data);}
bool
IsIdenticalTo
cY1
nV2
b)const;void
l41}
;yD
cP2
x42{int
nW3;nW2
Opcode;l62
Value;unsigned
iQ1
nX2
Params;nE
fphash_t
Hash;size_t
Depth;const
tV
iS1;x42();x42
cY1
x42&b);i5
nW2
o);i5
nW2
o,unsigned
f);i5
const
l62&i);
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
i5
l62&&i);x42(x42&&b);
#endif
bool
IsIdenticalTo
cY1
x42&b)const;void
Sort();void
Recalculate_Hash_NoRecursion();private:void
eD1=cY1
x42&b);}
;yD
l11
CodeTreeImmed
cX1
i)yS
xG(i
lL1
x23());}
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
yD
l11
CodeTreeImmed(l62&&i)yS
xG
cX2
i)lL1
x23());}
#endif
yD
l11
CodeTreeOp(nW2
opcode)yS
xG(opcode
lL1
OpcodeTag());}
yD
l11
CodeTreeFuncOp(nW2
cW3
unsigned
f)yS
xG(cW3
f
lL1
FuncOpcodeTag());}
yD
l11
CodeTreeVar
nD2
varno)yS
xG(varno
lL1
VarTag());}
#ifdef FUNCTIONPARSER_SUPPORT_DEBUGGING
x73
DumpHashes(cU);x73
DumpTree(cU);x73
DumpTreeWithIndent(cU,const
std
yE3&indent="\\"
);
#endif
}
#endif
#endif
#ifndef FPOptimizer_GrammarHH
#define FPOptimizer_GrammarHH
#include <iostream>
tC{x13
CodeTree;}
iR2
l21{enum
ImmedConstraint_Value{ValueMask=0x07,Value_AnyNum=0x0,nK2=0x1,Value_OddInt=0x2,tM1=0x3,Value_NonInteger=0x4,eQ1=0x5
nO3
ImmedConstraint_Sign{SignMask=0x18,Sign_AnySign=0x00,n91=0x08,eR1=0x10,Sign_NoIdea=0x18
nO3
ImmedConstraint_Oneness{OnenessMask=0x60,Oneness_Any=0x00,Oneness_One=0x20,Oneness_NotOne=0x40
nO3
ImmedConstraint_Constness{ConstnessMask=0x180,Constness_Any=0x00,tL1=0x80,Constness_NotConst=0x100
nO3
Modulo_Mode{Modulo_None=0,Modulo_Radians=1
nO3
Situation_Flags{LogicalContextOnly=0x01,NotForIntegers=0x02,OnlyForIntegers=0x04,OnlyForComplex=0x08,NotForComplex=0x10
nO3
nB2{NumConstant,ParamHolder,SubFunction
nO3
ParamMatchingType{PositionalParams,SelectedParams,AnyParams,GroupFunction
nO3
RuleType{ProduceNewTree,ReplaceParams}
;
#ifdef __GNUC__
# define PACKED_GRAMMAR_ATTRIBUTE __attribute__((packed))
#else
# define PACKED_GRAMMAR_ATTRIBUTE
#endif
typedef
std::pair<nB2,const
void*>cS2;yD
cS2
cR1
nD2
paramlist,lB1);yD
bool
ParamSpec_Compare
cY1
void*nX3
void*b,nB2
type);unsigned
ParamSpec_GetDepCode
cY1
cS2&b);cP2
ParamSpec_ParamHolder{lB1:8;unsigned
constraints:9;unsigned
depcode:15;e6
yD
cP2
ParamSpec_NumConstant{l62
constvalue;unsigned
modulo;}
;cP2
iY{unsigned
param_count:2;unsigned
param_list:30;nW2
subfunc_opcode:8;ParamMatchingType
match_type:3;unsigned
n0:5;e6
cP2
ParamSpec_SubFunction{iY
data;unsigned
constraints:9;unsigned
depcode:7;e6
cP2
Rule{RuleType
ruletype:2;unsigned
situation_flags:5;unsigned
repl_param_count:2+9;unsigned
repl_param_list:30;iY
match_tree;e6
cP2
Grammar{unsigned
rule_count;unsigned
short
rule_list[999
cV
extern
const
Rule
grammar_rules[];}
x73
DumpParam
cY1
cS2&p,std::ostream&o=std::cout);x73
DumpParams
nD2
paramlist,unsigned
count,std::ostream&o=std::cout);}
#endif
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
#define CONSTANT_POS_INF HUGE_VAL
#define CONSTANT_NEG_INF (-HUGE_VAL)
iR2
FUNCTIONPARSERTYPES{yD
inline
l62
fp_const_pihalf()yS
fp_const_pi
yB()*l62(0.5);}
yD
inline
l62
fp_const_twopi(){l62
nT3(fp_const_pi
yB());nT3
xP
fp_const_twoe(){l62
nT3(fp_const_e
yB());nT3
xP
fp_const_twoeinv(){l62
nT3(fp_const_einv
yB());nT3
xP
fp_const_negativezero()yS-Epsilon
yB::value;}
}
#ifdef FP_SUPPORT_OPTIMIZER
#include <vector>
#include <utility>
#include <iostream>
cV1{using
iR2
l21;using
tC;using
iR2
FUNCTIONPARSERTYPES;x13
MatchInfo{cV3
std::vector<std::pair<bool,nX2> >lR;nX2
yL
e72
unsigned>n51;cV3
MatchInfo():lR(),yL(),n51(){}
cV3
bool
SaveOrTestRestHolder
nD2
n0,x43
tV1){cE1{lR
x53
n0+1);lR
tW=tV1
cT2
if(lR[n0
e03==false){lR
tW=tV1
cT2
x43
found=lR[n0].second;if(tV1.size()!=found.size()c5
n72
0;a<tV1.size();++a)if(!tV1[a]xI
found[a])c5
return
true
n63
SaveRestHolder
nD2
n0,nX2&tV1){cE1
lR
x53
n0+1);lR
tW.swap(tV1);}
bool
SaveOrTestParamHolder
nD2
x1,iT1
x63
cI3
yL.size()<=x1){yL.xD3
x1+1);yL
x53
x1);yL
yS2
x63)cT2
if(!yL[x1].n12){yL[x1]=x63
cT2
return
x63
xI
yL[x1]n33
SaveMatchedParamIndex(lB1){n51
yS2
index);}
iT1
GetParamHolderValueIfFound
nD2
x1)const{static
const
xG
dummytree;if(yL.size()<=x1)return
dummytree
eO
yL[x1];}
iT1
GetParamHolderValue
nD2
x1
yK1
yL[x1];}
bool
HasRestHolder
nD2
n0
yK1
lR.size()>n0&&lR[n0
e03==true;}
x43
GetRestHolderValues
nD2
n0)const{static
yL1
xG>empty_result;cE1
return
empty_result
eO
lR[n0].second;}
yL1
unsigned>&GetMatchedParamIndexes(yK1
n51
n63
swap(y1
b){lR.swap(b.lR);yL.swap(b.yL);n51.swap(b.n51);}
y1
eD1=cY1
y1
b){lR=b.lR;yL=b.yL;n51=b.n51
eO*this;}
}
;class
e9
tO3
xQ<e9>e3;class
e9{cV3
int
nW3;cV3
e9():nW3(0){}
virtual~e9(){}
}
;cP2
lW1{bool
found;e3
specs;lW1(bool
f):found(f),specs(){}
lW1(bool
f
n43
s):found(f),specs(s){}
}
;x73
SynthesizeRule
cY1
eU2
xG&tree,y1
info);yD
lW1
TestParam
cY1
cS2&x02,xB3
n43
start_at,y1
info);yD
lW1
TestParams(eP&nT,xB3
n43
start_at,y1
info,bool
y82;yD
bool
ApplyGrammar
cY1
Grammar&tD2,FPoptimizer_CodeTree::xG&tree,bool
from_logical_context=false);x73
ApplyGrammars(FPoptimizer_CodeTree::cV2;yD
bool
IsLogisticallyPlausibleParamsMatch(eP&e13,xB3);}
iR2
l21{x73
DumpMatch
cY1
eU2
nU,const
FPoptimizer_Optimize::y1
info,bool
DidMatch,std::ostream&o=std::cout);x73
DumpMatch
cY1
eU2
nU,const
FPoptimizer_Optimize::y1
info,nA3
eH3,std::ostream&o=std::cout);}
#endif
#include <string>
nB3
l21::nB2
yD1=false);nB3
nW2
yD1=false);
#include <string>
#include <sstream>
#include <assert.h>
#include <iostream>
using
iR2
l21;using
iR2
FUNCTIONPARSERTYPES;nB3
l21::nB2
yD1){
#if 1
nA3
p=0;switch(opcode
e53
NumConstant:p="NumConstant"
;lD
ParamHolder:p="ParamHolder"
;lD
SubFunction:p="SubFunction"
;yY3
std::ostringstream
tmp;assert(p);tmp<<p;if(pad)while(tmp.str().size()<12)tmp<<' '
eO
tmp.str();
#else
std::ostringstream
tmp;tmp<<opcode;if(pad)while(tmp.str().size()<5)tmp<<' '
eO
tmp.str();
#endif
}
nB3
nW2
yD1){
#if 1
nA3
p=0;switch(opcode
e53
cAbs:p="cAbs"
;lD
cAcos:p="cAcos"
;lD
cAcosh:p="cAcosh"
;lD
cArg:p="cArg"
;lD
cAsin:p="cAsin"
;lD
cAsinh:p="cAsinh"
;lD
cAtan:p="cAtan"
;lD
cAtan2:p="cAtan2"
;lD
cAtanh:p="cAtanh"
;lD
cCbrt:p="cCbrt"
;lD
cCeil:p="cCeil"
;lD
cConj:p="cConj"
;lD
cCos:p="cCos"
;lD
cCosh:p="cCosh"
;lD
cCot:p="cCot"
;lD
cCsc:p="cCsc"
;lD
cExp:p="cExp"
;lD
cExp2:p="cExp2"
;lD
cFloor:p="cFloor"
;lD
cHypot:p="cHypot"
;lD
cIf:p="cIf"
;lD
cImag:p="cImag"
;lD
cInt:p="cInt"
;lD
cLog:p="cLog"
;lD
cLog2:p="cLog2"
;lD
cLog10:p="cLog10"
;lD
cMax:p="cMax"
;lD
cMin:p="cMin"
;lD
cPolar:p="cPolar"
;lD
cPow:p="cPow"
;lD
cReal:p="cReal"
;lD
cSec:p="cSec"
;lD
cSin:p="cSin"
;lD
cSinh:p="cSinh"
;lD
cSqrt:p="cSqrt"
;lD
cTan:p="cTan"
;lD
cTanh:p="cTanh"
;lD
cTrunc:p="cTrunc"
;lD
cImmed:p="cImmed"
;lD
cJump:p="cJump"
;lD
cNeg:p="cNeg"
;lD
cAdd:p="cAdd"
;lD
cSub:p="cSub"
;lD
cMul:p="cMul"
;lD
cDiv:p="cDiv"
;lD
cMod:p="cMod"
;lD
cEqual:p="cEqual"
;lD
tP1:p="cNEqual"
;lD
cLess:p="cLess"
;lD
cLessOrEq:p="cLessOrEq"
;lD
cGreater:p="cGreater"
;lD
cGreaterOrEq:p="cGreaterOrEq"
;lD
cNot:p="cNot"
;lD
cAnd:p="cAnd"
;lD
cOr:p="cOr"
;lD
cDeg:p="cDeg"
;lD
cRad:p="cRad"
;lD
cFCall:p="cFCall"
;lD
cPCall:p="cPCall"
;break;
#ifdef FP_SUPPORT_OPTIMIZER
case
cFetch:p="cFetch"
;lD
cPopNMov:p="cPopNMov"
;lD
cLog2by:p="cLog2by"
;lD
cNop:p="cNop"
;break;
#endif
case
cSinCos:p="cSinCos"
;lD
cSinhCosh:p="cSinhCosh"
;lD
c03:p="cAbsNot"
;lD
cAbsNotNot:p="cAbsNotNot"
;lD
cAbsAnd:p="cAbsAnd"
;lD
cAbsOr:p="cAbsOr"
;lD
cAbsIf:p="cAbsIf"
;lD
cDup:p="cDup"
;lD
cInv:p="cInv"
;lD
cSqr:p="cSqr"
;lD
cRDiv:p="cRDiv"
;lD
cRSub:p="cRSub"
;lD
cNotNot:p="cNotNot"
;lD
cRSqrt:p="cRSqrt"
;lD
iE2:p="VarBegin"
;yY3
std::ostringstream
tmp;assert(p);tmp<<p;if(pad)while(tmp.str().size()<12)tmp<<' '
eO
tmp.str();
#else
std::ostringstream
tmp;tmp<<opcode;if(pad)while(tmp.str().size()<5)tmp<<' '
eO
tmp.str();
#endif
}
#ifdef FP_SUPPORT_OPTIMIZER
#include <vector>
#include <utility>
#ifndef FP_GENERATING_POWI_TABLE
enum{MAX_POWI_BYTECODE_LENGTH=20}
;
#else
enum{MAX_POWI_BYTECODE_LENGTH=999}
;
#endif
enum{MAX_MULI_BYTECODE_LENGTH=3}
;iR2
x51{x13
ByteCodeSynth{cV3
ByteCodeSynth():eE1(),Immed(),cL(),xR(0),StackMax(0){eE1.xD3
64);Immed.xD3
8);cL.xD3
16
n33
Pull(std::vector<unsigned>&bc,std::vector
yB&imm,size_t&StackTop_max){for
nD2
a=0;a<lX1;++a){eE1[a]&=~0x80000000u;}
eE1.swap(bc);Immed.swap(imm);StackTop_max=StackMax;}
size_t
GetByteCodeSize(yK1
lX1;}
size_t
GetStackTop(yK1
xR
n63
PushVar
nD2
varno){eE1
yS2
varno);eV2}
void
PushImmed(l62
immed
nN
eE1
yS2
cImmed);Immed
yS2
immed);eV2}
void
StackTopIs(nU,int
offset=0
cI3(int)xR>offset){cL[xR
lT2
first=true;cL[xR
lT2
second=tree;}
}
bool
IsStackTop(nU,int
offset=0
yK1(int)xR>offset&&cL[xR
lT2
first&&cL[xR
lT2
second
xI
tree);x61
void
EatNParams
nD2
eat_count){xR-=eat_count
n63
ProducedNParams
nD2
produce_count){x71
xR+produce_count
n33
DoPopNMov
x72
cU2,size_t
srcpos
nN
eE1
yS2
cPopNMov
nC2
cU2
nC2
srcpos);x71
srcpos+1);cL[cU2]=cL[srcpos];x71
cU2+1
n33
DoDup
x72
xE3
nN
if(xE3==xR-1){eE1
yS2
cDup);}
else{eE1
yS2
cFetch
nC2
xE3);}
eV2
cL[xR-1]=cL[xE3];}
#ifdef FUNCTIONPARSER_SUPPORT_DEBUGGING
t91
int>void
Dump(){std::ostream&o=std::cout;o<<"Stack state now("
<<xR<<"):\n"
;n72
0;a<xR;++a){o<<a<<": "
;if(cL[a
e03){nU=cL[a].second;o<<'['<<std::hex<<(void*)(&tree.iP1))<<std::dec<<','<<tree.GetRefCount()<<']'
yR2
tree,o);}
else
o<<"?"
;o<<"\n"
;}
o<<std::flush;}
#endif
size_t
xF3
nU)const{n72
xR;a-->0;)if(cL[a
e03&&cL[a].second
xI
tree
tJ2
a
eO~size_t(0);}
bool
Find(nU
yK1
xF3
tree)!=~size_t(0);}
bool
FindAndDup(nU){size_t
pos=xF3
tree);if(pos!=~size_t(0)){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<tF3"duplicate at ["
<<pos<<"]: "
yR2
tree)xO1" -- issuing cDup or cFetch\n"
;
#endif
DoDup(pos)cT2
return
e83
cP2
IfData{size_t
ofs;}
;void
SynthIfStep1(IfData&nP3,nW2
op
nT1
nP3.ofs=lX1;eE1
yS2
op
cY
cY
n33
SynthIfStep2(IfData&nP3
nT1
eE1
tW1
xG3+2);eE1
tW1
2
y8
iU1
nP3.ofs=lX1;eE1
yS2
cJump
cY
cY
n33
SynthIfStep3(IfData&nP3
nT1
eE1.back()|=0x80000000u;eE1
tW1
xG3-1);eE1
tW1
2
y8
iU1
eV2
n72
0;a<nP3.ofs;++a
cI3
eE1[a]==cJump&&eE1[a+1]==(0x80000000u|(nP3.ofs-1))){eE1[a+xG3-1);eE1[a+2
y8
iU1
yX3(eE1[a]e53
cAbsIf:case
cIf:case
cJump:case
cPopNMov:a+=2;lD
cFCall:case
cPCall:case
cFetch:a+=1;break;y13
yN3}
protected:void
x71
size_t
value){xR=value;if(xR>iT2{StackMax=xR;cL
x53
iT2;}
}
protected:std::vector<unsigned>eE1;std::vector
yB
Immed
e72
std::pair<bool,FPoptimizer_CodeTree::xG> >cL;size_t
xR;size_t
StackMax;private:void
incStackPtr(){if(xR+2>iT2
cL
x53
StackMax=xR+2);}
t91
bool
IsIntType,bool
IsComplexType>cP2
yZ2{}
;cV3
void
AddOperation
nD2
cW3
unsigned
eat_count,unsigned
produce_count=1){EatNParams(eat_count);lJ1
opcode);ProducedNParams(produce_count
n33
lJ1
unsigned
cW3
yZ2<false,false>yT
false,true>yT
true,false>yT
true,true>);inline
void
lJ1
unsigned
opcode){lJ1
cW3
yZ2<bool(nE
IsIntType
yB::nT3),bool(nE
IsComplexType
yB::nT3)>());}
}
;yD
cP2
SequenceOpCode;yD
cP2
tH1{static
y0
AddSequence;static
y0
MulSequence;}
;x73
nQ1
long
count,y0&eR,yM1;}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
using
iR2
FUNCTIONPARSERTYPES;iR2
x51{yD
cP2
SequenceOpCode{l62
basevalue;unsigned
op_flip;unsigned
op_normal,op_normal_flip;unsigned
op_inverse,op_inverse_flip;}
;yD
y0
tH1
yB::AddSequence={tD1),cNeg
xC
cAdd,cSub,cRSub}
;yD
y0
tH1
yB::MulSequence={l62(1),cInv,cMul,cMul,cDiv,cRDiv}
;
#define findName(a,b,c) "var"
#define TryCompilePowi(o) false
#define mData this
#define mByteCode eE1
#define mImmed Immed
lU2
false,false>){x81
# define FP_FLOAT_VERSION 1
# define FP_COMPLEX_VERSION 0
# include "extrasrc/fp_opcode_add.inc"
# undef FP_COMPLEX_VERSION
# undef FP_FLOAT_VERSION
}
lU2
true,false>){x81
# define FP_FLOAT_VERSION 0
# define FP_COMPLEX_VERSION 0
# include "extrasrc/fp_opcode_add.inc"
# undef FP_COMPLEX_VERSION
# undef FP_FLOAT_VERSION
}
#ifdef FP_SUPPORT_COMPLEX_NUMBERS
lU2
false,true>){x81
# define FP_FLOAT_VERSION 1
# define FP_COMPLEX_VERSION 1
# include "extrasrc/fp_opcode_add.inc"
# undef FP_COMPLEX_VERSION
# undef FP_FLOAT_VERSION
}
lU2
true,true>){x81
# define FP_FLOAT_VERSION 0
# define FP_COMPLEX_VERSION 1
# include "extrasrc/fp_opcode_add.inc"
# undef FP_COMPLEX_VERSION
# undef FP_FLOAT_VERSION
}
#endif
#undef findName
#undef mImmed
#undef mByteCode
#undef mData
#undef TryCompilePowi
}
using
iR2
x51;
#define POWI_TABLE_SIZE 256
#define POWI_WINDOW_SIZE 3
iR2
x51{
#ifndef FP_GENERATING_POWI_TABLE
extern
const
unsigned
char
powi_table[POWI_TABLE_SIZE];const
#endif
unsigned
char
powi_table[POWI_TABLE_SIZE]={0,1,1,1,2
i41
1,4,1,2,131,8
i41
1,8,cA3
131,4,1,15,1,16
i41
131,8,1,2,1,4,cA3
1,16,1,25,131,4,1,27,5,8,3,2,1,30,1,31,3,32
i41
1,8,1,2,131,4,1,39,1,16,137,2,1,4,cA3
131,8,1,45,135,4,31,2,5,32,1,2,131,50,1,51,1,8,3,2,1,54,1,55,3,16,1,57,133,4,137,2,135,60,1,61,3,62,133,63,1,tX1
131,tX1
139,iV1
eA
30,1,130,137,2,31,iV1
eA
eA
130,cA3
1,eA
eA
2,1,130,133,tX1
61,130,133,62,139,130,137,eA
iV1
eA
eA
tX1
131,eA
eA
130,131,2,133,iV1
130,141,eA
130,cA3
1,eA
5,135,eA
iV1
eA
iV1
130,133,130,141,130,131,eA
eA
2,131}
;}
static
nK3
c1=256;
#define FPO(x)
iR2{class
PowiCache{private:int
iC[c1];int
tY1[c1];cV3
PowiCache():iC(),tY1(){iC[1]=1;}
bool
Plan_Add(yO1,int
count){yW1>=c1
c5
tY1[eW2+=count
eO
iC[eW2!=0
n63
iU2
yO1){yW1<c1)iC[eW2=1
n63
Start
x72
value1_pos){for(int
n=2;n<c1;++n)iC[n]=-1;Remember(1,value1_pos);DumpContents();}
int
Find(yO1)const{yW1<c1
cI3
iC[eW2>=0){FPO(t03(t43,"* I found %ld from cache (%u,%d)\n",value,(unsigned)cache[value],t13 value]))eO
iC[eW2;}
}
return-1
n63
Remember(yO1,size_t
tD3){yW1>=c1)return;FPO(t03(t43,"* Remembering that %ld can be found at %u (%d uses remain)\n",value,(unsigned)tD3,t13 value]));iC[eW2=(int)tD3
n63
DumpContents()const{FPO(for(int a=1;a<POWI_CACHE_SIZE;++a)if(cache[a]>=0||t13 a]>0){t03(t43,"== cache: sp=%d, val=%d, needs=%d\n",cache[a],a,t13 a]);})}
int
UseGetNeeded(yO1){yW1>=0&&value<c1)return--tY1[eW2
eO
0;}
}
;yD
size_t
y9
long
count
cC1
y0&eR,yM1;x73
yP1
size_t
apos,long
aval,size_t
bpos,long
bval
cC1
unsigned
cumulation_opcode,unsigned
cimulation_opcode_flip,yM1;void
l31
yO1
cC1
int
need_count,int
iZ=0){yW1<1)return;
#ifdef FP_GENERATING_POWI_TABLE
if(iZ>32)throw
false;
#endif
if(iC.Plan_Add(value,need_count
tJ2;long
xH3
1;yW1<POWI_TABLE_SIZE){xH3
powi_table[eW2
n73&128
xA2
if(xI3
xH3-eX2
FPO(t03(t43,"value=%ld, half=%ld, otherhalf=%ld\n",value,half,value/half));l31
half,c02
iC.iU2
half)eO;}
i81
xI3{xH3-eX2}
}
else
yW1&1)xH3
value&((1<<POWI_WINDOW_SIZE)-1);else
xH3
value/2;long
cO=value-half
n73>cO||half<0)std::swap(half,cO);FPO(t03(t43,"value=%ld, half=%ld, otherhalf=%ld\n",value,half,otherhalf))n73==cO){l31
half,iC,2,iZ+1);}
else{l31
half,c02
l31
cO>0?cO:-cO,c02}
iC.iU2
value);}
yD
size_t
y9
yO1
cC1
y0&eR,yM1{int
xJ3=iC.Find(value);if(xJ3>=0)yS
xJ3;}
long
xH3
1;yW1<POWI_TABLE_SIZE){xH3
powi_table[eW2
n73&128
xA2
if(xI3
xH3-eX2
FPO(t03(t43,"* I want %ld, my plan is %ld * %ld\n",value,half,value/half));size_t
xB2=y9
half
e41
if(iC
n62
half)>0||xB2!=e51){tZ1
xB2)xE2
half,e51);}
nQ1
value/half
n32
size_t
tD3=e51
xE2
value,tD3);iC.DumpContents()eO
tD3;}
i81
xI3{xH3-eX2}
}
else
yW1&1)xH3
value&((1<<POWI_WINDOW_SIZE)-1);else
xH3
value/2;long
cO=value-half
n73>cO||half<0)std::swap(half,cO);FPO(t03(t43,"* I want %ld, my plan is %ld + %ld\n",value,half,value-half))n73==cO){size_t
xB2=y9
half
e41
yP1
xB2,half,xB2,half,iC,eR.op_normal,eR.op_normal_flip,synth);}
else{long
part1=half;long
part2=cO>0?cO:-cO;size_t
part1_pos=y9
part1
e41
size_t
part2_pos=y9
part2
e41
FPO(t03(t43,"Subdivide(%ld: %ld, %ld)\n",value,half,otherhalf));yP1
part1_pos,part1,part2_pos,part2,iC,cO>0?eR.op_normal:eR.op_inverse,cO>0?eR.op_normal_flip:eR.op_inverse_flip,synth);}
size_t
tD3=e51
xE2
value,tD3);iC.DumpContents()eO
tD3;}
x73
yP1
size_t
apos,long
aval,size_t
bpos,long
bval
cC1
unsigned
cumulation_opcode,unsigned
cumulation_opcode_flip,yM1{int
a_needed=iC
n62
aval);int
xK3=iC
n62
bval);bool
flipped
e23
#define DUP_BOTH() do{if(apos<bpos){size_t tmp=apos;apos=bpos;bpos=tmp e81 FPO(t03(t43,"-> " tB3 tB3"op\n",(unsigned)apos,(unsigned)bpos));tZ1 apos);tZ1 apos==bpos?e51:bpos);}while(0)
#define DUP_ONE(p) do{FPO(t03(t43,"-> " tB3"op\n",(unsigned)p));tZ1 p);}while(0)
if(a_needed>0
cI3
xK3>0){lV2}
eN3
bpos!=e51)lV2
else{nF2
e81}
}
i81
xK3>0
cI3
apos!=e51)lV2
else
DUP_ONE(bpos);}
eN3
apos==bpos&&apos==e51)nF2;i81
apos==e51&&bpos==synth.xH
2){FPO(t03(t43,"-> op\n"))e81
i81
apos==synth.xH
2&&bpos==e51)FPO(t03(t43,"-> op\n"));i81
apos==e51)DUP_ONE(bpos);i81
bpos==e51){nF2
e81
else
lV2}
n42
flipped?cumulation_opcode_flip:cumulation_opcode,2);}
x73
yX1
long
count,y0&eR,yM1{while(count<256){int
xH3
x51::powi_table[count]n73&128
xA2
yX1
half
n32
count/=half;}
else
yY3
if(count==1)return;if(!(count&1)){n42
cSqr,1);yX1
count/2
n32}
else{tZ1
e51);yX1
count-1
n32
n42
cMul,2);}
}
}
iR2
x51{x73
nQ1
long
count,y0&eR,yM1{if(count==0)eV1
eR.basevalue);else{bool
eY2
e23
if(count<0){eY2=true;count=-count;}
if(false)yX1
count
n32
i81
count>1){PowiCache
iC;l31
count,iC,1);size_t
x91
eJ3
GetStackTop();iC.Start(e51);FPO(t03(t43,"Calculating result for %ld...\n",count));size_t
xC2=y9
count
e41
size_t
n_excess
eJ3
xH
x91;if(n_excess>0||xC2!=x91-1){synth.DoPopNMov(x91-1,xC2);}
}
if(eY2)n42
eR.op_flip,1);}
}
}
#endif
#ifndef FPOptimizer_ValueRangeHH
#define FPOptimizer_ValueRangeHH
tC{iR2
iV2{iN
cP2
Comp{}
;t91>cP2
Comp<nE
cLess>{t91
lF<nC
cLessOrEq>{t91
lF<=nC
cGreater>{t91
lF>nC
cGreaterOrEq>{t91
lF>=nC
cEqual>{t91
lF==nC
tP1>{t91
lF!=b;}
}
;}
yD
cP2
rangehalf{l62
val;bool
known;rangehalf():val(),known(false){}
rangehalf
cX1
v):val(v),known(true){x61
void
set
cX1
v){known=true;val=v
n63
set(l62(i01(l62
nP)val=i11
void
set(l62(i01
cX1
nP)val=i11
iN
void
set_if(l62
v,l62(i01(l62
nP&&iV2::Comp<Compare>()(val,v))val=i11
iN
void
set_if
cX1
v,l62(i01
cX1
nP&&iV2::Comp<Compare>()(val,v))val=i11}
;yD
cP2
range{rangehalf
yB
min,max;range():min(),max(){}
range(l62
mi,l62
ma):min(mi),max(ma){}
range(bool,l62
ma):min(),max(ma){}
range(l62
mi,bool):min(mi),max(){}
void
set_abs();void
set_neg();}
;yD
bool
IsLogicalTrueValue
cY1
range
yB&p
eZ2;yD
bool
IsLogicalFalseValue
cY1
range
yB&p
eZ2;}
#endif
#ifndef FPOptimizer_RangeEstimationHH
#define FPOptimizer_RangeEstimationHH
tC{enum
TriTruthValue{IsAlways,xL3,Unknown}
;yD
range
yB
iO
xB3);yD
bool
IsLogicalValue
cY1
cV2;yD
TriTruthValue
GetIntegerInfo
cY1
cV2;yD
xA1
GetEvennessInfo
cY1
cV2{if(!tree
yP2
tJ2
Unknown;yB1=tree.xE1;if(nE
isEvenInteger(value
tJ2
IsAlways;if(nE
isOddInteger(value
tJ2
xL3
xD2
yD
xA1
GetPositivityInfo
cY1
cV2{range
yB
p=iO
tree);if(p
eY
cF3>=l62(tJ2
IsAlways;if(p
t6
yG1
tJ2
xL3
xD2
yD
xA1
GetLogicalValue
iX2
tree
eZ2{range
yB
p=iO
tree);if(IsLogicalTrueValue(p,abs
tJ2
IsAlways;if(IsLogicalFalseValue(p,abs
tJ2
xL3
xD2}
#endif
#ifndef FPOptimizer_ConstantFoldingHH
#define FPOptimizer_ConstantFoldingHH
tC{x73
ConstantFolding(cV2;}
#endif
iR2{using
iR2
FUNCTIONPARSERTYPES;using
tC;cP2
ComparisonSetBase{enum{cX3=0x1,Eq_Mask=0x2,Le_Mask=0x3,cY3=0x4,cZ3=0x5,Ge_Mask=0x6}
;static
int
Swap_Mask(int
m)yS(m&Eq_Mask)|((m&cX3)?cY3:0)|((m&cY3)?cX3:0);}
enum
yQ1{Ok,BecomeZero,BecomeOne,n21
nO3
nG2{cond_or,i92,iA2,iB2}
;}
;yD
cP2
ComparisonSet:public
ComparisonSetBase{cP2
t02{xG
a;xG
b;int
relationship;t02():a(),b(),relationship(){}
}
e72
t02>eB;cP2
Item{xG
value;bool
c12;Item():value(),c12(false){}
}
e72
Item>yY1;int
xB1;ComparisonSet():eB(),yY1(),xB1(0){}
yQ1
AddItem
iX2
a,bool
c12,nG2
type){for
x72
c=0;c<yY1.size();++c)if(yY1[c].value
xI
a)cI3
c12!=yY1[c].c12){iP
cF1
case
iB2:yY1.erase(yY1.begin()+c);xB1
c22
case
i92:case
iA2:cG1}
}
return
n21;}
Item
pole;pole.value=a;pole.c12=c12;yY1
yS2
pole)eO
Ok;}
yQ1
AddRelationship(xG
a,xG
b,int
tI1,nG2
type){iP
if(tI1==7)cF1
lD
iB2:if(tI1==7){xB1
c22}
lD
i92:case
iA2:if(tI1==0)cG1
yY3
if(!(a.GetHash()<b.GetHash())){a.swap(b);tI1=Swap_Mask(tI1);}
for
x72
c=0;c<eB.size();++c
cI3
eB[c].a
xI
a)&&eB[c].b
xI
b)){iP{int
xM3=yQ2|tI1;if(xM3==7)cF1
yQ2=xM3;break
cT1
i92:case
iA2:{int
xM3=yQ2&tI1;if(xM3==0)cG1
yQ2=xM3;break
cT1
iB2:{int
newrel_or=yQ2|tI1;int
xF2=yQ2&tI1;iW1
5&&xF2==0){yQ2=cZ3
eO
n21;}
iW1
7&&xF2==0){xB1+=1;eB.erase(eB.begin()+c)eO
n21;}
iW1
7&&xF2==Eq_Mask){yQ2=Eq_Mask;xB1
c22}
y41}
}
return
n21;}
}
t02
comp;comp.a=a;comp.b=b;comp.relationship=tI1;eB
yS2
comp)eO
Ok;}
}
;nH1
l62,typename
CondType>bool
ConstantFolding_LogicCommon(xG&tree,CondType
x11,bool
xG2){bool
should_regenerate
e23
ComparisonSet
yB
comp;for
c6
typename
yC
yQ1
cB3=yC
Ok;iT1
atree=xP1
a);switch(atree
nF
e53
cEqual
lH
Eq_Mask,x11);lD
tP1
lH
cZ3,x11);lD
cLess
lH
cX3,x11);lD
cLessOrEq
lH
Le_Mask,x11);lD
cGreater
lH
cY3,x11);lD
cGreaterOrEq
lH
Ge_Mask,x11);lD
cNot:cB3
e31
l8
0),true,x11);lD
cNotNot:cB3
e31
l8
0),false,x11
eP3;y13
if(xG2||IsLogicalValue(atree))cB3
e31,false,x11);yX3(cB3){ReplaceTreeWithZero
e93.ReplaceWithImmed(0)eO
true;ReplaceTreeWithOne
e93.ReplaceWithImmed(1);x0
yC
Ok:lD
yC
BecomeZero
tE
yC
BecomeOne:iA
yC
n21:yT1
yN3
if(should_regenerate){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before ConstantFolding_LogicCommon: "
xY
#endif
if(xG2){tree.DelParams();}
else{for
c7{iT1
atree=xP1
a);if(IsLogicalValue(atree))e61);}
}
n72
0;a<comp.yY1.size();++a
cI3
comp.yY1[a].c12
iW2
cNot);r
i8
r.nU1
i81!xG2
iW2
cNotNot);r
i8
r.nU1
else
tree
i8}
n72
0;a<comp.eB.size();++a
iW2
cNop);switch(comp.eB[a
i1
e53
yC
cX3:r
c4
cLess);lD
yC
Eq_Mask:r
c4
cEqual);lD
yC
cY3:r
c4
cGreater);lD
yC
Le_Mask:r
c4
cLessOrEq);lD
yC
cZ3:r
c4
tP1);lD
yC
Ge_Mask:r
c4
cGreaterOrEq);yY3
r
y7
comp.eB[a].a);r
y7
comp.eB[a].b);r.nU1
if(comp.xB1!=0)tree.yJ
l62(comp.xB1)));
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After ConstantFolding_LogicCommon: "
xY
#endif
return
true
yC3
e83
yD
bool
ConstantFolding_AndLogic(t53(tree
tC3()==cAnd||tree
tC3()==cAbsAnd)eO
nJ
i92,true
nD3
ConstantFolding_OrLogic(t53(tree
tC3()==cOr||tree
tC3()==cAbsOr)eO
nJ
cond_or,true
nD3
ConstantFolding_AddLogicItems(t53(tree
tC3()==cAdd)eO
nJ
iB2,false
nD3
ConstantFolding_MulLogicItems(t53(tree
tC3()==cMul)eO
nJ
iA2,false);}
}
#include <vector>
#include <map>
#include <algorithm>
iR2{using
iR2
FUNCTIONPARSERTYPES;using
tC;cP2
CollectionSetBase{enum
xC1{Ok,n21}
;}
;yD
cP2
CollectionSet:public
CollectionSetBase{cP2
yR1{xG
value;xG
xH2;bool
e4;yR1():value(),xH2(),e4(false){}
yR1
iX2
v,iT1
f):value(v),xH2(f),e4(false){}
}
;std::multimap<fphash_t,yR1>iD
tO3
typename
std::multimap<fphash_t,yR1>::xR3
xD1;CollectionSet():iD(){}
xD1
FindIdenticalValueTo
iX2
value){fphash_t
hash=value.GetHash();for(xD1
i=iD.xI2
hash);i!=iD.cH1
hash;++i){yW1
xI
i
cW2.value
tJ2
i
yC3
iD.end();}
bool
Found
cY1
xD1&b)yS
b!=iD.end();}
xC1
AddCollectionTo
iX2
xH2,const
xD1&into_which){yR1&c=into_which
cW2;if(c.e4)c.xH2
cM
xH2);else{xG
add;add
c4
cAdd);add
y7
c.xH2);add
cM
xH2);c.xH2.swap(add);c.e4=true
yC3
n21;}
xC1
nH2
iX2
value,iT1
xH2){const
fphash_t
hash=value.GetHash();xD1
i=iD.xI2
hash);for(;i!=iD.cH1
hash;++i
cI3
i
cW2.value
xI
value
tJ2
AddCollectionTo(xH2,i);}
iD.xN3,std::make_pair(hash,yR1(value,xH2)))eO
Ok;}
xC1
nH2
iX2
a)yS
nH2(a,n31
1)));}
}
;yD
cP2
ConstantExponentCollection{typedef
nX2
xP3
tO3
std::nV1
xJ2
e72
xJ2>data;ConstantExponentCollection():data(){}
void
MoveToSet_Unique
cX1
eF1&eG1){data
yS2
std::nV1(eF1()));data.back().second.swap(eG1
n33
MoveToSet_NonUnique
cX1
eF1&eG1){typename
std::vector<xJ2>::xR3
i=std::xI2
data.iC2
data.end(),nU3,Compare1st());if(i!=data.cH1
x32{i
cW2.xN3
cW2.end(),eG1.iC2
eG1.end());}
else{data.xN3,std::nV1(nU3,eG1));}
}
bool
i22{bool
changed
e23
std::sort(data.iC2
data.end(),Compare1st());redo:n72
0;a<data.size();++a){l62
exp_a=data[a
e03;if(fp_equal(exp_a,l62(1)))y41
x62
a+1;b<data.size();++b){l62
exp_b=data[b
e03;l62
xK2=exp_b-exp_a;if(xK2>=fp_abs(exp_a
lE3
exp_diff_still_probable_integer=xK2*l62(16);if(t12
exp_diff_still_probable_integer)&&!(t12
exp_b)&&!t12
xK2))){xP3&a_set=iX1;xP3&b_set=data[b].second;
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before ConstantExponentCollection iteration:\n"
;t22
cout);
#endif
if(isEvenInteger(exp_b)&&!isEvenInteger(xK2+exp_a)nQ
tmp2;tmp2
c4
cMul);tmp2
iY1
b_set);tmp2
eM1
t21
cAbs);tmp
c43;tmp
eM1
b_set
x53
1);b_set[0].tV2}
a_set.insert(a_set.end(),b_set.iC2
b_set.end());xP3
b_copy=b_set;data.erase(data.begin()+b);MoveToSet_NonUnique(xK2,b_copy);nS2
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After ConstantExponentCollection iteration:\n"
;t22
cout);
#endif
cN}
}
}
return
changed;}
#ifdef DEBUG_SUBSTITUTIONS
void
t22
ostream&out){n72
0;a<data.size();++a){out.precision(12);out<<data[a
e03<<": "
;xS3
iX1.size();++b
cI3
b>0)out<<'*'
yR2
iX1[b],out);}
out<<std::endl;}
}
#endif
}
;yD
static
xG
nW1
xG&value,bool&xL){switch(value
nF
e53
cPow:{xG
cY2
value
l8
1);value.y31
eO
nU3
cT1
cRSqrt:value.y31;xL=true
eO
n31-0.5));case
cInv:value.y31;xL=true
eO
n31-1));y13
break
yC3
n31
1));}
yD
static
void
eH1
l12&mul,xB3,iT1
xH2,bool&yS1
bool&xL){n72
0;a<t7;++a
nQ
value
tN3);xG
nU3(nW1
value,xL));if(!xH2
yP2)||xH2.xE1!=l62(1.0)nQ
cI1;cI1
c4
cMul);cI1
cM
x32;cI1
cM
xH2);cI1
eM1
nU3.swap(cI1);}
#if 0 /* FIXME: This does not work */
yW1
nF==cMul
cI3
1){bool
exponent_is_even=nU3
yP2)&&isEvenInteger(nU3.xE1);xS3
value.x31++b){bool
tmp
e23
xG
val(value
l8
b));xG
exp(nW1
val,tmp));if(exponent_is_even||(exp
yP2)&&isEvenInteger(exp.xE1))nQ
cI1;cI1
c4
cMul);cI1
cM
x32;cI1
y7
exp);cI1.ConstantFolding();if(!cI1
yP2)||!isEvenInteger(cI1.xE1)){goto
cannot_adopt_mul;}
}
}
}
eH1
mul,value,nU3,yS1
xL);}
else
cannot_adopt_mul:
#endif
{if(mul.nH2(value,x32==CollectionSetBase::n21)yT1}
}
}
yD
bool
ConstantFolding_MulGrouping(cV2{bool
xL
e23
bool
should_regenerate
e23
l12
mul;eH1
mul,tree,n31
1)),yS1
xL)tO3
std::pair<xG,nX2>eI1
tO3
std::multimap<fphash_t,eI1>yZ1;yZ1
iE;xL2
l12::xD1
j=mul.iD.xQ3
j!=mul.iD.end();++j
nQ&value=j
cW2.value;xG&cY2
j
cW2.xH2;if(j
cW2.e4)nU3
eM1
const
fphash_t
eJ1=nU3.GetHash();typename
yZ1::xR3
i=iE.xI2
eJ1);for(;i!=iE.cH1
eJ1;++i)if(i
cW2.first
xI
x32
cI3!nU3
yP2)||!cJ1.xE1,l62(1)))yT1
i
cW2.second
yS2
value);goto
skip_b;}
iE.xN3,std::make_pair(eJ1,std::make_pair(nU3,nX2
x72(1),value))));skip_b:;}
#ifdef FP_MUL_COMBINE_EXPONENTS
ConstantExponentCollection
yB
cK1;xL2
yZ1::xR3
j,i=iE.xQ3
i!=iE.end();i=j){j=i;++j;eI1&list=i
cW2;if(list
y93
lK1
cY2
list
y93
xE1;if(!(nU3==tD1)))cK1.MoveToSet_Unique(nU3,list.second);iE.erase(i);}
}
if(cK1.i22)yT1
#endif
if(should_regenerate
nQ
before=tree;before.l41
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before ConstantFolding_MulGrouping: "
yR2
before)xO1"\n"
;
#endif
tree.DelParams();xL2
yZ1::xR3
i=iE.xQ3
i!=iE.end();++i){eI1&list=i
cW2;
#ifndef FP_MUL_COMBINE_EXPONENTS
if(list
y93
lK1
cY2
list
y93
xE1;if(nU3==tD1))y41
if(cJ1
n53
tree.AddParamsMove(list.second);y41}
}
#endif
xG
mul;mul
c4
cMul);mul
iY1
list.second);mul
eM1
if(xL&&list
y93
lO2
list
y93
xE1==l62(1)/l62(3)nQ
cbrt;cbrt
c4
cCbrt);cbrt
xM2
cbrt
eM1
tree
y7
cbrt);y41
cP
0.5)nQ
sqrt;sqrt
c4
cSqrt);sqrt
xM2
sqrt
eM1
tree
y7
sqrt);y41
cP-0.5)nQ
rsqrt;rsqrt
c4
cRSqrt);rsqrt
xM2
rsqrt
eM1
tree
y7
rsqrt);y41
cP-1)nQ
inv;inv
c4
cInv);inv
xM2
inv
eM1
tree
y7
inv);y41}
}
xG
pow;pow
c4
cPow);pow
xM2
pow
y7
list.first);pow
eM1
tree
y7
pow);}
#ifdef FP_MUL_COMBINE_EXPONENTS
iE.clear();n72
0;a<i6.size();++a){l62
cY2
i6[a
e03;if(cJ1
n53
tree.AddParamsMove(i6[a].second);y41}
xG
mul;mul
c4
cMul);mul
iY1
i6[a].second);mul
eM1
xG
pow;pow
c4
cPow);pow
xM2
pow.yJ
x32);pow
eM1
tree
y7
pow);}
#endif
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After ConstantFolding_MulGrouping: "
xY
#endif
return!tree
xI
before)yC3
e83
yD
bool
ConstantFolding_AddGrouping(cV2{bool
should_regenerate
e23
l12
add;for
c6
if
tN3
nF==cMul)y41
if(add.nH2
tN3)==CollectionSetBase::n21)yT1}
std::vector<bool>remaining(t7);size_t
tJ=0;for
c6
iT1
i82=xP1
a);if(i82
nF==cMul){xS3
xY1
x31++b
cI3
i82
l8
b)yP2))y41
typename
l12::xD1
c=add.FindIdenticalValueTo(i82
l8
b));if(add.Found(c)nQ
tmp(i82
lL1
CloneTag());tmp.DelParam(b);tmp
eM1
add.AddCollectionTo(tmp,c);yT1
goto
done_a;}
}
remaining[a]=true;tJ+=1;done_a:;}
}
if(tJ>0
cI3
tJ>1){std::vector<std::pair<xG,size_t> >nZ;std::multimap<fphash_t,size_t>eK1;bool
iY2
e23
for
eX{xS3
xP1
a).x31++b){iT1
p=xP1
a)l8
b);const
fphash_t
p_hash=p.GetHash();for(std::multimap<fphash_t,size_t>::const_iterator
i=eK1.xI2
p_hash);i!=eK1.cH1
p_hash;++i
cI3
nZ[i
cW2
e03
xI
p)){nZ[i
cW2].second+=1;iY2=true;goto
found_mulgroup_item_dup;}
}
nZ
yS2
std::make_pair(p,size_t(1)));eK1.insert(std::make_pair(p_hash,nZ.size()-1));found_mulgroup_item_dup:;}
}
if(iY2
nQ
cZ2;{size_t
max=0;for
x72
p=0;p<nZ.size();++p)if(nZ[p].second<=1)nZ[p].second=0;else{nZ[p].second*=nZ[p]y93
xT2;if(nZ[p].second>max){cZ2=nZ[p
e03;max=nZ[p].second;}
}
}
xG
group_add;group_add
c4
cAdd);
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Duplicate across some trees: "
yR2
cZ2)xO1" in "
xY
#endif
for
eX
xS3
xP1
a).x31++b)if(cZ2
xI
xP1
a)l8
b))nQ
tmp
tN3
lL1
CloneTag());tmp.DelParam(b);tmp
eM1
group_add
y7
tmp);remaining[a]e23
yY3
group_add
eM1
xG
group;group
c4
cMul);group
y7
cZ2);group
y7
group_add);group
eM1
add.nH2(group);yT1}
}
for
eX{if(add.nH2
tN3)==CollectionSetBase::n21)yT1}
}
if(should_regenerate){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before ConstantFolding_AddGrouping: "
xY
#endif
tree.DelParams();xL2
l12::xD1
j=add.iD.xQ3
j!=add.iD.end();++j
nQ&value=j
cW2.value;xG&coeff=j
cW2.xH2;if(j
cW2.e4)coeff
eM1
if(coeff.lO2
fp_equal(coeff.xE1,tD1)))y41
if(fp_equal(coeff.xE1
n53
tree
y7
value);y41}
}
xG
mul;mul
c4
cMul);mul
y7
value);mul
y7
coeff);mul
eM1
tree
xM2}
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After ConstantFolding_AddGrouping: "
xY
#endif
return
true
yC3
e83}
iR2{using
iR2
FUNCTIONPARSERTYPES;using
tC;yD
bool
ConstantFolding_IfOperations(t53(tree
tC3()==cIf||tree
tC3()==cAbsIf);for(;;cI3
e43
cNot){tree
c4
cIf);tQ
iD2
tQ
xN2
t32.swap(cC3);}
i81
tQ
c71{tree
c4
e63;tQ
iD2
tQ
xN2
t32.swap(cC3);}
else
break;yX3
nR1
tQ,i21
e63)i71
tree
iD2
t32);x0
iO2
tree
iD2
cC3);x0
lS1
if(e43
cIf||e43
cAbsIf
nQ
cond=tQ;xG
iZ2;iZ2
c4
cond
nF==cIf?cNotNot:cAbsNotNot);iZ2
xP2
1));ConstantFolding(iZ2);xG
l03;l03
c4
cond
nF==cIf?cNotNot:cAbsNotNot);l03
xP2
2));ConstantFolding(l03);if(iZ2
yP2)||l03
yP2)nQ
eS;eS
c4
cond
nF);eS
xP2
1));eS.n7
1));eS.n7
2));eS
eM1
xG
eT;eT
c4
cond
nF);eT
xP2
2));eT.n7
1));eT.n7
2));eT
eM1
tree
c4
cond
nF)y51
0,cond
xN2
tree.n41
1,eS);tree.n41
2,eT)cT2}
if(t32
nF==cC3
nF&&(t32
nF==cIf||t32
nF==e63
nQ&i72=t32;xG&leaf2=cC3;if(i72
l8
0)x21
0))&&(i72
l8
1)x21
1))||i72
l8
2)x21
2)))nQ
eS;eS
n82
eS.n7
0));eS
cM
i72
l8
1));eS
cM
leaf2
l8
1));eS
eM1
xG
eT;eT
n82
eT.n7
0));eT
cM
i72
l8
2));eT
cM
leaf2
l8
2));eT
t5
SetParam(0,i72
xN2
tree.n41
1,eS);tree.n41
2,eT)cT2
if(i72
l8
1)x21
1))&&i72
l8
2)x21
2))nQ
eU;eU
n82
eU
y7
tQ);eU
cM
i72
xN2
eU
cM
leaf2
xN2
eU
t5
n41
0,eU)y51
2,i72
l8
2))y51
1,i72
l8
1))cT2
if(i72
l8
1)x21
2))&&i72
l8
2)x21
1))nQ
e02;e02
c4
leaf2
nF==cIf?cNot:c03);e02
cM
leaf2
xN2
e02
eM1
xG
eU;eU
n82
eU
y7
tQ);eU
cM
i72
xN2
eU
y7
e02);eU
t5
n41
0,eU)y51
2,i72
l8
2))y51
1,i72
l8
1))cT2}
xG&y2=t32;xG&y6=cC3;if(y2
xI
y6)){tree
iD2
t32)cT2
const
OPCODE
op1=y2
nF;const
OPCODE
op2=y6
nF;cL3
op2
cI3
y2
yU1
1
nQ
lP
0));xQ2
0))iZ1
n4
if(y2
yU1
2&&y6
yU1
2
cI3
y2
l8
0)xI
y6
l8
0))nQ
param0=y2
l8
0);xG
lP
1));xQ2
1))iZ1
e12
param0)n4
if(y2
l8
1)xI
y6
l8
1))nQ
param1=y2
l8
1);xG
lP
0));xQ2
0))iZ1
e12
n11)e12
param1)cT2}
cL3
xT3
cMul
l02
cAnd
l02
cOr
l02
cAbsAnd
l02
cAbsOr
l02
cMin
l02
cMax){nX2
l13;c3{x62
y6.x31
b-->0;){nI2
y6
l8
b))cI3
l13
yU3){y2.e73
l41}
l13
yS2
y2
lS3
y6.DelParam(b);y2.DelParam(a);yN3}
if(!l13
yU3){y2.Rehash(eL1
n11;n11
n82
n11
iY1
tree.iP1))yF
SetParamsMove(l13)n4}
}
cL3
xT3
cMul||(op1==cAnd
nK1
y6))||(op1==cOr
nK1
y6))){c3
nI2
y6)){y2.l41
y2.DelParam(a);y2
eM1
xG
c01=y6;y6=tK
op1==xT3
cOr)l9
op1)e12
c01)n4}
if((op1==cAnd
l02
cOr)&&op2==cNotNot
nQ&l23=y6
l8
0);c3
nI2
l23)){y2.l41
y2.DelParam(a);y2
eM1
xG
c01=l23;y6=tK
op1
xU3
op1)e12
c01)n4}
if(op2==cAdd||op2==cMul||(op2==cAnd
nK1
y2))||(op2==cOr
nK1
y2))){n72
y6.n93
y6
l8
a)xI
y2)){y6.e73
DelParam(a
eL1
c11=y2;y2=tK
op2==cAdd||op2
xU3
op2)e12
c11)n4}
if((op2==cAnd||op2==cOr)&&op1==cNotNot
nQ&l33=y2
l8
0);n72
y6.n93
y6
l8
a)xI
l33)){y6.e73
DelParam(a
eL1
c11=l33;y2=tK
op2
xU3
op2)e12
c11)n4}
return
e83}
#include <limits>
iR2{using
iR2
FUNCTIONPARSERTYPES;using
tC;yD
int
maxFPExponent()yS
std::numeric_limits
yB::max_exponent;}
yD
bool
tE1
l62
base,l62
x32{if(base<tD1
t41;if(fp_equal(base,tD1))||fp_equal(base,l62(1))c5
return
nU3>=l62(maxFPExponent
yB())/fp_log2(base);}
yD
int
nY1
l62
val){int
ex=0;l62
t=std::frexp(val,&ex);unsigned
yO1=fp_abs(t)*(1u<<30),v0=value;unsigned
int
nT3=0;while(!(value&1))value>>=1;for(;value!=0;value>>=1)++nT3
eO
nT3;}
yD
bool
ConstantFolding_PowOperations(t53(tree
tC3()==cPow);cD3
t32.lK1
const_value
xV3
lS,tM));tree.ReplaceWithImmed(const_value)eO
e83
if
lM1
fp_equal(tM)n53
tree
iD2
tQ)cT2
cD3
fp_equal(lS,l62(1)nR
1)eO
e83
cD3
t32
nF==cMul){bool
xR2
e23
l62
eO1=lS;xG
i82=t32;n72
xY1
n93
i82
l8
a).lK1
imm=i82
l8
a).xE1;{if(tE1
eO1,imm
lE3
c21
xV3
eO1,imm);if(fp_equal(c21,tD1)))break;if(nY1
c21)<(nY1
eO1)+nY1
imm))/4){yY3
if(!xR2){xR2=true;xY1
l41}
eO1=c21;xY1
DelParam(a);yN3
if(xR2){xY1
Rehash();
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before pow-mul change: "
xY
#endif
tQ
iD2
cL1
eO1));t32
iD2
i82);
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After pow-mul change: "
xY
#endif
}
}
if
lM1
e43
cMul){l62
l72=tM);l62
xS2=1.0;bool
xR2
e23
xG&i82=tQ;n72
xY1
n93
i82
l8
a).lK1
imm=i82
l8
a).xE1;{if(tE1
imm,l72
lE3
eP1
xV3
imm,l72);if(fp_equal(eP1,tD1)))break;if(!xR2){xR2=true;xY1
l41}
xS2*=eP1;xY1
DelParam(a);yN3
if(xR2){xY1
Rehash();xG
cH3;cH3
c4
cPow);cH3
iY1
tree.iP1));cH3.lF2
tree
c4
cMul)e12
cH3);tree
cM
cL1
xS2))cT2}
if(e43
cPow&&xP1
yO2&&tQ
l8
1).lK1
a=tQ
l8
1).xE1;l62
b=tM);l62
c=a*b;if(isEvenInteger(a)&&!isEvenInteger(c)nQ
l43;l43
c4
cAbs);l43.n7
0)xN2
l43
eM1
tree.n41
0,l43);}
else
tree.SetParam(0,tQ
l8
0))y51
1,cL1
c))yC3
e83}
iR2{using
iR2
FUNCTIONPARSERTYPES;using
tC;cP2
l4{enum
e22{MakeFalse=0,MakeTrue=1,t42=2,l73=3,MakeNotNotP0=4,MakeNotNotP1=5,MakeNotP0=6,MakeNotP1=7,xK=8
nO3
l82{Never=0,Eq0=1,Eq1=2,xW3=3,xX3=4}
;e22
if_identical;e22
l92
4];cP2{e22
what:4;l82
when:4;}
iA1,iB1,iC1,iD1;yD
e22
Analyze
iX2
a,iT1
b)const{if(a
xI
b
tJ2
if_identical;range
yB
p0=iO
a);range
yB
p1=iO
b);if(p0
t6&&p1
eY
cI3
p0
nV<p1
e32&&l92
0]i7
0];if(p0
nV<=p1
e32&&l92
1]i7
1];}
if(p0
eY&&p1
t6
cI3
p0
e32>p1
nV&&l92
2]i7
2];if(p0
e32>=p1
nV&&l92
3]i7
3];}
if(IsLogicalValue(a)cI3
iA1
e92
iA1.when,p1
tJ2
iA1.what;if(iC1
e92
iC1.when,p1
tJ2
iC1.what;}
if(IsLogicalValue(b)cI3
iB1
e92
iB1.when,p0
tJ2
iB1.what;if(iD1
e92
iD1.when,p0
tJ2
iD1.what
yC3
xK;}
yD
static
bool
TestCase(l82
when,const
range
yB&p
cI3!p
eY||!p
t6
c5
switch(when
e53
Eq0
nM1==l62(0.0
eB3==p
e32;case
Eq1
nM1==l62(1.0
eB3==p
nV;case
xW3
nM1>tD1
eB3<=l62(1);case
xX3
nM1
cW1
yG1
1);y13
yC3
e83}
;iR2
RangeComparisonsData{static
const
l4
Data[6]={{l4
l53
i2
xK,l4::i2
xK}
,n71
Eq1}
,n81
Eq1}
,{l4::c31
Eq0}
,{l4::c41
Eq0}
}
,{l4::nJ2
l63
xK,l4
l63
xK}
,n71
Eq0}
,n81
Eq0}
,{l4::c31
Eq1}
,{l4::c41
Eq1}
}
,{l4::nJ2
l63
t42,l4::i2
MakeFalse}
,{l4::c31
xW3}
,n81
xX3
yZ,{l4
l53
xK,l4
l63
i2
l73}
,{l4::c31
xX3}
,n81
xW3
yZ,{l4::nJ2::i2
i2
MakeTrue,l4::t42}
,n71
xX3}
,{l4::c41
xW3
yZ,{l4
l53
i2
l73,l4::xK,l4
n61}
,n71
xW3}
,{l4::c41
xX3
yZ}
;}
yD
bool
ConstantFolding_Comparison(cV2{using
iR2
RangeComparisonsData;assert(tree tC3()>=cEqual&&tree tC3()<=cGreaterOrEq);switch(Data[tree
nF-cEqual].Analyze(tQ,t32)e53
l4::MakeFalse
e93.ReplaceWithImmed(0);x0
l4
n61
e93.ReplaceWithImmed(1
l52
l73
e93
c4
cEqual
l52
t42
e93
c4
tP1
l52
MakeNotNotP0
e93
c4
cNotNot)lP2
1
l52
MakeNotNotP1
e93
c4
cNotNot)lP2
0
l52
MakeNotP0
e93
c4
cNot)lP2
1
l52
MakeNotP1
e93
c4
cNot)lP2
0
l52
xK:;}
if(xP1
yO2)switch(tQ
nF
e53
cAsin:lN
fp_sin
xY3
cAcos:lN
fp_cos
tL));tree
c4
i21
cLess?cGreater:i21
cLessOrEq?cGreaterOrEq:i21
cGreater?cLess:i21
cGreaterOrEq?cLessOrEq
e93
nF);x0
cAtan:lN
fp_tan
xY3
cLog:lN
fp_exp
xY3
cSinh:lN
fp_asinh
xY3
cTanh:if(fp_less(fp_abs
tL
n53
lN
fp_atanh
tL))cT2
break;y13
break
yC3
e83}
#include <list>
#include <algorithm>
#ifdef FP_SUPPORT_OPTIMIZER
using
iR2
FUNCTIONPARSERTYPES;iR2{
#ifdef DEBUG_SUBSTITUTIONS
yM
double
d){union{double
d;uint_least64_t
h;}
t52
d=d;lG1
h
nN1
#ifdef FP_SUPPORT_FLOAT_TYPE
yM
float
f){union{float
f;uint_least32_t
h;}
t52
f=f;lG1
h
nN1
#endif
#ifdef FP_SUPPORT_LONG_DOUBLE_TYPE
yM
long
double
ld){union{long
double
ld;cP2{uint_least64_t
a;unsigned
short
b;}
s;}
t52
ld=ld;lG1
s.b<<data.s.a
nN1
#endif
#ifdef FP_SUPPORT_LONG_INT_TYPE
yM
long
ld){o<<"("
<<std::hex<<ld
nN1
#endif
#endif
}
tC{lO
nG)){}
lO
const
l62&i
lL1
x23
nG
i)){data
xE
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
lO
l62&&i
lL1
x23
nG
std::move(i))){data
xE
#endif
lO
unsigned
v
lL1
VarTag
nG
iE2,v))c32
nW2
o
lL1
OpcodeTag
nG
o))c32
nW2
o,unsigned
f
lL1
FuncOpcodeTag
nG
o,f))c32
iT1
b
lL1
CloneTag
nG*b.data)){}
yD
xG::~CodeTree(){}
lC
ReplaceWithImmed
cX1
i){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Replacing "
yR2*this);if(IsImmed())OutFloatHex(std::cout,xE1)xO1" with const value "
<<i;OutFloatHex(std::cout,i)xO1"\n"
;
#endif
data=new
x42
yB(i);}
yD
cP2
ParamComparer{bool
eD1()iX2
a,iT1
b)const{if(a.xT2!=b.xT2)return
a.xT2<b.xT2
eO
a.GetHash()<b.GetHash();}
}
;x73
x42
yB::Sort(){switch(Opcode
e53
cAdd:case
cMul:case
cMin:case
cMax:case
cAnd:case
cAbsAnd:case
cOr:case
cAbsOr:case
cHypot:case
cEqual:case
tP1:std::sort(x83
iC2
x83
end(),ParamComparer
yB());lD
cLess
lZ
cGreater;}
lD
cLessOrEq
lZ
cGreaterOrEq;}
lD
cGreater
lZ
cLess;}
lD
cGreaterOrEq
lZ
cLessOrEq;}
break;y13
yN3
lC
AddParam
iX2
param){y3
yS2
param);}
lC
eE
xG&param){y3
yS2
xG());y3.back().swap(param);}
lC
SetParam
x72
which,iT1
b)nO1
which
xZ3
y3[which]=b;}
lC
n41
size_t
which,xG&b)nO1
which
xZ3
y3[which
eO3
b);}
lC
AddParams
cY1
nK){y3.insert(y3.end(),lA2.iC2
lA2.end());}
lC
AddParamsMove(nK){size_t
endpos=y3.size(),added=lA2.size();y3
x53
endpos+added,xG());for
x72
p=0;p<added;++p)y3[endpos+p
eO3
lA2[p]);}
lC
AddParamsMove(nK,size_t
lB2)nO1
lB2
xZ3
DelParam(lB2);AddParamsMove(t81}
lC
SetParams
cY1
nK){nX2
tmp(t81
y3.tV2}
lC
SetParamsMove(nK){y3.swap(t81
lA2.clear();}
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
lC
SetParams(nX2&&lA2){SetParamsMove(t81}
#endif
lC
DelParam
x72
index){nX2&Params=y3;
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
x83
erase(x83
begin()+index);
#else
Params[index].data=0;for
x72
p=index;p+1<y03;++p)Params[p].data.UnsafeSetP(Params[p+1]t62));Params[y03-1].data.UnsafeSetP(0);Params
x53
y03-1);
#endif
}
lC
DelParams(){y3.clear(nD3
xG::IsIdenticalTo
iX2
b)const{if(data.get()==b
t62
t41
eO
data->IsIdenticalTo(*b.data
nD3
x42
yB::IsIdenticalTo
cY1
x42
yB&b)const{if(Hash!=b.Hash
c5
if(Opcode!=eL3
c5
switch(Opcode
e53
cImmed:return
fp_equal(Value,eM3;case
iE2:return
iR1==b.iQ1
case
cFCall:case
cPCall:if(iR1!=b.iR1
c5
break;y13
yY3
if(y03!=b.y03
c5
n72
0;a<y03;++a
cI3!x93
xI
b.x93)c5}
return
true;}
lC
Become
iX2
b
cI3&b!=this&&data.get()!=b
t62)){DataP
tmp=b.data;l41
data.tV2}
}
lC
CopyOnWrite(cI3
GetRefCount()>1)data=new
x42
yB(*data);}
yD
xG
xG::GetUniqueRef(cI3
GetRefCount()>1)return
xG(*this,CloneTag())eO*this;}
yD
nO):yU
cNop),Value(),n9
yD
nO
const
x42&b):yU
eL3),Value(eM3,iR1(b.c51,Params(b.Params),Hash(b.Hash),Depth(b.Depth),tJ1
b.iS1){}
yD
nO
const
l62&i):yU
cImmed),Value(i),n9
#if defined(__GXX_EXPERIMENTAL_CXX0X__) || __cplusplus >= 201103L
yD
nO
x42
yB&&b):yU
eL3),Value
cX2
eM3),iR1(b.c51,Params
cX2
b.Params)),Hash(b.Hash),Depth(b.Depth),tJ1
b.iS1){}
yD
nO
l62&&i):yU
cImmed),Value
cX2
i)),n9
#endif
yD
nO
nW2
o):yU
o),Value(),n9
yD
nO
nW2
o,unsigned
f):yU
o),Value(),iR1(f),Params(),Hash(),Depth(1),tJ1
0){}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <iostream>
using
iR2
FUNCTIONPARSERTYPES;
#ifdef FUNCTIONPARSER_SUPPORT_DEBUGGING
iR2{x73
tK1
nU,std
cF&done,std::ostream&o){n72
0
tH2
tK1
tJ3
done,o);std::ostringstream
buf
yR2
tree,buf);done[xC3].insert(buf.str());}
}
#endif
tC{
#ifdef FUNCTIONPARSER_SUPPORT_DEBUGGING
x73
DumpHashes(i9){std
cF
done;tK1
tree,done,o);for(std
cF::const_iterator
i=done.xQ3
i!=done.end();++i){const
std::set<std
yE3>&flist=i
cW2;if(flist.size()!=1)o<<"ERROR - HASH COLLISION?\n"
;for(std::set<std
yE3>::const_iterator
j=flist.xQ3
j!=flist.end();++j){o<<'['<<std::hex<<i->first.hash1<<','<<i->first.hash2<<']'<<std::dec;o<<": "
<<*j<<"\n"
;}
}
}
x73
DumpTree(i9){nA3
t33;switch
l83{case
cImmed:o<<tree.xE1
eA3
iE2:o<<"Var"
<<(tree.GetVar()-iE2)eA3
cAdd:t33"+"
;lD
cMul:t33"*"
;lD
cAnd:t33"&"
;lD
cOr:t33"|"
;lD
cPow:t33"^"
;break;y13
t33;o<<eK3
l83;if(i21
cFCall||i21
cPCall)o<<':'<<tree.GetFuncNo();}
o<<'(';if(t7<=1&&sep2[1])o<<(sep2+1)<<' ';for
c6
if(a>0)o<<' '
yR2
tJ3
o);if(a+1<t7)o<<sep2;}
o<<')';}
x73
DumpTreeWithIndent(i9,const
std
yE3&indent){o<<'['<<std::hex<<(void*)(&tree.iP1))<<std::dec<<','<<tree.GetRefCount()<<']';o<<indent<<'_';switch
l83{case
cImmed:o<<"cImmed "
<<tree.xE1;o<<'\n'
eA3
iE2:o<<"VarBegin "
<<(tree.GetVar()-iE2);o<<'\n'
eO;y13
o<<eK3
l83;if(i21
cFCall||i21
cPCall)o<<':'<<tree.GetFuncNo();o<<'\n';}
for
c6
std
yE3
ind=indent;for
x72
p=0;p<ind.size();p+=2)if(ind[p]=='\\')ind[p]=' ';ind+=(a+1<t7)?" |"
:" \\"
;DumpTreeWithIndent(tJ3
o,ind);}
o<<std::flush;}
#endif
}
#endif
using
iR2
l21;using
iR2
FUNCTIONPARSERTYPES;
#include <cctype>
iR2
l21{unsigned
ParamSpec_GetDepCode
cY1
cS2&b){switch(b.first
e53
ParamHolder:{cQ*s=(cQ*)b.second
eO
s->depcode
cT1
SubFunction:{cR*s=(cR*)b.second
eO
s->depcode;}
y13
break
yC3
0;}
x73
DumpParam
cY1
cS2&x02,std::ostream&o){static
const
char
ParamHolderNames[][2]={"%"
,"&"
,"x"
,"y"
,"z"
,"a"
,"b"
,"c"
}
;unsigned
xU2
0;switch(x02.first
e53
NumConstant:{const
ParamSpec_NumConstant
yB
y63
cY1
ParamSpec_NumConstant
yB*e62;using
iR2
FUNCTIONPARSERTYPES;o.precision(12);o<<yA3
constvalue;break
cT1
ParamHolder:{cQ
y63(cQ*e62;o<<ParamHolderNames[yA3
index];xU2
yA3
constraints;break
cT1
SubFunction:{cR
y63(cR*e62;xU2
yA3
constraints;yN
GroupFunction){l93
l51==cNeg){o<<"-"
;n1}
i81
yA3
l51==cInv){o<<"/"
;n1}
else{std
yE3
opcode=eK3((nW2)yA3
l51).substr(1);n72
0;a<opcode.size();++a)opcode[a]=(char)std::toupper(opcode[a]);o<<opcode<<"( "
;n1
o<<" )"
;}
}
else{o<<'('<<eK3((nW2)yA3
l51)<<' ';yN
PositionalParams)o<<'[';yN
SelectedParams)o<<'{';n1
l93
data.n0!=0)o<<" <"
<<yA3
data.n0<<'>';yN
PositionalParams)o<<"]"
;yN
SelectedParams)o<<"}"
;o<<')';}
yN3
switch(ImmedConstraint_Value(constraints&ValueMask)e53
ValueMask:lD
Value_AnyNum:lD
nK2:o<<"@E"
;lD
Value_OddInt:o<<"@O"
;lD
tM1:o<<"@I"
;lD
Value_NonInteger:o<<"@F"
;lD
eQ1:o<<"@L"
;y73
ImmedConstraint_Sign(constraints&SignMask)e53
SignMask:lD
Sign_AnySign:lD
n91:o<<"@P"
;lD
eR1:o<<"@N"
;y73
ImmedConstraint_Oneness(constraints&OnenessMask)e53
OnenessMask:lD
Oneness_Any:lD
Oneness_One:o<<"@1"
;lD
Oneness_NotOne:o<<"@M"
;y73
ImmedConstraint_Constness(constraints&ConstnessMask)e53
ConstnessMask:lD
tL1:if(x02.first==ParamHolder){cQ
y63(cQ*e62;l93
index<2)yY3
o<<"@C"
;lD
Constness_NotConst:o<<"@V"
;lD
Oneness_Any:yN3
x73
DumpParams
nD2
paramlist,unsigned
count,std::ostream&o){for
nD2
a=0;a<count;++a
cI3
a>0)o<<' ';const
cS2&param=cR1
yB(paramlist,a);DumpParam
yB(param,o);unsigned
depcode=ParamSpec_GetDepCode(param);if(depcode!=0)o<<"@D"
<<depcode;}
}
}
#include <algorithm>
using
iR2
l21;using
iR2
FUNCTIONPARSERTYPES;iR2{cQ
plist_p[37]={{2,0,0x0}
x9
0,0x4}
x9
n91,0x0}
x9
eR1|Constness_NotConst,0x0}
x9
Sign_NoIdea,0x0}
x9
eQ1,0x0}
,{3,Sign_NoIdea,0x0}
,{3,0,0x0}
,{3,eQ1,0x0}
,{3,0,0x8}
,{3,Value_OddInt,0x0}
,{3,Value_NonInteger,0x0}
,{3,nK2,0x0}
,{3,n91,0x0}
,{0,eR1|lW{0,lW{0,n91|lW{0,nK2|lW{0,tL1,0x1}
,{0,tM1|n91|lW{0,tN1
tL1,0x1}
,{0,tN1
lW{0,Oneness_One|lW{0,eQ1|lW{1,lW{1,nK2|lW{1,tN1
lW{1,tM1|lW{1,n91|lW{1,eR1|lW{6,0,0x0}
,{4,0,0x0}
,{4,tM1,0x0}
,{4,lW{4,0,0x16}
,{5,0,0x0}
,{5,lW}
;yD
cP2
plist_n_container{static
const
ParamSpec_NumConstant
yB
plist_n[20];}
;yD
const
ParamSpec_NumConstant
yB
plist_n_container
yB::plist_n[20]={{l62(-2
i3-1
i3-0.5
i3-0.25
i3
0
t72
fp_const_deg_to_rad
yB(t72
fp_const_einv
yB(t72
fp_const_log10inv
yB(i3
0.5
t72
fp_const_log2
yB(i3
1
t72
fp_const_log2inv
yB(i3
2
t72
fp_const_log10
yB(t72
fp_const_e
yB(t72
fp_const_rad_to_deg
yB(t72-fp_const_pihalf
yB(),xG1{tD1),xG1{fp_const_pihalf
yB(),xG1{fp_const_pi
yB(),xG1}
;cR
plist_s[517]={{{1,15,t82
398,t82
477,t82
15,cNeg,GroupFunction,0}
,tL1,0x1
tL3
15,xV2
24,xV2
465,xV2
466,xV2
498,cInv,lU
2,327995
xC
eB2
48276
xC
l6
260151
xC
l6
470171
xC
l6
169126
xC
l6
48418
xC
l6
1328
xC
l6
283962
xC
l6
169275
xC
l6
39202
xC
l6
283964
xC
l6
283973
xC
l6
476619
xC
l6
296998
xC
l6
47
xC
SelectedParams,0}
,0,0x4
nM
161839
xC
l6
25036
xC
l6
35847
xC
l6
60440
xC
l6
30751
xC
l6
183474
xC
l6
259318
xC
l6
270599
xC
l6
60431
xC
l6
259119
xC
l6
332066
xC
l6
7168
xC
l6
197632
xC
l6
291840
xC
l6
283648
xC
l6
238866
xC
l6
239902
xC
l6
31751
xC
l6
244743
xC
l6
384022
xC
SelectedParams,0}
,0,0x4
nM
385262
xC
l6
386086
xC
l6
393254
xC
SelectedParams,0}
,0,0x5
nM
393254
xC
l6
386095
xC
l6
387312
xC
l6
18662
xC
l6
61670
xC
l6
387397
xC
l6
247855
xC
SelectedParams,0}
,0,0x1
nM
342063
xC
l6
297007
xC
l6
15820
xC
l6
393263
xC
l6
393263
xC
SelectedParams,0}
,0,0x5
nM
161847
xC
l6
258103
xC
l6
249073
xC
l6
249076
xC
iF
0,0
xC
nL
0,0
tO1
1,45
xC
nL
1,53
xC
nL
1,54
xC
nL
1,55
xC
nL
1,56
xC
nL
1,26
xC
nL
1,259
tP
1}
,0,0x16
tL3
272
tO1
1,323
tP
1}
,0,0x16
tL3
0
xC
nL
1,21
xC
nL
1,447
tP
1}
nE2
449
tP
1}
nE2
0
tP
1}
nE2
0
tP
2}
nE2
15
xC
nL
1,24
tP
2}
,0,0x0
nM
58392
tO1
0,0
tP
1}
,n91,0x0
nM
24591
nC3
33807
nC3
48143
nC3
285720
nC3
290840
nC3
305152,lA
312400,lA
39202,lA
122918,lA
421926,lA
429094,lA
443430,lA
317834,lA
329098,lA
7633,lA
7706,lA
7730,lA
38,lA
50587,lA
406528,lA
24583,lA
31751,lA
405511,lA
321551,xH1
327713,lA
322596,lA
90409,lA
335174,lA
327050,lA
493606,lA
496678,lA
503846,lA
516134,lA
7217,lA
333875,lA
336896,lA
524326,lA
509952,lA
286727,lA
89103,lA
92175,lA
296976,tA1
324623,l1
0x14
nM
332815,l1
0x10}
,{{3,7340056,tA1
289092,lA
93200,xH1
337935
tF1
7340060,l1
t92
7340176,lA
338959
tF1
7340061,xH1
7206,lA
7168,lA
357414,lA
368678,lA
370745
eV3}
,{{3,7340177,lA
39277,tA1
426398,l1
t92
40272286,xH1
490910,l1
t92
40336798,xH1
50600,lA
426462,xH1
490974,xH1
370726,l1
0x6
nM
371750,l1
0x6
nM
428070
tF1
40336862,xH1
38378,lA
50671
tF1
47662080,lA
477184,lA
568320,lA
371727
eV3}
,{{3,15779306,lA
370703
eV3
nM
39277,lA
39279,l1
0x4}
,{{3,15779238,lA
39338,tA1
436262,lA
508966,lA
39409,tA1
296998,tA1
35847,lA
15,tA1
377894,lA
386063,l1
0x1
nM
15,lA
7192,lA
123928,lA
122904,lA
30751,lA
57,lA
7456,lA
15674
tF1
67579935,lA
39237,lA
58768,lA
62924,lA
122880,lA
15760
tF1
64009216,l1
0x0}
,{{0,0,xM
0,0,iM
2,cM1
2,cN1
3,cM1
3,cN1
38,xM
1,38,iM
14,xM
1,57,xM
1,16,eA2
0x0
nM
471103,eA2
0x1
tL3
303,xM
1,323,yF3
0x0
nM
471363,eA2
0x16
tL3
293,cM1
294,cN1
295,xM
1,296,iM
400,xM
1,0,xM
1,460,xM
1,465,xM
1,16,eA2
0x1
tL3
57,yF3
0x1
tL3
0,iM
21,xM
1,15,eA2
0x0
nM
24591,xM
1,24,iM
517,yF3
0x0
nM
46095,lL
46104,lL
15397,lL
287789,lL
66584,lL
404763,lL
62504,lL
15409,lL
39951,lL
24591,lL
33807,lL
50200,lL
62509,lL
50176,lG,178176,eS1
0x12
nM
283648,lG,19456,lG,27648,lG,91136,lG,86016,lG,488448,lG,14342,lG,58375,lG,46147
iX
46151,lG,284679,lG,7183,lG,46159
iX
38993
iX
50262,lG,50249,lG,283808,lG,284835,lG,24822,lG,10240,lG,11264,lG,7170,lG,7168,lG,17408,lG,164864,lG,237568,lG,242688,eS1
0x14
nM
476160,lG,25607,lG,122895,lG,50252,lG,39374,lG,50183,lG,7192,lG,122911,lG,252979,lG,46155,lG,38919,lG,50268,lG,50269,lG,50253,lG,46191,lG,50296,lG,7563,eS1
0x10
nM
416811,lG,416819,lG,40047,lG,46192
iX
415795,lG,40048
iX
415787,lG,39016,eS1
0x5
nM
39326
iX
39326,lG,39332,eS1
0x5
nM
39333,eS1
0x1
nM
50590
iX
50590,lG,39338
iX
39338,lG,39335,eS1
0x5
nM
15786
iX
146858,lG,39372,lG,39379,lG,39380,lG,39390
iX
50654
iX
50654,lG,24,eS1
0x6
nM
62,lG,24,lG,62,eS1
0x6
nM
43,lG,43
iX
51,lG,51
iX
50270,lG,50176
iX
50271,lG,39159,lG,39183
iX
7168
iX
31744,lG,100352,lG,31746,lG,101400,lG,39409
iX
39411
iX
39411,lG,39420,lG,39420
iX
15,lG,39026,eS1
0x5
nM
39422,lG,16384,lG,62853,lG,15360,lG,15,eS1
0x1
nM
16,lG,7183,eS1
0x1
nM
7172,cPow,y21,n91,0x0
nM
24591,cPow,lU
2,50200,cPow,lU
2,63521,cPow,lU
2,62500,cPow,lU
2,50453,cPow,lU
2,62488,cPow,lU
1,0,eC3
7,eC3
194,eC3
0,cAcos
tH3
cAcosh
tH3
cAsin
tH3
cAsinh
nW
120,cAsinh
tH3
cAtan,eB2
306176,cAtan2
iG2,cAtan2
tH3
cAtanh
nW
246,cCeil
tH3
cCeil,eV
1,0,c42
0,cCos,eV
1,7,c42
92,c42
93,c42
120,c42
236,c42
255,c42
214,iF2
236,iF2
464,iF2
0,cCosh,eV
1,0,iF2
0,eD3
7,eD3
92,eD3
0
yG3
7
yG3
92
yG3
246,cFloor
tH3
cFloor
xZ
309540,eE3
eB2
316708,eE3
eB2
316724,eE3
l0
3,32513024,eC2
34627584
eW
31493120,eC2
89213952
eW
149042176
eW
246647808
eW
301234176
eW
494360576
eW
498558976
eW
62933520
eW
62933520,eC2
62933526
eW
62933526,eC2
24670208
eW
579378176
eW
573578240
eW
32513024
eW
566254592
eW
7900160
eW
588822528,cIf
nW
120,cInt
nW
246,tA2
0,tA2
7,tA2
31,tA2
194,tA2
363,tA2
15,cLog,lU
1,24,cLog,lU
1,0,cLog10
tH3
cLog2
iG2,cMax,eB2
35847,cMax,eB2
30751,cMax
tH3
cMax,AnyParams,1}
,0,0x4
nM
7168,cMin,eB2
35847,cMin,eB2
30751,cMin
tH3
cMin,AnyParams,1}
,0,0x4
nM
24591,cMin,lU
1,0,nL2
7,nL2
92,nL2
93,nL2
120,nL2
149,nL2
231,cSin,lB
0x5
tL3
246,nL2
255,nL2
254,nL2
0,cSin,eV
1,273,cSin,lB
0x1
tL3
214
xW2
231,cSinh,lB
0x5
tL3
246
xW2
254
xW2
255
xW2
464
xW2
0,cSinh,eV
1,0
xW2
15,cSqrt,lU
1,0
c62
0,cTan,eV
1,116,cTan,eV
1,117
c62
231
c62
246
c62
273
c62
254
c62
255
c62
0,xX2
0,cTanh,eV
1,213,xX2
231,xX2
246,xX2
254,xX2
255,xX2
0,cTrunc,eB2
15384,cSub,lU
2,15384,cDiv,lU
2,476626,cDiv,lU
2,122937
x82
7168
i52
tB2
7168
x82
31744
i52,lB
0x20
nM
31751
i52,lB
0x24
nM
31751
x82
122937,tP1
iG2,cLess
tB2
41984,cLess
xZ
41984,cLess,eB2
7,cLess
iG2,cLessOrEq,eB2
296182,cLessOrEq
iG2
tC2
tB2
41984
tC2
xZ
41984
tC2,eB2
7
tC2
iG2
xO2,eB2
296182
xO2
nW
0
tB1
245
tB1
7
tB1
550
tB1
553
tB1
554
tB1
556
tB1
31
tB1
559
tB1
15
tB1
560,cNot,eB2
7706,lA3
7168,lA3
35847,lA3
30751,lA3
463903,lA3
466975,cAnd,iF
0,0,cAnd,nL
2,7168,cO3
7706,cO3
35847,cO3
463903,cO3
466975,cO3
30751,cOr,iF
1,0,lC2
92,lC2
131,lC2
245,lC2
215,lC2
246,cDeg
nW
246,cRad
iG2,cAbsAnd,l6
7168,cAbsOr,iF
1,0,c03
tH3
cAbsNotNot,l0
3,32513024,cAbsIf,lB
0x0}
,}
;}
iR2
l21{const
Rule
grammar_rules[262]={{ProduceNewTree,17,1,0,{1,0,cAbs
eD2
409,{1,146,cAtan
eD2
403
x9
1324,cAtan2
eD2
405
x9
307201,cAtan2
cA
253174
x9
255224,cAtan2
cA
259324
x9
257274,cAtan2
eD2
152,{1,252,cCeil
l3
2,1,486,{1,68,xI1
482,{1,123,xI1
483,{1,125,xI1
151,{1,126,xI1
419,{1,124,xI1
0,{1,403,cCos,l2
2,1,246,{1,252,cCos,l2
18,1,0,{1,400,xI1
301,{1,406,cCosh,l2
2,1,246,{1,252,cCosh,l2
18,1,0,{1,400,cCosh
l3
2,1,458,{1,122,cFloor
eD2
150,{1,252,cFloor
l3
0,1,156,{3,7382016,eI
549,{3,8430592,eI
556,{3,8436736,eI
157,{3,42998784,eI
550,{3,42999808,eI
562,{3,43039744,eI
557,{3,49291264,eI
538,{3,49325056,eI
469,{3,1058318,eI
473,{3,1058324,eI
473,{3,9438734,eI
469,{3,9438740,cIf,l2
0,3,32542225,{3,36732434,cIf,l2
0,3,32542231,{3,36732440,cIf
l3
16,1,573,{3,32513026,cIf
l3
16,1,515,{3,455505423,cIf
l3
16,1,515,{3,433506837,cIf
l3
2,1,78,{1,256,xY2
69,{1,258,xY2
404,{1,72,xY2
159,{1,147,cLog,l2
0,1,0
x9
487425,cMax,tX
16,1,445
x9
eF3
cMax,tX
0,1,0
x9
483329,cMin,tX
16,1,446
x9
eF3
cMin,cB
0,1,153
x9
24832,nM2
0,1,153
x9
25854,nM2
0,1,154
x9
130063,iI2
32055,iI2
32056,iI2
32057,iJ2
166288
x9
32137,iI2
33082,iJ2
7168
x9
12688,iJ2
7434
x9
12553
nY2
435
x9
46146
nY2
436
x9
46154
nY2
437
x9
46150
nY2
169
x9
83983
nY2
168
x9
131106
nY2
175
x9
133154
nZ2
476160
x9
471055
nZ2
274432
x9
273423
nZ2
251904
x9
266274
nZ2
251904
x9
263186
nY2
171,{1,252,lN1
421,{1,68,lN1
151,{1,123,lN1
419,{1,125,lN1
170,{1,126,lN1
482,{1,124,lN1
0,{1,405,lN1
172,{1,252,cSinh
l3
2,1,328,{1,404,cSinh
l3
2,1,173,{1,252,xZ2
0,{1,408,xZ2
176,{1,410,xZ2
177,{1,252,cTanh,l2
0,1,442
x9
449551
cP3
441
x9
450998
cP3
167
x9
268549
cP3
180
x9
276749
cP3
181
x9
276500
cQ3
190770
x9
189622
cQ3
194748
x9
193723
cQ3
202943
x9
196795
cQ3
59699
x9
298148
cQ3
59714
x9
325815
cQ3
59724
x9
343224
xC
cB
2,1,337,{1,333
tP
1
tT
336,{1,338
tP
1}
}
iH2
2,1,340
x9
1363
tS
342
x9
1365
tS
463
x9
472524
tS
47
x9
356711
tS
349
x9
200751
tS
360
x9
199727
tS
480
x9
207053
tS
481
x9
208077
tS
417
x9
211144
tS
209
x9
211145
tS
418
x9
215240
tS
212
x9
212329
tS
204
x9
373097
tS
211
x9
372944
tS
217
x9
201944
tS
221
x9
223448
tS
367
x9
508329
tS
219
x9
508126
tS
224
x9
225705
tS
223
x9
225776
tS
365
x9
230825
tS
426
x9
377057
tS
497
x9
377054
tS
497
x9
204201
tS
426
x9
375280
tS
224
x9
375006,l7
2,2,407781
x9
233698,l7
2,2,59763
x9
233842
cP3
372
x9
1397,c72
96
x9
24705,c72
97
x9
24708,c72
444
x9
449551,c72
443
x9
eF3
c72
101
x9
102774,c72
109
x9
107845,c72
106
x9
104773,l5
0,2,111631
x9
109893,l5
0,2,108559
x9
110917,lK
0
tT
113
x9
112658,cMul,SelectedParams,0
tT
567,{1,52,lK
1
tT
568,{1,42,lK
1}
}
iH2
2,1,467
x9
45516
i51
356
x9
51555
i51
468
x9
49612
i51
357
x9
47459
i51
429
x9
438699
i51
432
x9
441774
i51
486
x9
498726
i51
494
x9
504870
i51
382
x9
435579
i51
497
x9
435709
i51
426
x9
508287
i51
414
x9
500092
i51
499
x9
352744
i51
345
x9
367092
i51
381
x9
425318
i51
478
x9
425460
i51
47
x9
512501
i51
505
x9
355817
i51
47
x9
516598
i51
507
x9
518182
i51
508
x9
358896
i51
351
x9
388605
i51
511
x9
360939
i51
503
x9
354788
i51
514
x9
525350
i51
510
x9
394343
i51
386
x9
351347,l5
2,2,363004
x9
361968,l5
16,1,118
x9
1157,l5
16,1,119
x9
1158,l5
16,1,402
x9
411024,l5
16,2,58768
x9
1472,l5
16,2,15760
x9
1474,l5
17,1,0,{1,400,l5
17,1,57,{1,14,lK
0}
}
,{ProduceNewTree,4,1,538
x9
41
i52
l3
4,1,0
x9
5167
i52
c9
41984
x9
409641
i52
c9
tY
cEqual
c9
t0
cEqual
c9
t1
cEqual
xL1
24849
i52
cA
tZ
cEqual
cA
lD2
281873
i52
cA
lA1
i52
cA
l61
cEqual
l3
4,1,562
x9
41,tP1
l3
4,1,538
x9
5167,tP1
c9
41984
x9
409641,tP1
c9
tY
tP1
c9
t0
tP1
c9
t1
tP1
xL1
24849,tP1
cA
tZ
tP1
cA
lD2
281873,tP1
cA
lA1,tP1
cA
l61
tP1
c9
tY
yH3
t0
yH3
t1
cLess
eD2
571
x9
46080,cLess
xL1
24832,cLess
cA
xM1
cLess
cA
tZ
cLess
cA
lD2
281856,cLess
cA
nP1
cLess
cA
lA1,cLess
cA
l61
cLess
l3
20,1,562
x9
409641,yH3
tY
y02
t0
y02
t1
cLessOrEq
eD2
565
x9
409615,cLessOrEq
xL1
24832,cLessOrEq
cA
xM1
cLessOrEq
cA
tZ
cLessOrEq
cA
lD2
281856,cLessOrEq
cA
nP1
cLessOrEq
cA
lA1,cLessOrEq
cA
l61
cLessOrEq
l3
20,1,562
x9
409647,y02
tY
c82
t0
c82
t1
cGreater
eD2
539
x9
409615
tC2
xL1
24832
tC2
cA
xM1
cGreater
cA
tZ
cGreater
cA
lD2
281856
tC2
cA
nP1
cGreater
cA
lA1
tC2
cA
l61
cGreater
l3
20,1,538
x9
409647,c82
tY
cGreaterOrEq
c9
t0
cGreaterOrEq
c9
t1
cGreaterOrEq
eD2
572
x9
46080
xO2
xL1
24832
xO2
cA
xM1
cGreaterOrEq
cA
tZ
cGreaterOrEq
cA
lD2
281856
xO2
cA
nP1
cGreaterOrEq
cA
lA1
xO2
cA
l61
cGreaterOrEq
l3
20,1,538
x9
409641
xO2
l3
4,1,519,{1,137,cNot
l3
16,1,571,{1,2,cNot,l2
0,1,452
x9
eF3
xF
0,2,537097,{3,547892744,cAnd,cB
16,1,566,{1,5,cAnd,AnyParams,1}
}
iH2
16,1,569
x9
13314,xF
16,1,544
x9
553498,xF
16,1,546
x9
462369,xF
16,1,548
x9
466465,xF
0,1,457
x9
eF3
nH
570
x9
13314,nH
563
x9
8197,nH
541
x9
553498,nH
542
x9
462369,nH
543
x9
466465,nH
564
x9
143365,cOr,cB
4,1,525,{1,137,cNotNot
l3
16,1,572,{1,2,cNotNot
l3
17,1,0,{1,0,cNotNot
eD2
537,{1,256,cAbsNotNot,cB
18,1,531,{1,254,cAbsNotNot,cB
0,1,572,{3,43039744,cAbsIf
l3
0,1,571,{3,49325056,cAbsIf
l3
16,1,454,{3,32513586,cAbsIf,l2
16,3,32542225,{3,36732434,cAbsIf,y21}
,}
;cP2
grammar_optimize_abslogical_type{y5
9
cV
grammar_optimize_abslogical_type
grammar_optimize_abslogical={9,{34,192,228,238,242,247,254,260,261}
}
;}
cP2
grammar_optimize_ignore_if_sideeffects_type{y5
59
cV
grammar_optimize_ignore_if_sideeffects_type
grammar_optimize_ignore_if_sideeffects={59,{0,20,21,22,23,24,25,26,cS
iE1
78,cT
cW
cP2
grammar_optimize_nonshortcut_logical_evaluation_type{y5
56
cV
grammar_optimize_nonshortcut_logical_evaluation_type
grammar_optimize_nonshortcut_logical_evaluation={56,{0,25,cS
iE1
78,cT
241,243,244,245,246,248,249,250,251,252,253,255,256,257,258,259}
}
;}
cP2
grammar_optimize_recreate_type{y5
22
cV
grammar_optimize_recreate_type
grammar_optimize_recreate={22,{18,55,56,57,80,81,82,83,84,85,117,118,120,121,130,131,132,133,134,135,136,137}
}
;}
cP2
grammar_optimize_round1_type{y5
125
cV
grammar_optimize_round1_type
grammar_optimize_round1={125,{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,19,25,cS
37,38,iE1
45,46,47,48,49,50,51,52,53,54,58,59,60,61,62,63,64,65,66,67,68,69,70,71,78,79,80,81,82,83,84,85,86,87,88,93,94,95,96,97,98,99,100,101,117,118,119,120,121,122,123,124,125,126,127,128,129,138,160,161,162,163,164,165,166,167,168,169,178,179,180,200,204,212,216,224,236,237,239,240,cW
cP2
grammar_optimize_round2_type{y5
103
cV
grammar_optimize_round2_type
grammar_optimize_round2={103,{0,15,16,17,25,cS
39,40,iE1
45,46,47,48,49,50,51,52,53,54,59,60,72,73,78,79,86,87,88,89,90,91,92,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,119,122,123,124,125,126,127,128,139,159,160,161,162,163,164,165,166,167,168,169,178,179,180,200,204,212,216,224,236,237,239,240,cW
cP2
grammar_optimize_round3_type{y5
79
cV
grammar_optimize_round3_type
grammar_optimize_round3={79,{74,75,76,77,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,170,171,172,173,174,175,176,177,181,182,183,184,185,186,187,188,189,190,191,193,194,195,196,197,198,199,201,202,203,205,206,207,208,209,210,211,213,214,215,217,218,219,220,221,222,223,225,226,227,229,230,231,232,233,234,235}
}
;}
cP2
grammar_optimize_round4_type{y5
12
cV
grammar_optimize_round4_type
grammar_optimize_round4={12,{18,55,56,57,130,131,132,133,134,135,136,137}
}
;}
cP2
grammar_optimize_shortcut_logical_evaluation_type{y5
53
cV
grammar_optimize_shortcut_logical_evaluation_type
grammar_optimize_shortcut_logical_evaluation={53,{0,25,cS
iE1
78,cT
cW}
iR2
l21{yD
cS2
cR1
nD2
paramlist,lB1){index=(paramlist>>(index*10))&1023;if(index>=57)return
cS2(SubFunction,c92
plist_s[index-57]);if(index>=37)return
cS2(NumConstant,c92
plist_n_container
yB::plist_n[index-37])eO
cS2(ParamHolder,c92
plist_p[index]);}
}
#ifdef FP_SUPPORT_OPTIMIZER
#include <stdio.h>
#include <algorithm>
#include <map>
#include <sstream>
using
iR2
FUNCTIONPARSERTYPES;using
iR2
l21;using
tC;using
cV1;iR2{nH1
It,typename
T,typename
Comp>eT1
MyEqualRange(It
first,It
last,const
T&val,Comp
comp){size_t
len=last-first;while(len>0){size_t
xH3
len/2;It
nQ3(first);nQ3+=half;if(comp(*nQ3,val)){first=nQ3;++first;len=len-half-1;}
i81
comp(val,*nQ3)){len=half;}
else{It
left(first);{It&eL2=left;It
last2(nQ3);size_t
len2=last2-eL2;while(len2>0){size_t
half2=len2/2;It
middle2(eL2);middle2+=half2;if(comp(*middle2,val)){eL2=middle2;++eL2;len2=len2-half2-1;}
else
len2=half2;}
}
first+=len;It
right(++nQ3);{It&eL2=right;It&last2=first;size_t
len2=last2-eL2;while(len2>0){size_t
half2=len2/2;It
middle2(eL2);middle2+=half2;if(comp(val,*middle2))len2=half2;else{eL2=middle2;++eL2;len2=len2-half2-1;}
}
}
return
eT1(left,right);}
}
return
eT1(first,first);}
yD
cP2
OpcodeRuleCompare{bool
eD1()iX2
tree,unsigned
y12)const{const
Rule&rule=grammar_rules[y12]eO
tree
nF<rule
cB2.subfunc_opcode;}
bool
eD1()nD2
y12,xB3)const{const
Rule&rule=grammar_rules[y12]eO
rule
cB2.subfunc_opcode<tree
nF;}
}
;yD
bool
TestRuleAndApplyIfMatch
cY1
eU2
xG&tree,bool
cC{MatchInfo
yB
info;lW1
found(false,e3());if((rule.l71
LogicalContextOnly)&&!cC{tC1
if(nE
IsIntType
yB::nT3
cI3
rule.l71
NotForIntegers)tC1
eN3
rule.l71
OnlyForIntegers)tC1
if(nE
IsComplexType
yB::nT3
cI3
rule.l71
NotForComplex)tC1
eN3
rule.l71
OnlyForComplex)tC1
for(;;){
#ifdef DEBUG_SUBSTITUTIONS
#endif
found=TestParams(rule
cB2,tree,found.specs,info,true);if(found.found)break;if(found.specs.isnull()){fail:;
#ifdef DEBUG_SUBSTITUTIONS
DumpMatch
cA2,false);
#endif
return
e83}
#ifdef DEBUG_SUBSTITUTIONS
DumpMatch
cA2,true);
#endif
SynthesizeRule
cA2)cT2}
cV1{yD
bool
ApplyGrammar
cY1
Grammar&tD2,xG&tree,bool
cC{if(tree.GetOptimizedUsing()==&tD2){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Already optimized:  "
yR2
tree)xO1"\n"
<<std::flush;
#endif
return
e83
if(true){bool
changed
e23
switch
l83{case
cNot:case
cNotNot:case
cAnd:case
cOr:n72
0;lB3
true))nS2
lD
cIf:case
cAbsIf:if(ApplyGrammar(tD2,tQ,i21
cIf))nS2
n72
1;lB3
cC)nS2
break;y13
n72
0;lB3
false))nS2}
if(changed){tree.Mark_Incompletely_Hashed()cT2}
typedef
const
unsigned
short*lG3;std::pair<lG3,lG3>range=MyEqualRange(tD2.rule_list,tD2.rule_list+tD2.rule_count,tree,OpcodeRuleCompare
yB())e72
unsigned
short>rules;rules.xD3
range.second-range.first);cX
if(IsLogisticallyPlausibleParamsMatch(e91
cB2,tree))rules
yS2*r);}
range.first=!rules
yU3?&rules[0]:0;range.second=!rules
yU3?&rules[rules.size()-1]+1:0;if(range.first!=range.second){
#ifdef DEBUG_SUBSTITUTIONS
if(range.first!=range.second)lC3"Input ("
<<eK3
l83<<")["
<<t7<<"]"
;if(cC
std::cout<<"(Logical)"
;unsigned
first=iF1,prev=iF1;nA3
sep=", rules "
;cX
if(first==iF1)first=prev=*r;i81*r==prev+1)prev=*r;else
lC3
sep<<first;sep=","
;if(prev!=first)std::cout<<'-'<<prev;first=prev=*r;}
}
if(first!=iF1)lC3
sep<<first;if(prev!=first)std::cout<<'-'<<prev;}
std::cout<<": "
yR2
tree)xO1"\n"
<<std::flush;}
#endif
bool
changed
e23
cX
#ifndef DEBUG_SUBSTITUTIONS
if(!IsLogisticallyPlausibleParamsMatch(e91
cB2,tree))y41
#endif
if(TestRuleAndApplyIfMatch(e91,tree,cC){nS2
yN3
if(changed){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Changed."
<<std::endl
xO1"Output: "
yR2
tree)xO1"\n"
<<std::flush;
#endif
tree.Mark_Incompletely_Hashed()cT2}
tree.SetOptimizedUsing(&tD2)eO
e83
yD
bool
ApplyGrammar
cY1
void*p,FPoptimizer_CodeTree::cV2
yS
ApplyGrammar(*cY1
Grammar*)p,tree);}
x73
ApplyGrammars(FPoptimizer_CodeTree::cV2{
#ifdef DEBUG_SUBSTITUTIONS
std
t83"grammar_optimize_round1\n"
;
#endif
n6
grammar_optimize_round1
y22
#ifdef DEBUG_SUBSTITUTIONS
std
t83"grammar_optimize_round2\n"
;
#endif
n6
grammar_optimize_round2
y22
#ifdef DEBUG_SUBSTITUTIONS
std
t83"grammar_optimize_round3\n"
;
#endif
n6
grammar_optimize_round3
y22
#ifndef FP_ENABLE_SHORTCUT_LOGICAL_EVALUATION
#ifdef DEBUG_SUBSTITUTIONS
std
t83"grammar_optimize_nonshortcut_logical_evaluation\n"
;
#endif
n6
grammar_optimize_nonshortcut_logical_evaluation
y22
#endif
#ifdef DEBUG_SUBSTITUTIONS
std
t83"grammar_optimize_round4\n"
;
#endif
n6
grammar_optimize_round4
y22
#ifdef FP_ENABLE_SHORTCUT_LOGICAL_EVALUATION
#ifdef DEBUG_SUBSTITUTIONS
std
t83"grammar_optimize_shortcut_logical_evaluation\n"
;
#endif
n6
grammar_optimize_shortcut_logical_evaluation
y22
#endif
#ifdef FP_ENABLE_IGNORE_IF_SIDEEFFECTS
#ifdef DEBUG_SUBSTITUTIONS
std
t83"grammar_optimize_ignore_if_sideeffects\n"
;
#endif
n6
grammar_optimize_ignore_if_sideeffects
y22
#endif
#ifdef DEBUG_SUBSTITUTIONS
std
t83"grammar_optimize_abslogical\n"
;
#endif
n6
grammar_optimize_abslogical
y22
#undef C
}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
#include <algorithm>
#include <assert.h>
#include <cstring>
#include <cmath>
#include <memory> /* for auto_ptr */
using
iR2
FUNCTIONPARSERTYPES;using
iR2
l21;using
tC;using
cV1;iR2{yD
bool
TestImmedConstraints
nD2
bitmask,xB3){switch(bitmask&ValueMask
e53
Value_AnyNum:case
ValueMask:lD
nK2:if(GetEvennessInfo
cK3
lE2
Value_OddInt:if(GetEvennessInfo
cK3
nN2
tM1:if(GetIntegerInfo
cK3
lE2
Value_NonInteger:if(GetIntegerInfo
cK3
nN2
eQ1:if(!IsLogicalValue(tree)c5
nA1
SignMask
e53
Sign_AnySign:lD
n91:if(l01
lE2
eR1:if(l01
nN2
Sign_NoIdea:if(l01
Unknown
c5
nA1
OnenessMask
e53
Oneness_Any:case
OnenessMask:lD
Oneness_One:if(!e01
if(!fp_equal(fp_abs(tree.xE1),l62(1))c5
lD
Oneness_NotOne:if(!e01
if(fp_equal(fp_abs(tree.xE1),l62(1))c5
nA1
ConstnessMask
e53
Constness_Any:lD
tL1:if(!e01
lD
Constness_NotConst:if(e01
break
yC3
true;}
t91
unsigned
extent,unsigned
nbits,typename
eM2=unsigned
int>cP2
nbitmap{private:static
const
unsigned
bits_in_char=8;static
const
unsigned
eN2=(yI3
eM2)*bits_in_char)/nbits;eM2
data[(extent+eN2-1)/eN2];cV3
void
inc(lB1,int
by=1){data[pos(index)]+=by*eM2(1<<y32);x61
void
dec(lB1){inc(index,-1);}
int
get(lB1
yK1(data[pos(index)]>>y32)&mask()lD3
pos(lB1)yS
index/eN2
lD3
shift(lB1)yS
nbits*(index%eN2)lD3
mask()yS(1<<nbits)-1
lD3
mask(lB1)yS
mask()<<y32;}
}
;cP2
c93{int
SubTrees:8;int
yB3:8;int
y42:8;int
Immeds:8;nbitmap<iE2,2>SubTreesDetail;c93(){std::memset(this,0,yI3*this));}
c93
cY1
c93&b){std::memcpy(this,&b,yI3
b));}
c93&eD1=cY1
c93&b){std::memcpy(this,&b,yI3
b))eO*this;}
}
;yD
c93
CreateNeedList_uncached(eP&tE2{c93
cU1;for
nD2
a=0;a<e13
y52;++a){const
cS2&x02=cR1
yB(e13.param_list,a);switch(x02.first
e53
SubFunction:{cR
y63(cR*e62;yN
GroupFunction)++cU1.Immeds;else{++cU1.SubTrees;assert(param.data.subfunc_opcode<VarBegin);cU1.SubTreesDetail.inc
y83
l51);}
++cU1.y42;break
cT1
NumConstant:case
ParamHolder:++cU1.yB3;++cU1.y42;yN3
return
cU1;}
yD
c93&CreateNeedList(eP&tE2{typedef
std::map<eP*,c93>cO1;static
cO1
yF1;cO1::xR3
i=yF1.xI2&tE2;if(i!=yF1.cH1&tE2
return
i
cW2
eO
yF1.xN3,std::make_pair(&e13,CreateNeedList_uncached
yB(tE2))cW2;}
yD
xG
CalculateGroupFunction
cY1
cS2&x02,const
y1
info){switch(x02.first
e53
NumConstant:{const
ParamSpec_NumConstant
yB
y63
cY1
ParamSpec_NumConstant
yB*e62
eO
CodeTreeImmed
y83
constvalue)cT1
ParamHolder:{cQ
y63(cQ*e62
eO
cS3
GetParamHolderValueIfFound
y83
index)cT1
SubFunction:{cR
y63(cR*e62;xG
nT3;nT3
c4
yA3
l51);tX2
iP1).xD3
yA3
data
y52);for
nD2
a=0;a<yA3
data
y52;++a
nQ
tmp(CalculateGroupFunction(cR1
yB
y83
data.param_list,a),info));nT3
y7
tmp);}
tX2
Rehash()eO
nT3;}
}
return
xG();}
}
cV1{yD
bool
IsLogisticallyPlausibleParamsMatch(eP&e13,xB3){c93
cU1(CreateNeedList
yB(tE2);size_t
eG3=t7;if(eG3<size_t(cU1.y42))tF2
n72
0;a<eG3;++a){unsigned
opcode=xP1
a)nF;switch(opcode
e53
cImmed:if(cU1.Immeds>0)cR3
Immeds;else
cR3
yB3;lD
iE2:case
cFCall:case
cPCall:cR3
yB3;break;y13
assert(opcode<VarBegin);if(cU1.SubTrees>0&&cU1.SubTreesDetail.get(opcode)>0){cR3
SubTrees;cU1.SubTreesDetail.dec(opcode);}
else
cR3
yB3;}
}
if(cU1.Immeds>0||cU1.SubTrees>0||cU1.yB3>0)tF2
if(e13.match_type!=AnyParams
cI3
0||cU1.SubTrees<0||cU1.yB3<0)tF2}
return
true;}
yD
lW1
TestParam
cY1
cS2&x02,xB3
n43
start_at,y1
info){switch(x02.first
e53
NumConstant:{const
ParamSpec_NumConstant
yB
y63
cY1
ParamSpec_NumConstant
yB*e62;if(!e01
l62
imm=tree.xE1;switch
y83
modulo
e53
Modulo_None:lD
Modulo_Radians:imm=fp_mod(imm,yG
imm<tD1))imm
yQ
if(imm>fp_const_pi
yB())imm-=fp_const_twopi
yB(eP3
yC3
fp_equal(imm,yA3
constvalue)cT1
ParamHolder:{cQ
y63(cQ*e62;if(!x5
return
cS3
SaveOrTestParamHolder
y83
index,tree)cT1
SubFunction:{cR
y63(cR*e62;yN
GroupFunction
cI3!x5
xG
xN1=CalculateGroupFunction(x02,info);
#ifdef DEBUG_SUBSTITUTIONS
DumpHashes(xN1)xO1*cY1
void**)&xN1.xE1
xO1"\n"
xO1*cY1
void**)&tree.xE1
xO1"\n"
;DumpHashes(tree)xO1"Comparing "
yR2
xN1)xO1" and "
yR2
tree)xO1": "
xO1(xN1
xI
tree)?"true"
:"false"
)xO1"\n"
;
#endif
return
xN1
xI
tree);}
eN3
start_at.isnull()cI3!x5
if(tree
nF!=yA3
l51
c5}
return
TestParams
y83
data,tree,start_at,info,false);}
}
}
return
e83
yD
cP2
iR
x92
MatchInfo
yB
info;iR(y62,info(){}
}
;x13
MatchPositionSpec_PositionalParams:eN1
iR
yB>{cV3
iK2
MatchPositionSpec_PositionalParams
x72
n
yN1
iR
yB>(n){}
}
;cP2
iG1
x92
iG1(y62{}
}
;class
yO:eN1
iG1>{cV3
unsigned
trypos;iK2
yO
x72
n
yN1
iG1>(n),trypos(0){}
}
;yD
lW1
TestParam_AnyWhere
cY1
cS2&x02,xB3
n43
start_at,y1
info,std::vector<bool>&used,bool
y82{xQ<yO>xB;unsigned
a;if(!start_at.isnull()){xB=(yO
eE2
a=xB->trypos;goto
retry_anywhere_2;}
tG2
yO(t7);a=0;}
for(tH2{if(used[a])y41
retry_anywhere
yJ3
TestParam(x02,tJ3(eF2);(eG2
used[a]=true;if(y82
cS3
SaveMatchedParamIndex(a);xB->trypos=a
eO
lW1(true,xB.get());}
}
retry_anywhere_2
yK3
goto
retry_anywhere;}
}
return
e83
yD
cP2
y61
x92
MatchInfo
yB
info
e72
bool>used;iK2
y61
x72
eG3
y62,info(),used(eG3){}
}
;x13
MatchPositionSpec_AnyParams:eN1
y61
yB>{cV3
iK2
MatchPositionSpec_AnyParams
x72
n,size_t
m
yN1
y61
yB>(n,y61
yB(m)){}
}
;yD
lW1
TestParams(eP&nT,xB3
n43
start_at,y1
info,bool
y82{if(nT.match_type!=AnyParams
cI3
c2!=t7
c5}
if(!IsLogisticallyPlausibleParamsMatch(nT,tree))tF2
switch(nT.match_type
e53
PositionalParams:{xQ<yA>xB;unsigned
a;if(start_at.get()){xB=(yA
eE2
a=c2-1;goto
l81;}
tG2
yA(c2);a=0;}
for(;a<c2;++a){(*xB)[a
y72
retry_positionalparams
yJ3
TestParam(cZ
a),tJ3(eF2);(eG2
y41}
}
l81
yK3
eH2
a].info;goto
retry_positionalparams;}
yL3--a;goto
l81;}
eH2
0].info
eO
e83
if(y82
for
lR1
cS3
SaveMatchedParamIndex(a)eO
lW1(true,xB.get())cT1
SelectedParams:case
AnyParams:{xQ<tD>xB
e72
bool>used(t7)e72
unsigned>iL2(c2)e72
unsigned>y92(c2)tE3{const
cS2
x02=cZ
a);iL2[a]=ParamSpec_GetDepCode(x02);}
{unsigned
b=0
tE3
if(iL2[a]!=0)y92[b++]=a
tE3
if(iL2[a]==0)y92[b++]=a;}
unsigned
a;if(start_at.get()){xB=(tD
eE2
if(c2==0){a=0;goto
retry_anyparams_4;}
a=c2-1;goto
cP1;}
tG2
tD(c2,t7);a=0;if(c2!=0){(*xB)[0
y72(*xB)[0].used=used;}
}
for(;a<c2;++a){yL3(*xB)[a
y72(*xB)[a].used=used;}
retry_anyparams
yJ3
TestParam_AnyWhere
yB(cZ
y92[a]),tree,(eF2,used,y82;(eG2
y41}
}
cP1
yK3
eH2
a].info;used=(*xB)[a].used;goto
retry_anyparams;}
cQ1:yL3--a;goto
cP1;}
eH2
0].info
eO
e83
retry_anyparams_4:if(nT.n0!=0
cI3!TopLevel||!cS3
HasRestHolder(nT.n0)){nX2
cC2;cC2.xD3
t7);for
nD2
b=0;b<t7;++b
cI3
yM3)y41
cC2
yS2
xP1
b));yM3=true;if(y82
cS3
SaveMatchedParamIndex(b);}
if(!cS3
SaveOrTestRestHolder(nT.n0,cC2)){goto
cQ1;}
}
else{x43
cC2=cS3
GetRestHolderValues(nT.n0);n72
0;a<cC2.size();++a){bool
found
e23
for
nD2
b=0;b<t7;++b
cI3
yM3)y41
if(cC2[a]xI
xP1
b))){yM3=true;if(y82
cS3
SaveMatchedParamIndex(b);found=true;yN3
if(!found){goto
cQ1;}
}
}
}
return
lW1(true,c2?xB.get():0)cT1
GroupFunction:break
yC3
e83}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
#include <algorithm>
#include <assert.h>
using
tC;using
cV1;iR2{yD
xG
xR1
const
cS2&x02,y1
info,bool
inner=true){switch(x02.first
e53
NumConstant:{const
ParamSpec_NumConstant
yB
y63
cY1
ParamSpec_NumConstant
yB*e62
eO
CodeTreeImmed
y83
constvalue)cT1
ParamHolder:{cQ
y63(cQ*e62
eO
cS3
GetParamHolderValue
y83
index)cT1
SubFunction:{cR
y63(cR*e62;xG
tree;tree
c4
yA3
l51);for
nD2
a=0;a<yA3
data
y52;++a
nQ
nparam=xR1
cR1
yB
y83
data.param_list,a),info,true)e12
nparam);}
l93
data.n0!=0){nX2
trees(cS3
GetRestHolderValues
y83
data.n0));tree.AddParamsMove(trees);if(t7==1){assert(tree tC3()==cAdd||tree tC3()==cMul||tree tC3()==cMin||tree tC3()==cMax||tree tC3()==cAnd||tree tC3()==cOr||tree tC3()==cAbsAnd||tree tC3()==cAbsOr);tree
iD2
tQ);}
i81
t7==0){switch
l83{case
cAdd:case
cOr
e93=n31
0));lD
cMul:case
cAnd
e93=n31
1));y13
yN3}
if(inner)tree.Rehash()eO
tree;}
}
return
xG();}
}
cV1{x73
SynthesizeRule
cY1
eU2
xG&tree,y1
info){switch(rule.ruletype
e53
ProduceNewTree:{tree
iD2
xR1
cR1
l91
0),info,false)eP3
cT1
ReplaceParams:y13{std::vector<unsigned>list=cS3
GetMatchedParamIndexes();std::sort(list.iC2
list.end());n72
list.size();a-->0;)tree.DelParam(list[a]);for
nD2
a=0;a<rule.repl_param_count;++a
nQ
nparam=xR1
cR1
l91
a),info,true)e12
nparam);}
yN3}
}
#endif
#ifdef DEBUG_SUBSTITUTIONS
#include <sstream>
#include <cstring>
using
iR2
FUNCTIONPARSERTYPES;using
iR2
l21;using
tC;using
cV1;iR2
l21{x73
DumpMatch
cY1
eU2
xB3,const
y1
info,bool
DidMatch,std::ostream&o){DumpMatch
cA2,DidMatch?tF3"match"
:tF3"mismatch"
,o);}
x73
DumpMatch
cY1
eU2
xB3,const
y1
info,nA3
eH3,std::ostream&o){static
const
char
ParamHolderNames[][2]={"%"
,"&"
,"x"
,"y"
,"z"
,"a"
,"b"
,"c"
}
;o<<eH3<<" (rule "
<<(&rule-grammar_rules)<<")"
<<":\n  Pattern    : "
;{cS2
tmp;tmp.first=SubFunction;ParamSpec_SubFunction
tmp2;tmp2.data=rule
cB2;tmp.second=c92
tmp2;DumpParam
yB(tmp,o);}
o<<"\n  Replacement: "
;DumpParams
l91
rule.repl_param_count
eI2
o<<"  Tree       : "
yR2
tree
eI2
if(!std::strcmp(eH3,tF3"match"
))DumpHashes(tree,o);n72
0;a<cS3
yL.size();++a
cI3!cS3
yL[a].n12)y41
o<<"           "
<<ParamHolderNames[a]<<" = "
yR2
cS3
yL[a]eI2}
xS3
cS3
lR.size();++b
cI3!cS3
lR[b
e03)y41
n72
0;a<cS3
lR[b].second.size();++a){o<<"         <"
<<b<<"> = "
yR2
cS3
lR[b].second[a],o);o<<std::endl;}
}
o<<std::flush;}
}
#endif
#include <list>
#include <algorithm>
#ifdef FP_SUPPORT_OPTIMIZER
using
iR2
FUNCTIONPARSERTYPES;iR2{yD
bool
MarkIncompletes(FPoptimizer_CodeTree::cV2{if(tree.Is_Incompletely_Hashed(t41;bool
iH1
e23
n72
0
tH2
iH1|=MarkIncompletes
tN3);if(iH1)tree.Mark_Incompletely_Hashed()eO
iH1;}
x73
FixIncompletes(FPoptimizer_CodeTree::cV2{if(tree.Is_Incompletely_Hashed()){n72
0
tH2
FixIncompletes
tN3);tree
eM1}
}
}
tC{lC
Sort()tI3
Sort();}
lC
Rehash(bool
constantfolding
cI3
constantfolding)ConstantFolding(*this);else
Sort();data
xE
yD
cP2
cG
yO3
l62
yP3
cB1=0;
#if 0
long
double
value=Value;eC=crc32::calc(cY1
unsigned
char*)&value,yI3
value));key^=(key<<24);
#elif 0
union{cP2{unsigned
char
filler1[16];l62
v;unsigned
char
filler2[16];}
buf2;cP2{unsigned
char
filler3[yI3
l62)+16-yI3
x01)];eC;}
buf1;}
data;memset(&data,0,yI3
data));data.buf2.v=Value;eC=data.buf1.key;
#else
int
nU3;l62
lW2=std::frexp(Value,&x32;eC=nD2(nU3+0x8000)&0xFFFF);if(lW2<0){lW2=-lW2;key=key^0xFFFF;}
else
key+=0x10000;lW2-=l62(0.5);key<<=39;key|=lY1(lW2+lW2)*l62(1u<<31))<<8;
#endif
lQ
#ifdef FP_SUPPORT_COMPLEX_NUMBERS
nH1
T
cD2
std::complex<T> >yO3
std::complex<T>yP3
cG<T>::lH3
NewHash,Value.real());nE
fphash_t
temp;cG<T>::lH3
temp,Value.imag());cB1^=temp.hash2;NewHash.hash2^=temp.hash1;}
}
;
#endif
#ifdef FP_SUPPORT_LONG_INT_TYPE
t91
cD2
long>yK
long
Value){eC=Value;lQ
#endif
#ifdef FP_SUPPORT_GMP_INT_TYPE
t91
cD2
GmpInt>yO3
GmpInt
yP3
eC=Value.toInt();lQ
#endif
x73
x42
yB::Recalculate_Hash_NoRecursion(){fphash_t
NewHash(lY1
Opcode)<<56,Opcode*t23(0x1131462E270012B));Depth=1;switch(Opcode
e53
cImmed:{cG
yB::lH3
NewHash,Value
eP3
cT1
iE2:{cB1|=lY1
c51<<48
cA1((lY1
c51)*11)^t23(0x3A83A83A83A83A0);break
cT1
cFCall:case
cPCall:{cB1|=lY1
c51<<48
cA1((~lY1
c51)*7)^3456789;}
y13{size_t
eU1=0;n72
0;a<y03;++a
cI3
x93.xT2>eU1)eU1=x93.xT2;cB1+=((x93
n02
hash1*(a+1))>>12)cA1
x93
n02
hash1
cA1(3)*t23(0x9ABCD801357);NewHash.hash2*=t23(0xECADB912345)cA1(~x93
n02
hash2)^4567890;}
Depth+=eU1;}
}
if(Hash!=NewHash){Hash=NewHash;iS1=0;}
}
lC
FixIncompleteHashes(){MarkIncompletes(*this);FixIncompletes(*this);}
}
#endif
#include <cmath>
#include <list>
#include <cassert>
#ifdef FP_SUPPORT_OPTIMIZER
using
iR2
FUNCTIONPARSERTYPES;iR2{using
tC;yD
bool
nQ1
xB3,long
count,const
x51::SequenceOpCode
yB&eR,x51
yZ3&synth,size_t
max_bytecode_grow_length);static
const
cP2
SinCosTanDataType{OPCODE
whichopcode;OPCODE
inverse_opcode;enum{nominator,lX2,inverse_nominator,lC1}
;OPCODE
codes[4];}
SinCosTanData[12]={{cTan,cCot,{cSin,cCos,cCsc,cSec}
}
,{cCot,cCot,{cCos,cSin,cSec,cCsc}
}
,{cCos,cSec,{cSin,cTan,cCsc,cCot}
}
,{cSec,cCos,{cTan,cSin,cCot,cCsc}
}
,{cSin,cCsc,{cCos,cCot,cSec,cTan}
}
,{cCsc,cSin,{cCot,cCos,cTan,cSec}
}
,{cE2{cSinh,cCosh,cF2,{cSinh,cNop,{cE2
cNop,cCosh}
}
,{cCosh,cNop,{cSinh,cE2
cNop}
}
,{cNop,cTanh,{cCosh,cSinh,cF2,{cNop,cSinh,{cNop,cTanh,cCosh,cNop}
}
,{cNop,cCosh,{cTanh,cSinh,cF2}
;}
tC{lC
SynthesizeByteCode(std::vector<unsigned>&eE1,std::vector
yB&Immed,size_t&stacktop_max){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Making bytecode for:\n"
;iQ
#endif
while(RecreateInversionsAndNegations()){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"One change issued, produced:\n"
;iQ
#endif
FixIncompleteHashes();using
cV1;using
iR2
l21;const
void*g=c92
grammar_optimize_recreate;while(ApplyGrammar(*cY1
Grammar*)g,*this)){FixIncompleteHashes();}
}
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Actually synthesizing, after recreating inv/neg:\n"
;iQ
#endif
x51
yZ3
synth;SynthesizeByteCode(synth,false
i62.Pull(eE1,Immed,stacktop_max);}
lC
SynthesizeByteCode(x51
yZ3&synth,bool
MustPopTemps)const{xS1*this))yS;}
n72
0;a<12;++a){const
SinCosTanDataType&data=SinCosTanData[a];if(data.whichopcode!=cNop)l22!=data.whichopcode)y41
CodeTree
lI3;lI3.lH1
lI3
yQ3
inverse_opcode);lI3.lF2
xS1
lI3)){synth.iU
else
l22!=cInv)y41
if(GetParam(0)nF!=data.inverse_opcode)y41
xS1
GetParam(0))){synth.iU
size_t
found[4];xS3
4;++b){CodeTree
tree;if(data.eI3]==cNop){tree
c4
cInv);CodeTree
lJ3;lJ3.lH1
lJ3
yQ3
eI3^2]);lJ3.lF2
tree
y7
lJ3);}
else{tree.lH1
tree
yQ3
eI3]);}
tree.lF2
found[b]eJ3
xF3
tree);}
if(found[data.yA2!=tR
lX2]!=iS
yA2
i62.yV
lX2
iT
cDiv
iV
yA2!=tR
lC1]!=iS
yA2
i62.yV
lC1
iT
cMul
iV
lO1!=tR
lC1]!=iS
lO1
i62.yV
lC1
iT
cRDiv
iV
lO1!=tR
lX2]!=iS
lO1
i62.yV
lX2
iT
cMul,2,1
i62.iU
size_t
n_subexpressions_synthesized=SynthCommonSubExpressions(synth);switch(l42{case
iE2:synth.PushVar(GetVar());lD
cImmed:eV1
xE1);lD
cAdd:case
cMul:case
cMin:case
cMax:case
cAnd:case
cOr:case
cAbsAnd:case
cAbsOr:l22==cMul){bool
yR3
e23
yP
lP1
yP2)&&isLongInteger(lP1.xE1)){yO1=makeLongInteger(lP1.xE1);CodeTree
tmp(*this,typename
CodeTree::CloneTag());tmp.DelParam(a);tmp
eM1
if(nQ1
tmp,value,x51::tH1
yB::AddSequence,synth,MAX_MULI_BYTECODE_LENGTH)){yR3=true;yN3}
if(yR3)yY3
int
y71=0
e72
bool>done(GetParamCount(),false);CodeTree
iG;iG
c4
l42;for(;;){bool
found
e23
yP
done[a])y41
if(synth.IsStackTop(lP1)){found=true;done[a]=true;lP1.n8
iG
cM
lP1);if(++y71>1){yW
2);iG.Rehash(false)cZ1
iG);y71=y71-2+1;}
}
}
if(!found)yY3
yP
done[a])y41
lP1.n8
iG
cM
lP1);if(++y71>1){yW
2);iG.Rehash(false)cZ1
iG);y71=y71-2+1;}
}
if(y71==0){switch(l42{case
cAdd:case
cOr:case
cAbsOr:eV1
0);lD
cMul:case
cAnd:case
cAbsAnd:eV1
1);lD
cMin:case
cMax:eV1
0
eP3;y13
yY3++y71;}
assert(n_stacked==1);break
cT1
cPow:{iS2
p0
nT2
0);iS2
p1
nT2
1);if(!p1
yP2)||!isLongInteger(p1.xE1)||!nQ1
p0,makeLongInteger(p1.xE1),x51::tH1
yB::MulSequence,synth,MAX_POWI_BYTECODE_LENGTH)){p0.n8
p1.n8
yW
2);i31
cIf:case
cAbsIf:{typename
x51
yZ3::IfData
nP3;GetParam(0).n8
synth.SynthIfStep1(nP3,l42;GetParam(1).n8
synth.SynthIfStep2(nP3);GetParam(2).n8
synth.SynthIfStep3(nP3
eP3
cT1
cFCall:case
cPCall:{n72
0;a<x31++a)lP1.n8
yW
nD2)GetParamCount());n42
0x80000000u|GetFuncNo(),0,0);yY3
y13{n72
0;a<x31++a)lP1.n8
yW
nD2)GetParamCount());yN3
synth.StackTopIs(*this);if(MustPopTemps&&n_subexpressions_synthesized>0){size_t
top
eJ3
GetStackTop(i62.DoPopNMov(top-1-n_subexpressions_synthesized,top-1);}
}
}
iR2{yD
bool
nQ1
xB3,long
count,const
x51::SequenceOpCode
yB&eR,x51
yZ3&synth,size_t
max_bytecode_grow_length
cI3
count!=0){x51
yZ3
backup=synth;tree.n8
size_t
bytecodesize_backup
eJ3
GetByteCodeSize();x51::nQ1
count
n32
size_t
bytecode_grow_amount
eJ3
GetByteCodeSize()-bytecodesize_backup;if(bytecode_grow_amount>max_bytecode_grow_length){synth=backup
eO
false
yC3
true;}
else{x51::nQ1
count,eR,synth)cT2}
}
#endif
#include <cmath>
#include <cassert>
#ifdef FP_SUPPORT_OPTIMIZER
using
iR2
FUNCTIONPARSERTYPES;iR2{using
tC;
#define FactorStack std::vector
const
cP2
PowiMuliType{unsigned
opcode_square;unsigned
opcode_cumulate;unsigned
opcode_invert;unsigned
opcode_half;unsigned
opcode_invhalf;}
iseq_powi={cSqr,cMul,cInv,cSqrt,cRSqrt}
,iseq_muli={iF1
xC
cNeg,iF1,iF1}
;yD
l62
c61
const
PowiMuliType&yS3,t51,lG2&stack){l62
nT3(1);while(IP<limit
cI3
lK3
yS3.opcode_square
cI3!t12
nT3))lH2
2;c0
opcode_invert){nT3=-nT3;c0
opcode_half
cI3
nT3>tD1)&&isEvenInteger(nT3))lH2
l62(0.5);c0
opcode_invhalf
cI3
nT3>tD1)&&isEvenInteger(nT3))lH2
l62(-0.5);++IP;y41}
size_t
nO2=IP;l62
lhs(1);if(lK3
cFetch){lB1=lL3;if(index<y4||size_t(index-y4)>=iK1){IP=nO2;yY3
lhs=stack[index-y4];goto
yB2;}
if(lK3
cDup){lhs=nT3;goto
yB2;yB2:eW1
nT3);++IP;l62
subexponent=c61
yS3
i0
if(IP>=limit||eE1[IP]!=yS3.opcode_cumulate){IP=nO2;yY3++IP;stack.pop_back();nT3+=lhs*subexponent;y41}
break
yC3
nT3;}
yD
l62
ParsePowiSequence(t51){lG2
stack;eW1
l62(1))eO
c61
iseq_powi
i0}
yD
l62
ParseMuliSequence(t51){lG2
stack;eW1
l62(1))eO
c61
iseq_muli
i0}
x13
CodeTreeParserData{cV3
iK2
CodeTreeParserData(bool
k_powi):stack(),clones(),keep_powi(k_powi){}
void
Eat
x72
eG3,OPCODE
opcode
nQ
xN;xN
c4
opcode);nX2
e13=Pop(eG3);xN
iY1
tE2;if(!keep_powi)switch(opcode
e53
cTanh:{xG
sinh,cosh;sinh
c4
cSinh);sinh
cM
xN
xN2
sinh
eM1
cosh
c4
cCosh);cosh
y7
xN
xN2
cosh
eM1
xG
pow;pow
c4
cPow);pow
y7
cosh);pow.yJ
l62(-1)));pow
eM1
xN
c4
yT3.n41
0,sinh);xN
y7
pow
eP3
cT1
cTan:{xG
sin,cos;sin
c4
cSin);sin
cM
xN
xN2
sin
eM1
cos
c4
cCos);cos
y7
xN
xN2
cos
eM1
xG
pow;pow
c4
cPow);pow
y7
cos);pow.yJ
l62(-1)));pow
eM1
xN
c4
yT3.n41
0,sin);xN
y7
pow
eP3
cT1
cPow:{iT1
p0=xN
l8
0);iT1
p1=xN
l8
1);if(p1
nF==cAdd){nX2
i82(p1.GetParamCount());n72
0;a<p1.x31++a
nQ
pow;pow
c4
cPow);pow
cM
p0);pow
cM
p1
lS3
pow
eM1
i82[a
eO3
pow);}
xN
c4
yT3
iY1
i82);}
yY3
y13
yY3
xN.Rehash(!keep_powi);iI1,false);
#ifdef DEBUG_SUBSTITUTIONS
iJ1<<eG3<<", "
<<eK3(opcode)<<"->"
<<eK3(xN
nF)<<": "
t93
xN)iW
xN);
#endif
eW1
xN
n33
EatFunc
x72
eG3,OPCODE
cW3
unsigned
funcno
nQ
xN=CodeTreeFuncOp
yB(cW3
funcno);nX2
e13=Pop(eG3);xN
iY1
tE2;xN.lF2
#ifdef DEBUG_SUBSTITUTIONS
iJ1<<eG3<<", "
t93
xN)iW
xN);
#endif
iI1);eW1
xN
n33
xU
yB1
nQ
xN=CodeTreeImmed(value);iI1);Push(xN
n33
AddVar
nD2
varno
nQ
xN=CodeTreeVar
yB(varno);iI1);Push(xN
n33
SwapLastTwoInStack(){eX1
1
eO3
eX1
2]n33
Dup(){Fetch(iK1-1
n33
Fetch
x72
which){Push(stack[which]);}
nH1
T>void
Push(T
tree){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<t93
tree)iW
tree);
#endif
eW1
tree
n33
PopNMov
x72
target,size_t
source){stack[target]=stack[source];stack
x53
target+1);}
xG
yC2{clones.clear();xG
nT3(stack.back());stack
x53
iK1-1)eO
nT3;}
nX2
Pop
x72
n_pop){nX2
nT3(n_pop);for
nD2
n=0;n<n_pop;++n)nT3[n
eO3
eX1
n_pop+n]);
#ifdef DEBUG_SUBSTITUTIONS
for
x72
n=n_pop;n-->0;){iJ1
yR2
nT3[n])iW
nT3[n]);}
#endif
stack
x53
iK1-n_pop)eO
nT3;}
size_t
GetStackTop(yK1
iK1;}
private:void
FindClone(xG&,bool=true)yS;}
private:nX2
stack;std::multimap<fphash_t,xG>clones;bool
keep_powi;private:CodeTreeParserData
cY1
CodeTreeParserData&);CodeTreeParserData&eD1=cY1
CodeTreeParserData&);}
;yD
cP2
IfInfo{xG
eO2;xG
thenbranch;size_t
endif_location;IfInfo():eO2(),thenbranch(),endif_location(){}
}
;}
tC{lC
GenerateFrom
cY1
typename
FunctionParserBase
yB::Data&nR3,bool
keep_powi){nX2
x12;x12.xD3
nR3.mVariablesAmount);for
nD2
n=0;n<nR3.mVariablesAmount;++n){x12
yS2
CodeTreeVar
yB(n+iE2));}
GenerateFrom(nR3,x12,keep_powi);}
lC
GenerateFrom
cY1
typename
FunctionParserBase
yB::Data&nR3,const
x7&x12,bool
keep_powi){yL1
unsigned>&eE1=nR3.mByteCode;const
std::vector
yB&Immed=nR3.mImmed;
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"ENTERS GenerateFrom()\n"
;
#endif
CodeTreeParserData
yB
sim(keep_powi)e72
IfInfo
yB>eN;for
x72
IP=0,DP=0;;++IP){tK2:while(!eN
yU3&&(eN.eD==IP||(IP<lX1&&lK3
cJump&&eN.eA1.n12))){CodeTree
elsebranch=sim.yC2
n92
eN.back().eO2)n92
eN.eA1)n92
elsebranch);sim.Eat(3,cIf);eN.pop_back();}
if(IP>=lX1)break;unsigned
opcode=eE1[IP];if((opcode==cSqr||opcode==cDup||(opcode==cInv&&!IsIntType
yB::nT3)||opcode==cNeg||opcode==cSqrt||opcode==cRSqrt||opcode==cFetch)){size_t
was_ip=IP;l62
cY2
ParsePowiSequence
yB(eE1,IP,eN
yU3?lX1:eN.eD,sim.xH
1);if(nU3!=l62(1.0)){sim.xU
x32
xX
cPow);goto
tK2;}
if(opcode==cDup||opcode==cFetch||opcode==cNeg){l62
xH2=ParseMuliSequence
yB(eE1,IP,eN
yU3?lX1:eN.eD,sim.xH
1);if(xH2!=l62(1.0)){sim.xU
xH2)xX
cMul);goto
tK2;}
}
IP=was_ip;}
if(lI2>=iE2){lB1=opcode-iE2
n92
x12[index]);}
else{switch(lI2
e53
cIf:case
cAbsIf:{eN
x53
eN.size()+1);CodeTree
res(sim.yC2);eN.back().eO2.swap(res);eN.eD=lX1;IP+=2;y41}
case
cJump:{CodeTree
res(sim.yC2);eN.eA1.swap(res);eN.eD=eE1[IP+1]+1;IP+=2;y41}
case
cImmed:sim.xU
Immed[DP++]);lD
cDup:sim.Dup();lD
cNop:lD
cFCall:{unsigned
funcno=lL3;assert(funcno<fpdata.mFuncPtrs.size());unsigned
e13=nR3.mFuncPtrs
lM3
mParams;sim.EatFunc(e13,lI2,funcno
eP3
cT1
cPCall:{unsigned
funcno=lL3;assert(funcno<fpdata.t63.size());const
FunctionParserBase
yB&p=*nR3.t63
lM3
mParserPtr;unsigned
e13=nR3.t63
lM3
mParams;x7
paramlist=sim.Pop(tE2;CodeTree
tL2;tL2.GenerateFrom(*p.mData,paramlist)n92
tL2
eP3
cT1
cInv:sim.xU
1
lY2
cDiv);lD
cNeg:nA2
cNeg
eP3;sim.xU
0
lY2
cSub);lD
cSqr:sim
c52;lD
cSqrt
tM3
eY1
cRSqrt
tM3-eY1
cCbrt
tM3
1)/l62(3))xX
yV3
cDeg:sim.xU
fp_const_rad_to_deg
e11
cRad:sim.xU
fp_const_deg_to_rad
e11
cExp:iH)goto
default_function_handling;sim.xU
fp_const_e
yB()lY2
yV3
cExp2:iH)goto
default_function_handling;sim.xU
2.0
lY2
yV3
cCot:nA2
cTan);iH)x2
cCsc:nA2
cSin);iH)x2
cSec:nA2
cCos);iH)x2
cInt:
#ifndef __x86_64
iH){nA2
cInt);yY3
#endif
sim
i61
0.5))xX
cAdd);nA2
cFloor);lD
cLog10:nA2
yD2
fp_const_log10inv
e11
cLog2:nA2
yD2
fp_const_log2inv
e11
cLog2by:lN3
yD2
fp_const_log2inv
yB());sim.Eat(3,cMul);lD
cHypot:sim
c52;sim.nX1
sim
c52
xX
cAdd);sim
i61
eY1
cSinCos:sim.Dup();nA2
cSin);lN3
cCos);lD
cSinhCosh:sim.Dup();nA2
cSinh);lN3
cCosh);lD
cRSub:sim.nX1
case
cSub:iH){sim.Eat(2,cSub);yY3
sim.xU-1)xX
cMul)xX
cAdd);lD
cRDiv:sim.nX1
case
cDiv:iH||IsIntType
yB::nT3){sim.Eat(2,cDiv);yY3
sim.xU-1)xX
cPow)xX
cMul);lD
cAdd:case
cMul:case
cMod:case
cPow:case
cEqual:case
cLess:case
cGreater:case
tP1:case
cLessOrEq:case
cGreaterOrEq:case
cAnd:case
cOr:case
cAbsAnd:case
cAbsOr:sim.Eat(2
cS1
lD
cNot:case
cNotNot:case
c03:case
cAbsNotNot:sim.Eat(1
cS1
lD
cFetch:sim.Fetch(lL3);lD
cPopNMov:{unsigned
stackOffs_target=lL3;unsigned
stackOffs_source=lL3;sim.PopNMov(stackOffs_target,stackOffs_source);yY3
y13
default_function_handling:;unsigned
funcno=opcode-cAbs;assert(funcno<FUNC_AMOUNT);const
FuncDefinition&func=Functions[funcno];sim.Eat(func.e13
cS1
yN3}
Become(sim.yC2);
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Produced tree:\n"
;iQ
#endif
}
}
#endif
#include <algorithm>
#ifdef FP_SUPPORT_OPTIMIZER
#include <assert.h>
#define FP_MUL_COMBINE_EXPONENTS
iR2{using
iR2
FUNCTIONPARSERTYPES;using
tC;yD
static
void
AdoptChildrenWithSameOpcode(cV2{
#ifdef DEBUG_SUBSTITUTIONS
bool
lZ2
e23
#endif
for
c7
if
tN3
nF==tree
nF){
#ifdef DEBUG_SUBSTITUTIONS
if(!lZ2)lC3"Before assimilation: "
xY
lZ2=true;}
#endif
tree.AddParamsMove
tN3.GetUniqueRef().iP1),a);}
#ifdef DEBUG_SUBSTITUTIONS
if(lZ2)lC3"After assimilation:   "
xY}
#endif
}
}
tC{x73
ConstantFolding(cV2{tree.Sort();
#ifdef DEBUG_SUBSTITUTIONS
void*yW3=0
xO1"["
<<(&yW3)<<"]Runs ConstantFolding for: "
xY
DumpHashes(tree)xO1
std::flush;
#endif
if(false){redo:;tree.Sort();
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"["
<<(&yW3)<<"]Re-runs ConstantFolding: "
xY
DumpHashes(tree);
#endif
}
if(tree
nF!=cImmed){range
yB
p=iO
tree);if(p
eY&&p
t6
cF3==p
nV){tree.ReplaceWithImmed(p
e32
l32}
}
if(false){ReplaceTreeWithOne
e93.ReplaceWithImmed(l62(1)l32
ReplaceTreeWithZero
e93.ReplaceWithImmed(tD1)l32
ReplaceTreeWithParam0:
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before replace: "
xO1
std::hex<<'['<<tree
n02
hash1<<','<<tree
n02
hash2<<']'<<std::dec
xY
#endif
tree
iD2
tQ);
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After replace: "
xO1
std::hex<<'['<<tree
n02
hash1<<','<<tree
n02
hash2<<']'<<std::dec
xY
#endif
cN
yX3
l83{case
cImmed:lD
iE2:lD
cAnd:case
cAbsAnd:e0
bool
cD
e23
for
c7{if(!IsLogicalValue
tN3))cD=true;cN3
tJ3
i21
cAbsAnd)e53
xL3
tE
IsAlways:e61);lD
lS1
yX3(t7
e53
0:iA
1
e93
c4
i21
cAnd?cNotNot:cAbsNotNot);cN
y13
if(i21
cAnd||!cD)if(ConstantFolding_AndLogic(e82
i31
cOr:case
cAbsOr:e0
bool
cD
e23
for
c7{if(!IsLogicalValue
tN3))cD=true;cN3
tJ3
i21
cAbsOr))i71
iA
iO2
e61);lD
lS1
yX3(t7
e53
0
tE
1
e93
c4
i21
cOr?cNotNot:cAbsNotNot);cN
y13
if(i21
cOr||!cD)if(ConstantFolding_OrLogic(e82
i31
cNot:case
c03:{unsigned
lZ1
0;switch(tQ
nF
e53
cEqual:lZ1
tP1;lD
tP1:lZ1
cEqual;lD
cLess:lZ1
cGreaterOrEq;lD
cGreater:lZ1
cLessOrEq;lD
cLessOrEq:lZ1
cGreater;lD
cGreaterOrEq:lZ1
cLess;lD
cNotNot:lZ1
cNot;lD
cNot:lZ1
cNotNot;lD
c03:lZ1
cAbsNotNot;lD
cAbsNotNot:lZ1
c03;break;y13
yY3
if(opposite){tree
c4
OPCODE(opposite));tree
iY1
tQ.GetUniqueRef().iP1));cN
yX3
nR1
tQ,tree
c71
e53
IsAlways
tE
iO2
iA
lS1
if(i21
cNot&&GetPositivityInfo(tQ)==IsAlways)tree
c4
c03);if(e43
cIf||e43
cAbsIf
nQ
iM2=tQ;iT1
ifp1=iM2
l8
1);iT1
ifp2=iM2
l8
2);if(ifp1
nF==cNot||ifp1
c71
tG3
ifp1
nF==cNot?cNotNot:cAbsNotNot);tM2
l8
0))lT1
cM3
n82
p2
lO3
p2.lP3
if(ifp2
nF==cNot||ifp2
c71
tG3
tree
nF);tM2)lT1
cM3
c4
ifp2
nF==cNot?cNotNot:cAbsNotNot);p2
cM
ifp2
xN2
p2.lP3
i31
cNotNot:case
cAbsNotNot:{if(IsLogicalValue(tQ))goto
e5
cN3
tQ,i21
cAbsNotNot)e53
xL3
tE
IsAlways:iA
lS1
if(i21
cNotNot&&GetPositivityInfo(tQ)==IsAlways)tree
c4
cAbsNotNot);if(e43
cIf||e43
cAbsIf
nQ
iM2=tQ;iT1
ifp1=iM2
l8
1);iT1
ifp2=iM2
l8
2);if(ifp1
nF==cNot||ifp1
c71{tree.SetParam(0,iM2
xN2
tree
cM
ifp1);cM3
n82
p2
lO3
p2.lP3
if(ifp2
nF==cNot||ifp2
c71
tG3
tree
nF);tM2)lT1
tree
lO3
tree
c4
iM2
nF);cN}
i31
cIf:case
cAbsIf:{if(ConstantFolding_IfOperations(e82
break
cT1
cMul:{NowWeAreMulGroup:;AdoptChildrenWithSameOpcode(tree);l62
nB1=l62(1);size_t
iL1=0;bool
nC1
e23
for
c6
if(!xQ1
y41
l62
immed=yE2;if(immed==tD1))goto
ReplaceTreeWithZero;nB1*=immed;++iL1;}
if(iL1>1||(iL1==1&&fp_equal(nB1,l62(1))))nC1=true;if(nC1){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"cMul: Will add new "
t73
nB1<<"\n"
;
#endif
for
c7
if(xQ1{
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<" - For that, deleting "
t73
yE2
xO1"\n"
;
#endif
lQ3!fp_equal(nB1,l62(1)))tree
cM
cL1
nB1));yX3(t7
e53
0:iA
1:goto
e5
y13
if(ConstantFolding_MulGrouping(e82
if(ConstantFolding_MulLogicItems(e82
i31
cAdd:e0
l62
lJ2=0.0;size_t
iL1=0;bool
nC1
e23
for
c6
if(!xQ1
y41
l62
immed=yE2;lJ2+=immed;++iL1;}
if(iL1>1||(iL1==1&&lJ2==tD1)))nC1=true;if(nC1){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"cAdd: Will add new "
t73
lJ2<<"\n"
xO1"In: "
xY
#endif
for
c7
if(xQ1{
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<" - For that, deleting "
t73
yE2
xO1"\n"
;
#endif
lQ3!(lJ2==l62(0.0)))tree
cM
cL1
lJ2));yX3(t7
e53
0
tE
1:goto
e5
y13
if(ConstantFolding_AddGrouping(e82
if(ConstantFolding_AddLogicItems(e82
i31
cMin:e0
size_t
yF2=0;range
yB
e7;for
c6
while(a+1<t7&&xP1
a)xI
xP1
a+1)))e61+1);range<l62
nY
max
iN2(!e7
t6||(p
nV)<e7
nV)){e7
nV=p
nV;e7
t6=true;yF2=a;}
}
if(e7
t6)for
c7{range<l62
nY
min
iN2
a!=yF2
cF3>=e7
nV)lQ3
t7==1){goto
e5
i31
cMax:e0
size_t
yF2=0;range
yB
t2;for
c6
while(a+1<t7&&xP1
a)xI
xP1
a+1)))e61+1);range<l62
nY
min
iN2(!t2
eY||p
e32>t2
e32)){t2
e32=p
e32;t2
eY=true;yF2=a;}
}
if(t2
eY){for
c7{range<l62
nY
max
iN2
a!=yF2&&(p
nV)<t2
e32){e61);}
}
}
if(t7==1){goto
e5
i31
cEqual:case
tP1:case
cLess:case
cGreater:case
cLessOrEq:case
cGreaterOrEq:if(ConstantFolding_Comparison(e82
lD
cAbs:{range
yB
p0=tO
0));if(p0.cK
goto
e5
if(p0.c8{tree
c4
cMul);tree.yJ
l62(1)));goto
NowWeAreMulGroup;}
if(e43
cMul){iT1
p=tQ;nX2
lR3;nX2
cG2;n72
0;a<p.x31++a){p0=iO
p
lS3
if(p0.cK{lR3
yS2
p
lS3}
if(p0.c8{cG2
yS2
p
lS3}
}
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Abs: mul group has "
<<lR3.size()<<" pos, "
<<cG2.size()<<"neg\n"
;
#endif
if(!lR3
yU3||!cG2
yU3){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"AbsReplace-Before: "
yR2
tree)xO1"\n"
<<std::flush;DumpHashes(tree
cH2;
#endif
xG
cT3;cT3
c4
cMul);n72
0;a<p.x31++a){p0=iO
p
lS3
if((p0.cK||(p0.c8){}
else
cT3
cM
p
lS3}
cT3
eM1
xG
lT3;lT3
c4
cAbs);lT3
y7
cT3);lT3
eM1
xG
xX1
cMul);i82
y7
lT3);xY1
AddParamsMove(lR3);if(!cG2
yU3
cI3
cG2.size()%2)xY1
yJ
l62(-1)));xY1
AddParamsMove(cG2);}
tree
iD2
i82);
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"AbsReplace-After: "
yR2
tree
cH2
xO1"\n"
<<std::flush;DumpHashes(tree
cH2;
#endif
goto
NowWeAreMulGroup;}
}
yY3
#define HANDLE_UNARY_CONST_FUNC(funcname) y23 funcname(lS nD
case
cLog:eW3(fp_log);if(e43
cPow
nQ
pow=tQ;if(GetPositivityInfo(pow
l8
0))==IsAlways
xA3
pow
xF1
tree.lV
if(GetEvennessInfo(pow
l8
1))==IsAlways
xA3
xG
abs;abs
c4
cAbs);abs
y7
pow
xN2
abs
eM1
pow
xF1
pow.n41
0,abs);tree.lV}
i81
e43
cAbs
nQ
pow=tQ
l8
0);if(pow
nF==cPow
xA3
xG
abs;abs
c4
cAbs);abs
y7
pow
xN2
abs
eM1
pow
xF1
pow.n41
0,abs);tree.lV}
lD
cAcosh:eW3(fp_acosh);lD
cAsinh:eW3(fp_asinh);lD
cAtanh:eW3(fp_atanh);lD
cAcos:eW3(fp_acos);lD
cAsin:eW3(fp_asin);lD
cAtan:eW3(fp_atan);lD
cCosh:eW3(fp_cosh);lD
cSinh:eW3(fp_sinh);lD
cTanh:eW3(fp_tanh);lD
cSin:eW3(fp_sin);lD
cCos:eW3(fp_cos);lD
cTan:eW3(fp_tan);lD
cCeil:if(n5
eW3(fp_ceil);lD
cTrunc:if(n5
eW3(fp_trunc);lD
cFloor:if(n5
eW3(fp_floor);lD
cInt:if(n5
eW3(fp_int);lD
cCbrt:eW3(fp_cbrt);lD
cSqrt:eW3(fp_sqrt);lD
cExp:eW3(fp_exp);lD
cLog2:eW3(fp_log2);lD
cLog10:eW3(fp_log10);lD
cLog2by:if
lJ
fp_log2(lS)*c13
cArg:eW3(fp_arg);lD
cConj:eW3(fp_conj);lD
cImag:eW3(fp_imag);lD
cReal:eW3(fp_real);lD
cPolar:if
lJ
fp_polar
e42
cMod:if
lJ
fp_mod
e42
cAtan2:{range
yB
p0
tN
p1=tO
1));cD3
fp_equal(lS,tD1))cI3
p1
t6&&(p1
nV)<tD1
nR
fp_const_pi
yB(nD
if(p1
eY&&p1
e32>=tN2
tD1
nD}
if
lM1
fp_equal(tM),tD1))cI3
p0
t6&&(p0
nV)<tD1
nR-fp_const_pihalf
yB(nD
if(p0
eY&&p0
e32>tD1
nR
fp_const_pihalf
yB(nD}
if
lJ
fp_atan2(lS,tM)nD
if((p1
eY&&p1
e32>tD1))||(p1
t6&&(p1
nV)<fp_const_negativezero
yB())nQ
yG2;yG2
c4
cPow);yG2
y7
t32);yG2.yJ
l62(-1)));yG2
eM1
xG
yH2;yH2
c4
cMul);yH2
y7
tQ);yH2
y7
yG2);yH2
eM1
tree
c4
cAtan);tree.n41
0,yH2)lP2
1);i31
cPow:{if(ConstantFolding_PowOperations(e82
break
cT1
cDiv:cD3
xP1
yO2&&tM)!=tN2
lS/c13
cInv:cD3
lS!=tN2
l62(1)/lS);yV1
cSub:if
lJ
lS-c13
cNeg:y23-lS);yV1
cRad:y23
RadiansToDegrees(lS
nD
lD
cDeg:y23
DegreesToRadians(lS
nD
lD
cSqr:y23
lS*lS);yV1
cExp2:eW3(fp_exp2);lD
cRSqrt:y23
l62(1)/fp_sqrt(lS
nD
lD
cCot
tK3
fp_tan(n3
cSec
tK3
fp_cos(n3
cCsc
tK3
fp_sin(n3
cHypot:if
lJ
fp_hypot
e42
cRDiv:case
cRSub:case
cDup:case
cFetch:case
cPopNMov:case
cSinCos:case
cSinhCosh:case
cNop:case
cJump:lD
cPCall:case
cFCall:yY3
do_return:;
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"["
<<(&yW3)<<"]Done ConstantFolding, result: "
xY
DumpHashes(tree);
#endif
}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
tC{x73
range
yB::set_abs(nN
bool
has_negative=!min.known||min.val<l62();bool
has_positive=!max.known||max.val>l62();bool
crosses_axis=has_negative&&has_positive;rangehalf
yB
newmax;if(min
iN2
max.known)newmax.set(fp_max(tQ1,tS1);if(crosses_axis)min.set(l62());eN3
min
iN2
max.known)min.set(fp_min(tQ1,tS1);i81
min.known)min.set(tQ1);else
min.set(tS1;}
max=newmax;}
x73
range
yB::set_neg(){std::swap(min,max);min.val=-min.val;max.val=-max.val;}
yD
bool
IsLogicalTrueValue
cY1
range
yB&p
eZ2{if(nE
IsIntType
yB::nT3
cI3
p
eY
cF3>=l62(1
t41;if(!abs&&p
t6&&p
nV<=l62(-1
t41;}
eN3
p
eY
cF3>=l62(0.5
t41;if(!abs&&p
t6&&p
nV<=l62(-0.5
t41
yC3
e83
yD
bool
IsLogicalFalseValue
cY1
range
yB&p
eZ2{if(nE
IsIntType
yB::nT3
cI3
abs)yD3
t6
yG1
1);else
yD3
eY&&p
t6
cF3>l62(-1)yG1
1);}
eN3
abs)yD3
t6
yG1
0.5);else
yD3
eY&&p
t6
cF3>l62(-0.5)yG1
0.5);}
}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
using
iR2
FUNCTIONPARSERTYPES;using
tC;tC{yD
range
yB
iO
xB3)
#ifdef DEBUG_SUBSTITUTIONS_extra_verbose
{using
iR2
FUNCTIONPARSERTYPES;range
yB
tmp=CalculateResultBoundaries_do(tree)xO1"Estimated boundaries: "
;if(tmp
eY)std::cout<<tmp
e32;else
std::cout<<"-inf"
xO1" .. "
;if(tmp
t6)std::cout<<tmp
nV;else
std::cout<<"+inf"
xO1": "
yR2
tree)xO1
std::endl
eO
tmp;}
yD
range
yB
xG::CalculateResultBoundaries_do
cY1
cV2
#endif
{iI
y81(-fp_const_pihalf
yB(),fp_const_pihalf
yB());iI
pi_limits(-fp_const_pi
yB(),fp_const_pi
yB());iI
abs_pi_limits(tD1),fp_const_pi
yB());iI
plusminus1_limits(l62(-n23
using
iR2
std;switch
l83{case
cImmed:nS
tree.xE1,tree.xE1);case
cAnd:case
cAbsAnd:case
cOr:case
cAbsOr:case
cNot:case
c03:case
cNotNot:case
cAbsNotNot:case
cEqual:case
tP1:case
cLess:case
cLessOrEq:case
cGreater:case
cGreaterOrEq:{nS
tD1),l62(1))cT1
cAbs:lE
m.set_abs(tR1
cLog:lE
lU3
fp_log
tP2
fp_log
tR1
cLog2:lE
lU3
fp_log2
tP2
fp_log2
tR1
cLog10:lE
lU3
fp_log10
tP2
fp_log10
tR1
cAcosh:lE
yI
xT1
cGreaterOrEq
eZ1
fp_acosh
eR3
cGreaterOrEq
eZ1
fp_acosh
tR1
cAsinh:lE
yI
set(fp_asinh
tI2
set(fp_asinh
tR1
cAtanh:lE
yI
n2-1),fp_atanh
eR3
cLess
eZ1
fp_atanh
tR1
cAcos:lE
nS(m
t6&&(m
nV)<l62(1))?fp_acos(m
nV):tD1),(eJ2&&cI2)>=l62(-1))?fp_acos
cI2):fp_const_pi
yB())cT1
cAsin:lE
yI
n2-1),fp_asin,y81
e32
eR3
cLess
eZ1
fp_asin,y81
nV
tR1
cAtan:lE
yI
set(fp_atan,y81
e32
tI2
set(fp_atan,y81
nV
tR1
cAtan2:{cD3
fp_equal(lS,tD1)))yS
abs_pi_limits;}
if
lM1
fp_equal(tM),tD1)))yS
y81
yC3
pi_limits
cT1
cSin:lE
bool
nS1=!eJ2||!m
t6||(m
nV-yI
val)>=(yG
nS1)t8
l62
min=fp_mod
cI2,yG
min<tD1))min
yQ
l62
max=fp_mod(m
nV,yG
max<tD1))max
yQ
if(max<min)max
yQ
bool
xU1=(min<=fp_const_pihalf
yB()&&max>=fp_const_pihalf
yB());bool
nD1=(min<=cI&&max>=cI);if(xU1&&nD1)t8
if(nD1)nS
l62(-1),yI2;if(xU1)nS
yK2
l62(1));nS
yK2
yI2
cT1
cCos:lE
if(eJ2)yI
val+=fp_const_pihalf
yB();if(m
t6)m
nV+=fp_const_pihalf
yB();bool
nS1=!eJ2||!m
t6||(m
nV-yI
val)>=(yG
nS1)t8
l62
min=fp_mod
cI2,yG
min<tD1))min
yQ
l62
max=fp_mod(m
nV,yG
max<tD1))max
yQ
if(max<min)max
yQ
bool
xU1=(min<=fp_const_pihalf
yB()&&max>=fp_const_pihalf
yB());bool
nD1=(min<=cI&&max>=cI);if(xU1&&nD1)t8
if(nD1)nS
l62(-1),yI2;if(xU1)nS
yK2
l62(1));nS
yK2
yI2
cT1
cTan:{nS)cT1
cCeil:lE
m
eM
cFloor:lE
yJ2
tR1
cTrunc:lE
yJ2);m
eM
cInt:lE
yJ2);m
eM
cSinh:lE
yI
set(fp_sinh
tI2
set(fp_sinh
tR1
cTanh:lE
yI
set(fp_tanh,plusminus1_limits.min
tI2
set(fp_tanh,plusminus1_limits.max
tR1
cCosh:lE
if(eJ2
cI3
m
t6){if
cI2
cW1&&m
nV
cW1){yI
val=e1}
i81
cI2)<tD1)&&m
nV
cW1){l62
tmp=e1
if(tmp>m
nV)m
nV=tmp;yI
val=l62(1);}
else{yI
val=e1
std::swap
cI2,m
nV);}
}
else{if
cI2
cW1){m.cJ
yI
val=fp_cosh
cI2);}
else{m.cJ
yI
val=l62(1);}
}
}
else{eJ2=true;yI
val=l62(1);if(m
t6){yI
val=fp_cosh(m
nV);m.cJ}
else
m.cJ}
return
m
cT1
cIf:case
cAbsIf:{range
yB
res1=tO
1));range
yB
res2=tO
2));if(!res2
eY)res1
eY
e23
i81
res1
eY&&(res2
e32)<res1
e32)res1
e32=res2
e32;if(!res2
t6)res1.cJ
i81
res1
t6&&(res2
nV)>res1
nV)res1
nV=res2
nV
eO
res1
cT1
cMin:{bool
iJ
e23
bool
iK
e23
range
lV3;x8
m=tO
c23
eJ2)iJ
e33
eY||cI2)<lW3)lW3=yI
val;if(!m
t6)iK
e33
t6||(m
nV)<lX3)lX3=m
nV;}
if(iJ)lY3
iK)tX2
cJ
return
lZ3
cMax:{bool
iJ
e23
bool
iK
e23
range
lV3;x8
m=tO
c23
eJ2)iJ
e33
eY||yI
val>lW3)lW3=yI
val;if(!m
t6)iK
e33
t6||m
nV>lX3)lX3=m
nV;}
if(iJ)lY3
iK)tX2
cJ
return
lZ3
cAdd:{range
lV3(tD1),tD1));x8
item=tO
a));if(item
eY)lW3+=item
e32;else
lY3
item
t6)lX3+=item
nV;else
tX2
cJ
if(!n03&&!n13)yY3
if(n03&&n13&&lW3>lX3)std::swap
tQ2,lX3)eO
lZ3
cMul:{cP2
Value{enum
nE3{tR2,iM1,tS2}
;nE3
eZ;l62
value;Value(nE3
t):eZ(t),value(0){}
Value(l62
v):eZ(tR2),value(v){}
bool
cJ2
yK1
eZ==iM1||(eZ==tR2&&value<tD1)n33
eD1*=cY1
Value&rhs
cI3
eZ==tR2&&rhs.eZ==tR2)value*=rhs.value;else
eZ=(cJ2)!=rhs.cJ2)?iM1:tS2);}
bool
eD1<cY1
Value&rhs
yK1(eZ==iM1&&rhs.eZ!=iM1)||(eZ==tR2&&(rhs.eZ==tS2||(rhs.eZ==tR2&&value<rhs.value)));}
}
;cP2
y91{Value
yL2,yM2;y91():yL2(Value::tS2),yM2(Value::iM1){}
void
x22
Value
c33,const
Value&value2){c33*=value2;if(c33<yL2)yL2=c33;if(yM2<c33)yM2=c33;}
}
;range
lV3(l62(n23
x8
item=tO
c23
item
eY&&!item
t6)nS);Value
nF3=n03?Value
tQ2):t01
iM1);Value
nG3=n13?Value(lX3):t01
tS2);Value
nH3=item
eY?Value(item
e32):t01
iM1);Value
nI3=item
t6?Value(item
nV):t01
tS2);y91
range;range.x22
nF3,nH3
e52
nF3,nI3
e52
nG3,nH3
e52
nG3,nI3);if(range.yL2.eZ==Value::tR2)lW3=range.yL2.value;else
lY3
range.yM2.eZ==Value::tR2)lX3=range.yM2.value;else
tX2
cJ
if(!n03&&!n13)yY3
if(n03&&n13&&lW3>lX3)std::swap
tQ2,lX3)eO
lZ3
cMod:{range
yB
x
tN
y=tO
1));if(y
t6
cI3
y
nV
cW1
cI3!x
eY||(x
e32)<tD1))nS-y
nV,y
nV);else
nS
tD1),y
nV);}
eN3!x
t6||(x
nV)cW1)nS
y
nV,-y
nV);else
nS
y
nV,fp_const_negativezero
yB());}
}
else
nS)cT1
cPow:{if
lM1
tM)==tD1)){nS
l62(n23}
cD3
lS==tD1)){nS
tD1),tD1));}
cD3
fp_equal(lS
n53
nS
l62(n23}
if
lM1
tM)>tD1)&&GetEvennessInfo(t32)==IsAlways){l62
cY2
tM);range
yB
tmp
tN
nT3;n03=true;lW3=0;if(tmp
eY&&tmp
e32
cW1)lW3
xV3
tmp
e32,x32;i81
tmp
t6&&tmp
nV<=tD1))lW3
xV3
tmp
nV,x32;tX2
cJ
if(tmp
eY&&tmp
t6){n13=true;lX3=fp_max(fp_abs(tmp
e32),fp_abs(tmp
nV));lX3
xV3
lX3,x32
yC3
nT3;}
range
yB
p0
tN
p1=tO
1
n52
p0_positivity=(p0
eY&&(p0
e32)cW1)?IsAlways:(p0
t6&&(p0
nV)<tD1)?iO2
Unknown);TriTruthValue
cK2=GetEvennessInfo(xP1
1
n52
t3=Unknown;switch(p0_positivity)i71
t3=IsAlways;lD
iO2{t3=cK2;yY3
y13
switch(cK2)i71
t3=IsAlways;lD
iO2
lD
Unknown:{if
lM1!isInteger
tL&&tM)cW1){t3=IsAlways;}
yN3
yX3(t3)i71{if((p1
t6&&p1
nV<0)||(p1
eY&&p1
e32<0)){nS);}
l62
min=tD1);if(p0
eY&&p1
eY){min
xV3
p0
e32,p1
e32);if(p0
e32<tD1)&&(!p1
t6||p1
nV
cW1)&&min
cW1)min=tD1);}
if(p0
eY&&p0
e32
cW1&&p0
t6&&p1
t6){l62
max
xV3
p0
nV,p1
nV);if(min>max)std::swap(min,max);nS
min,max);}
nS
min,false)cT1
iO2{nS
false,fp_const_negativezero
yB());}
y13{yY3
i31
cNeg:lE
m.set_neg(tR1
cSub:{nA
cNeg
y53
1
nY3
cAdd);nZ3;tmp
c43
eO
lI
cInv:{nB-1
cE3
cDiv:{nA
cInv
y53
1
nY3
xV1
c43
eO
lI
cRad:{t21
xV1.yJ
fp_const_rad_to_deg
yB(cE3
cDeg:{t21
xV1.yJ
fp_const_deg_to_rad
yB(cE3
cSqr:{nB
2
cE3
cExp:{t21
cPow);tmp.yJ
fp_const_e
yB()));nZ3
eO
lI
cExp2:{t21
cPow);tmp
eB1
nZ3
eO
lI
cCbrt:lE
yI
set(fp_cbrt
tI2
set(fp_cbrt
tR1
cSqrt:lE
if(eJ2)yI
val=cI2)<tD1)?0:fp_sqrt
cI2);if(m
t6)m
nV=(m
nV)<tD1)?0:fp_sqrt(m
nV
tR1
cRSqrt:{nB-0.5
cE3
cHypot:{xG
xsqr,ysqr,add,sqrt;xsqr.n7
0));xsqr
eB1
ysqr.n7
1));ysqr
eB1
xsqr
c4
cPow);ysqr
c4
cPow);add
y7
xsqr);add
y7
ysqr);add
c4
cAdd);sqrt
y7
add);sqrt
c4
cSqrt)eO
iO
sqrt)cT1
cLog2by:{nA
cLog2
y53
0
nY3
cMul);tmp
c43;tmp.n7
1))eO
lI
cCot:{nA
cTan
x4
lI
cSec:{nA
cCos
x4
lI
cCsc:{nA
cSin
x4
iO
tmp);}
lD
cRDiv:case
cRSub:case
cDup:case
cFetch:case
cPopNMov:case
cSinCos:case
cSinhCosh:case
cNop:case
cJump:case
iE2:lD
cArg:case
cConj:case
cImag:case
cReal:case
cPolar:lD
cPCall:lD
cFCall:yY3
nS);}
yD
TriTruthValue
GetIntegerInfo
cY1
cV2{switch
l83{case
cImmed:return
t12
tree.xE1)?IsAlways:xL3;case
cFloor:case
cCeil:case
cTrunc:case
cInt:return
IsAlways;case
cAnd:case
cOr:case
cNot:case
cNotNot:case
cEqual:case
tP1:case
cLess:case
cLessOrEq:case
cGreater:case
cGreaterOrEq:return
IsAlways;case
cIf:{TriTruthValue
a=GetIntegerInfo(xP1
1
n52
b=GetIntegerInfo(cC3);if(a==b)return
a
eO
Unknown
cT1
cAdd:case
cMul:{for
c7
if(GetIntegerInfo
tN3)!=IsAlways)return
Unknown
eO
IsAlways;}
y13
break
yC3
Unknown;}
yD
bool
IsLogicalValue
cY1
cV2{switch
l83{case
cImmed:return
fp_equal(tree.xE1,tD1))||fp_equal(tree.xE1,l62(1));case
cAnd:case
cOr:case
cNot:case
cNotNot:case
cAbsAnd:case
cAbsOr:case
c03:case
cAbsNotNot:case
cEqual:case
tP1:case
cLess:case
cLessOrEq:case
cGreater:case
cGreaterOrEq:x0
cMul:{for
c7
if(!IsLogicalValue
tN3)c5
return
true
cT1
cIf:case
cAbsIf:yS
IsLogicalValue(t32)nK1
cC3);}
y13
break
yC3
e83}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
using
iR2
FUNCTIONPARSERTYPES;
#if defined(__x86_64) || !defined(FP_SUPPORT_CPLUSPLUS11_MATH_FUNCS)
# define CBRT_IS_SLOW
#endif
#if defined(DEBUG_POWI) || defined(DEBUG_SUBSTITUTIONS)
#include <cstdio>
#endif
iR2
x51{extern
const
unsigned
char
powi_table[256];}
iR2{using
tC;yD
bool
IsOptimizableUsingPowi(long
immed,long
penalty=0){x51
yZ3
synth;synth.PushVar(iE2);size_t
bytecodesize_backup
eJ3
GetByteCodeSize();x51::nQ1
immed,x51::tH1
yB::MulSequence,synth);size_t
bytecode_grow_amount
eJ3
GetByteCodeSize()-bytecodesize_backup
eO
bytecode_grow_amount<size_t(MAX_POWI_BYTECODE_LENGTH-penalty);}
x73
ChangeIntoRootChain(xG&tree,bool
iP2,long
tT2,long
tU2){while(tU2>0
nP2
cCbrt);nQ2
tmp
eM1
tree.tV2--tU2;}
while(tT2>0
nP2
cSqrt);if(iP2){tmp
c4
cRSqrt);iP2
e23}
nQ2
tmp
eM1
tree.tV2--tT2;}
if(iP2
nP2
cInv);nQ2
tree.tV2}
}
yD
cP2
RootPowerTable{static
const
l62
RootPowers[(1+4)*(1+3)];}
;yD
const
l62
tF(1+4)*(1+3)]={l62(1)lT
eQ3
eQ3
2*eQ3
2*2*2)lT
3)lT
3*2)lT
3*2*2)lT
3*2*2*2)lT
3*2*2*2*2)lT
3*3
i32
2
i32
2*2
i32
2*2*2
i32
2*2*2*2
i32
3
i32
3*2
i32
3*2*2
i32
3*2*2*2
i32
3*2*2*2*2)}
;cP2
PowiResolver{static
const
unsigned
MaxSep=4;static
nK3
MaxOp=5
tO3
int
c53
tO3
long
nS3
tO3
long
tH;cP2
yN2{yN2():n_int_sqrt(0),n_int_cbrt(0),sep_list(),lU1(0){}
int
n_int_sqrt;int
n_int_cbrt;int
tG1
MaxSep];tH
lU1;}
;yD
static
yN2
CreatePowiResult(l62
x32{yN2
nT3;c53
tG=FindIntegerFactor(x32;if(tG==0){
#ifdef DEBUG_POWI
tW2"no factor found for %Lg\n"
,e21
x32;
#endif
return
nT3;}
cG3=xW1
nU3,tG);nS3
eP2=EvaluateFactorCost(tG,0,0,0)+cH
cG3);int
eS3=0;int
eT3=0;int
nJ3=0;
#ifdef DEBUG_POWI
tW2"orig = %Lg\n"
,e21
x32;tW2"plain factor = "
tA3"%ld\n"
,(int)tG,(long)eP2);
#endif
for
nD2
n_s=0;n_s<MaxSep;++n_s){int
xJ=0;nS3
yA1=eP2;c53
yI1=tG;for(int
s=1;s<MaxOp*4;++s){
#ifdef CBRT_IS_SLOW
if(s>=MaxOp)break;
#endif
int
n_sqrt=s%MaxOp;int
n_cbrt=s/MaxOp;if(n_sqrt+n_cbrt>4)y41
l62
lD1=nU3;lD1-=tF
s];tT1=FindIntegerFactor(lD1);if(xH2!=0){tH
xO=xW1
lD1,xH2);nS3
cost=EvaluateFactorCost(xH2,eS3+n_sqrt,eT3+n_cbrt,nJ3+1)+cH
xO);
#ifdef DEBUG_POWI
tW2"Candidate sep %u (%d*sqrt %d*cbrt)factor = "
tA3"%ld (for %Lg to %ld)\n"
,s,n_sqrt,n_cbrt,xH2,(long)cost,e21
lD1,(long)xO);
#endif
if(cost<yA1){xJ=s;yI1=xH2;yA1=cost;}
}
}
if(!xJ)break;
#ifdef DEBUG_POWI
tW2"CHOSEN sep %u (%d*sqrt %d*cbrt)factor = "
tA3"%ld, exponent %Lg->%Lg\n"
,xJ,xJ%MaxOp,xJ/MaxOp,yI1,yA1,e21(x32,e21(nU3-tF
xJ]));
#endif
tX2
tG1
n_s]=xJ
t11-=tF
xJ];eS3+=xJ%MaxOp;eT3+=xJ/MaxOp;eP2=yA1;tG=yI1;nJ3+=1;}
cG3=xW1
nU3,tG);
#ifdef DEBUG_POWI
tW2"resulting exponent is %ld (from exponent=%Lg, best_factor=%Lg)\n"
,cG3,e21
nU3,e21
tG);
#endif
while(tG%2==0){++tX2
n_int_sqrt;tG/=2;}
while(tG%3==0){++tX2
n_int_cbrt;tG/=3
yC3
nT3;}
private:static
nS3
cH
tH
xO){static
std::map
cL2
iC;if(xO<0){nS3
cost=22
eO
cost+cH-xO);}
std::map
cL2::xR3
i=iC.xI2
xO);if(i!=iC.cH1
xO)return
i
cW2;std::pair
cL2
nT3(xO,0.0);nS3&cost=tX2
second;while(xO>1){int
xH2=0;if(xO<256){xH2=x51::powi_table[xO];if(xH2&128)xH2&=127;else
xH2=0;if(xH2&64)xH2=-(xH2&63)-1;}
if(xH2){cost+=cH
xH2);xO/=xH2;y41}
if(!(xO&1)){xO/=2;cost+=6;}
else{cost+=7;xO-=1;}
}
iC.xN3,nT3)eO
cost;}
yD
static
tH
xW1
yB1,tT1)yS
makeLongInteger(value*l62(xH2));}
yD
static
bool
yC1
yB1,tT1){l62
v=value*l62(xH2)eO
isLongInteger(v);}
yD
static
c53
FindIntegerFactor(yB1){tT1=(2*2*2*2);
#ifdef CBRT_IS_SLOW
#else
xH2*=(3*3*3);
#endif
c53
nT3=0;if(yC1
value,xH2)){nT3=xH2;while((xH2%2)==0&&yC1
value,xH2/2))nT3=xH2/=2;while((xH2%3)==0&&yC1
value,xH2/3))nT3=xH2/=3;}
#ifdef CBRT_IS_SLOW
if(nT3==0
cI3
yC1
value,3
tJ2
3;}
#endif
return
nT3;}
static
int
EvaluateFactorCost(int
xH2,int
s,int
c,int
nmuls){nK3
nL3=6;
#ifdef CBRT_IS_SLOW
nK3
eQ2=25;
#else
nK3
eQ2=8;
#endif
int
nT3=s*nL3+c*eQ2;while(xH2%2==0){xH2/=2;nT3+=nL3;}
while(xH2%3==0){xH2/=3;nT3+=eQ2;}
nT3+=nmuls
eO
nT3;}
}
;}
tC{yD
bool
xG::RecreateInversionsAndNegations(bool
prefer_base2){bool
changed
e23
n72
0;a<x31++a)if(lP1.RecreateInversionsAndNegations(prefer_base2))nS2
if(changed){exit_changed:Mark_Incompletely_Hashed()cT2
switch(l42{case
cMul:{nX2
lK2;xG
lL2,c81;if(true){bool
nE1
e23
l62
nR2=0;n72
x31
a
nX
y33
0)y43
tU
yO2){nE1=true;nR2=tU
1).xE1;yN3
if(nE1){l62
immeds=1.0;n72
x31
a
nX
yP2)){immeds*=powgroup.xE1;e71}
n72
x31
a-->0;nQ&powgroup=lP1;if(powgroup
y33
0)y43
tU
yO2
nQ&log2=tU
0);log2.l41
log2
c4
nM3
log2.yJ
fp_pow(immeds,l62(1)/nR2)));log2
eM1
yN3}
}
n72
x31
a
nX
y33
yO2){iT1
exp_param=tU
1);l62
cY2
exp_param.xE1;if(cJ1,l62(-1))){l41
lK2
yS2
lP1
xN2
e71
i81
nU3<tD1)&&t12
x32
nQ
iL;iL
c4
cPow);iL
cM
tU
0));iL.yJ-x32);iL
eM1
lK2
yS2
iL);l41
e71}
i81
powgroup
y43!lL2.n12){lL2=tU
0);l41
e71
i81
powgroup
nF==cLog2by&&!c81.n12){c81=powgroup;l41
e71}
if(!lK2
yU3){nS2
xG
i91;i91
c4
cMul);i91
iY1
lK2);i91
eM1
xG
xX1
cMul);xY1
SetParamsMove(t4
if(xY1
IsImmed()&&fp_equal(xY1
xE1
n53
lM2
cInv);eE
i91);}
eN3
xY1
xT2>=i91.xT2){lM2
cDiv)c91
eE
i91);}
else{lM2
cRDiv);eE
i91)c91}
}
}
if(lL2.n12
nQ
xX1
l42;xY1
SetParamsMove(t4
while(xY1
RecreateInversionsAndNegations(prefer_base2))xY1
FixIncompleteHashes();lM2
nM3
eE
lL2)c91
nS2}
if(c81.n12
nQ
xX1
cMul);i82
y7
c81
l8
1));xY1
AddParamsMove(t4
while(xY1
RecreateInversionsAndNegations(prefer_base2))xY1
FixIncompleteHashes();DelParams();lM2
nM3
eE
c81
l8
0))c91
nS2
i31
cAdd:{nX2
tY2;n72
n93
c63
cMul){lN2
xZ1:;xG&i82
c73
x62
xY1
x31
b-->0;cI3
i82
l8
b).lK1
xH2=i82
l8
b).xE1;if(fp_equal(xH2
xV
xZ1;}
xY1
l41
xY1
DelParam(b
yT2
i81
fp_equal(xH2,l62(-2)))eG
xZ1;}
xY1
l41
xY1
DelParam(b);xY1
yJ
l62(2))yT2}
}
if(t9){xY1
tI
i82);e71}
i81
c63
cDiv&&!IsIntType
yB::nT3){lN2
y01:;xG&i91
c73
if(i91
l8
0).lO2
fp_equal(i91
l8
0).xE1
xV
y01;}
i91.l41
i91.DelParam(0);i91
c4
cInv
yT2}
if(t9)eG
y01;}
i91.tI
i91);e71}
i81
c63
cRDiv&&!IsIntType
yB::nT3){lN2
nZ1:;xG&i91
c73
if(i91
l8
1).lO2
fp_equal(i91
l8
1).xE1
xV
nZ1;}
i91.l41
i91.DelParam(1);i91
c4
cInv
yT2}
if(t9)eG
nZ1;}
i91.tI
i91);e71}
if(!tY2
yU3){
#ifdef DEBUG_SUBSTITUTIONS
tW2"Will make a Sub conversion in:\n"
);fflush(stdout);iQ
#endif
xG
t71;t71
c4
cAdd);t71
iY1
tY2);t71
eM1
xG
cD1;cD1
c4
cAdd);cD1
iY1
iP1));cD1
eM1
if(cD1
yP2)&&fp_equal(cD1.xE1,tD1))){lM2
cNeg
nN3);}
eN3
cD1.xT2==1){lM2
cRSub
nN3);eE
cD1);}
i81
t71
nF==cAdd){lM2
cSub);eE
cD1
nN3
xN2
n72
1;a<t71.x31++a
nQ
eR2;eR2
c4
cSub);eR2
iY1
iP1));eR2.lF2
eE
eR2
nN3
lS3}
}
else{lM2
cSub);eE
cD1
nN3);}
}
#ifdef DEBUG_SUBSTITUTIONS
tW2"After Sub conversion:\n"
);fflush(stdout);iQ
#endif
i31
cPow:{iT1
p0
nT2
0);iT1
p1
nT2
1);if(p1.lO2
p1.xE1!=tD1)&&!t12
p1.xE1)){eJ
yN2
r=eJ
CreatePowiResult(fp_abs(p1.xE1));if(r.lU1!=0){bool
iN1
e23
if(p1.xE1<tD1)&&r.tG1
0]==0&&r.n_int_sqrt>0){iN1=true;}
#ifdef DEBUG_POWI
tW2"Will resolve powi %Lg as powi(chain(%d,%d),%ld)"
,e21
fp_abs(p1.xE1),r.n_int_sqrt,r.n_int_cbrt,r.lU1);for
nD2
n=0;n<eJ
MaxSep;++n
cI3
r.tG1
n]==0)break;int
n_sqrt=r.tG1
n]%eJ
MaxOp;int
n_cbrt=r.tG1
n]/eJ
MaxOp;tW2"*chain(%d,%d)"
,n_sqrt,n_cbrt);}
tW2"\n"
);
#endif
xG
cM2
nT2
0);xG
yU2=cM2;yU2.l41
ChangeIntoRootChain(yU2,iN1,r.n_int_sqrt,r.n_int_cbrt);yU2
eM1
xG
pow;if(r.lU1!=1){pow
c4
cPow);pow
y7
yU2);pow.yJ
l62(r.lU1)));}
else
pow.swap(yU2);xG
mul;mul
c4
cMul);mul
y7
pow);for
nD2
n=0;n<eJ
MaxSep;++n
cI3
r.tG1
n]==0)break;int
n_sqrt=r.tG1
n]%eJ
MaxOp;int
n_cbrt=r.tG1
n]/eJ
MaxOp;xG
eS2=cM2;eS2.l41
ChangeIntoRootChain(eS2,false,n_sqrt,n_cbrt);eS2
eM1
mul
y7
eS2);}
if(p1.xE1<tD1)&&!iN1){mul
eM1
lM2
cInv);n41
0,mul);DelParam(1);}
else{lM2
cMul);SetParamsMove(mul.iP1));}
#ifdef DEBUG_POWI
iQ
#endif
nS2
yN3}
if(GetOpcode()==cPow&&(!p1
yP2)||!isLongInteger(p1.xE1)||!IsOptimizableUsingPowi
yB(makeLongInteger(p1.xE1)))cI3
p0
yP2)&&p0.xE1>l62(0.0)cI3
prefer_base2){l62
yV2=fp_log2(p0.xE1);if(fp_equal(yV2
n53
DelParam(0);}
else{lX
cL1
yV2))t11
cM
p1)t11.Rehash(x41}
lM2
cExp2);nS2}
else{l62
yV2=fp_log(p0.xE1);if(fp_equal(yV2
n53
DelParam(0);}
else{lX
cL1
yV2))t11
cM
p1)t11.Rehash(x41}
lM2
cExp);nS2}
}
i81
GetPositivityInfo(p0)==IsAlways
cI3
prefer_base2
nQ
log;log
c4
cLog2);log
cM
p0);log
eM1
lX
p1)t11
y7
log)t11
eM1
lM2
cExp2
x41
nS2}
else{xG
log;log
c4
cLog);log
cM
p0);log
eM1
lX
p1)t11
y7
log)t11
eM1
lM2
cExp
x41
nS2}
}
i31
cDiv:{if(GetParam(0)yP2)&&fp_equal(GetParam(0).xE1
n53
lM2
cInv);DelParam(0);}
yY3
y13
yY3
if(changed)goto
exit_changed
eO
changed;}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
using
iR2
FUNCTIONPARSERTYPES;iR2{using
tC;class
cJ3{size_t
nF1;size_t
eK;size_t
eL;size_t
lE1;size_t
tA;size_t
tB;size_t
n01;cV3
cJ3():nF1(0),eK(0),eL(0),lE1(0),tA(0),tB(0),n01(0){}
void
AddFrom(OPCODE
op){nF1+=1;if(op==cCos)++eK;if(op==cSin)++eL;if(op==cSec)++eK;if(op==cCsc)++eL;if(op==cTan)++lE1;if(op==cCot)++lE1;if(op==cSinh)++tB;if(op==cCosh)++tA;if(op==cTanh)++n01;}
size_t
GetCSEscore()const{size_t
nT3=nF1
eO
nT3;}
int
NeedsSinCos()const{bool
y11=(nF1==(eK+eL+lE1));if((lE1&&(eL||eK))||(eL&&eK)cI3
y11)return
1
eO
2
yC3
0;}
int
NeedsSinhCosh()const{bool
y11=(nF1==(tA+tB+n01));if((n01&&(tB||tA))||(tB&&tA)cI3
y11)return
1
eO
2
yC3
0;}
size_t
MinimumDepth()const{size_t
n_sincos=std::min(eK,eL);size_t
n_sinhcosh=std::min(tA,tB);if(n_sincos==0&&n_sinhcosh==0)return
2
eO
1;}
}
;x13
TreeCountType:public
std::multimap<fphash_t,std::pair<cJ3,xG> >{}
;x73
FindTreeCounts(t31&lQ2,xB3,OPCODE
nU2,bool
skip_root=false){e2
i=lQ2.xI2
xC3);if(!skip_root){bool
found
e23
for(;i!=lQ2.cH1
xC3;++i
cI3
tree
xI
i
cW2.second)){i
cW2
y93
AddFrom(nU2);found=true;yN3
if(!found){cJ3
count;count.AddFrom(nU2);lQ2.xN3,std::make_pair(xC3,std::make_pair(count,tree)));}
}
n72
0
tH2
FindTreeCounts(lQ2,tJ3
tree
nF);}
cP2
yX{bool
BalanceGood;bool
FoundChild;}
;yD
yX
lF1
iT1
root,iT1
child
cI3
root
xI
child)){yX
nT3={true,true}
eO
nT3;}
yX
nT3={true,false}
;if(root
nF==cIf||root
nF==e63{yX
cond=lF1
root
l8
0
x03
yX
y2=lF1
root
l8
1
x03
yX
y6=lF1
root
l8
2
x03
if
lF3||y2
yY||y6
yY){nT3
yY=true;}
nT3
eH=((y2
yY==y6
yY)||lF3
cN2&&(cond
eH||(y2
yY&&y6
yY))&&(y2
eH||lF3
cN2&&(y6
eH||lF3
cN2;}
else{bool
tU1
e23
bool
nG1
e23
x62
root.GetParamCount(),a=0;a<b;++a){yX
tmp=lF1
root
l8
a
x03
if(tmp
yY)nT3
yY=true;if(tmp
eH==false)tU1=true;i81
tmp
yY)nG1=true;}
if(tU1&&!nG1)nT3
eH=false
yC3
nT3;}
yD
bool
n83
iX2
eU3
xB3,const
x51
yZ3&synth,const
t31&lQ2){x62
t7,a=0;a<b;++a){iT1
leaf=xP1
a);e2
synth_it;xL2
t31::const_iterator
i=lQ2.xQ3
i!=lQ2.end();++i
cI3
i->first!=leaf.GetHash())y41
const
cJ3&occ
n22
first;size_t
score=occ.GetCSEscore();iT1
candidate
n22
second;lS2
candidate))y41
if(leaf.xT2<occ.MinimumDepth())y41
if(score<2)y41
if(lF1
eU3
leaf)eH==false)continue
cT2
if(n83(eU3
leaf,synth,lQ2
t41
yC3
e83
yD
bool
lR2
iX2
xO3,iT1
expr){yE
xO3
l8
a)xI
expr
t41;yE
lR2(xO3
l8
a),expr
t41
eO
e83
yD
bool
GoodMomentForCSE
iX2
xO3,iT1
expr
cI3
xO3
nF==cIf)return
true;yE
xO3
l8
a)xI
expr
t41;size_t
tZ2=0;yE
lR2(xO3
l8
a),expr))++tZ2
eO
tZ2!=1;}
}
tC{yD
size_t
xG::SynthCommonSubExpressions(x51::yM1
const{if(GetParamCount()==0)return
0;size_t
stacktop_before
eJ3
GetStackTop();t31
lQ2;FindTreeCounts(lQ2,*this,GetOpcode(),true);for(;;){size_t
yW2=0;
#ifdef DEBUG_SUBSTITUTIONS_CSE
std::cout<<"Finding a CSE candidate, root is:"
<<std::endl;DumpHashes(*this);
#endif
e2
cs_it(lQ2.end());for(e2
j=lQ2.xQ3
j!=lQ2.end();){e2
i(j++);const
cJ3&occ
n22
first;size_t
score=occ.GetCSEscore();xB3
n22
second;
#ifdef DEBUG_SUBSTITUTIONS_CSE
std::cout<<"Score "
<<score<<":\n"
<<std::flush;DumpTreeWithIndent(tree);
#endif
lS2
tree))xS
if(tree.xT2<occ.MinimumDepth())xS
if(score<2)xS
if(lF1*this,tree)eH==false)xS
if(n83(*this,tree,synth,lQ2)){y41}
if(!GoodMomentForCSE(*this,tree))xS
score*=tree.xT2;if(score>yW2){yW2=score;cs_it=i;}
}
if(yW2<=0){
#ifdef DEBUG_SUBSTITUTIONS_CSE
std::cout<<"No more CSE candidates.\n"
<<std::flush;
#endif
yY3
xB3=cs_it
cW2.second;
#ifdef DEBUG_SUBSTITUTIONS_CSE
std::cout<<tF3"Common Subexpression:"
;DumpTree
yB(tree)xO1
std::endl;
#endif
#if 0
int
lV1=occ.NeedsSinCos();int
i4=occ.NeedsSinhCosh();xG
i02,i12,yX2,yY2;if(lV1){i02
eT2
i02
c4
cSin);i02
eM1
i12
eT2
i12
c4
cCos);i12
eM1
lS2
i02)||synth.Find(i12)cI3
lV1==2){t61
y41}
lV1=0;}
}
if(i4){yX2
eT2
yX2
c4
cSinh);yX2
eM1
yY2
eT2
yY2
c4
cCosh);yY2
eM1
lS2
yX2)||synth.Find(yY2)cI3
i4==2){t61
y41}
i4=0;}
}
#endif
tree.SynthesizeByteCode(synth,false);t61
#ifdef DEBUG_SUBSTITUTIONS_CSE
synth.template
Dump<0>()xO1"Done with Common Subexpression:"
;DumpTree
yB(tree)xO1
std::endl;
#endif
#if 0
if(lV1
cI3
lV1==2||i4){synth.eC1}
n42
cSinCos,1,2)cZ1
i02,1)cZ1
i12,0);}
if(i4
cI3
lV1)synth.eC1
if(i4==2){synth.eC1}
n42
cSinhCosh,1,2)cZ1
yX2,1)cZ1
yY2,0);}
#endif
}
return
synth.xH
stacktop_before;}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
yD
lI1
yB::i22{using
tC;l41
xG
tree;tree.GenerateFrom(*mData);FPoptimizer_Optimize::ApplyGrammars(tree)e72
unsigned>c83;std::vector
yB
immed;size_t
stacktop_max=0;tree.SynthesizeByteCode(c83,immed,stacktop_max);if(mData->mStackSize!=stacktop_max){mData->mStackSize=unsigned(stacktop_max);
#if !defined(FP_USE_THREAD_SAFE_EVAL) && \
    !defined(FP_USE_THREAD_SAFE_EVAL_WITH_ALLOCA)
mData->mStack
x53
stacktop_max);
#endif
}
mData->mByteCode.swap(c83);mData->mImmed.swap(immed);}
#define FUNCTIONPARSER_INSTANTIATE_EMPTY_OPTIMIZE(type) t91>lI1<type>::i22{}
#ifdef FP_SUPPORT_MPFR_FLOAT_TYPE
eX3(MpfrFloat)
#endif
#ifdef FP_SUPPORT_GMP_INT_TYPE
eX3(GmpInt)
#endif
#ifdef FP_SUPPORT_COMPLEX_DOUBLE_TYPE
eX3(std::complex<double>)
#endif
#ifdef FP_SUPPORT_COMPLEX_FLOAT_TYPE
eX3(std::complex<float>)
#endif
#ifdef FP_SUPPORT_COMPLEX_LONG_DOUBLE_TYPE
eX3(std::complex<long
double>)
#endif
#define FUNCTIONPARSER_INSTANTIATE_OPTIMIZE(type) template lI1<type>::i22;
#ifndef FP_DISABLE_DOUBLE_TYPE
eY3(double)
#endif
#ifdef FP_SUPPORT_FLOAT_TYPE
eY3(float)
#endif
#ifdef FP_SUPPORT_LONG_DOUBLE_TYPE
eY3(long
double)
#endif
#ifdef FP_SUPPORT_LONG_INT_TYPE
eY3(long)
#endif
#endif // FP_SUPPORT_OPTIMIZER

#endif
