/***************************************************************************\
|* Function Parser for C++ v4.4.3                                          *|
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
#define lN4 if(n6 l24
#define lM4 .lD1 y3
#define lL4 Needs
#define lK4 tree.xQ1
#define lJ4 "Found "
#define lI4 stackpos
#define lH4 "PUSH "yA3
#define lG4 "dup(%u) "
#define lF4 eV{assert
#define lE4 "%d, cost "
#define lD4 ::cout<<l44
#define lC4 "immed "<<
#define lB4 mFuncParsers
#define lA4 stderr
#define l94 sep2=" "
#define l84 -iW2 63)-1;
#define l74 FPHASH_CONST
#define l64 cache_needed[
#define l54 fprintf
#define l44 "Applying "
#define l34 ||tree.GetOpcode
#define l24 HANDLE_UNARY_CONST_FUNC
#define l14 {cSinh,
#define l04 },0,0x4
#define iZ3 xZ a)
#define iY3 xI(i42
#define iX3 tO info
#define iW3 &&IsLogicalValue(
#define iV3 xZ 1)nD==
#define iU3 .n8 synth.
#define iT3 known||(nW2
#define iS3 ),Value(
#define iR3 );if(
#define iQ3 within,
#define iP3 ;return
#define iO3 )iP3 Ok;}
#define iN3 tK1){if
#define iM3 cLog2&&
#define iL3 eL3 yJ2
#define iK3 c_count
#define iJ3 s_count
#define iI3 MaxOp
#define iH3 2)lT 2*
#define iG3 0));t0
#define iF3 nM 0));
#define iE3 tmp2.nM
#define iD3 Unknown
#define iC3 else nP
#define iB3 known&&
#define iA3 lT2 val
#define i93 tree.lV
#define i83 default_function_handling
#define i73 sim.x71
#define i63 ].swap(
#define i53 codes[b
#define i43 whydump
#define i33 nparams
#define i23 444848,
#define i13 l2 2,2,
#define i03 ,cIf,cI3
#define tZ3 l3 0,1,
#define tY3 cHypot,
#define tX3 t1 1,0,
#define tW3 nU 0,
#define tV3 cAbs nU
#define tU3 b.Value)
#define tT3 b.Opcode
#define tS3 Params[
#define tR3 Params(
#define tQ3 )xC1 2)
#define tP3 leaf1
#define tO3 cAbsIf)
#define tN3 AddFrom(
#define tM3 =fp_pow(
#define tL3 ,l5 2,1,
#define tK3 =false;
#define tJ3 ==cOr)l43
#define tI3 cI cMul);
#define tH3 ;}static n02
#define tG3 .size()
#define tF3 ].first
#define tE3 Ne_Mask
#define tD3 7168,
#define tC3 +=1;e31
#define tB3 Gt_Mask
#define tA3 Lt_Mask
#define t93 opcode,
#define t83 public:
#define t73 {data->
#define t63 },{l4::
#define t53 pclone
#define t43 Immeds
#define t33 c0 nA2
#define t23 c0 iE,
#define t13 cOr,l6
#define t03 info.
#define eZ3 ){if(nA1
#define eY3 );Value
#define eX3 };enum
#define eW3 ;}void
#define eV3 xR)eW3
#define eU3 ParamHolder
#define eT3 NumConstant:
#define eS3 ){case
#define eR3 newpow
#define eQ3 change
#define eP3 (count
#define eO3 value)
#define eN3 ,lB1+1);
#define eM3 value]
#define eL3 value,
#define eK3 133,2,
#define eJ3 eH3 l73
#define eI3 eH3 c1 val
#define eH3 result
#define eG3 byteCode
#define eF3 eS cV1);
#define eE3 xR2 a eQ
#define eD3 n51 nD==
#define eC3 cLog2by
#define eB3 cPow&&tT
#define eA3 factor_t
#define e93 value1
#define e83 Finite
#define e73 a)iR3!
#define e63 fp_mod(
#define e53 else{if(
#define e43 xI());nE
#define e33 c1 val)<
#define e23 p c1 val
#define e13 iZ);}if(
#define e03 {tree.xE
#define cZ3 cAbsNot
#define cY3 stackptr
#define cX3 (tree)!=
#define cW3 1),l63(1));
#define cV3 FP_GetOpcodeName(
#define cU3 cLog);xJ
#define cT3 .empty()
#define cS3 opcodes
#define cR3 did_muli
#define cQ3 &Value){
#define cP3 yG const
#define cO3 used[b]
#define cN3 :if(&*lV1){
#define cM3 :{nB1 r=
#define cL3 sizeof(
#define cK3 iY2 lI1
#define cJ3 cAbsIf,
#define cI3 l3 16,1,
#define cH3 281856,
#define cG3 cLess,cS
#define cF3 cTan,yV2
#define cE3 Ge0Lt1
#define cD3 switch(n61
#define cC3 )){data xB
#define cB3 xZ 0)nD
#define cA3 ))return
#define c93 IsLogicalValue(xZ
#define c83 cLog,yV2
#define c73 lK 2},0,
#define c63 middle2
#define c53 ::string
#define c43 param.
#define c33 &param=*
#define c23 l63(2)));
#define c13 ){switch(
#define c03 ()const{
#define yZ3 .tV1 n]
#define yY3 break;}
#define yX3 default:
#define yW3 {l4::xD2
#define yV3 =*(cX*xY1
#define yU3 ;else eH3
#define yT3 range x23
#define yS3 )yT1 Rehash()
#define yR3 range<cG2
#define yQ3 range xI
#define yP3 cAdd lX2
#define yO3 (op1==
#define yN3 ))==IsAlways)
#define yM3 cZ1;++b)
#define yL3 iterator
#define yK3 begin();
#define yJ3 TreeSet
#define yI3 parent
#define yH3 insert(i
#define yG3 newrel
#define yF3 void set
#define yE3 b_needed
#define yD3 cachepos
#define yC3 half,iB,
#define yB3 half=
#define yA3 ;DumpTree(
#define y93 ;o<<"\n";
#define y83 fp_equal(
#define y73 (y83
#define y63 131,4,1,
#define y53 131,8,1,
#define y43 4,1,2,1,
#define y33 ::vector
#define y23 FindPos(
#define y13 src_pos
#define y03 reserve(
#define xZ3 const std::eT
#define xY3 const char*
#define xX3 yP1 void
#define xW3 treeptr
#define xV3 .resize(
#define xU3 tS1 void
#define xT3 ImmedTag
#define xS3 );m c1 nV2
#define xR3 ){half&=127;
#define xQ3 a,const
#define xP3 RefCount
#define xO3 Birth();
#define xN3 exponent
#define xM3 template
#define xL3 unsigned
#define xK3 start_at
#define xJ3 cost_t
#define xI3 lE3 size()
#define xH3 ;}case
#define xG3 .known
#define xF3 fpdata
#define xE3 middle
#define xD3 sqrt_cost
#define xC3 const int
#define xB3 mul_count
#define xA3 yR1.SubTrees
#define x93 yR1.Others
#define x83 ;eH3
#define x73 maxValue1
#define x63 minValue1
#define x53 maxValue0
#define x43 minValue0
#define x33 ValueType
#define x23 xI eH3
#define x13 c1 n4 0),
#define x03 yO n4 0),
#define nZ3 iV);nE lD
#define nY3 abs_mul
#define nX3 pos_set
#define nW3 goto e3
#define nV3 sim.x3 1,
#define nU3 {sim.Eat(
#define nT3 subtree
#define nS3 invtree
#define nR3 MakeHash(
#define nQ3 ,lV1,info
#define nP3 parampair
#define nO3 rulenumit
#define nN3 l7 0,2,
#define nM3 l7 0,1,
#define nL3 cEqual,
#define nK3 nL3 lB
#define nJ3 lB 0x4 nJ
#define nI3 0x4},{{3,
#define nH3 cNeg,lU 1
#define nG3 MakeEqual
#define nF3 nJ1,l4::
#define nE3 nJ1,{l4::
#define nD3 newbase
#define nC3 branch1op
#define nB3 branch2op
#define nA3 ContainsOtherCandidates
#define n93 l9 a)xL
#define n83 overlap
#define n73 truth_b
#define n63 truth_a
#define n53 found_dup
#define n43 eW r;r cI
#define n33 eH3=yJ2
#define n23 .second
#define n13 ,tree x6
#define n03 n23);
#define lZ3 e11 eW&
#define lY3 comp.eX[a
#define lX3 rangeutil
#define lW3 synth lS2
#define lV3 Plan_Has(
#define lU3 StackMax)
#define lT3 eH3 xT
#define lS3 const xU2
#define lR3 namespace
#define lQ3 ByteCode[
#define lP3 tC=!tC;}
#define lO3 inverted
#define lN3 IsNever:
#define lM3 return p
#define lL3 iftree
#define lK3 }switch(
#define lJ3 depcodes
#define lI3 explicit
#define lH3 cCosh nU
#define lG3 VarBegin
#define lF3 tS3 a]
#define lE3 Params.
#define lD3 ].data);
#define lC3 iV)));nY
#define lB3 PlusInf
#define lA3 =true;c21
#define l93 .what nW1
#define l83 (tP3 l9
#define l73 .min.val
#define l63 Value_t
#define l53 ;l63
#define l43 ?0:1))l8
#define l33 (mulgroup
#define l23 begin(),
#define l13 cond_add
#define l03 cond_mul
#define iZ2 cond_and
#define iY2 {if(rule.
#define iX2 ;tree eR2
#define iW2 (half&
#define iV2 const eN
#define iU2 bool eM1
#define iT2 costree
#define iS2 sintree
#define iR2 leaf_count
#define iQ2 sub_params
#define iP2 printf(
#define iO2 swap(tmp);
#define iN2 cbrt_count
#define iM2 sqrt_count
#define iL2 .min xG3
#define iK2 c3 2,cAdd)
#define iJ2 pcall_tree
#define iI2 after_powi
#define iH2 GetHash().
#define iG2 ;lV1=r.specs;if(r.found){
#define iF2 eY1=0;y02
#define iE2 x9)[0].info
#define iD2 eY1;if xX2
#define iC2 yM false;}
#define iB2 params)
#define iA2 grammar
#define i92 cLog nU
#define i82 ,cAnd,l6
#define i72 l1 xF2
#define i62 x2 lU 2,
#define i52 },{{1,
#define i42 ),0},{
#define i32 std::move(
#define i22 data;data.
#define i12 ;synth.yU
#define i02 cI cond nD
#define tZ2 tree cI
#define tY2 tree))tG1
#define tX2 tree nD
#define tW2 MakeNEqual
#define tV2 Dump(std::
#define tU2 isInteger(
#define tT2 Comparison
#define tS2 needs_flip
#define tR2 (l63
#define tQ2 iW1 apos==
#define tP2 ~size_t(0)
#define tO2 xJ1 xV+1);
#define tN2 Rule&rule,
#define tM2 >::res,b8<
#define tL2 ),child);
#define tK2 mul_item
#define tJ2 innersub
#define tI2 cbrt_cost
#define tH2 best_cost
#define tG2 2)lT 3*3*
#define tF2 preserve=
#define tE2 condition
#define tD2 TopLevel)
#define tC2 per_item
#define tB2 item_type
#define tA2 first2
#define t92 cGreater,
#define t82 cIf,t1 3,
#define t72 t03 lR[b].
#define t62 if(tX2==
#define t52 l0 2,
#define t42 lK 1},0,
#define t32 t8 1},0,
#define t22 Decision
#define t12 not_tree
#define t02 Become(xZ
#define eZ2 group_by
#define eY2 xN3=
#define eX2 );sim.x3 2,
#define eW2 eH3))nD2
#define eV2 eU2 eH3(
#define eU2 ){l63
#define eT2 nT eU2 tmp=
#define eS2 (lS));nE lD
#define eR2 .SetParam(
#define eQ2 ->second
#define eP2 return nL
#define eO2 targetpos
#define eN2 ParamSpec
#define eM2 )continue;if(
#define eL2 ;iB.Remember(
#define eK2 rhs.hash2;}
#define eJ2 rhs.hash1
#define eI2 struct
#define eH2 Forget()
#define eG2 AddParam(
#define eF2 source_tree
#define eE2 .n_int_sqrt
#define eD2 <tM,xJ3>
#define eC2 CodeTree lX
#define eB2 p1_evenness
#define eA2 isNegative(
#define e92 c1 iB3(
#define e82 ,std::cout)
#define e72 ByteCodeSynth xI
#define e62 cNop,cNop}}
#define e52 cTanh,cNop,
#define e42 >eI2 cM<
#define e32 matches
#define e22 else{x9=new
#define e12 (c43 data
#define e02 t92 cS
#define cZ2 cTan nU
#define cY2 cCos nU
#define cX2 return;case
#define cW2 negated
#define cV2 Specializer
#define cU2 coshtree
#define cT2 sinhtree
#define cS2 best_score
#define cR2 mulvalue
#define cQ2 pow_item
#define cP2 subgroup
#define cO2 IsDefined(
#define cN2 PowiResult
#define cM2 maxValue
#define cL2 minValue
#define cK2 yO known
#define cJ2 fp_min(yQ,
#define cI2 div_tree
#define cH2 pow_tree
#define cG2 l63 nW
#define cF2 );for iD1 a=
#define cE2 {std::cout<<
#define cD2 ,e11 void*)&
#define cC2 ,l5 0,1,
#define cB2 ifdata.ofs
#define cA2 (IfData&ifdata
#define c92 i8 push_back(
#define c82 i8 size()
#define c72 y33<xL3>&tD1
#define c62 );yY3
#define c52 PullResult()
#define c42 dup_or_fetch
#define c32 nominator]
#define c22 test_order
#define c12 nP3,
#define c02 .param_count
#define yZ2 shift(index)
#define yY2 rulenumber
#define yX2 cLessOrEq,cS
#define yW2 nL3 t52
#define yV2 l3 2,1,
#define yU2 ,l3 18,1,
#define yT2 cTanh nU
#define yS2 cSinh nU
#define yR2 cInv,lU 1,
#define yQ2 constraints=
#define yP2 factor_immed
#define yO2 changes
#define yN2 for(typename
#define yM2 exp_diff
#define yL2 ExponentInfo
#define yK2 lower_bound(
#define yJ2 factor
#define yI2 is_logical
#define yH2 newrel_and
#define yG2 eX[c eE
#define yF2 res_stackpos
#define yE2 half_pos
#define yD2 fphash_t
#define yC2 (tA==e83&&
#define yB2 ));yQ3
#define yA2 >>1)):(
#define y92 CodeTreeData
#define y82 if(list.first.
#define y72 xN3)
#define y62 var_trees
#define y52 );nY l4::
#define y42 std y33<bool>
#define y32 *)&*xK3;
#define y22 ,l63(1))){
#define y12 ,eU,synth);
#define y02 a<y5;++a)
#define xZ2 .match_tree
#define xY2 (rule,tree,info
#define xX2 (&*xK3){x9=(
#define xW2 const eW&
#define xV2 nC OPCODE
#define xU2 CodeTree&
#define xT2 parent_opcode
#define xS2 =i eQ2.
#define xR2 =GetParam(
#define xQ2 changed=true;
#define xP2 log2_exponent
#define xO2 l63(0.5))
#define xN2 dup_fetch_pos
#define xM2 .eG2
#define xL2 tree xM2
#define xK2 (*x9)[a].info
#define xJ2 Rehash(false)
#define xI2 {e1 xK3;
#define xH2 IsNever cJ lD
#define xG2 cSin nU
#define xF2 0x12 nJ
#define xE2 Value_EvenInt
#define xD2 MakeFalse,{l4
#define xC2 AddCollection
#define xB2 ConditionType
#define xA2 (xL3
#define x92 iA|xA2)
#define x82 SpecialOpcode
#define x72 fp_max(yQ);
#define x62 c1 iB3 p
#define x52 assimilated
#define x42 denominator
#define x32 fraction
#define x22 l2 18,2,
#define x12 .GetDepth()
#define x02 DUP_BOTH();
#define nZ2 xM3 lM
#define nY2 0x80000000u
#define nX2 .UseGetNeeded(
#define nW2 m c1 val
#define nV2 xM3 set_if<
#define nU2 xM3 lY
#define nT2 if(synth.Find(
#define nS2 IsDescendantOf
#define nR2 bool tC tK3
#define nQ2 SetOpcode(
#define nP2 found_log2
#define nO2 div_params
#define nN2 set(fp_floor);
#define nM2 immed_sum
#define nL2 Rehash cT
#define nK2 minimum_need
#define nJ2 .Rehash();
#define nI2 .DelParam(
#define nH2 :sim.Eat(1,
#define nG2 lQ3++IP]
#define nF2 OPCODE(opcode)
#define nE2 ;sim.Push(
#define nD2 break x83*=
#define nC2 FactorStack xI
#define nB2 IsAlways cJ lD
#define nA2 282870 xD
#define n92 cNotNot nU
#define n82 cNot nU
#define n72 replacing_slot
#define n62 RefParams
#define n52 if_always[
#define n42 WhatDoWhenCase
#define n32 exponent_immed
#define n22 new_base_immed
#define n12 base_immed
#define n02 inline xL3
#define lZ2 }inline
#define lY2 return false;}
#define lX2 ||op1==
#define lW2 data[a]n23
#define lV2 nL2 r);}
#define lU2 if(newrel_or==
#define lT2 )eH3.min.
#define lS2 .AddOperation(
#define lR2 DUP_ONE(apos);
#define lQ2 flipped
#define lP2 e7 2,131,
#define lO2 [xV-1-offset].
#define lN2 lQ3 a
#define lM2 y6 Immed tG3);
#define lL2 1 y6 c82
#define lK2 OptimizedUsing
#define lJ2 Var_or_Funcno
#define lI2 lJ2;
#define lH2 GetParams(
#define lG2 crc32_t
#define lF2 signed_chain
#define lE2 {case IsAlways:
#define lD2 known?Value(
#define lC2 MinusInf
#define lB2 n_immeds
#define lA2 stack tG3
#define l92 FindClone(xR
#define l82 lQ3 IP]
#define l72 GetOpcode())
#define l62 needs_rehash
#define l52 AnyWhere_Rec
#define l42 lS,xZ iV));nE
#define l32 AddParamMove(
#define l22 mulgroup.
#define l12 .l32
#define l02 ~xL3(0)
#define iZ1 TreeCountItem
#define iY1 ;if(op==
#define iX1 divgroup
#define iW1 else if(
#define iV1 41,42,43,44,
#define iU1 p1_logical_b
#define iT1 p0_logical_b
#define iS1 p1_logical_a
#define iR1 p0_logical_a
#define iQ1 cI tX2);
#define iP1 func(val);nV1
#define iO1 *const func)
#define iN1 synth.DoDup(
#define iM1 cache_needed
#define iL1 e7 2,1,e7 2,
#define iK1 treelist
#define iJ1 has_bad_balance
#define iI1 {if(GetOpcode()
#define iH1 2*2*2)lT 3
#define iG1 if(remaining[a])
#define iF1 ;eZ1.hash2+=
#define iE1 TreeCounts
#define iD1 (size_t
#define iC1 for iD1 b=0;b<
#define iB1 return true;}
#define iA1 .SetParamsMove(
#define i91 eA3 yJ2
#define i81 set(fp_ceil);t9
#define i71 fp_abs(max.val))
#define i61 fp_abs(min.val)
#define i51 cNEqual
#define i41 },0,0x0},{{
#define i31 t8 2 i41
#define i21 Oneness_NotOne|
#define i11 Value_IsInteger
#define i01 Constness_Const
#define tZ1 DumpHashesFrom(
#define tY1 lK2(
#define tX1 reltype
#define tW1 SequenceOpcodes
#define tV1 sep_list[
#define tU1 goto fail;}
#define tT1 xM3<
#define tS1 n62);
#define tR1 TreeCountType xI
#define tQ1 ,l63(-1)))xO
#define tP1 back().thenbranch
#define tO1 c1 known)
#define tN1 >tR2(1),
#define tM1 l63(0.0)){xM
#define tL1 CollectionSet xI
#define tK1 .IsImmed()
#define tJ1 a)tK1)
#define tI1 xZ 1)tK1&&
#define tH1 iL2&&
#define tG1 goto redo;
#define tF1 eG2 ifp2
#define tE1 }break xH3
#define tD1 ByteCode,size_t&IP,size_t limit,size_t y1
#define tC1 <<tree.iH2
#define tB1 nU2 void
#define tA1 ,cPow,
#define t91 synth.xK 1
#define t81 ,iB y12
#define t71 (cond yW&&cond eC))
#define t61 p1.nL2 p1);
#define t51 p1 xM2 ifp1
#define t41 std::cout<<"POP "
#define t31 stack[lA2-
#define t21 stack.push_back(
#define t11 synth.PushImmed(
#define t01 MaxChildDepth
#define eZ1 NewHash
#define eY1 xL3 a
#define eX1 std::pair<It,It>
#define eW1 Sign_Negative
#define eV1 Value_Logical
#define eU1 (constraints&
#define eT1 new_factor_immed
#define eS1 occurance_pos
#define eR1 exponent_hash
#define eQ1 exponent_list
#define eP1 CollectMulGroup(
#define eO1 source_set
#define eN1 xN3,yJ3
#define eM1 operator
#define eL1 ;synth.StackTopIs(
#define eK1 FindAndDup(tree);
#define eJ1 ParamSpec_Extract
#define eI1 retry_anyparams_3
#define eH1 retry_anyparams_2
#define eG1 e6(),std y33<
#define eF1 needlist_cached_t
#define eE1 grammar_rules[*r]
#define eD1 return eN2(
#define eC1 c73 0x4 i52
#define eB1 t42 0x4 i52
#define eA1 CodeTreeImmed xI(
#define e91 by_float_exponent
#define e81 y83 xN3
#define e71 new_exp
#define e61 end()&&i->first==
#define e51 return BecomeZero;
#define e41 =comp.AddItem(atree
#define e31 return nH1;
#define e21 return BecomeOne;
#define e11 (const
#define e01 e11 l63&
#define cZ1 .GetParamCount()
#define cY1 iE1.erase(cs_it);
#define cX1 cA3 true;
#define cW1 if(lR tG3<=n2)
#define cV1 addgroup
#define cU1 found_log2by
#define cT1 nD==cZ3)
#define cS1 ParsePowiMuli(
#define cR1 lJ2)
#define cQ1 c0 523510 xD
#define cP1 branch1_backup
#define cO1 branch2_backup
#define cN1 exponent_map
#define cM1 plain_set
#define cL1 LightWeight(
#define cK1 ,PowiCache&iB,
#define cJ1 if(value
#define cI1 nU2 cB
#define cH1 nU2 static
#define cG1 l32 tmp2);
#define cF1 yO val
#define cE1 cCos,l3 2,1
#define cD1 e72&synth)
#define cC1 should_regenerate=true;
#define cB1 should_regenerate,
#define cA1 Collection
#define c91 RelationshipResult
#define c81 Subdivide_Combine(
#define c71 );tree nI2
#define c61 )const yM
#define c51 rhs c61 hash1
#define c41 eG2 tree);
#define c31 best_sep_factor
#define c21 iW1!eH3
#define c11 needlist_cached
#define c01 return IsNever;yY1}
#define yZ1 return IsAlways;if(
#define yY1 return iD3;
#define yX1 t93 bool pad
#define yW1 );xR iA1
#define yV1 eZ1.hash1
#define yU1 ;SetParamMove xF1
#define yT1 ;xN3.
#define yS1 ;range.multiply(
#define yR1 NeedList
#define yQ1 nU2 bool
#define yP1 ;nU2
#define yO1 DelParam(a);}
#define yN1 MakesInteger(
#define yM1 const l63&value
#define yL1 best_sep_cost
#define yK1 MultiplicationRange
#define yJ1 pihalf_limits
#define yI1 c3 2,cMul);lD
#define yH1 c3 2,cPow);lD
#define yG1 n_stacked
#define yF1 AnyParams_Rec
#define yE1 79,122,123,158,159,
#define yD1 PositionalParams,0}
#define yC1 always_sincostan
#define yB1 Recheck_RefCount_Div
#define yA1 Recheck_RefCount_Mul
#define y91 mulgroup;mulgroup cI
#define y81 MultiplyAndMakeLong(
#define y71 cMul);tmp.iF3 tmp
#define y61 ,l1 0x0},{{3,
#define y51 l63(0)
#define y41 covers_plus1
#define y31 nF2);
#define y21 if(synth.FindAndDup(
#define y11 SynthesizeParam(
#define y01 grammar_func
#define xZ1 lR3 FPoptimizer_Optimize
#define xY1 )nP3 n23;
#define xX1 tree tK1 cJ
#define xW1 252415 xD 24830,
#define xV1 l2 0,2,165888 xD
#define xU1 Modulo_Radians},
#define xT1 eG2 leaf2 l9
#define xS1 eG2 tP3 l9
#define xR1 eG2 cond l9
#define xQ1 GetImmed()
#define xP1 Become(value l9 0));
#define xO1 PositionType
#define xN1 CollectionResult
#define xM1 const_offset
#define xL1 inline TriTruthValue
#define xK1 stacktop_desired
#define xJ1 SetStackTop(
#define xI1 FPoptimizer_ByteCode
#define xH1 ;std::cout<<
#define xG1 1)?(poly^(
#define xF1 (0,y72;DelParam(1);
#define xE1 t63 MakeNotP0,l4::
#define xD1 y51)
#define xC1 xL leaf2 l9
#define xB1 return lI
#define xA1 cond_type
#define x91 fphash_value_t
#define x81 Recheck_RefCount_RDiv
#define x71 SwapLastTwoInStack();
#define x61 fPExponentIsTooLarge(
#define x51 CollectMulGroup_Item(
#define x41 pair<l63,yJ3>
#define x31 nO xJ1 xV-1);
#define x21 covers_full_cycle
#define x11 AssembleSequence(
#define x01 252180 xD 281854,
#define nZ1 {DataP slot_holder(y0[
#define nY1 <<std::dec<<")";}
#define nX1 :lM3 l73
#define nW1 !=xN)if(TestCase(
#define nV1 else*this=model;}
#define nU1 std::pair<T1,T2>&
#define nT1 tT1 typename
#define nS1 has_good_balance_found
#define nR1 n_occurrences
#define nQ1 found_log2_on_exponent
#define nP1 covers_minus1
#define nO1 needs_resynth
#define nN1 immed_product
#define nM1 ,2,1)nS if(found[data.
#define nL1 break;lK3 bitmask&
#define nK1 Sign_Positive
#define nJ1 ::MakeTrue
#define nI1 CodeTreeImmed tR2(
#define nH1 Suboptimal
#define nG1 changed_if
#define nF1 >::Optimize(){}
#define nE1 n_as_tanh_param
#define nD1 opposite=
#define nC1 x91(
#define nB1 MatchResultType
#define nA1 needs_sincos
#define n91 resulting_exponent
#define n81 val):Value(Value::
#define n71 iD3:yX3;}
#define n61 GetLogicalValue(xZ
#define n51 GetParam(a)
#define n41 inverse_nominator]
#define n31 (p0 tH1 p0 l73>=l63(0.0))
#define n21 cSin,yV2
#define n11 IsImmed()eU2
#define n01 AddFunctionOpcode(
#define lZ1 t63 MakeNotNotP1,l4::
#define lY1 t63 MakeNotNotP0,l4::
#define lX1 public e6,public std y33<
#define lW1 cZ1;a-->0;)if(
#define lV1 (*x9)[a].xK3
#define lU1 (long double)
#define lT1 tmp l12 tree);
#define lS1 SetParams(lH2));
#define lR1 o<<"("<<std::hex<<data.
#define lQ1 IfBalanceGood(
#define lP1 n_as_tan_param
#define lO1 changed_exponent
#define lN1 &&e23<l63(
#define lM1 inverse_denominator
#define lL1 xI(rule.repl_param_list,
#define lK1 retry_positionalparams_2
#define lJ1 xL3 index
#define lI1 situation_flags&
#define lH1 512 xD 400412,
#define lG1 data.subfunc_opcode
#define lF1 nJ2 tZ2 tP3 nD);
#define lE1 ;if(tD2 t03 SaveMatchedParamIndex(
#define lD1 CopyOnWrite();
#define lC1 for iD1 a=0;a<tree.iR
#define lB1 recursioncount
#define lA1 PlanNtimesCache(
#define l91 >){int mStackPtr=0;
#define l81 FPoptimizer_Grammar
#define l71 AddOperation(cInv,1,1)nS}
#define l61 GetPositivityInfo cX3
#define l51 nG1 xM2 y3 l9
#define l41 ParamSpec_SubFunctionData
#define l31 tP2){synth.yU
#define l21 PositionalParams_Rec
#define l11 ,{l4::MakeNotP1,l4::
#define l01 lQ3 cB2+
#define iZ tree nI2 a
#define iY ]);lW3
#define iX ,ByteCode,IP,limit,y1,stack);
#define iW }},{ProduceNewTree,2,1,
#define iV 1).xQ1
#define iU tree cZ1
#define iT ,l7 2,1,
#define iS ;tree l12
#define iR GetParamCount();++a)
#define iQ DumpTreeWithIndent(*this);
#define iP iD1 a=GetParamCount();a
#define iO CalculateResultBoundaries(
#define iN tT1 xL3 Compare>
#define iM lK 2 i41 1,
#define iL edited_powgroup
#define iK has_unknown_max
#define iJ has_unknown_min
#define iI static const yQ3
#define iH if(keep_powi
#define iG synthed_tree
#define iF 7168 xD 401798,
#define iE 408963 xD 24959
#define iD collections
#define iC {switch(type eS3 cond_or:
#define iB cache
#define iA ;c92 nY2
#define i9 )iA);
#define i8 ByteCode.
#define i7 goto ReplaceTreeWithOne;case
#define i6 ,SelectedParams,0 i41
#define i5 !=xN)return n52
#define i4 e91.data
#define i3 lI3 y92(
#define i2 needs_sinhcosh
#define i1 i42 l63(
#define i0 nU2 nA
#define tZ MakeFalse,l4::
#define tY 522359 xD 24713,
#define tX AnyParams,0}},{ReplaceParams,
#define tW matched_params
#define tV [n2 tF3=true;lR[n2]n23
#define tU l81::Grammar*
#define tT powgroup l9
#define tS tP2&&found[data.
#define tR nI1(
#define tQ has_mulgroups_remaining
#define tP by_exponent
#define tO MatchInfo xI&
#define tN Rehash();iQ2.push_back(
#define tM int_exponent_t
#define tL best_factor
#define tK RootPowerTable xI::RootPowers[
#define tJ tree.SetParamMove(
#define tI );p2.nL2 p2);tZ2 lL3 nD);tG1}
#define tH :goto ReplaceTreeWithZero;case
#define tG MatchPositionSpec_AnyParams xI
#define tF lR3 FPoptimizer_CodeTree
#define tE n_as_sinh_param
#define tD n_as_cosh_param
#define tC is_signed
#define tB result_positivity
#define tA valueType
#define t9 return m xH3
#define t8 x2 AnyParams,
#define t7 =iO xZ
#define t6 c1 known tK3
#define t5 biggest_minimum
#define t4 const l41
#define t3 122999 xD 139399,
#define t2 142455 xD 141449,
#define t1 lB 0x4},{{
#define t0 cond_tree
#define eZ else_tree
#define eY then_tree
#define eX relationships
#define eW CodeTree xI
#define eV eW&tree)
#define eU sequencing
#define eT string cV3
#define eS );l32
#define eR (lH2));l22 Rehash();
#define eQ );bool needs_cow=GetRefCount()>1;
#define eP {AdoptChildrenWithSameOpcode(tree);
#define eO )){eW
#define eN std y33<eW>
#define eM if_stack
#define eL n_as_sin_param
#define eK n_as_cos_param
#define eJ PowiResolver::
#define eI ];};extern"C"{
#define eH nP l63(-cW3
#define eG cIf,tZ3
#define eF ,cIf,l0 3,
#define eE ].relationship
#define eD PACKED_GRAMMAR_ATTRIBUTE;
#define eC .BalanceGood
#define eB l32 cP2
#define eA back().endif_location
#define e9 x91 key
#define e8 l32 mul);
#define e7 130,1,
#define e6 MatchPositionSpecBase
#define e5 lI3 CodeTree(
#define e4 smallest_maximum
#define e3 ReplaceTreeWithParam0;
#define e2 factor_needs_rehashing
#define e1 MatchPositionSpecBaseP
#define e0 typename tR1::yL3
#define cZ eJ1 xI(nQ.param_list,
#define cY 27,28,29,30,31,32,33,35,36,
#define cX const ParamSpec_SubFunction
#define cW const ParamSpec_ParamHolder
#define cV otherhalf
#define cU StackState
#define cT ()iS
#define cS l2 16,2,
#define cR const SequenceOpCode xI
#define cQ MatchPositionSpec_PositionalParams xI
#define cP xW2 tree,std::ostream&o
#define cO l63(1.5)*fp_const_pi xI()
#define cN CalculatePowiFactorCost(
#define cM ImmedHashGenerator
#define cL ::map<yD2,std::set<std c53> >
#define cK l32 comp.cM1[a].eO3;
#define cJ )return false;
#define cI .nQ2
#define cH T1,typename T2>inline iU2()(
#define cG has_nonlogical_values
#define cF from_logical_context)
#define cE AnyParams,0}},{ProduceNewTree,
#define cD for iD1 a=xY cZ1;a-->0;)
#define cC POWI_CACHE_SIZE
#define cB static inline eW
#define cA ++IP;continue;}if(l82==cS3.
#define c9 paramholder_matches
#define c8 {eW tmp;tmp cI
#define c7 c1 iB3 p0 c1 val<=fp_const_negativezero xI())
#define c6 ComparisonSetBase::
#define c5 iD1 a=iU;a-->0;)
#define c4 yA3 tree)xH1"\n";
#define c3 ;sim.Eat(
#define c2 );void n01 xL3 t93 cV2<
#define c1 .max.
#define c0 ,x22
#define yZ xL3 c;xL3 short l[
#define yY 161,162,163,164,165,166,167,176,177,178,198,202,210,214,222,234,235,237,238,241,242,243,244,247,248,249,251,253,254,255,256,257}};}eI2
#define yX t63 xN,l4::Never t63 xN,l4::Never}}
#define yW .FoundChild
#define yV BalanceResultType
#define yU DoDup(found[data.
#define yT +=fp_const_twopi xI();
#define yS ();pow cI cLog);tZ2 cMul);
#define yR fp_const_twopi xI()iR3
#define yQ fp_sin(min),fp_sin(max))
#define yP eW tmp,tmp2;tmp2 cI
#define yO m.min.
#define yN ,y4 0x4 nJ
#define yM {return
#define yL const yM data->
#define yK )xH1 std::endl;DumpHashes(
#define yJ lW3 GetOpcode(),
#define yI for iD1 a=0;a<iR{if(
#define yH ):xP3(0),Opcode(
#define yG static void nR3 nC yD2&eZ1,
#define yF MatchPositionSpec_AnyWhere
#define yE if e12.match_type==
#define yD void OutFloatHex(std::ostream&o,
#define yC cGreaterOrEq,
#define yB ,typename eW::
#define yA AssembleSequence_Subdivide(
#define y9 {iE1.erase(i);continue;}
#define y8 =0;a<yI3.iR if(
#define y7 );continue;}y82 xQ1==l63(
#define y6 ]=nY2|xL3(
#define y5 nQ c02
#define y4 cPow,lB
#define y3 branch2
#define y2 xM2 CodeTreeImmed(
#define y1 factor_stack_base
#define y0 data->Params
#define xZ tree l9
#define xY branch1
#define xX (nO3 r=range.first;r!=range n23;++r){
#define xW =fp_cosh(cF1);nW2=fp_cosh(nW2);
#define xV StackTop
#define xU FPOPT_autoptr
#define xT +=eH3 iP3 eH3;}nU2 inline l63
#define xS int_exponent
#define xR newnode
#define xQ lK 1 i41
#define xP has_highlevel_opcodes
#define xO {if(needs_cow){lD1 goto
#define xN Unchanged
#define xM tree.ReplaceWithImmed(
#define xL .IsIdenticalTo(
#define xK GetStackTop()-
#define xJ sim.AddConst(
#define xI <l63>
#define xH cP=std::cout
#define xG best_selected_sep
#define xF tT1>void FunctionParserBase<
#define xE SetParam(0,lL3 l9 0));eW p1;p1 cI
#define xD ,{2,
#define xC cAnd,tX
#define xB ->Recalculate_Hash_NoRecursion();}
#define xA iR if(ApplyGrammar(iA2,iZ3,
#define x9 position
#define x8 ;iE3 0));tmp cI cInv);tmp.cG1 return
#define x7 lC1{yQ3
#define x6 )){tree.FixIncompleteHashes();}
#define x5 std y33<CodeTree>
#define x4 TestImmedConstraints(c43 constraints,tree)cJ
#define x3 SwapLastTwoInStack()c3
#define x2 ,cAdd,
#define x1 nU3 1,cInv c62 xJ-1)yH1
#define x0 ));nG1 nJ2 tZ2 op1);tree.DelParams()
#define nZ paramholder_index
#define nY return true;case
#define nX occurance_counts
#define nW >p t7 a)iR3 p.
#define nV -->0;){xW2 powgroup=n51;if(powgroup
#define nU ,l0 1,
#define nT if(xZ 0)tK1
#define nS eL1*this)iP3;}
#define nR const FPoptimizer_CodeTree::eW&tree
#define nQ model_tree
#define nP return yQ3(
#define nO ){using lR3 FUNCTIONPARSERTYPES;
#define nN eN&n62
#define nM eG2 xZ
#define nL ConstantFolding_LogicCommon(tree,c6
#define nK ),rangehalf xI model=rangehalf xI()){if(known
#define nJ },{{2,
#define nI nT1 Ref>inline void xU<Ref>::
#define nH AnyParams,1 i41
#define nG cOr,tX 16,1,
#define nF ):data(new y92 xI(
#define nE goto do_return;}
#define nD .GetOpcode()
#define nC FUNCTIONPARSERTYPES::
#define nB b;}};tT1>eI2 Comp<nC
#define nA y92 xI::y92(
#define n9 lJ2(),tR3),Hash(),Depth(1),tY1 0){}
#define n8 SynthesizeByteCode(synth);
#define n7 while(ApplyGrammar(e11 void*)&
#define n6 GetIntegerInfo(xZ 0 yN3 nW3
#define n5 iS nG1);iB1
#define n4 nV2 cGreater>tR2(
#define n3 DumpParams xI e12.param_list,c43 data c02,o);
#define n2 restholder_index
#define n1 eW xN3;xN3 cI cMul)yT1 eG2
#define n0 lS iR3 fp_nequal(tmp,xD1){xM l63(1)/tmp);nE}lD
#define lZ :if(ParamComparer xI()(tS3 1],tS3 0])){std::swap(tS3 0],tS3 1]);Opcode=
#define lY <typename l63>
#define lX xI tmp;tmp cI cPow);tmp.iF3 tmp y2 l63(
#define lW i01,0x0},
#define lV l32 pow l9 1));pow nI2 1);pow nJ2 tJ 0,pow);goto NowWeAreMulGroup;}
#define lU GroupFunction,0},lW{{
#define lT ,l63(1)/l63(
#define lS xZ 0).xQ1
#define lR restholder_matches
#define lQ yV1|=key;x91 crc=(key>>10)|(key<<(64-10))iF1((~nC1 crc))*3)^1234567;}};
#define lP nG1;nG1 iQ1 nG1 l12 xZ 0));nG1 xM2 xY l9
#define lO nU2 eW::CodeTree(
#define lN tree eR2 0,xZ 0)l9 0))iX2 1,CodeTreeImmed(
#define lM lY void e72::n01 xL3 t93 cV2<
#define lL cMul,lU 2,
#define lK cMul,AnyParams,
#define lJ (xZ 0)tK1&&xZ 1)tK1){xM
#define lI iO tmp)xH3
#define lH :eQ3=comp.AddRelationship(atree l9 0),atree l9 1),c6
#define lG cPow,l0 2
#define lF typename l63>inline iU2()e01 xQ3 l63&b)yM a
#define lE {yQ3 m t7 0));
#define lD break;case
#define lC tB1 eW::
#define lB yD1,0,
#define lA l1 0x0 nJ
#define l9 .GetParam(
#define l8 ;eW nG1;nG1 iQ1 nG1 iA1 tree.lH2));nG1 nJ2 tZ2
#define l7 cAdd,tX
#define l6 SelectedParams,0},0,0x0 nJ
#define l5 lK 0}},{ReplaceParams,
#define l4 RangeComparisonData
#define l3 yD1},{ProduceNewTree,
#define l2 yD1},{ReplaceParams,
#define l1 cMul,SelectedParams,0},0,
#define l0 lB 0x0},{{
#ifdef _MSC_VER
typedef
xL3
int
lG2;
#else
#include <stdint.h>
typedef
uint_least32_t
lG2;
#endif
lR3
crc32{enum{startvalue=0xFFFFFFFFUL,poly=0xEDB88320UL}
;tT1
lG2
crc>eI2
b8{enum{b1=(crc&xG1
crc
yA2
crc>>1),b2=(b1&xG1
b1
yA2
b1>>1),b3=(b2&xG1
b2
yA2
b2>>1),b4=(b3&xG1
b3
yA2
b3>>1),b5=(b4&xG1
b4
yA2
b4>>1),b6=(b5&xG1
b5
yA2
b5>>1),b7=(b6&xG1
b6
yA2
b6>>1),res=(b7&xG1
b7
yA2
b7>>1)}
;}
;inline
lG2
update(lG2
crc,xL3
b){
#define B4(n) b8<n tM2 n+1 tM2 n+2 tM2 n+3>::res
#define R(n) B4(n),B4(n+4),B4(n+8),B4(n+12)
static
const
lG2
table[256]={R(0x00),R(0x10),R(0x20),R(0x30),R(0x40),R(0x50),R(0x60),R(0x70),R(0x80),R(0x90),R(0xA0),R(0xB0),R(0xC0),R(0xD0),R(0xE0),R(0xF0)}
;
#undef R
#undef B4
return((crc>>8))^table[(crc^b)&0xFF];lZ2
lG2
calc_upd(lG2
c,const
xL3
char*buf,size_t
size){lG2
value=c;for
iD1
p=0;p<size;++p)value=update(eL3
buf[p])iP3
value;lZ2
lG2
calc
e11
xL3
char*buf,size_t
size)yM
calc_upd(startvalue,buf,size);}
}
#ifndef FPOptimizerAutoPtrHH
#define FPOptimizerAutoPtrHH
nT1
Ref>class
xU{t83
xU():p(0){}
xU(Ref*b):p(b){xO3}
xU
e11
xU&b):p(b.p){xO3
lZ2
Ref&eM1*(c61*p;lZ2
Ref*eM1->(c61
p;}
xU&eM1=(Ref*b){Set(b)iP3*this;}
xU&eM1=e11
xU&b){Set(b.p)iP3*this;}
#ifdef __GXX_EXPERIMENTAL_CXX0X__
xU(xU&&b):p(b.p){b.p=0;}
xU&eM1=(xU&&b){if(p!=b.p){eH2;p=b.p;b.p=0;}
return*this;}
#endif
~xU(){eH2
eW3
UnsafeSetP(Ref*newp){p=newp
eW3
swap(xU<Ref>&b){Ref*tmp=p;p=b.p;b.p=tmp;}
private:inline
static
void
Have(Ref*p2);inline
void
eH2;inline
void
xO3
inline
void
Set(Ref*p2);private:Ref*p;}
;nI
eH2{if(!p)return;p->xP3-=1;if(!p->xP3)delete
p;}
nI
Have(Ref*p2){if(p2)++(p2->xP3);}
nI
Birth(){Have(p);}
nI
Set(Ref*p2){Have(p2);eH2;p=p2;}
#endif
#include <utility>
eI2
Compare2ndRev{nT1
T>inline
iU2()e11
T&xQ3
T&b
c61
a
n23>b
n23;}
}
;eI2
Compare1st{nT1
cH
const
nU1
xQ3
nU1
b
c61
a.first<b.first;}
nT1
cH
const
nU1
a,T1
b
c61
a.first<b;}
nT1
cH
T1
xQ3
nU1
b
c61
a<b.first;}
}
;
#ifndef FPoptimizerHashHH
#define FPoptimizerHashHH
#ifdef _MSC_VER
typedef
xL3
long
long
x91;
#define FPHASH_CONST(x) x##ULL
#else
#include <stdint.h>
typedef
uint_fast64_t
x91;
#define FPHASH_CONST(x) x##ULL
#endif
lR3
FUNCTIONPARSERTYPES{eI2
yD2{x91
hash1,hash2;yD2():hash1(0),hash2(0){}
yD2
e11
x91&xQ3
x91&b):hash1(a),hash2(b){}
iU2==e11
yD2&c51==eJ2&&hash2==eK2
iU2!=e11
yD2&c51!=eJ2||hash2!=eK2
iU2<e11
yD2&c51!=eJ2?hash1<eJ2:hash2<eK2}
;}
#endif
#ifndef FPOptimizer_CodeTreeHH
#define FPOptimizer_CodeTreeHH
#ifdef FP_SUPPORT_OPTIMIZER
#include <vector>
#include <utility>
lR3
l81{eI2
Grammar;}
lR3
xI1{nU2
class
ByteCodeSynth;}
tF{nU2
class
CodeTree
yP1
eI2
y92
yP1
class
CodeTree{typedef
xU<y92
xI>DataP;DataP
data;t83
CodeTree();~CodeTree();eI2
OpcodeTag{}
;e5
xV2
o,OpcodeTag);eI2
FuncOpcodeTag{}
;e5
xV2
o,xL3
f,FuncOpcodeTag);eI2
xT3{}
;e5
const
l63&v,xT3);
#ifdef __GXX_EXPERIMENTAL_CXX0X__
e5
l63&&v,xT3);
#endif
eI2
VarTag{}
;e5
xL3
varno,VarTag);eI2
CloneTag{}
;e5
lS3
b,CloneTag);void
GenerateFrom
e11
typename
FunctionParserBase
xI::Data&data,bool
keep_powi=false);void
GenerateFrom
e11
typename
FunctionParserBase
xI::Data&data,const
x5&y62,bool
keep_powi=false);void
SynthesizeByteCode(std
y33<xL3>&eG3,std
y33
xI&immed,size_t&stacktop_max);void
SynthesizeByteCode(xI1::e72&synth,bool
MustPopTemps=true)const;size_t
SynthCommonSubExpressions(xI1::cD1
const;void
SetParams
e11
x5&xU3
SetParamsMove(x5&tS1
CodeTree
GetUniqueRef();
#ifdef __GXX_EXPERIMENTAL_CXX0X__
void
SetParams(x5&&tS1
#endif
void
SetParam
iD1
which,lS3
b);void
SetParamMove
iD1
which,xU2
b);void
AddParam
e11
xU2
param);void
l32
xU2
param);void
AddParams
e11
x5&xU3
AddParamsMove(x5&xU3
AddParamsMove(x5&n62,size_t
n72);void
DelParam
iD1
index);void
DelParams();void
Become
e11
xU2
b);inline
size_t
GetParamCount(c61
lH2)tG3;lZ2
xU2
GetParam
iD1
n)yM
lH2)[n];lZ2
lS3
GetParam
iD1
n
c61
lH2)[n];lZ2
void
nQ2
xV2
o)t73
Opcode=o;lZ2
xV2
GetOpcode()yL
Opcode;lZ2
nC
yD2
GetHash()yL
Hash;lZ2
const
x5&lH2
c61
y0;lZ2
x5&lH2)yM
y0;lZ2
size_t
GetDepth()yL
Depth;lZ2
const
l63&xQ1
yL
Value;lZ2
xL3
GetVar()yL
lI2
lZ2
xL3
GetFuncNo()yL
lI2
lZ2
bool
cO2
c61
GetOpcode()!=nC
cNop;lZ2
bool
IsImmed(c61
GetOpcode()==nC
cImmed;lZ2
bool
IsVar(c61
GetOpcode()==nC
lG3;lZ2
xL3
GetRefCount()yL
xP3
eW3
ReplaceWithImmed
e01
i);void
Rehash(bool
constantfolding=true);void
Sort();inline
void
Mark_Incompletely_Hashed()t73
Depth=0;lZ2
bool
Is_Incompletely_Hashed()yL
Depth==0;lZ2
const
tU
GetOptimizedUsing()yL
lK2;lZ2
void
SetOptimizedUsing
e11
tU
g)t73
lK2=g;}
bool
RecreateInversionsAndNegations(bool
prefer_base2=false);void
FixIncompleteHashes();void
swap(xU2
b){data.swap(b.data);}
bool
IsIdenticalTo
e11
xU2
b)const;void
lD1}
yP1
eI2
y92{int
xP3;xV2
Opcode
l53
Value;xL3
lI2
eN
Params;nC
yD2
Hash;size_t
Depth;const
tU
lK2;y92();y92
e11
y92&b);i3
xV2
o);i3
xV2
o,xL3
f);i3
const
l63&i);
#ifdef __GXX_EXPERIMENTAL_CXX0X__
i3
l63&&i);y92(y92&&b);
#endif
bool
IsIdenticalTo
e11
y92&b)const;void
Sort();void
Recalculate_Hash_NoRecursion();private:void
eM1=e11
y92&b);}
yP1
cB
CodeTreeImmed
e01
i)yM
eW(i
yB
xT3());}
#ifdef __GXX_EXPERIMENTAL_CXX0X__
cI1
CodeTreeImmed
tR2&&i)yM
eW(i32
i)yB
xT3());}
#endif
cI1
CodeTreeOp(xV2
opcode)yM
eW(opcode
yB
OpcodeTag());}
cI1
CodeTreeFuncOp(xV2
t93
xL3
f)yM
eW(t93
f
yB
FuncOpcodeTag());}
cI1
CodeTreeVar
xA2
varno)yM
eW(varno
yB
VarTag());}
#ifdef FUNCTIONPARSER_SUPPORT_DEBUGGING
tB1
DumpHashes(xH)xX3
DumpTree(xH)xX3
DumpTreeWithIndent(xH,const
std
c53&indent="\\"
);
#endif
}
#endif
#endif
#ifndef FPOptimizer_GrammarHH
#define FPOptimizer_GrammarHH
#include <iostream>
tF{nU2
class
CodeTree;}
lR3
l81{enum
ImmedConstraint_Value{ValueMask=0x07,Value_AnyNum=0x0,xE2=0x1,Value_OddInt=0x2,i11=0x3,Value_NonInteger=0x4,eV1=0x5
eX3
ImmedConstraint_Sign{SignMask=0x18,Sign_AnySign=0x00,nK1=0x08,eW1=0x10,Sign_NoIdea=0x18
eX3
ImmedConstraint_Oneness{OnenessMask=0x60,Oneness_Any=0x00,Oneness_One=0x20,Oneness_NotOne=0x40
eX3
ImmedConstraint_Constness{ConstnessMask=0x180,Constness_Any=0x00,i01=0x80,Constness_NotConst=0x100
eX3
Modulo_Mode{Modulo_None=0,Modulo_Radians=1
eX3
Situation_Flags{LogicalContextOnly=0x01,NotForIntegers=0x02,OnlyForIntegers=0x04,OnlyForComplex=0x08,NotForComplex=0x10
eX3
x82{NumConstant,eU3,SubFunction
eX3
ParamMatchingType{PositionalParams,SelectedParams,AnyParams,GroupFunction
eX3
RuleType{ProduceNewTree,ReplaceParams}
;
#ifdef __GNUC__
# define PACKED_GRAMMAR_ATTRIBUTE __attribute__((packed))
#else
# define PACKED_GRAMMAR_ATTRIBUTE
#endif
typedef
std::pair<x82,const
void*>eN2
yP1
eN2
eJ1
xA2
paramlist,lJ1)yP1
bool
ParamSpec_Compare
e11
void*xQ3
void*b,x82
type);xL3
ParamSpec_GetDepCode
e11
eN2&b);eI2
ParamSpec_ParamHolder{lJ1:8;xL3
constraints:9;xL3
depcode:15;}
eD
nU2
eI2
ParamSpec_NumConstant{l63
constvalue;xL3
modulo;}
;eI2
l41{xL3
param_count:2;xL3
param_list:30;xV2
subfunc_opcode:8;ParamMatchingType
match_type:3;xL3
n2:5;}
eD
eI2
ParamSpec_SubFunction{l41
data;xL3
constraints:9;xL3
depcode:7;}
eD
eI2
Rule{RuleType
ruletype:2;xL3
situation_flags:5;xL3
repl_param_count:2+9;xL3
repl_param_list:30;l41
match_tree;}
eD
eI2
Grammar{xL3
rule_count;xL3
short
rule_list[999
eI
extern
const
Rule
grammar_rules[];}
tB1
DumpParam
e11
eN2&p,std::ostream&o=std::cout)xX3
DumpParams
xA2
paramlist,xL3
count,std::ostream&o=std::cout);}
#endif
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif
#define CONSTANT_POS_INF HUGE_VAL
#define CONSTANT_NEG_INF (-HUGE_VAL)
lR3
FUNCTIONPARSERTYPES{nU2
inline
l63
fp_const_pihalf()yM
fp_const_pi
xI()*l63(0.5);}
nU2
inline
l63
fp_const_twopi(eV2
fp_const_pi
xI());lT3
fp_const_twoe(eV2
fp_const_e
xI());lT3
fp_const_twoeinv(eV2
fp_const_einv
xI());lT3
fp_const_negativezero(){
#ifdef FP_EPSILON
return-fp_epsilon
xI();
#else
return
l63(-1e-14);
#endif
}
}
#ifdef FP_SUPPORT_OPTIMIZER
#include <vector>
#include <utility>
#include <iostream>
xZ1{using
lR3
l81;using
tF;using
lR3
FUNCTIONPARSERTYPES
yP1
class
MatchInfo{t83
std
y33<std::pair<bool,eN> >lR;eN
c9;std
y33<xL3>tW;t83
MatchInfo():lR(),c9(),tW(){}
t83
bool
SaveOrTestRestHolder
xA2
n2,iV2&iK1){cW1{lR
xV3
n2+1);lR
tV=iK1;iB1
if(lR[n2
tF3==false){lR
tV=iK1;iB1
iV2&found=lR[n2]n23;if(iK1
tG3!=found
tG3
cJ
for
iD1
a=0;a<iK1
tG3;++a)if(!iK1[a]xL
found[a])cJ
iB1
void
SaveRestHolder
xA2
n2,eN&iK1){cW1
lR
xV3
n2+1);lR
tV.swap(iK1);}
bool
SaveOrTestParamHolder
xA2
nZ,xW2
xW3){if(c9
tG3<=nZ){c9.y03
nZ+1);c9
xV3
nZ);c9.push_back(xW3);iB1
if(!c9[nZ].cO2)){c9[nZ]=xW3;iB1
return
xW3
xL
c9[nZ])eW3
SaveMatchedParamIndex(lJ1){tW.push_back(index);}
xW2
GetParamHolderValueIfFound
xA2
nZ)const{static
const
eW
dummytree;if(c9
tG3<=nZ)return
dummytree
iP3
c9[nZ];}
xW2
GetParamHolderValue
xA2
nZ
c61
c9[nZ];}
bool
HasRestHolder
xA2
n2
c61
lR
tG3>n2&&lR[n2
tF3==true;}
iV2&GetRestHolderValues
xA2
n2)const{static
iV2
empty_result;cW1
return
empty_result
iP3
lR[n2]n23;}
const
std
y33<xL3>&GetMatchedParamIndexes(c61
tW
eW3
swap(tO
b){lR.swap(b.lR);c9.swap(b.c9);tW.swap(b.tW);}
tO
eM1=e11
tO
b){lR=b.lR;c9=b.c9;tW=b.tW
iP3*this;}
}
;class
e6;typedef
xU<e6>e1;class
e6{t83
int
xP3;t83
e6():xP3(0){}
virtual~e6(){}
}
;eI2
nB1{bool
found;e1
specs;nB1(bool
f):found(f),specs(){}
nB1(bool
f,const
e1&s):found(f),specs(s){}
}
xX3
SynthesizeRule
e11
tN2
eW&tree,iX3)yP1
nB1
TestParam
e11
eN2&c12
xW2
tree,const
e1&xK3,iX3)yP1
nB1
TestParams(t4&nQ,xW2
tree,const
e1&xK3,iX3,bool
tD2
yP1
bool
ApplyGrammar
e11
Grammar&iA2,FPoptimizer_CodeTree::eW&tree,bool
from_logical_context=false)xX3
ApplyGrammars(FPoptimizer_CodeTree::eV
yP1
bool
IsLogisticallyPlausibleParamsMatch(t4&params,const
eV;}
lR3
l81{tB1
DumpMatch
e11
tN2
nR,const
FPoptimizer_Optimize::iX3,bool
DidMatch,std::ostream&o=std::cout)xX3
DumpMatch
e11
tN2
nR,const
FPoptimizer_Optimize::iX3,xY3
i43,std::ostream&o=std::cout);}
#endif
#include <string>
xZ3
l81::x82
yX1=false);xZ3
xV2
yX1=false);
#include <string>
#include <sstream>
#include <assert.h>
#include <iostream>
using
lR3
l81;using
lR3
FUNCTIONPARSERTYPES;xZ3
l81::x82
yX1){
#if 1
xY3
p=0;switch(opcode
eS3
eT3
p="NumConstant"
;lD
eU3:p="ParamHolder"
;lD
SubFunction:p="SubFunction"
;yY3
std::ostringstream
tmp;assert(p);tmp<<p;if(pad)while(tmp.str()tG3<12)tmp<<' 'iP3
tmp.str();
#else
std::ostringstream
tmp;tmp<<opcode;if(pad)while(tmp.str()tG3<5)tmp<<' 'iP3
tmp.str();
#endif
}
xZ3
xV2
yX1){
#if 1
xY3
p=0;switch(opcode
eS3
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
cEval:p="cEval"
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
i51:p="cNEqual"
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
eC3:p="cLog2by"
;lD
cNop:p="cNop"
;break;
#endif
case
cSinCos:p="cSinCos"
;lD
cSinhCosh:p="cSinhCosh"
;lD
cZ3:p="cAbsNot"
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
lG3:p="VarBegin"
;yY3
std::ostringstream
tmp;assert(p);tmp<<p;if(pad)while(tmp.str()tG3<12)tmp<<' 'iP3
tmp.str();
#else
std::ostringstream
tmp;tmp<<opcode;if(pad)while(tmp.str()tG3<5)tmp<<' 'iP3
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
;lR3
xI1{nU2
class
ByteCodeSynth{t83
ByteCodeSynth():ByteCode(),Immed(),cU(),xV(0),StackMax(0){i8
y03
64);Immed.y03
8);cU.y03
16)eW3
Pull(std
y33<xL3>&bc,std
y33
xI&imm,size_t&StackTop_max){for(eY1=0;a<c82;++a){lN2]&=~nY2;}
i8
swap(bc);Immed.swap(imm);StackTop_max=StackMax;}
size_t
GetByteCodeSize(c61
c82;}
size_t
GetStackTop(c61
xV
eW3
PushVar
xA2
varno){c92
varno);tO2}
void
PushImmed
tR2
immed
nO
c92
cImmed);Immed.push_back(immed);tO2}
void
StackTopIs(nR,int
offset=0){if((int)xV>offset){cU
lO2
first=true;cU
lO2
second=tree;}
}
bool
IsStackTop(nR,int
offset=0
c61(int)xV>offset&&cU
lO2
first&&cU
lO2
second
xL
tree);lZ2
void
EatNParams
xA2
eat_count){xV-=eat_count
eW3
ProducedNParams
xA2
produce_count){xJ1
xV+produce_count)eW3
DoPopNMov
iD1
eO2,size_t
srcpos
nO
c92
cPopNMov)x92
eO2)x92
srcpos);xJ1
srcpos+1);cU[eO2]=cU[srcpos];xJ1
eO2+1)eW3
DoDup
iD1
y13
nO
if(y13==xV-1){c92
cDup);}
else{c92
cFetch)x92
y13);}
tO2
cU[xV-1]=cU[y13];}
#ifdef FUNCTIONPARSER_SUPPORT_DEBUGGING
tT1
int>void
Dump(){std::ostream&o=std::cout;o<<"Stack state now("
<<xV<<"):\n"
;for
iD1
a=0;a<xV;++a){o<<a<<": "
;if(cU[a
tF3){nR=cU[a]n23;o<<'['<<std::hex<<(void*)(&tree.lH2))<<std::dec<<','<<tree.GetRefCount()<<']'yA3
tree,o);}
else
o<<"?"
y93}
o<<std::flush;}
#endif
size_t
y23
nR)const{for
iD1
a=xV;a-->0;)if(cU[a
tF3&&cU[a]n23
xL
tree
cA3
a
iP3
tP2;}
bool
Find(nR
c61
y23
tree)!=tP2;}
bool
FindAndDup(nR){size_t
pos=y23
tree
iR3
pos!=tP2){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<lJ4"duplicate at ["
<<pos<<"]: "
yA3
tree)xH1" -- issuing cDup or cFetch\n"
;
#endif
DoDup(pos);iB1
lY2
eI2
IfData{size_t
ofs;}
;void
SynthIfStep1
cA2,xV2
op
x31
cB2=c82;c92
op
i9
c92
nY2)eW3
SynthIfStep2
cA2
x31
l01
lL2+2);l01
2
lM2
cB2=c82;c92
cJump
i9
c92
nY2)eW3
SynthIfStep3
cA2
x31
i8
back()|=nY2;l01
lL2-1);l01
2
lM2
xJ1
xV+1
cF2
0;a<cB2;++a){if(lN2]==cJump&&lN2+1]==(nY2|(cB2-1))){lN2+lL2-1);lN2+2
lM2
lK3
lN2]eS3
cAbsIf:case
cIf:case
cJump:case
cPopNMov:a+=2;lD
cFCall:case
cPCall:case
cFetch:a+=1;break;yX3
yY3}
}
protected:void
xJ1
size_t
eO3{xV=value;if(xV>lU3{StackMax=xV;cU
xV3
lU3;}
}
protected:std
y33<xL3>ByteCode;std
y33
xI
Immed;std
y33<std::pair<bool,FPoptimizer_CodeTree::eW> >cU;size_t
xV;size_t
StackMax;private:void
incStackPtr(){if(xV+2>lU3
cU
xV3
StackMax=xV+2);}
tT1
bool
IsIntType,bool
IsComplexType>eI2
cV2{}
;t83
void
AddOperation
xA2
t93
xL3
eat_count,xL3
produce_count=1){EatNParams(eat_count);n01
opcode);ProducedNParams(produce_count)eW3
n01
xL3
t93
cV2<false,false>c2
false,true>c2
true,false>c2
true,true>);inline
void
n01
xL3
opcode){n01
t93
cV2<bool(nC
IsIntType
xI::eH3),bool(nC
IsComplexType
xI::eH3)>());}
}
yP1
eI2
SequenceOpCode
yP1
eI2
tW1{static
cR
AddSequence;static
cR
MulSequence;}
xX3
x11
long
count,cR&eU,cD1;}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
using
lR3
FUNCTIONPARSERTYPES;lR3
xI1{nU2
eI2
SequenceOpCode{l63
basevalue;xL3
op_flip;xL3
op_normal,op_normal_flip;xL3
op_inverse,op_inverse_flip;}
yP1
cR
tW1
xI::AddSequence={y51,cNeg
x2
cAdd,cSub,cRSub}
yP1
cR
tW1
xI::MulSequence={l63(1),cInv,cMul,cMul,cDiv,cRDiv}
;
#define findName(a,b,c) "var"
#define TryCompilePowi(o) false
#define mData this
#define mByteCode ByteCode
#define mImmed Immed
nZ2
false,false
l91
# define FP_FLOAT_VERSION 1
# define FP_COMPLEX_VERSION 0
# include "extrasrc/fp_opcode_add.inc"
# undef FP_COMPLEX_VERSION
# undef FP_FLOAT_VERSION
}
nZ2
true,false
l91
# define FP_FLOAT_VERSION 0
# define FP_COMPLEX_VERSION 0
# include "extrasrc/fp_opcode_add.inc"
# undef FP_COMPLEX_VERSION
# undef FP_FLOAT_VERSION
}
#ifdef FP_SUPPORT_COMPLEX_NUMBERS
nZ2
false,true
l91
# define FP_FLOAT_VERSION 1
# define FP_COMPLEX_VERSION 1
# include "extrasrc/fp_opcode_add.inc"
# undef FP_COMPLEX_VERSION
# undef FP_FLOAT_VERSION
}
nZ2
true,true
l91
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
lR3
xI1;
#define POWI_TABLE_SIZE 256
#define POWI_WINDOW_SIZE 3
lR3
xI1{
#ifndef FP_GENERATING_POWI_TABLE
extern
const
xL3
char
powi_table[POWI_TABLE_SIZE];const
#endif
xL3
char
powi_table[POWI_TABLE_SIZE]={0,1,1,1,2,1,2,1,y43
4,1,2,y53
2,1,y43
8,eK3
y63
15,1,16,1,2,1,4,1,2,y53
2,1,4,eK3
1,16,1,25,y63
27,5,8,3,2,1,30,1,31,3,32,1,2,1,y43
8,1,2,y63
39,1,16,137,2,1,4,eK3
y53
45,135,4,31,2,5,32,1,2,131,50,1,51,1,8,3,2,1,54,1,55,3,16,1,57,133,4,137,2,135,60,1,61,3,62,133,63,1,iL1
131,iL1
139,lP2
e7
30,1,130,137,2,31,lP2
e7
e7
130,eK3
1,e7
e7
2,1,130,133,iL1
61,130,133,62,139,130,137,e7
lP2
e7
e7
iL1
131,e7
e7
130,131,2,133,lP2
130,141,e7
130,eK3
1,e7
5,135,e7
lP2
e7
lP2
130,133,130,141,130,131,e7
e7
2,131}
;}
static
xC3
cC=256;
#define FPO(x)
lR3{class
PowiCache{private:int
iB[cC];int
iM1[cC];t83
PowiCache():iB(),iM1(){iB[1]=1;}
bool
Plan_Add(long
eL3
int
count){cJ1>=cC
cJ
iM1[eM3+=count
iP3
iB[eM3!=0
eW3
lV3
long
eO3{cJ1<cC)iB[eM3=1
eW3
Start
iD1
value1_pos){for(int
n=2;n<cC;++n)iB[n]=-1;Remember(1,value1_pos);DumpContents();}
int
Find(long
eO3
const{cJ1<cC){if(iB[eM3>=0){FPO(l54(lA4,"* I found %ld from cache (%u,%d)\n",value,(unsigned)cache[value],l64 value]))iP3
iB[eM3;}
}
return-1
eW3
Remember(long
eL3
size_t
lI4){cJ1>=cC)return;FPO(l54(lA4,"* Remembering that %ld can be found at %u (%d uses remain)\n",value,(unsigned)lI4,l64 value]));iB[eM3=(int)lI4
eW3
DumpContents
c03
FPO(for(int a=1;a<POWI_CACHE_SIZE;++a)if(cache[a]>=0||l64 a]>0){l54(lA4,"== cache: sp=%d, val=%d, needs=%d\n",cache[a],a,l64 a]);})}
int
UseGetNeeded(long
eO3{cJ1>=0&&value<cC)return--iM1[eM3
iP3
0;}
}
yP1
size_t
yA
long
count
cK1
cR&eU,cD1
xX3
c81
size_t
apos,long
aval,size_t
bpos,long
bval
cK1
xL3
cumulation_opcode,xL3
cimulation_opcode_flip,cD1;void
lA1
long
value
cK1
int
need_count,int
lB1=0){cJ1<1)return;
#ifdef FP_GENERATING_POWI_TABLE
if(lB1>32)throw
false;
#endif
if(iB.Plan_Add(eL3
need_count
cA3;long
yB3
1;cJ1<POWI_TABLE_SIZE){yB3
powi_table[eM3;if
iW2
128
xR3
if
iW2
64)yB3
l84
FPO(l54(lA4,"value=%ld, half=%ld, otherhalf=%ld\n",value,half,value/half));lA1
yC3
1
eN3
iB.lV3
half)iP3;}
iW1
half&64){yB3
l84}
}
else
cJ1&1)yB3
value&((1<<POWI_WINDOW_SIZE)-1);else
yB3
value/2;long
cV=value-half;if(half>cV||half<0)std::swap(half,cV);FPO(l54(lA4,"value=%ld, half=%ld, otherhalf=%ld\n",value,half,otherhalf));if(half==cV){lA1
yC3
2
eN3}
else{lA1
yC3
1
eN3
lA1
cV>0?cV:-cV,iB,1
eN3}
iB.lV3
eO3;}
nU2
size_t
yA
long
value
cK1
cR&eU,cD1{int
yD3=iB.Find(eO3;if(yD3>=0)yM
yD3;}
long
yB3
1;cJ1<POWI_TABLE_SIZE){yB3
powi_table[eM3;if
iW2
128
xR3
if
iW2
64)yB3
l84
FPO(l54(lA4,"* I want %ld, my plan is %ld * %ld\n",value,half,value/half));size_t
yE2=yA
half
t81
if(iB
nX2
half)>0||yE2!=t91){iN1
yE2)eL2
half,t91);}
x11
value/half
y12
size_t
lI4=t91
eL2
eL3
lI4);iB.DumpContents()iP3
lI4;}
iW1
half&64){yB3
l84}
}
else
cJ1&1)yB3
value&((1<<POWI_WINDOW_SIZE)-1);else
yB3
value/2;long
cV=value-half;if(half>cV||half<0)std::swap(half,cV);FPO(l54(lA4,"* I want %ld, my plan is %ld + %ld\n",value,half,value-half));if(half==cV){size_t
yE2=yA
half
t81
c81
yE2,half,yE2,yC3
eU.op_normal,eU.op_normal_flip,synth);}
else{long
part1=half;long
part2=cV>0?cV:-cV;size_t
part1_pos=yA
part1
t81
size_t
part2_pos=yA
part2
t81
FPO(l54(lA4,"Subdivide(%ld: %ld, %ld)\n",value,half,otherhalf));c81
part1_pos,part1,part2_pos,part2,iB,cV>0?eU.op_normal:eU.op_inverse,cV>0?eU.op_normal_flip:eU.op_inverse_flip,synth);}
size_t
lI4=t91
eL2
eL3
lI4);iB.DumpContents()iP3
lI4;}
tB1
c81
size_t
apos,long
aval,size_t
bpos,long
bval
cK1
xL3
cumulation_opcode,xL3
cumulation_opcode_flip,cD1{int
a_needed=iB
nX2
aval);int
yE3=iB
nX2
bval);bool
lQ2
tK3
#define DUP_BOTH() do{if(apos<bpos){size_t tmp=apos;apos=bpos;bpos=tmp;lQ2=!lQ2;}FPO(l54(lA4,"-> "lG4 lG4"op\n",(unsigned)apos,(unsigned)bpos));iN1 apos);iN1 apos==bpos?t91:bpos);}while(0)
#define DUP_ONE(p) do{FPO(l54(lA4,"-> "lG4"op\n",(unsigned)p));iN1 p);}while(0)
if(a_needed>0){if(yE3>0){x02}
e53
bpos!=t91)x02
else{lR2
lQ2=!lQ2;}
}
}
iW1
yE3>0){if(apos!=t91)x02
else
DUP_ONE(bpos);}
e53
apos==bpos&&apos==t91)lR2
tQ2
t91&&bpos==synth.xK
2){FPO(l54(lA4,"-> op\n"));lQ2=!lQ2;}
tQ2
synth.xK
2&&bpos==t91)FPO(l54(lA4,"-> op\n"));tQ2
t91)DUP_ONE(bpos);iW1
bpos==t91){lR2
lQ2=!lQ2;}
else
x02}
lW3
lQ2?cumulation_opcode_flip:cumulation_opcode,2);}
tB1
cL1
long
count,cR&eU,cD1{while
eP3<256){int
yB3
xI1::powi_table[count];if
iW2
128
xR3
cL1
half
y12
count/=half;}
else
yY3
if
eP3==1)return;if(!eP3&1)){lW3
cSqr,1);cL1
count/2
y12}
else{iN1
t91);cL1
count-1
y12
lW3
cMul,2);}
}
}
lR3
xI1{tB1
x11
long
count,cR&eU,cD1{if
eP3==0)t11
eU.basevalue);else{bool
tS2
tK3
if
eP3<0){tS2=true;count=-count;}
if(false)cL1
count
y12
iW1
count>1){PowiCache
iB;lA1
count,iB,1);size_t
xK1=synth.GetStackTop();iB.Start(t91);FPO(l54(lA4,"Calculating result for %ld...\n",count));size_t
yF2=yA
count
t81
size_t
n_excess=synth.xK
xK1;if(n_excess>0||yF2!=xK1-1){synth.DoPopNMov(xK1-1,yF2);}
}
if(tS2)lW3
eU.op_flip,1);}
}
}
#endif
#ifndef FPOptimizer_ValueRangeHH
#define FPOptimizer_ValueRangeHH
tF{lR3
lX3{iN
eI2
Comp{}
;tT1>eI2
Comp<nC
cLess>{tT1
lF<nB
cLessOrEq>{tT1
lF<=nB
cGreater>{tT1
lF>nB
cGreaterOrEq>{tT1
lF>=nB
cEqual>{tT1
lF==nB
i51>{tT1
lF!=b;}
}
;}
nU2
eI2
rangehalf{l63
val;bool
known;rangehalf():val(),known(false){}
rangehalf
e01
v):val(v),known(true){lZ2
yF3
e01
v){known=true;val=v;}
yF3
tR2(iO1
tR2
nK)val=iP1
yF3
tR2(iO1
e01
nK)val=iP1
iN
void
set_if
tR2
v,l63(iO1
tR2
nK&&lX3::Comp<Compare>()(val,v))val=iP1
iN
void
set_if
e01
v,l63(iO1
e01
nK&&lX3::Comp<Compare>()(val,v))val=iP1}
yP1
eI2
range{rangehalf
xI
min,max;range():min(),max(){}
range
tR2
mi,l63
ma):min(mi),max(ma){}
range(bool,l63
ma):min(),max(ma){}
range
tR2
mi,bool):min(mi),max(){}
void
set_abs();void
set_neg();}
yP1
bool
IsLogicalTrueValue
e11
yQ3&p,bool
abs)yP1
bool
IsLogicalFalseValue
e11
yQ3&p,bool
abs);}
#endif
#ifndef FPOptimizer_RangeEstimationHH
#define FPOptimizer_RangeEstimationHH
tF{enum
TriTruthValue{IsAlways,IsNever,iD3}
yP1
yQ3
iO
const
eV
yP1
bool
IsLogicalValue
e11
eV
yP1
TriTruthValue
GetIntegerInfo
e11
eV
yP1
xL1
GetEvennessInfo
e11
eV{if(!tree
tK1)yY1
yM1=lK4;if(isEvenInteger(eO3)yZ1
isOddInteger(eO3)c01
nU2
xL1
GetPositivityInfo
e11
eV{yQ3
p=iO
tree
iR3
p
tH1
p
l73>=l63())yZ1
p
c1
known
lN1))c01
nU2
xL1
GetLogicalValue
lZ3
tree,bool
abs){yQ3
p=iO
tree
iR3
IsLogicalTrueValue(p,abs))yZ1
IsLogicalFalseValue(p,abs))c01}
#endif
#ifndef FPOptimizer_ConstantFoldingHH
#define FPOptimizer_ConstantFoldingHH
tF{tB1
ConstantFolding(eV;}
#endif
lR3{using
lR3
FUNCTIONPARSERTYPES;using
tF;eI2
ComparisonSetBase{enum{tA3=0x1,Eq_Mask=0x2,Le_Mask=0x3,tB3=0x4,tE3=0x5,Ge_Mask=0x6}
;static
int
Swap_Mask(int
m)yM(m&Eq_Mask)|((m&tA3)?tB3:0)|((m&tB3)?tA3:0);}
enum
c91{Ok,BecomeZero,BecomeOne,nH1
eX3
xB2{cond_or,iZ2,l03,l13}
;}
yP1
eI2
ComparisonSet:public
ComparisonSetBase{eI2
tT2{eW
a;eW
b;int
relationship;tT2():a(),b(),relationship(){}
}
;std
y33<tT2>eX;eI2
Item{eW
value;bool
cW2;Item():value(),cW2(false){}
}
;std
y33<Item>cM1;int
xM1;ComparisonSet():eX(),cM1(),xM1(0){}
c91
AddItem
lZ3
a,bool
cW2,xB2
type){for
iD1
c=0;c<cM1
tG3;++c)if(cM1[c].value
xL
a)){if(cW2!=cM1[c].cW2)iC
e21
case
l13:cM1.erase(cM1.begin()+c);xM1
tC3
case
iZ2:case
l03:e51}
}
e31}
Item
pole;pole.value=a;pole.cW2=cW2;cM1.push_back(pole
iO3
c91
AddRelationship(eW
a,eW
b,int
tX1,xB2
type)iC
if(tX1==7)e21
lD
l13:if(tX1==7){xM1
tC3}
lD
iZ2:case
l03:if(tX1==0)e51
yY3
if(!(a.GetHash()<b.GetHash())){a.swap(b);tX1=Swap_Mask(tX1);}
for
iD1
c=0;c<eX
tG3;++c){if(eX[c].a
xL
a)&&eX[c].b
xL
b))iC{int
yG3=yG2|tX1;if(yG3==7)e21
yG2=yG3;break
xH3
iZ2:case
l03:{int
yG3=yG2&tX1;if(yG3==0)e51
yG2=yG3;break
xH3
l13:{int
newrel_or=yG2|tX1;int
yH2=yG2&tX1;lU2
5&&yH2==0){yG2=tE3;e31}
lU2
7&&yH2==0){xM1+=1;eX.erase(eX.begin()+c);e31}
lU2
7&&yH2==Eq_Mask){yG2=Eq_Mask;xM1
tC3}
continue;}
}
e31}
}
tT2
comp;comp.a=a;comp.b=b;comp.relationship=tX1;eX.push_back(comp
iO3}
;nT1
l63,typename
CondType>bool
ConstantFolding_LogicCommon(eW&tree,CondType
xA1,bool
yI2){bool
should_regenerate
tK3
ComparisonSet
xI
comp;lC1{typename
c6
c91
eQ3=c6
Ok;xW2
atree=iZ3;switch(atree
nD
eS3
cEqual
lH
Eq_Mask,xA1);lD
i51
lH
tE3,xA1);lD
cLess
lH
tA3,xA1);lD
cLessOrEq
lH
Le_Mask,xA1);lD
cGreater
lH
tB3,xA1);lD
cGreaterOrEq
lH
Ge_Mask,xA1);lD
cNot:eQ3
e41
l9
0),true,xA1);lD
cNotNot:eQ3
e41
l9
0),false,xA1);break;yX3
if(yI2||IsLogicalValue(atree))eQ3
e41,false,xA1);lK3
eQ3){ReplaceTreeWithZero:xM
0)iP3
true;ReplaceTreeWithOne:xM
1);nY
c6
Ok:lD
c6
BecomeZero
tH
c6
BecomeOne:i7
c6
nH1:cC1
yY3}
if(should_regenerate){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before ConstantFolding_LogicCommon: "
c4
#endif
if(yI2){tree.DelParams();}
else{for
c5{xW2
atree=xZ
a
iR3
IsLogicalValue(atree))iZ);}
}
for
iD1
a=0;a<comp.cM1
tG3;++a){if(comp.cM1[a].cW2){n43
cNot);r.cK
r.lV2
iW1!yI2){n43
cNotNot);r.cK
r.lV2
else
tree.cK}
for
iD1
a=0;a<comp.eX
tG3;++a){n43
cNop);switch(lY3
eE
eS3
c6
tA3:r
cI
cLess);lD
c6
Eq_Mask:r
cI
cEqual);lD
c6
tB3:r
cI
cGreater);lD
c6
Le_Mask:r
cI
cLessOrEq);lD
c6
tE3:r
cI
i51);lD
c6
Ge_Mask:r
cI
cGreaterOrEq
c62
r
l12
lY3].a);r
l12
lY3].b);r.lV2
if(comp.xM1!=0)tree
y2
l63(comp.xM1)));
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After ConstantFolding_LogicCommon: "
c4
#endif
iB1
lY2
yQ1
ConstantFolding_AndLogic(lF4(tree.GetOpcode()==cAnd
l34()==cAbsAnd);eP2
iZ2,true);}
yQ1
ConstantFolding_OrLogic(lF4(tree.GetOpcode()==cOr
l34()==cAbsOr);eP2
cond_or,true);}
yQ1
ConstantFolding_AddLogicItems(lF4(tree.GetOpcode()==cAdd);eP2
l13,false);}
yQ1
ConstantFolding_MulLogicItems(lF4(tree.GetOpcode()==cMul);eP2
l03,false);}
}
#include <vector>
#include <map>
#include <algorithm>
lR3{using
lR3
FUNCTIONPARSERTYPES;using
tF;eI2
CollectionSetBase{enum
xN1{Ok,nH1}
;}
yP1
eI2
CollectionSet:public
CollectionSetBase{eI2
cA1{eW
value;eW
yJ2;bool
e2;cA1():value(),yJ2(),e2(false){}
cA1
lZ3
v,xW2
f):value(v),yJ2(f),e2(false){}
}
;std::multimap<yD2,cA1>iD;typedef
typename
std::multimap<yD2,cA1>::yL3
xO1;CollectionSet():iD(){}
xO1
FindIdenticalValueTo
lZ3
eO3{yD2
hash=value.GetHash();for(xO1
i=iD.yK2
hash);i!=iD.e61
hash;++i){cJ1
xL
i
eQ2.value
cA3
i;}
return
iD.end();}
bool
Found
e11
xO1&b)yM
b!=iD.end();}
xN1
AddCollectionTo
lZ3
yJ2,const
xO1&into_which){cA1&c=into_which
eQ2;if(c.e2)c.yJ2
xM2
yJ2);else{eW
add;add
cI
cAdd);add
l12
c.yJ2);add
xM2
yJ2);c.yJ2.swap(add);c.e2=true;}
e31}
xN1
xC2
lZ3
eL3
xW2
yJ2){const
yD2
hash=value.GetHash();xO1
i=iD.yK2
hash);for(;i!=iD.e61
hash;++i){if(i
eQ2.value
xL
value
cA3
AddCollectionTo(yJ2,i);}
iD.yH3,std::make_pair(hash,cA1(iL3))iO3
xN1
xC2
lZ3
a)yM
xC2(a,nI1
1)));}
}
yP1
eI2
ConstantExponentCollection{typedef
eN
yJ3;typedef
std::x41
yL2;std
y33<yL2>data;ConstantExponentCollection():data(){}
void
MoveToSet_Unique
e01
eN1&eO1){data.push_back(std::x41(eN1()));data.back()n23.swap(eO1)eW3
MoveToSet_NonUnique
e01
eN1&eO1){typename
std
y33<yL2>::yL3
i=std::yK2
data.l23
data.end(),xN3,Compare1st()iR3
i!=data.e61
y72{i
eQ2.yH3
eQ2.end(),eO1.l23
eO1.end());}
else{data.yH3,std::x41(xN3,eO1));}
}
bool
Optimize(){bool
changed
tK3
std::sort(data.l23
data.end(),Compare1st());redo:for
iD1
a=0;a<data
tG3;++a
eU2
exp_a=data[a
tF3;if
y73
exp_a,l63(1)))continue;for
iD1
b=a+1;b<data
tG3;++b
eU2
exp_b=data[b
tF3
l53
yM2=exp_b-exp_a;if(yM2>=fp_abs(exp_a))break
l53
exp_diff_still_probable_integer=yM2*l63(16
iR3
tU2
exp_diff_still_probable_integer)&&!(tU2
exp_b)&&!tU2
yM2))){yJ3&a_set=lW2;yJ3&b_set=data[b]n23;
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before ConstantExponentCollection iteration:\n"
;tV2
cout);
#endif
if(isEvenInteger(exp_b)&&!isEvenInteger(yM2+exp_a
eO
tmp2;tmp2
tI3
tmp2
iA1
b_set);tmp2
nJ2
eW
tmp;tmp
cI
cAbs);tmp.cG1
tmp
nJ2
b_set
xV3
1);b_set[0].iO2}
a_set.insert(a_set.end(),b_set.l23
b_set.end());yJ3
b_copy=b_set;data.erase(data.begin()+b);MoveToSet_NonUnique(yM2,b_copy);xQ2
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After ConstantExponentCollection iteration:\n"
;tV2
cout);
#endif
tG1}
}
}
return
changed;}
#ifdef DEBUG_SUBSTITUTIONS
void
tV2
ostream&out){for
iD1
a=0;a<data
tG3;++a){out.precision(12);out<<data[a
tF3<<": "
;iC1
lW2
tG3;++b){if(b>0)out<<'*'yA3
lW2[b],out);}
out<<std::endl;}
}
#endif
}
yP1
static
eW
x51
eW&eL3
bool&xP
c13
value
nD
eS3
cPow:{eW
eY2
value
l9
1);value.xP1
return
xN3
xH3
cRSqrt:value.xP1
xP=true
iP3
nI1-0.5));case
cInv:value.xP1
xP=true
iP3
nI1-1));yX3
yY3
return
nI1
1));}
cH1
void
eP1
tL1&mul,xW2
tree,xW2
yJ2,bool&cB1
bool&xP){lC1{eW
value(iZ3);eW
xN3(x51
eL3
xP)iR3!yJ2
tK1||yJ2.xQ1!=l63(1.0
eO
e71;e71
tI3
e71
xM2
y72;e71
xM2
yJ2);e71.Rehash()yT1
swap(e71);}
#if 0 /* FIXME: This does not work */
cJ1
nD==cMul){if(1){bool
exponent_is_even=xN3
tK1&&isEvenInteger(xN3.xQ1);iC1
value
yM3{bool
tmp
tK3
eW
val(value
l9
b));eW
exp(x51
val,tmp)iR3
exponent_is_even||(exp
tK1&&isEvenInteger(exp.xQ1)eO
e71;e71
tI3
e71
xM2
y72;e71
l12
exp);e71.ConstantFolding(iR3!e71
tK1||!isEvenInteger(e71.xQ1)){goto
cannot_adopt_mul;}
}
}
}
eP1
mul,eL3
xN3,cB1
xP);}
else
cannot_adopt_mul:
#endif
{if(mul.xC2(eL3
y72==CollectionSetBase::nH1)cC1}
}
}
yQ1
ConstantFolding_MulGrouping(eV{bool
xP
tK3
bool
should_regenerate
tK3
tL1
mul;eP1
mul,tree,nI1
1)),cB1
xP);typedef
std::pair<eW,eN>eQ1;typedef
std::multimap<yD2,eQ1>cN1;cN1
tP;yN2
tL1::xO1
j=mul.iD.yK3
j!=mul.iD.end();++j){eW&value=j
eQ2.value;eW&eY2
j
eQ2.yJ2;if(j
eQ2.e2)xN3
nJ2
const
yD2
eR1=xN3.GetHash();typename
cN1::yL3
i=tP.yK2
eR1);for(;i!=tP.e61
eR1;++i)if(i
eQ2.first
xL
y72){if(!xN3
tK1||!e81.xQ1,l63(1)))cC1
i
eQ2
n23.push_back(eO3;goto
skip_b;}
tP.yH3,std::make_pair(eR1,std::make_pair(xN3,eN
iD1(1),eO3)));skip_b:;}
#ifdef FP_MUL_COMBINE_EXPONENTS
ConstantExponentCollection
xI
e91;yN2
cN1::yL3
j,i=tP.yK3
i!=tP.end();i=j){j=i;++j;eQ1&list=i
eQ2;y82
n11
eY2
list.first.xQ1;if(!(xN3==xD1)e91.MoveToSet_Unique(xN3,list
n03
tP.erase(i);}
}
if(e91.Optimize())cC1
#endif
if(should_regenerate){eW
before=tree;before.lD1
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before ConstantFolding_MulGrouping: "
yA3
before)xH1"\n"
;
#endif
tree.DelParams();yN2
cN1::yL3
i=tP.yK3
i!=tP.end();++i){eQ1&list=i
eQ2;
#ifndef FP_MUL_COMBINE_EXPONENTS
y82
n11
eY2
list.first.xQ1;if(xN3==xD1
continue;if(e81
y22
tree.AddParamsMove(list
n03
continue;}
}
#endif
eW
mul;mul
tI3
mul
iA1
list
n03
mul
nJ2
if(xP&&list.first
tK1){y82
xQ1==l63(1)/l63(3
eO
cbrt;cbrt
cI
cCbrt);cbrt.e8
cbrt.nL2
cbrt
y7
0.5
eO
sqrt;sqrt
cI
cSqrt);sqrt.e8
sqrt.nL2
sqrt
y7-0.5
eO
rsqrt;rsqrt
cI
cRSqrt);rsqrt.e8
rsqrt.nL2
rsqrt
y7-1
eO
inv;inv
cI
cInv);inv.e8
inv.nL2
inv);continue;}
}
eW
pow;pow
cI
cPow);pow.e8
pow
l12
list.first);pow.nL2
pow);}
#ifdef FP_MUL_COMBINE_EXPONENTS
tP.clear(cF2
0;a<i4
tG3;++a
eU2
eY2
i4[a
tF3;if(e81
y22
tree.AddParamsMove(i4[a]n03
continue;}
eW
mul;mul
tI3
mul
iA1
i4[a]n03
mul
nJ2
eW
pow;pow
cI
cPow);pow.e8
pow
y2
y72);pow.nL2
pow);}
#endif
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After ConstantFolding_MulGrouping: "
c4
#endif
return!tree
xL
before);}
lY2
yQ1
ConstantFolding_AddGrouping(eV{bool
should_regenerate
tK3
tL1
add;lC1{if(iZ3
nD==cMul
eM2
add.xC2(iZ3)==CollectionSetBase::nH1)cC1}
y42
remaining(iU);size_t
tQ=0;lC1{xW2
mulgroup=iZ3;if
l33
nD==cMul){iC1
mulgroup
yM3{if
l33
l9
b)tK1)continue;typename
tL1::xO1
c=add.FindIdenticalValueTo
l33
l9
b)iR3
add.Found(c
eO
tmp
l33
yB
CloneTag());tmp
nI2
b);tmp
nJ2
add.AddCollectionTo(tmp,c);cC1
goto
done_a;}
}
remaining[a]=true;tQ+=1;done_a:;}
}
if(tQ>0){if(tQ>1){std
y33<std::pair<eW,size_t> >nX;std::multimap<yD2,size_t>eS1;bool
n53
tK3
lC1
iG1{iC1
iZ3
yM3{xW2
p=iZ3
l9
b);const
yD2
p_hash=p.GetHash();for(std::multimap<yD2,size_t>::const_iterator
i=eS1.yK2
p_hash);i!=eS1.e61
p_hash;++i){if(nX[i
eQ2
tF3
xL
p)){nX[i
eQ2]n23+=1;n53=true;goto
found_mulgroup_item_dup;}
}
nX.push_back(std::make_pair(p,size_t(1)));eS1.insert(std::make_pair(p_hash,nX
tG3-1));found_mulgroup_item_dup:;}
}
if(n53){eW
eZ2;{size_t
max=0;for
iD1
p=0;p<nX
tG3;++p)if(nX[p]n23<=1)nX[p]n23=0;else{nX[p]n23*=nX[p
tF3
x12;if(nX[p]n23>max){eZ2=nX[p
tF3;max=nX[p]n23;}
}
}
eW
group_add;group_add
cI
cAdd);
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Duplicate across some trees: "
yA3
eZ2)xH1" in "
c4
#endif
lC1
iG1
iC1
iZ3
yM3
if(eZ2
xL
iZ3
l9
b)eO
tmp(iZ3
yB
CloneTag());tmp
nI2
b);tmp
nJ2
group_add
l12
tmp);remaining[a]tK3
yY3
group_add
nJ2
eW
group;group
tI3
group
l12
eZ2);group
l12
group_add);group
nJ2
add.xC2(group);cC1}
}
lC1
iG1{if(add.xC2(iZ3)==CollectionSetBase::nH1)cC1}
}
if(should_regenerate){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before ConstantFolding_AddGrouping: "
c4
#endif
tree.DelParams();yN2
tL1::xO1
j=add.iD.yK3
j!=add.iD.end();++j){eW&value=j
eQ2.value;eW&coeff=j
eQ2.yJ2;if(j
eQ2.e2)coeff
nJ2
if(coeff
iN3
y73
coeff.xQ1,xD1
eM2
y83
coeff.xQ1
y22
tree
l12
eO3;continue;}
}
eW
mul;mul
tI3
mul
l12
eO3;mul
l12
coeff);mul.nL2
mul);}
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After ConstantFolding_AddGrouping: "
c4
#endif
iB1
lY2}
lR3{using
lR3
FUNCTIONPARSERTYPES;using
tF
yP1
bool
ConstantFolding_IfOperations(lF4(tree.GetOpcode()==cIf
l34()==cAbsIf);for(;;){if(cB3==cNot){tZ2
cIf);xZ
0).t02
0)l9
0));xZ
1).swap(xZ
2));}
iW1
xZ
0)cT1{tZ2
tO3;xZ
0).t02
0)l9
0));xZ
1).swap(xZ
2));}
else
break;lK3
n61
0),tX2==tO3)lE2
tree.t02
1));nY
lN3
tree.t02
2));nY
n71
if(cB3==cIf||cB3==tO3{eW
cond=xZ
0);eW
n63;n63
i02==cIf?cNotNot:cAbsNotNot);n63.xR1
1));ConstantFolding(n63);eW
n73;n73
i02==cIf?cNotNot:cAbsNotNot);n73.xR1
2));ConstantFolding(n73
iR3
n63
tK1||n73.IsImmed(eO
eY;eY
i02);eY.xR1
1));eY.nM
1));eY.nM
2));eY
nJ2
eW
eZ;eZ
i02);eZ.xR1
2));eZ.nM
1));eZ.nM
2));eZ
nJ2
tZ2
cond
nD)iX2
0,cond
l9
0));tJ
1,eY);tJ
2,eZ);iB1}
if(iV3
xZ
2)nD&&(iV3
cIf||iV3
cAbsIf
eO&tP3=xZ
1);eW&leaf2=xZ
2);if
l83
0)xC1
0))&&l83
1)xC1
1))||tP3
l9
2
tQ3)eO
eY;eY
iQ1
eY.iF3
eY.xS1
1));eY.xT1
1));eY
nJ2
eW
eZ;eZ
iQ1
eZ.iF3
eZ.xS1
2));eZ.xT1
2));eZ
lF1
tree
eR2
0,tP3
l9
0));tJ
1,eY);tJ
2,eZ);iB1
if
l83
1)xC1
1))&&tP3
l9
2
tQ3
eO
t0;t0
iQ1
t0
l12
xZ
iG3.xS1
iG3.xT1
iG3
lF1
tJ
0,t0)iX2
2,tP3
l9
2))iX2
1,tP3
l9
1));iB1
if
l83
1
tQ3)&&tP3
l9
2)xC1
1)eO
t12;t12
cI
leaf2
nD==cIf?cNot:cZ3);t12.xT1
0));t12
nJ2
eW
t0;t0
iQ1
t0
l12
xZ
iG3.xS1
iG3
l12
t12);t0
lF1
tJ
0,t0)iX2
2,tP3
l9
2))iX2
1,tP3
l9
1));iB1}
eW&xY=xZ
1);eW&y3=xZ
2
iR3
xY
xL
y3)){tree.t02
1));iB1
const
OPCODE
op1=xY
nD;const
OPCODE
op2=y3
nD;if
yO3
op2){if(xY
cZ1==1){eW
lP
0));l51
0
x0
n5
if(xY
cZ1==2&&y3
cZ1==2){if(xY
l9
0)xL
y3
l9
0)eO
param0=xY
l9
0);eW
lP
1));l51
1
x0
iS
param0)n5
if(xY
l9
1)xL
y3
l9
1)eO
param1=xY
l9
1);eW
lP
0));l51
0
x0
iS
nG1)iS
param1);iB1}
if
yO3
yP3
cMul
lX2
cAnd
lX2
cOr
lX2
cAbsAnd
lX2
cAbsOr
lX2
cMin
lX2
cMax){eN
n83;cD{for
iD1
b=y3
cZ1;b-->0;){if(xY
n93
y3
l9
b))){if(n83
cT3){xY
lM4.lD1}
n83.push_back(xY
l9
a));y3
nI2
b);xY
nI2
a
c62}
}
if(!n83
cT3){xY
nJ2
y3.Rehash()l8
op1);tree
iA1
n83)n5}
}
if
yO3
yP3
cMul||yO3
cAnd
iW3
y3))||yO3
cOr
iW3
y3))){cD
if(xY
n93
y3)){xY.lD1
xY
nI2
a);xY
nJ2
eW
cO1=y3;y3=tR
op1==yP3
cOr)l43
op1)iS
cO1)n5}
if(yO3
cAnd
lX2
cOr)&&op2==cNotNot){eW&nB3=y3
l9
0);cD
if(xY
n93
nB3)){xY.lD1
xY
nI2
a);xY
nJ2
eW
cO1=nB3;y3=tR
op1
tJ3
op1)iS
cO1)n5}
if(op2==cAdd||op2==cMul||(op2==cAnd
iW3
xY))||(op2==cOr
iW3
xY))){for
iD1
a=y3
lW1
y3
n93
xY)){y3
lM4
nI2
a);y3
nJ2
eW
cP1=xY;xY=tR
op2==cAdd||op2
tJ3
op2)iS
cP1)n5}
if((op2==cAnd||op2==cOr)&&op1==cNotNot){eW&nC3=xY
l9
0
cF2
y3
lW1
y3
n93
nC3)){y3
lM4
nI2
a);y3
nJ2
eW
cP1=nC3;xY=tR
op2
tJ3
op2)iS
cP1)n5}
lY2}
#include <limits>
lR3{using
lR3
FUNCTIONPARSERTYPES;using
tF
yP1
int
maxFPExponent()yM
std::numeric_limits
xI::max_exponent;}
yQ1
x61
l63
base,l63
y72{if(base<xD1
return
true;if
y73
base,xD1||y83
base,l63(1))cJ
return
xN3>=l63(maxFPExponent
xI())/fp_log2(base);}
yQ1
ConstantFolding_PowOperations(lF4(tree.GetOpcode()==cPow);nT&&xZ
1).n11
const_value
tM3
lS,xZ
iV);xM
const_value);lY2
if(tI1
y83
xZ
iV
y22
tree.t02
0));iB1
nT&&y83
lS
y22
xM
1);lY2
nT&&iV3
cMul){bool
yO2=false
l53
n12=lS;eW
mulgroup=xZ
1
cF2
mulgroup
lW1
mulgroup
l9
a).n11
imm=mulgroup
l9
a).xQ1;{if(x61
n12,imm))break
l53
n22
tM3
n12,imm);if
y73
n22,xD1)break;if(!yO2){yO2=true;l22
lD1}
n12=n22;l22
DelParam(a
c62}
if(yO2){l22
Rehash();
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before pow-mul change: "
c4
#endif
xZ
0).Become(eA1
n12));xZ
1).Become
l33);
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After pow-mul change: "
c4
#endif
}
}
if(tI1
cB3==cMul
eU2
n32=xZ
iV
l53
yP2=1.0;bool
yO2
tK3
eW&mulgroup=xZ
0
cF2
mulgroup
lW1
mulgroup
l9
a).n11
imm=mulgroup
l9
a).xQ1;{if(x61
imm,n32))break
l53
eT1
tM3
imm,n32);if
y73
eT1,xD1)break;if(!yO2){yO2=true;l22
lD1}
yP2*=eT1;l22
DelParam(a
c62}
if(yO2){l22
Rehash();eW
eR3;eR3
cI
cPow);eR3
iA1
tree.lH2));eR3.xJ2;tZ2
cMul)iS
eR3);xL2
eA1
yP2));iB1}
if(cB3==cPow&&tI1
xZ
0)l9
1).n11
a=xZ
0)l9
iV
l53
b=xZ
iV
l53
c=a*b;if(isEvenInteger(a)&&!isEvenInteger(c
eO
nD3;nD3
cI
cAbs);nD3.nM
0)l9
0));nD3
nJ2
tJ
0,nD3);}
else
tree
eR2
0,xZ
0)l9
0))iX2
1,eA1
c));}
lY2}
lR3{using
lR3
FUNCTIONPARSERTYPES;using
tF;eI2
l4{enum
t22{MakeFalse=0,MakeTrue=1,tW2=2,nG3=3,MakeNotNotP0=4,MakeNotNotP1=5,MakeNotP0=6,MakeNotP1=7,xN=8
eX3
n42{Never=0,Eq0=1,Eq1=2,Gt0Le1=3,cE3=4}
;t22
if_identical;t22
n52
4];eI2{t22
what:4;n42
when:4;}
iR1,iS1,iT1,iU1
yP1
t22
Analyze
lZ3
a,xW2
b)const{if(a
xL
b
cA3
if_identical;yQ3
p0=iO
a);yQ3
p1=iO
b
iR3
p0
c1
iB3
p1
iL2){if(p0
c1
val<p1
l73&&n52
0]i5
0];if(p0
c1
val<=p1
l73&&n52
1]i5
1];}
if(p0
tH1
p1
tO1{if(p0
l73>p1
c1
val&&n52
2]i5
2];if(p0
l73>=p1
c1
val&&n52
3]i5
3];}
if(IsLogicalValue(a)){if(iR1
l93
iR1.when,p1
cA3
iR1.what;if(iT1
l93
iT1.when,p1
cA3
iT1.what;}
if(IsLogicalValue(b)){if(iS1
l93
iS1.when,p0
cA3
iS1.what;if(iU1
l93
iU1.when,p0
cA3
iU1.what;}
return
xN;}
cH1
bool
TestCase(n42
when,const
yQ3&p){if(!p
iL2||!p
c1
known
cJ
switch(when
eS3
Eq0
nX1==l63(0.0)&&e23==p
l73;case
Eq1
nX1==l63(1.0)&&e23==e23;case
Gt0Le1
nX1>y51&&e23<=l63(1);case
cE3
nX1>=y51
lN1
1);yX3;}
lY2}
;lR3
RangeComparisonsData{static
const
l4
Data[6]={{l4
nE3
tZ
xN,l4::tZ
xN
lY1
Eq1
lZ1
Eq1
xE1
Eq0}
l11
Eq0}
}
,yW3
nF3
xN,l4
nF3
xN
lY1
Eq0
lZ1
Eq0
xE1
Eq1}
l11
Eq1}
}
,yW3
nF3
tW2,l4::tZ
MakeFalse
xE1
Gt0Le1
lZ1
cE3
yX,{l4
nE3
xN,l4
nF3
tZ
nG3
xE1
cE3
lZ1
Gt0Le1
yX,yW3::tZ
tZ
MakeTrue,l4::tW2
lY1
cE3}
l11
Gt0Le1
yX,{l4
nE3
tZ
nG3,l4::xN,l4
nJ1
lY1
Gt0Le1}
l11
cE3
yX}
;}
yQ1
ConstantFolding_Comparison(eV{using
lR3
RangeComparisonsData;assert(tree.GetOpcode()>=cEqual&&tree.GetOpcode()<=cGreaterOrEq);switch(Data[tX2-cEqual].Analyze(xZ
0),xZ
1))eS3
l4::MakeFalse:xM
0);nY
l4
nJ1:xM
1
y52
nG3:tZ2
cEqual
y52
tW2:tZ2
i51
y52
MakeNotNotP0:tZ2
cNotNot
c71
1
y52
MakeNotNotP1:tZ2
cNotNot
c71
0
y52
MakeNotP0:tZ2
cNot
c71
1
y52
MakeNotP1:tZ2
cNot
c71
0
y52
xN:;}
if(xZ
1)tK1)switch(cB3
eS3
cAsin:lN
fp_sin(xZ
lC3
cAcos:lN
fp_cos(xZ
iV)));tZ2
tX2==cLess?cGreater:tX2==cLessOrEq?cGreaterOrEq:tX2==cGreater?cLess:tX2==cGreaterOrEq?cLessOrEq:tX2);nY
cAtan:lN
fp_tan(xZ
lC3
cLog:lN
fp_exp(xZ
lC3
cSinh:lN
fp_asinh(xZ
lC3
cTanh:if(fp_less(fp_abs(xZ
iV)y22
lN
fp_atanh(xZ
iV)));iB1
break;yX3
yY3
lY2}
#include <list>
#include <algorithm>
#ifdef FP_SUPPORT_OPTIMIZER
using
lR3
FUNCTIONPARSERTYPES;lR3{
#ifdef DEBUG_SUBSTITUTIONS
yD
double
d){union{double
d;uint_least64_t
h;}
i22
d=d;lR1
h
nY1
#ifdef FP_SUPPORT_FLOAT_TYPE
yD
float
f){union{float
f;uint_least32_t
h;}
i22
f=f;lR1
h
nY1
#endif
#ifdef FP_SUPPORT_LONG_DOUBLE_TYPE
yD
long
double
ld){union{long
double
ld;eI2{uint_least64_t
a;xL3
short
b;}
s;}
i22
ld=ld;lR1
s.b<<data.s.a
nY1
#endif
#ifdef FP_SUPPORT_LONG_INT_TYPE
yD
long
ld){o<<"("
<<std::hex<<ld
nY1
#endif
#endif
}
tF{lO
nF)){}
lO
const
l63&i
yB
xT3
nF
i
cC3
#ifdef __GXX_EXPERIMENTAL_CXX0X__
lO
l63&&i
yB
xT3
nF
i32
i)cC3
#endif
lO
xL3
v
yB
VarTag
nF
lG3,v
cC3
lO
xV2
o
yB
OpcodeTag
nF
o
cC3
lO
xV2
o,xL3
f
yB
FuncOpcodeTag
nF
o,f
cC3
lO
xW2
b
yB
CloneTag
nF*b.data)){}
nU2
eW::~CodeTree(){}
lC
ReplaceWithImmed
e01
i){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Replacing "
yA3*this
iR3
IsImmed())OutFloatHex(std::cout,xQ1)xH1" with const value "
<<i;OutFloatHex(std::cout,i)xH1"\n"
;
#endif
data=new
y92
xI(i);}
nU2
eI2
ParamComparer{iU2()lZ3
a,xW2
b)const{if(a
x12!=b
x12)return
a
x12<b
x12
iP3
a.GetHash()<b.GetHash();}
}
xX3
y92
xI::Sort(c13
Opcode
eS3
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
i51:std::sort(lE3
l23
lE3
end(),ParamComparer
xI());lD
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
break;yX3
yY3}
lC
AddParam
lZ3
param){y0.push_back(param);}
lC
l32
eW&param){y0.push_back(eW());y0.back().swap(param);}
lC
SetParam
iD1
which,xW2
b)nZ1
which
lD3
y0[which]=b;}
lC
SetParamMove
iD1
which,eW&b)nZ1
which
lD3
y0[which
i63
b);}
lC
AddParams
e11
nN){y0.insert(y0.end(),n62.l23
n62.end());}
lC
AddParamsMove(nN){size_t
endpos=y0
tG3,added=n62
tG3;y0
xV3
endpos+added,eW());for
iD1
p=0;p<added;++p)y0[endpos+p
i63
n62[p]);}
lC
AddParamsMove(nN,size_t
n72)nZ1
n72
lD3
DelParam(n72);AddParamsMove(tS1}
lC
SetParams
e11
nN){eN
tmp(tS1
y0.iO2}
lC
SetParamsMove(nN){y0.swap(tS1
n62.clear();}
#ifdef __GXX_EXPERIMENTAL_CXX0X__
lC
SetParams(eN&&n62){SetParamsMove(tS1}
#endif
lC
DelParam
iD1
index){eN&Params=y0;
#ifdef __GXX_EXPERIMENTAL_CXX0X__
lE3
erase(lE3
begin()+index);
#else
tS3
index].data=0;for
iD1
p=index;p+1<xI3;++p)tS3
p].data.UnsafeSetP(&*tS3
p+1
lD3
tS3
xI3-1].data.UnsafeSetP(0);lE3
resize(xI3-1);
#endif
}
lC
DelParams(){y0.clear();}
yQ1
eW::IsIdenticalTo
lZ3
b)const{if(&*data==&*b.data)return
true
iP3
data->IsIdenticalTo(*b.data);}
yQ1
y92
xI::IsIdenticalTo
e11
y92
xI&b)const{if(Hash!=b.Hash
cJ
if(Opcode!=tT3
cJ
switch(Opcode
eS3
cImmed:return
y83
Value,tU3;case
lG3:return
lJ2==b.lI2
case
cFCall:case
cPCall:if(lJ2!=b.lJ2
cJ
break;yX3
yY3
if(xI3!=b.xI3
cJ
for
iD1
a=0;a<xI3;++a){if(!lF3
xL
b.lF3)cJ}
iB1
lC
Become
lZ3
b){if(&b!=this&&&*data!=&*b.data){DataP
tmp=b.data;lD1
data.iO2}
}
lC
CopyOnWrite(){if(GetRefCount()>1)data=new
y92
xI(*data);}
nU2
eW
eW::GetUniqueRef(){if(GetRefCount()>1)return
eW(*this,CloneTag())iP3*this;}
i0
yH
cNop
iS3),n9
i0
const
y92&b
yH
tT3
iS3
tU3,lJ2(b.cR1,tR3
b.Params),Hash(b.Hash),Depth(b.Depth),tY1
b.lK2){}
i0
const
l63&i
yH
cImmed
iS3
i),n9
#ifdef __GXX_EXPERIMENTAL_CXX0X__
i0
y92
xI&&b
yH
tT3
iS3
i32
tU3),lJ2(b.cR1,tR3
i32
b.Params)),Hash(b.Hash),Depth(b.Depth),tY1
b.lK2){}
i0
l63&&i
yH
cImmed
iS3
i32
i)),n9
#endif
i0
xV2
o
yH
o
iS3),n9
i0
xV2
o,xL3
f
yH
o
iS3),lJ2(f),tR3),Hash(),Depth(1),tY1
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
lR3
FUNCTIONPARSERTYPES;
#ifdef FUNCTIONPARSER_SUPPORT_DEBUGGING
lR3{tB1
tZ1
nR,std
cL&done,std::ostream&o){lC1
tZ1
iZ3,done,o);std::ostringstream
buf
yA3
tree,buf);done[tree.GetHash()].insert(buf.str());}
}
#endif
tF{
#ifdef FUNCTIONPARSER_SUPPORT_DEBUGGING
tB1
DumpHashes(cP){std
cL
done;tZ1
tree,done,o);for(std
cL::const_iterator
i=done.yK3
i!=done.end();++i){const
std::set<std
c53>&flist=i
eQ2;if(flist
tG3!=1)o<<"ERROR - HASH COLLISION?\n"
;for(std::set<std
c53>::const_iterator
j=flist.yK3
j!=flist.end();++j){o<<'['<<std::hex<<i->first.hash1<<','<<i->first.hash2<<']'<<std::dec;o<<": "
<<*j<<"\n"
;}
}
}
tB1
DumpTree(cP){xY3
l94;switch(tX2
eS3
cImmed:o<<lK4;cX2
lG3:o<<"Var"
<<(tree.GetVar()-lG3);cX2
cAdd:l94"+"
;lD
cMul:l94"*"
;lD
cAnd:l94"&"
;lD
cOr:l94"|"
;lD
cPow:l94"^"
;break;yX3
l94;o<<cV3
tX2);t62
cFCall||tX2==cPCall)o<<':'<<tree.GetFuncNo();}
o<<'(';if(iU<=1&&sep2[1])o<<(sep2+1)<<' ';lC1{if(a>0)o<<' 'yA3
iZ3,o
iR3
a+1<iU)o<<sep2;}
o<<')';}
tB1
DumpTreeWithIndent(cP,const
std
c53&indent){o<<'['<<std::hex<<(void*)(&tree.lH2))<<std::dec<<','<<tree.GetRefCount()<<']';o<<indent<<'_';switch(tX2
eS3
cImmed:o<<"cImmed "
<<lK4;o<<'\n';cX2
lG3:o<<"VarBegin "
<<(tree.GetVar()-lG3);o<<'\n'iP3;yX3
o<<cV3
tX2);t62
cFCall||tX2==cPCall)o<<':'<<tree.GetFuncNo();o<<'\n';}
lC1{std
c53
ind=indent;for
iD1
p=0;p<ind
tG3;p+=2)if(ind[p]=='\\')ind[p]=' ';ind+=(a+1<iU)?" |"
:" \\"
;DumpTreeWithIndent(iZ3,o,ind);}
o<<std::flush;}
#endif
}
#endif
using
lR3
l81;using
lR3
FUNCTIONPARSERTYPES;
#include <cctype>
lR3
l81{xL3
ParamSpec_GetDepCode
e11
eN2&b
c13
b.first
eS3
eU3:{cW*s=(cW*)b
n23
iP3
s->depcode
xH3
SubFunction:{cX*s=(cX*)b
n23
iP3
s->depcode;}
yX3
yY3
return
0;}
tB1
DumpParam
e11
eN2&c12
std::ostream&o){static
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
;xL3
yQ2
0;switch(nP3.first
eS3
eT3{const
ParamSpec_NumConstant
xI
c33
e11
ParamSpec_NumConstant
xI*xY1
using
lR3
FUNCTIONPARSERTYPES;o.precision(12);o<<c43
constvalue;break
xH3
eU3:{cW
c33(cW*xY1
o<<ParamHolderNames[c43
index];yQ2
c43
constraints;break
xH3
SubFunction:{cX&param
yV3
yQ2
c43
constraints;yE
GroupFunction){if(c43
lG1==cNeg){o<<"-"
;n3}
iW1
c43
lG1==cInv){o<<"/"
;n3}
else{std
c53
opcode=cV3(xV2)c43
lG1).substr(1
cF2
0;a<opcode
tG3;++a)opcode[a]=(char)std::toupper(opcode[a]);o<<opcode<<"( "
;n3
o<<" )"
;}
}
else{o<<'('<<cV3(xV2)c43
lG1)<<' ';yE
PositionalParams)o<<'[';yE
SelectedParams)o<<'{';n3
if
e12.n2!=0)o<<" <"
<<c43
data.n2<<'>';yE
PositionalParams)o<<"]"
;yE
SelectedParams)o<<"}"
;o<<')';}
yY3
lK3
ImmedConstraint_Value
eU1
ValueMask)eS3
ValueMask:lD
Value_AnyNum:lD
xE2:o<<"@E"
;lD
Value_OddInt:o<<"@O"
;lD
i11:o<<"@I"
;lD
Value_NonInteger:o<<"@F"
;lD
eV1:o<<"@L"
;break;lK3
ImmedConstraint_Sign
eU1
SignMask)eS3
SignMask:lD
Sign_AnySign:lD
nK1:o<<"@P"
;lD
eW1:o<<"@N"
;break;lK3
ImmedConstraint_Oneness
eU1
OnenessMask)eS3
OnenessMask:lD
Oneness_Any:lD
Oneness_One:o<<"@1"
;lD
Oneness_NotOne:o<<"@M"
;break;lK3
ImmedConstraint_Constness
eU1
ConstnessMask)eS3
ConstnessMask:lD
i01:if(nP3.first==eU3){cW
c33(cW*xY1
if(c43
index<2)yY3
o<<"@C"
;lD
Constness_NotConst:o<<"@V"
;lD
Oneness_Any:yY3}
tB1
DumpParams
xA2
paramlist,xL3
count,std::ostream&o){for(eY1=0;a<count;++a){if(a>0)o<<' ';const
eN2&param=eJ1
xI(paramlist,a);DumpParam
xI(param,o);xL3
depcode=ParamSpec_GetDepCode(param
iR3
depcode!=0)o<<"@D"
<<depcode;}
}
}
#include <algorithm>
using
lR3
l81;using
lR3
FUNCTIONPARSERTYPES;lR3{cW
plist_p[37]={{2,0,0x0}
xD
0,0x4}
xD
nK1,0x0}
xD
eW1|Constness_NotConst,0x0}
xD
Sign_NoIdea,0x0}
xD
eV1,0x0}
,{3,Sign_NoIdea,0x0}
,{3,0,0x0}
,{3,eV1,0x0}
,{3,0,0x8}
,{3,Value_OddInt,0x0}
,{3,Value_NonInteger,0x0}
,{3,xE2,0x0}
,{3,nK1,0x0}
,{0,eW1|lW{0,lW{0,nK1|lW{0,xE2|lW{0,i01,0x1}
,{0,i11|nK1|lW{0,i21
i01,0x1}
,{0,i21
lW{0,Oneness_One|lW{0,eV1|lW{1,lW{1,xE2|lW{1,i21
lW{1,i11|lW{1,nK1|lW{1,eW1|lW{6,0,0x0}
,{4,0,0x0}
,{4,i11,0x0}
,{4,lW{4,0,0x16}
,{5,0,0x0}
,{5,lW}
yP1
eI2
plist_n_container{static
const
ParamSpec_NumConstant
xI
plist_n[20];}
yP1
const
ParamSpec_NumConstant
xI
plist_n_container
xI::plist_n[20]={{l63(-2
i1-1
i1-0.5
i1-0.25
i1
0
i42
fp_const_deg_to_rad
iY3
fp_const_einv
iY3
fp_const_log10inv
xI(i1
0.5
i42
fp_const_log2
xI(i1
1
i42
fp_const_log2inv
xI(i1
2
i42
fp_const_log10
iY3
fp_const_e
iY3
fp_const_rad_to_deg
iY3-fp_const_pihalf
xI(),xU1{y51,xU1{fp_const_pihalf
xI(),xU1{fp_const_pi
xI(),xU1}
;cX
plist_s[511]={{{1,15,nH3,397,nH3,471,nH3,15,cNeg,GroupFunction,0}
,i01,0x1
i52
15,yR2
24,yR2
459,yR2
460,yR2
492,cInv,lU
2,333122
x2
t52
48276
x2
l6
260151
x2
l6
464028
x2
l6
172201
x2
l6
48418
x2
l6
1331
x2
l6
172354
x2
l6
39202
x2
l6
312610
x2
l6
470469
x2
l6
296998
x2
l6
47
x2
SelectedParams,0
l04
nJ
162863
x2
l6
25030
x2
l6
7168
x2
l6
199680
x2
l6
35847
x2
l6
60440
x2
l6
30751
x2
l6
186549
x2
l6
270599
x2
l6
60431
x2
l6
259119
x2
l6
291840
x2
l6
283648
x2
l6
220360
x2
l6
220403
x2
l6
239890
x2
l6
240926
x2
l6
31751
x2
l6
382998
x2
SelectedParams,0
l04
nJ
384239
x2
l6
385071
x2
l6
386289
x2
l6
18674
x2
l6
61682
x2
l6
283969
x2
l6
283971
x2
l6
283976
x2
l6
386376
x2
l6
245767
x2
l6
385062
x2
l6
210991
x2
SelectedParams,0}
,0,0x1
nJ
15814
x2
l6
393254
x2
SelectedParams,0}
,0,0x5
nJ
393254
x2
l6
321583
x2
l6
297007
x2
l6
393263
x2
l6
393263
x2
SelectedParams,0}
,0,0x5
nJ
162871
x2
l6
258103
x2
SelectedParams,0
i41
0,0
x2
nH
0,0
i31
1,45
x2
nH
1,53
x2
nH
1,54
x2
nH
1,55
x2
nH
1,56
x2
nH
1,26
x2
nH
1,259
t32
0x16
i52
253
x2
nH
1,272
i31
1,327
t32
0x16
i52
0
x2
nH
1,21
x2
nH
1,441
t32
0x4
i52
443
t32
0x4
i52
0
t32
0x4
i52
0
t8
2
l04
i52
15
x2
nH
1,24
t8
2}
,0,0x0
nJ
58392
i31
0,0
t8
1}
,nK1,0x0
nJ
24591
i62
33807
i62
48143
i62
289816
i62
295960
i62
307200,lA
315470,lA
39202,lA
121894,lA
421926,lA
429094,lA
437286,lA
289073,lA
325002,lA
334218,lA
7627,lA
7700,lA
7724,lA
38,lA
50587,lA
405504,lA
31751,lA
404487,lA
76816,i72
318479,lA
319503
y61
7340060,i72
332833,lA
329764
y61
7340176,lA
89387,lA
339273,lA
332170,lA
487462,lA
490534,lA
497702,lA
509990,lA
336947,lA
342016,lA
518182,lA
503808,lA
286727,lA
91151,lA
131087,lA
328719,i72
7217,lA
304143,l1
0x14
nJ
296976,l1
nI3
7340056,l1
nI3
7340061,i72
7192,lA
7447,lA
24583,lA
335887,l1
0x10
nJ
7206,lA
tD3
lA
343078,l1
0x6
nJ
354342,lA
365606,lA
58768,lA
426398,l1
0x12}
,{{3,40272286,i72
484766,l1
0x12}
,{{3,40330654,i72
50600,lA
62918,lA
426456,i72
484824,i72
296998,l1
0x4
nJ
428070
y61
40330712,i72
50665,lA
367654,l1
0x6
nJ
38372,lA
121856
y61
47655936,lA
471040,lA
367631,l1
0x7
nJ
343097,l1
0x7
nJ
39275,lA
39278,l1
0x4
nJ
39338,l1
nI3
15779300,lA
343055,l1
0x7}
,{{3,15779238,lA
436262,lA
502822,lA
39275,l1
0x4
nJ
39403,l1
0x4
nJ
35847,lA
15,l1
0x4
nJ
376870,lA
15,lA
122904,lA
121880,lA
30751,lA
57
y61
7340177,lA
15681
y61
67573791,lA
39240,lA
385039,l1
0x1
nJ
15760
y61
64009216,lA
562176,l1
0x0}
,{{0,0,xQ
0,0,iM
2,eB1
2,eC1
3,eB1
3,eC1
38,xQ
1,38,iM
14,xQ
1,57,xQ
1,16,t42
0x0
nJ
464959,t42
0x1
i52
306,xQ
1,327,c73
0x0
nJ
465223,t42
0x16
i52
292,eB1
293,eC1
294,xQ
1,295,iM
400,xQ
1,0,xQ
1,454,xQ
1,459,xQ
1,16,t42
0x1
i52
57,c73
0x1
i52
0,iM
21,xQ
1,15,t42
0x0
nJ
24591,xQ
1,24,iM
511,c73
0x0
nJ
46095,lL
46104,lL
50200,lL
287789,lL
66584,lL
407836,lL
15397,lL
62504,lL
39951,lL
24591,lL
33807,lL
62509,lL
15409,lL
50176,lG,283648,lG,19456,lG,27648,lG,90112,lG,86016,lG,190464,y4
xF2
195584,lG,196608,y4
0x14
nJ
482304,lG,14342,lG,58375,lG,46147
yN
46151,lG,284679,lG,50183,lG,7183,lG,46157
yN
38991
yN
50279,lG,50280,lG,50257,lG,50258,lG,46193,lG,50295,lG,283809,lG,284835,lG,24822,lG,10240,lG,11264,lG,7170,lG,tD3
lG,17408,lG,197632,lG,470016,lG,25607,lG,121871,lG,39368,lG,7192,lG,121887,lG,416811,lG,252979,lG,50262,lG,46154,lG,38919,lG,62,lG,50281,lG,40050
yN
7566,y4
0x10
nJ
415787,lG,416819,lG,39326
yN
39326,lG,39332,y4
0x5
nJ
39333,y4
0x1
nJ
50590
yN
50590,lG,39338
yN
39338,lG,39335,y4
0x5
nJ
15786
yN
146858,lG,39366,lG,39373,lG,39374,lG,39384
yN
50648
yN
50648,lG,24,y4
0x6
nJ
24,lG,62,y4
0x6
nJ
40049,lG,46194
yN
43,lG,43
yN
415795,lG,51,lG,51
yN
50266,lG,50176
yN
50267,lG,39159,lG,39183
yN
7168
yN
31744,lG,98304,lG,31746,lG,109592,lG,39403
yN
39405
yN
39405,lG,39414,lG,39414
yN
16384,lG,15,lG,39024,y4
0x5
nJ
39027,y4
0x5
nJ
62853,lG,39416,lG,15360,lG,15,y4
0x1
nJ
16,lG,7183,y4
0x1
nJ
7172
tA1
yD1,nK1,0x0
nJ
24591
tA1
lU
2,63521
tA1
lU
2,62500
tA1
lU
2,50453
tA1
lU
2,50200
tA1
lU
2,62488
tA1
lU
1,0,tV3
7,tV3
196,tV3
0,cAcos
tW3
cAcosh
tW3
cAsin
tW3
cAsinh
nU
119,cAsinh
tW3
cAtan,t52
308224,cAtan2,t52
tD3
cAtan2
tW3
cAtanh
nU
246,cCeil
tW3
cCeil,tX3
cY2
0,cCos,t1
1,7,cY2
81,cY2
82,cY2
119,cY2
237,cY2
255,cY2
217,lH3
237,lH3
458,lH3
0,cCosh,tX3
lH3
246,cFloor
tW3
cFloor,nJ3
311587,tY3
t52
323875,tY3
t52
323899,tY3
l0
3,32513024,t82
34627584
eF
31493120,t82
89213952
eF
149042176
eF
247699456
eF
301234176
eF
488062976
eF
492261376
eF
62933514
eF
62933514,t82
62933520
eF
62933520,t82
24664064
eF
573080576
eF
565189632
eF
32513024
eF
559963136
eF
7891968
eF
582524928,cIf
nU
119,cInt
nU
246,i92
0,i92
7,i92
31,i92
196,i92
359,i92
15,cLog,lU
1,24,cLog,lU
1,0,cLog10
tW3
cLog2,t52
tD3
cMax,t52
35847,cMax,t52
30751,cMax
tW3
cMax,AnyParams,1
l04
nJ
tD3
cMin,t52
35847,cMin,t52
30751,cMin
tW3
cMin,AnyParams,1
l04
nJ
24591,cMin,lU
1,0,xG2
7,xG2
81,xG2
82,xG2
119,xG2
149,xG2
233,cSin,lB
0x5
i52
246,xG2
255,xG2
254,xG2
0,cSin,t1
1,273,cSin,lB
0x1
i52
217,yS2
233,cSinh,lB
0x5
i52
246,yS2
254,yS2
255,yS2
458,yS2
0,cSinh,tX3
yS2
15,cSqrt,lU
1,0,cZ2
0,cTan,t1
1,117,cTan,t1
1,118,cZ2
233,cZ2
246,cZ2
273,cZ2
254,cZ2
255,cZ2
0,yT2
0,cTanh,t1
1,216,yT2
233,yT2
246,yT2
254,yT2
255,yT2
0,cTrunc,t52
15384,cSub,lU
2,15384,cDiv,lU
2,470476,cDiv,lU
2,121913,yW2
tD3
nK3
xF2
tD3
yW2
31744,nK3
0x20
nJ
31751,nK3
0x24
nJ
31751,yW2
121913,i51,t52
tD3
cLess,lB
xF2
41984,cLess,nJ3
41984,cLess,t52
7,cLess,t52
tD3
cLessOrEq,t52
295158,cLessOrEq,t52
tD3
t92
lB
xF2
41984,t92
nJ3
41984,t92
t52
7,t92
t52
tD3
yC
t52
295158,cGreaterOrEq
tW3
n82
244,n82
7,n82
544,n82
547,n82
548,n82
550,n82
15,n82
31,n82
553,n82
554,cNot,t52
7700
i82
7168
i82
35847
i82
30751
i82
457759
i82
460831,cAnd
i6
0,0,cAnd,nH
2,tD3
t13
7700,t13
35847,t13
457759,t13
460831,t13
30751,cOr
i6
1,0,n92
81,n92
131,n92
244,n92
245,n92
246,cDeg
nU
246,cRad,t52
tD3
cAbsAnd,l6
tD3
cAbsOr
i6
1,0,cZ3
tW3
cAbsNotNot,l0
3,32513024,cJ3
lB
0x0}
,}
;}
lR3
l81{const
Rule
grammar_rules[260]={{ProduceNewTree,17,1,0,{1,0,cAbs
yU2
409,{1,146,cAtan
yU2
403
xD
1326,cAtan2
yU2
405
xD
309249,cAtan2
c0
253174
xD
255224,cAtan2
c0
259324
xD
257274,cAtan2
yU2
152,{1,252,cCeil,yV2
480,{1,68,cE1,476,{1,122,cE1,477,{1,124,cE1,151,{1,125,cE1,419,{1,123,cE1,0,{1,403,cCos,l2
2,1,246,{1,252,cCos,l2
18,1,0,{1,400,cE1,303,{1,406,cCosh,l2
2,1,246,{1,252,cCosh,l2
18,1,0,{1,400,cCosh,yV2
452,{1,121,cFloor
yU2
150,{1,252,cFloor,tZ3
157,{3,7382016,eG
543,{3,8430592,eG
550,{3,8436736,eG
158,{3,42998784,eG
544,{3,42999808,eG
556,{3,43039744,eG
551,{3,49291264,eG
532,{3,49325056,eG
463,{3,1058312,eG
467,{3,1058318,eG
467,{3,9438728,eG
463,{3,9438734,cIf,l2
0,3,32542219,{3,36732428,cIf,l2
0,3,32542225,{3,36732434
i03
567,{3,32513026
i03
509,{3,449213961
i03
509,{3,433500687,cIf,yV2
76,{1,256,c83
69,{1,258,c83
404,{1,72,c83
160,{1,147,cLog,l2
0,1,0
xD
481281,cMax,tX
16,1,439
xD
i23
cMax,tX
0,1,0
xD
477185,cMin,tX
16,1,440
xD
i23
cMin,cE
0,1,154
xD
24832
tA1
tZ3
154
xD
25854
tA1
tZ3
155
xD
129039
tA1
xV1
32062
tA1
xV1
32063
tA1
xV1
32064
tA1
l2
0,2,166288
xD
32137
tA1
xV1
33089
tA1
l2
0,2,7168
xD
12688
tA1
l2
0,2,7434
xD
12553
tA1
yV2
429
xD
46146
tA1
yV2
430
xD
46153
tA1
yV2
431
xD
46150
tA1
yV2
173
xD
81935
tA1
yV2
172
xD
130082
tA1
yV2
178
xD
133154
tA1
i13
470016
xD
464911
tA1
i13
274432
xD
273423
tA1
i13
251904
xD
266274
tA1
i13
251904
xD
263186
tA1
yV2
175,{1,252,n21
421,{1,68,n21
151,{1,122,n21
419,{1,124,n21
174,{1,125,n21
476,{1,123,n21
0,{1,405,n21
176,{1,252,cSinh,yV2
333,{1,404,cSinh,yV2
177,{1,252,cF3
0,{1,408,cF3
179,{1,410,cF3
180,{1,252,cTanh,l2
0,1,436
xD
443407,nM3
435
xD
i23
nM3
171
xD
268549,nM3
184
xD
276749,nM3
183
xD
276500,nN3
59701
xD
303289,nN3
59702
xD
305339,nN3
59728
xD
346306,nN3
157004
xD
193724,nN3
174245
xD
171172,nN3
243878
xD
194724
x2
cE
2,1,340,{1,313
t8
1
iW
330,{1,361
t8
1}
}
,{ReplaceParams,2,1,365
xD
1386
iT
339
xD
1361
iT
457
xD
466374
iT
47
xD
353636
iT
346
xD
203823
iT
357
xD
202799
iT
474
xD
208079
iT
475
xD
209103
iT
417
xD
213193
iT
210
xD
213194
iT
418
xD
216265
iT
212
xD
202086
iT
205
xD
368998
iT
214
xD
368853
iT
223
xD
224474
iT
225
xD
229594
iT
366
xD
502185
iT
220
xD
501982
iT
227
xD
226729
iT
226
xD
226794
iT
363
xD
234921
iT
426
xD
376037
iT
491
xD
376030
iT
491
xD
233897
iT
426
xD
373226
iT
227
xD
372958,l7
2,2,406760
xD
236753,l7
2,2,59762
xD
236913,nM3
371
xD
1396
cC2
94
xD
24705
cC2
95
xD
24708
cC2
438
xD
443407
cC2
437
xD
444848
cC2
98
xD
99701
cC2
106
xD
101704
cC2
100
xD
110920,l5
0,2,108559
xD
103752,l5
0,2,102415
xD
104776,lK
0
iW
110
xD
111634,cMul,SelectedParams,0
iW
561,{1,52,lK
1
iW
562,{1,42,lK
1}
}
,{ReplaceParams,2,1,480
xD
492582
tL3
488
xD
498726
tL3
381
xD
435578
tL3
491
xD
435703
tL3
426
xD
502142
tL3
414
xD
493947
tL3
493
xD
349666
tL3
342
xD
364014
tL3
380
xD
425315
tL3
472
xD
425454
tL3
47
xD
506351
tL3
499
xD
352739
tL3
47
xD
510448
tL3
501
xD
512038
tL3
502
xD
355818
tL3
348
xD
387575
tL3
505
xD
357861
tL3
497
xD
351710
tL3
508
xD
519206
tL3
504
xD
395375
tL3
388
xD
394356
tL3
461
xD
45510
tL3
353
xD
51552
tL3
462
xD
49606
tL3
354
xD
47456,l5
2,2,359926
xD
358890,l5
16,1,92
xD
1157,l5
16,1,93
xD
1158,l5
16,1,402
xD
411024,l5
16,2,58768
xD
1466,l5
16,2,15760
xD
1468,l5
17,1,0,{1,400,l5
17,1,57,{1,14,lK
0}
}
,{ProduceNewTree,4,1,532
xD
41,nL3
l3
4,1,0
xD
5167,nL3
cS
41984
xD
409641,nL3
cS
tY
nL3
cS
t2
nL3
cS
t3
cEqual
cQ1
24849,cEqual
t23
cEqual
t33
281873,cEqual
c0
iF
cEqual
c0
lH1
nL3
l3
4,1,556
xD
41,i51,l3
4,1,532
xD
5167,i51,cS
41984
xD
409641,i51,cS
tY
i51,cS
t2
i51,cS
t3
i51
cQ1
24849,i51
t23
i51
t33
281873,i51
c0
iF
i51
c0
lH1
i51,cS
tY
cG3
t2
cG3
t3
cLess
yU2
565
xD
46080,cLess
cQ1
24832,cLess
c0
xW1
cLess
t23
cLess
t33
cH3
cLess
c0
x01
cLess
c0
iF
cLess
c0
lH1
cLess,l3
20,1,556
xD
409641,cG3
tY
yX2
t2
yX2
t3
cLessOrEq
yU2
559
xD
409615,cLessOrEq
cQ1
24832,cLessOrEq
c0
xW1
cLessOrEq
t23
cLessOrEq
t33
cH3
cLessOrEq
c0
x01
cLessOrEq
c0
iF
cLessOrEq
c0
lH1
cLessOrEq,l3
20,1,556
xD
409647,yX2
tY
e02
t2
e02
t3
cGreater
yU2
533
xD
409615,cGreater
cQ1
24832,cGreater
c0
xW1
cGreater
t23
cGreater
t33
cH3
cGreater
c0
x01
cGreater
c0
iF
cGreater
c0
lH1
t92
l3
20,1,532
xD
409647,e02
tY
yC
cS
t2
yC
cS
t3
yC
l3
18,1,566
xD
46080,yC
x22
523510
xD
24832,yC
x22
xW1
yC
x22
iE,yC
x22
nA2
cH3
yC
x22
x01
yC
x22
iF
yC
x22
lH1
yC
l3
20,1,532
xD
409641,yC
l3
4,1,513,{1,137,cNot,cI3
565,{1,2,cNot,l2
0,1,446
xD
i23
xC
0,2,530947,{3,541595138,cAnd,cE
16,1,560,{1,5,cAnd,AnyParams,1}
}
,{ReplaceParams,16,1,563
xD
13314,xC
16,1,538
xD
547348,xC
16,1,541
xD
456220,xC
16,1,542
xD
460316,xC
0,1,451
xD
i23
nG
564
xD
13314,nG
557
xD
8197,nG
535
xD
547348,nG
536
xD
456220,nG
537
xD
460316,nG
558
xD
143365,cOr,cE
4,1,519,{1,137,cNotNot,cI3
566,{1,2,cNotNot,l3
17,1,0,{1,0,cNotNot
yU2
531,{1,256,cAbsNotNot,cE
18,1,525,{1,254,cAbsNotNot,cE
0,1,566,{3,43039744,cJ3
tZ3
565,{3,49325056,cJ3
cI3
448,{3,32513580,cJ3
l2
16,3,32542219,{3,36732428,cJ3
yD1}
,}
;eI2
grammar_optimize_abslogical_type{yZ
9
eI
grammar_optimize_abslogical_type
grammar_optimize_abslogical={9,{34,190,226,236,240,245,252,258,259}
}
;}
eI2
grammar_optimize_ignore_if_sideeffects_type{yZ
59
eI
grammar_optimize_ignore_if_sideeffects_type
grammar_optimize_ignore_if_sideeffects={59,{0,20,21,22,23,24,25,26,cY
iV1
78,yE1
yY
grammar_optimize_nonshortcut_logical_evaluation_type{yZ
56
eI
grammar_optimize_nonshortcut_logical_evaluation_type
grammar_optimize_nonshortcut_logical_evaluation={56,{0,25,cY
iV1
78,yE1
161,162,163,164,165,166,167,176,177,178,198,202,210,214,222,234,235,237,238,239,241,242,243,244,246,247,248,249,250,251,253,254,255,256,257}
}
;}
eI2
grammar_optimize_round1_type{yZ
125
eI
grammar_optimize_round1_type
grammar_optimize_round1={125,{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,19,25,cY
37,38,iV1
45,46,47,48,49,50,51,52,53,54,58,59,60,61,62,63,64,65,66,67,68,69,70,71,78,79,80,81,82,83,84,85,86,87,88,93,94,95,96,97,98,99,100,101,117,118,119,120,121,122,123,124,125,126,127,128,129,132,158,159,160,yY
grammar_optimize_round2_type{yZ
103
eI
grammar_optimize_round2_type
grammar_optimize_round2={103,{0,15,16,17,25,cY
39,40,iV1
45,46,47,48,49,50,51,52,53,54,59,60,72,73,78,79,83,84,85,89,90,91,92,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,119,122,123,124,125,126,127,128,133,157,158,159,160,yY
grammar_optimize_round3_type{yZ
79
eI
grammar_optimize_round3_type
grammar_optimize_round3={79,{74,75,76,77,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,168,169,170,171,172,173,174,175,179,180,181,182,183,184,185,186,187,188,189,191,192,193,194,195,196,197,199,200,201,203,204,205,206,207,208,209,211,212,213,215,216,217,218,219,220,221,223,224,225,227,228,229,230,231,232,233}
}
;}
eI2
grammar_optimize_round4_type{yZ
10
eI
grammar_optimize_round4_type
grammar_optimize_round4={10,{18,55,56,57,130,131,153,154,155,156}
}
;}
eI2
grammar_optimize_shortcut_logical_evaluation_type{yZ
53
eI
grammar_optimize_shortcut_logical_evaluation_type
grammar_optimize_shortcut_logical_evaluation={53,{0,25,cY
iV1
78,yE1
161,162,163,164,165,166,167,176,177,178,198,202,210,214,222,234,235,237,238,241,242,243,244,247,248,249,251,253,254,255,256,257}
}
;}
}
lR3
l81{nU2
eN2
eJ1
xA2
paramlist,lJ1){index=(paramlist>>(index*10))&1023;if(index>=57)eD1
SubFunction
cD2
plist_s[index-57]iR3
index>=37)eD1
NumConstant
cD2
plist_n_container
xI::plist_n[index-37]);eD1
eU3
cD2
plist_p[index]);}
}
#ifdef FP_SUPPORT_OPTIMIZER
#include <stdio.h>
#include <algorithm>
#include <map>
#include <sstream>
using
lR3
FUNCTIONPARSERTYPES;using
lR3
l81;using
tF;using
xZ1;lR3{nT1
It,typename
T,typename
Comp>eX1
MyEqualRange(It
first,It
last,const
T&val,Comp
comp){size_t
len=last-first;while(len>0){size_t
yB3
len/2;It
xE3(first);xE3+=half;if(comp(*xE3,val)){first=xE3;++first;len=len-half-1;}
iW1
comp(val,*xE3)){len=half;}
else{It
left(first);{It&tA2=left;It
last2(xE3);size_t
len2=last2-tA2;while(len2>0){size_t
half2=len2/2;It
c63(tA2);c63+=half2;if(comp(*c63,val)){tA2=c63;++tA2;len2=len2-half2-1;}
else
len2=half2;}
}
first+=len;It
right(++xE3);{It&tA2=right;It&last2=first;size_t
len2=last2-tA2;while(len2>0){size_t
half2=len2/2;It
c63(tA2);c63+=half2;if(comp(val,*c63))len2=half2;else{tA2=c63;++tA2;len2=len2-half2-1;}
}
}
return
eX1(left,right);}
}
return
eX1(first,first);}
nU2
eI2
OpcodeRuleCompare{iU2()lZ3
tree,xL3
yY2)const{const
Rule&rule=grammar_rules[yY2]iP3
tX2<rule
xZ2.subfunc_opcode;}
iU2()xA2
yY2,const
eV
const{const
Rule&rule=grammar_rules[yY2]iP3
rule
xZ2.subfunc_opcode<tX2;}
}
yP1
bool
TestRuleAndApplyIfMatch
e11
tN2
eW&tree,bool
cF{MatchInfo
xI
info;nB1
found(false,e1()iR3(rule.lI1
LogicalContextOnly)&&!cF{tU1
if(nC
IsIntType
xI::eH3)cK3
NotForIntegers)tU1
else
cK3
OnlyForIntegers)tU1
if(nC
IsComplexType
xI::eH3)cK3
NotForComplex)tU1
else
cK3
OnlyForComplex)tU1
for(;;){
#ifdef DEBUG_SUBSTITUTIONS
#endif
found=TestParams(rule
xZ2,tree,found.specs,info,true
iR3
found.found)break;if(!&*found.specs){fail:;
#ifdef DEBUG_SUBSTITUTIONS
DumpMatch
xY2,false);
#endif
lY2}
#ifdef DEBUG_SUBSTITUTIONS
DumpMatch
xY2,true);
#endif
SynthesizeRule
xY2);iB1}
xZ1{yQ1
ApplyGrammar
e11
Grammar&iA2,eW&tree,bool
cF{if(tree.GetOptimizedUsing()==&iA2){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Already optimized:  "
yA3
tree)xH1"\n"
<<std::flush;
#endif
lY2
if(true){bool
changed
tK3
switch(tX2
eS3
cNot:case
cNotNot:case
cAnd:case
cOr:for
iD1
a=0;a<tree.xA
true))xQ2
lD
cIf:case
cAbsIf:if(ApplyGrammar(iA2,xZ
0),tX2==cIf))xQ2
for
iD1
a=1;a<tree.xA
cF)xQ2
break;yX3
for
iD1
a=0;a<tree.xA
false))xQ2}
if(changed){tree.Mark_Incompletely_Hashed();iB1}
typedef
const
xL3
short*nO3;std::pair<nO3,nO3>range=MyEqualRange(iA2.rule_list,iA2.rule_list+iA2.rule_count,tree,OpcodeRuleCompare
xI());std
y33<xL3
short>rules;rules.y03
range
n23-range.first);for
xX
if(IsLogisticallyPlausibleParamsMatch(eE1
xZ2,tree))rules.push_back(*r);}
range.first=!rules
cT3?&rules[0]:0;range
n23=!rules
cT3?&rules[rules
tG3-1]+1:0;if(range.first!=range
n23){
#ifdef DEBUG_SUBSTITUTIONS
if(range.first!=range
n23)cE2"Input ("
<<cV3
tX2)<<")["
<<iU<<"]"
;if(cF
std::cout<<"(Logical)"
;xL3
first=l02,prev=l02;xY3
sep=", rules "
;for
xX
if(first==l02)first=prev=*r;iW1*r==prev+1)prev=*r;else
cE2
sep<<first;sep=","
;if(prev!=first)std::cout<<'-'<<prev;first=prev=*r;}
}
if(first!=l02)cE2
sep<<first;if(prev!=first)std::cout<<'-'<<prev;}
std::cout<<": "
yA3
tree)xH1"\n"
<<std::flush;}
#endif
bool
changed
tK3
for
xX
#ifndef DEBUG_SUBSTITUTIONS
if(!IsLogisticallyPlausibleParamsMatch(eE1
xZ2,tree))continue;
#endif
if(TestRuleAndApplyIfMatch(eE1,tree,cF){xQ2
yY3}
if(changed){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Changed."
<<std::endl
xH1"Output: "
yA3
tree)xH1"\n"
<<std::flush;
#endif
tree.Mark_Incompletely_Hashed();iB1}
tree.SetOptimizedUsing(&iA2);lY2
yQ1
ApplyGrammar
e11
void*p,FPoptimizer_CodeTree::eV
yM
ApplyGrammar(*e11
Grammar*)p,tree);}
tB1
ApplyGrammars(FPoptimizer_CodeTree::eV{
#ifdef DEBUG_SUBSTITUTIONS
std
lD4"grammar_optimize_round1\n"
;
#endif
n7
grammar_optimize_round1
n13
#ifdef DEBUG_SUBSTITUTIONS
std
lD4"grammar_optimize_round2\n"
;
#endif
n7
grammar_optimize_round2
n13
#ifdef DEBUG_SUBSTITUTIONS
std
lD4"grammar_optimize_round3\n"
;
#endif
n7
grammar_optimize_round3
n13
#ifndef FP_ENABLE_SHORTCUT_LOGICAL_EVALUATION
#ifdef DEBUG_SUBSTITUTIONS
std
lD4"grammar_optimize_nonshortcut_logical_evaluation\n"
;
#endif
n7
grammar_optimize_nonshortcut_logical_evaluation
n13
#endif
#ifdef DEBUG_SUBSTITUTIONS
std
lD4"grammar_optimize_round4\n"
;
#endif
n7
grammar_optimize_round4
n13
#ifdef FP_ENABLE_SHORTCUT_LOGICAL_EVALUATION
#ifdef DEBUG_SUBSTITUTIONS
std
lD4"grammar_optimize_shortcut_logical_evaluation\n"
;
#endif
n7
grammar_optimize_shortcut_logical_evaluation
n13
#endif
#ifdef FP_ENABLE_IGNORE_IF_SIDEEFFECTS
#ifdef DEBUG_SUBSTITUTIONS
std
lD4"grammar_optimize_ignore_if_sideeffects\n"
;
#endif
n7
grammar_optimize_ignore_if_sideeffects
n13
#endif
#ifdef DEBUG_SUBSTITUTIONS
std
lD4"grammar_optimize_abslogical\n"
;
#endif
n7
grammar_optimize_abslogical
n13
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
lR3
FUNCTIONPARSERTYPES;using
lR3
l81;using
tF;using
xZ1;lR3{yQ1
TestImmedConstraints
xA2
bitmask,const
eV{switch(bitmask&ValueMask
eS3
Value_AnyNum:case
ValueMask:lD
xE2:if(GetEvennessInfo
cX3
nB2
Value_OddInt:if(GetEvennessInfo
cX3
xH2
i11:if(GetIntegerInfo
cX3
nB2
Value_NonInteger:if(GetIntegerInfo
cX3
xH2
eV1:if(!IsLogicalValue(tree)cJ
nL1
SignMask
eS3
Sign_AnySign:lD
nK1:if(l61
nB2
eW1:if(l61
xH2
Sign_NoIdea:if(l61
iD3
cJ
nL1
OnenessMask
eS3
Oneness_Any:case
OnenessMask:lD
Oneness_One:if(!xX1
if(!y83
fp_abs(lK4),l63(1))cJ
lD
Oneness_NotOne:if(!xX1
if
y73
fp_abs(lK4),l63(1))cJ
nL1
ConstnessMask
eS3
Constness_Any:lD
i01:if(!xX1
lD
Constness_NotConst:if(xX1
yY3
iB1
tT1
xL3
extent,xL3
nbits,typename
tB2=xL3
int>eI2
nbitmap{private:static
const
xL3
bits_in_char=8;static
const
xL3
tC2=(cL3
tB2)*bits_in_char)/nbits;tB2
data[(extent+tC2-1)/tC2];t83
void
inc(lJ1,int
by=1){data[pos(index)]+=by*tB2(1<<yZ2);lZ2
void
dec(lJ1){inc(index,-1);}
int
get(lJ1
c61(data[pos(index)]>>yZ2)&mask()tH3
pos(lJ1)yM
index/tC2
tH3
shift(lJ1)yM
nbits*(index%tC2)tH3
mask()yM(1<<nbits)-1
tH3
mask(lJ1)yM
mask()<<yZ2;}
}
;eI2
lL4{int
SubTrees:8;int
Others:8;int
nK2:8;int
t43:8;nbitmap<lG3,2>SubTreesDetail;lL4(){std::memset(this,0,cL3*this));}
lL4
e11
lL4&b){std::memcpy(this,&b,cL3
b));}
lL4&eM1=e11
lL4&b){std::memcpy(this,&b,cL3
b))iP3*this;}
}
yP1
lL4
CreateNeedList_uncached(t4&iB2{lL4
yR1;for(eY1=0;a<params
c02;++a){const
eN2&nP3=eJ1
xI(params.param_list,a);switch(nP3.first
eS3
SubFunction:{cX&param
yV3
yE
GroupFunction)++yR1.t43;else{++xA3;assert(param.data.subfunc_opcode<VarBegin);yR1.SubTreesDetail.inc(c43
lG1);}
++yR1.nK2;break
xH3
eT3
case
eU3:++x93;++yR1.nK2;yY3}
return
yR1;}
nU2
lL4&CreateNeedList(t4&iB2{typedef
std::map<t4*,lL4>eF1;static
eF1
c11;eF1::yL3
i=c11.yK2&iB2;if(i!=c11.e61&iB2
return
i
eQ2
iP3
c11.yH3,std::make_pair(&params,CreateNeedList_uncached
xI(iB2))eQ2;}
nU2
eW
CalculateGroupFunction
e11
eN2&c12
const
iX3
c13
nP3.first
eS3
eT3{const
ParamSpec_NumConstant
xI
c33
e11
ParamSpec_NumConstant
xI*xY1
return
CodeTreeImmed(c43
constvalue)xH3
eU3:{cW
c33(cW*xY1
return
t03
GetParamHolderValueIfFound(c43
index)xH3
SubFunction:{cX&param
yV3
eW
eH3
x83
cI
c43
lG1)x83.lH2).reserve
e12
c02);for(eY1=0;a<c43
data
c02;++a){eW
tmp(CalculateGroupFunction(eJ1
xI
e12.param_list,a),info))x83
l12
tmp);}
eH3
nJ2
return
eH3;}
}
return
eW();}
}
xZ1{yQ1
IsLogisticallyPlausibleParamsMatch(t4&params,const
eV{lL4
yR1(CreateNeedList
xI(iB2);size_t
i33=iU;if(i33<size_t(yR1.nK2))iC2
for
iD1
a=0;a<i33;++a){xL3
opcode=iZ3
nD;switch(opcode
eS3
cImmed:if(yR1.t43>0)--yR1.t43;else--x93;lD
lG3:case
cFCall:case
cPCall:--x93;break;yX3
assert(opcode<VarBegin);if(xA3>0&&yR1.SubTreesDetail.get(opcode)>0){--xA3;yR1.SubTreesDetail.dec(opcode);}
else--x93;}
}
if(yR1.t43>0||xA3>0||x93>0)iC2
if(params.match_type!=AnyParams){if(0||xA3<0||x93<0)iC2}
iB1
nU2
nB1
TestParam
e11
eN2&c12
xW2
tree,const
e1&xK3,iX3
c13
nP3.first
eS3
eT3{const
ParamSpec_NumConstant
xI
c33
e11
ParamSpec_NumConstant
xI*xY1
if(!xX1
l63
imm=lK4;switch(c43
modulo
eS3
Modulo_None:lD
Modulo_Radians:imm=e63
imm,yR
imm<xD1
imm
yT
if(imm>fp_const_pi
xI())imm-=fp_const_twopi
xI(c62
return
y83
imm,c43
constvalue)xH3
eU3:{cW
c33(cW*xY1
if(!x4
return
t03
SaveOrTestParamHolder(c43
index,tree)xH3
SubFunction:{cX&param
yV3
yE
GroupFunction){if(!x4
eW
y01=CalculateGroupFunction(c12
info);
#ifdef DEBUG_SUBSTITUTIONS
DumpHashes(y01)xH1*e11
void**)&y01.xQ1
xH1"\n"
xH1*e11
void**)&lK4
xH1"\n"
;DumpHashes(tree)xH1"Comparing "
yA3
y01)xH1" and "
yA3
tree)xH1": "
xH1(y01
xL
tree)?"true"
:"false"
)xH1"\n"
;
#endif
return
y01
xL
tree);}
e53!&*xK3){if(!x4
if(tX2!=c43
lG1
cJ}
return
TestParams
e12,tree,xK3,info,false);}
}
}
lY2
nU2
eI2
l21
xI2
MatchInfo
xI
info;l21():xK3(),info(){}
}
yP1
class
MatchPositionSpec_PositionalParams:lX1
l21
xI>{t83
lI3
MatchPositionSpec_PositionalParams
iD1
n):eG1
l21
xI>(n){}
}
;eI2
l52
xI2
l52():xK3(){}
}
;class
yF:lX1
l52>{t83
xL3
trypos;lI3
yF
iD1
n):eG1
l52>(n),trypos(0){}
}
yP1
nB1
TestParam_AnyWhere
e11
eN2&c12
xW2
tree,const
e1&xK3,iX3,y42&used,bool
tD2{xU<yF>x9;iD2
yF
y32
a=x9->trypos;goto
retry_anywhere_2;}
e22
yF(iU);a=0;}
for(;a<tree.iR{if(used[a])continue;retry_anywhere
cM3
TestParam(c12
iZ3
nQ3)iG2
used[a]=true
lE1
a);x9->trypos=a
iP3
nB1(true,&*x9);}
}
retry_anywhere_2
cN3
goto
retry_anywhere;}
}
lY2
nU2
eI2
yF1
xI2
MatchInfo
xI
info;y42
used;lI3
yF1
iD1
i33):xK3(),info(),used(i33){}
}
yP1
class
MatchPositionSpec_AnyParams:lX1
yF1
xI>{t83
lI3
MatchPositionSpec_AnyParams
iD1
n,size_t
m):eG1
yF1
xI>(n,yF1
xI(m)){}
}
yP1
nB1
TestParams(t4&nQ,xW2
tree,const
e1&xK3,iX3,bool
tD2{if(nQ.match_type!=AnyParams){if(y5!=iU
cJ}
if(!IsLogisticallyPlausibleParamsMatch(nQ,tree))iC2
switch(nQ.match_type
eS3
PositionalParams:{xU<cQ>x9;iD2
cQ
y32
a=y5-1;goto
lK1;}
e22
cQ(y5);a=0;}
for(;y02{xK2=info;retry_positionalparams
cM3
TestParam(cZ
a),iZ3
nQ3)iG2
continue;}
}
lK1
cN3
info=xK2;goto
retry_positionalparams;}
if(a>0){--a;goto
lK1;}
info=(*iE2;lY2
if(tD2
for(iF2
t03
SaveMatchedParamIndex(a)iP3
nB1(true,&*x9)xH3
SelectedParams:case
AnyParams:{xU<tG>x9;y42
used(iU);std
y33<xL3>lJ3(y5);std
y33<xL3>c22(y5);for(iF2{const
eN2
nP3=cZ
a);lJ3[a]=ParamSpec_GetDepCode(nP3);}
{xL3
b=0;for(iF2
if(lJ3[a]!=0)c22[b++]=a;for(iF2
if(lJ3[a]==0)c22[b++]=a;}
iD2
tG
y32
if(y5==0){a=0;goto
retry_anyparams_4;}
a=y5-1;goto
eH1;}
e22
tG(y5,iU);a=0;if(y5!=0){(*iE2=info;(*x9)[0].used=used;}
}
for(;y02{if(a>0){xK2=info;(*x9)[a].used=used;}
retry_anyparams
cM3
TestParam_AnyWhere
xI(cZ
c22[a]),tree
nQ3,used,tD2
iG2
continue;}
}
eH1
cN3
info=xK2;used=(*x9)[a].used;goto
retry_anyparams;}
eI1:if(a>0){--a;goto
eH1;}
info=(*iE2;lY2
retry_anyparams_4:if(nQ.n2!=0){if(!TopLevel||!t03
HasRestHolder(nQ.n2)){eN
e32;e32.y03
iU);for
xA2
b=0;b<iU;++b){if(cO3)continue;e32.push_back(xZ
b));cO3=true
lE1
b);}
if(!t03
SaveOrTestRestHolder(nQ.n2,e32)){goto
eI1;}
}
else{iV2&e32=t03
GetRestHolderValues(nQ.n2
cF2
0;a<e32
tG3;++a){bool
found
tK3
for
xA2
b=0;b<iU;++b){if(cO3
eM2
e32[a]xL
xZ
b))){cO3=true
lE1
b);found=true;yY3}
if(!found){goto
eI1;}
}
}
}
return
nB1(true,y5?&*x9:0)xH3
GroupFunction:yY3
lY2}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
#include <algorithm>
#include <assert.h>
using
tF;using
xZ1;lR3{nU2
eW
y11
const
eN2&c12
iX3,bool
inner=true
c13
nP3.first
eS3
eT3{const
ParamSpec_NumConstant
xI
c33
e11
ParamSpec_NumConstant
xI*xY1
return
CodeTreeImmed(c43
constvalue)xH3
eU3:{cW
c33(cW*xY1
return
t03
GetParamHolderValue(c43
index)xH3
SubFunction:{cX&param
yV3
eW
tree;tZ2
c43
lG1);for(eY1=0;a<c43
data
c02;++a){eW
nparam=y11
eJ1
xI
e12.param_list,a),info,true)iS
nparam);}
if
e12.n2!=0){eN
trees(t03
GetRestHolderValues
e12.n2));tree.AddParamsMove(trees
iR3
iU==1){assert(tree.GetOpcode()==cAdd l34()==cMul l34()==cMin l34()==cMax l34()==cAnd l34()==cOr l34()==cAbsAnd l34()==cAbsOr);tree.t02
0));}
iW1
iU==0
c13
tX2
eS3
cAdd:case
cOr:tree=nI1
0));lD
cMul:case
cAnd:tree=nI1
1));yX3
yY3}
}
if(inner)tree
nJ2
return
tree;}
}
return
eW();}
}
xZ1{tB1
SynthesizeRule
e11
tN2
eW&tree,iX3
c13
rule.ruletype
eS3
ProduceNewTree:{tree.Become(y11
eJ1
lL1
0),info,false)c62
case
ReplaceParams:yX3{std
y33<xL3>list=t03
GetMatchedParamIndexes();std::sort(list.l23
list.end()cF2
list
tG3;a-->0;)tree
nI2
list[a]);for(eY1=0;a<rule.repl_param_count;++a){eW
nparam=y11
eJ1
lL1
a),info,true)iS
nparam);}
yY3}
}
}
#endif
#ifdef DEBUG_SUBSTITUTIONS
#include <sstream>
#include <cstring>
using
lR3
FUNCTIONPARSERTYPES;using
lR3
l81;using
tF;using
xZ1;lR3
l81{tB1
DumpMatch
e11
tN2
xW2
tree,const
iX3,bool
DidMatch,std::ostream&o){DumpMatch
xY2,DidMatch?lJ4"match"
:lJ4"mismatch"
,o);}
tB1
DumpMatch
e11
tN2
xW2
tree,const
iX3,xY3
i43,std::ostream&o){static
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
;o<<i43<<" (rule "
<<(&rule-grammar_rules)<<")"
<<":\n  Pattern    : "
;{eN2
tmp;tmp.first=SubFunction;ParamSpec_SubFunction
tmp2;tmp2.data=rule
xZ2;tmp
n23=e11
void*)&tmp2;DumpParam
xI(tmp,o);}
o<<"\n  Replacement: "
;DumpParams
lL1
rule.repl_param_count,o)y93
o<<"  Tree       : "
yA3
tree,o)y93
if(!std::strcmp(i43,lJ4"match"
))DumpHashes(tree,o
cF2
0;a<t03
c9
tG3;++a){if(!t03
c9[a].cO2))continue;o<<"           "
<<ParamHolderNames[a]<<" = "
yA3
t03
c9[a],o)y93}
iC1
t03
lR
tG3;++b){if(!t72
first)continue;for
iD1
a=0;a<t72
second
tG3;++a){o<<"         <"
<<b<<"> = "
yA3
t72
second[a],o);o<<std::endl;}
}
o<<std::flush;}
}
#endif
#include <list>
#include <algorithm>
#ifdef FP_SUPPORT_OPTIMIZER
using
lR3
FUNCTIONPARSERTYPES;lR3{yQ1
MarkIncompletes(FPoptimizer_CodeTree::eV{if(tree.Is_Incompletely_Hashed(cX1
bool
l62
tK3
lC1
l62|=MarkIncompletes(iZ3
iR3
l62)tree.Mark_Incompletely_Hashed()iP3
l62;}
tB1
FixIncompletes(FPoptimizer_CodeTree::eV{if(tree.Is_Incompletely_Hashed()){lC1
FixIncompletes(iZ3);tree
nJ2}
}
}
tF{lC
Sort()t73
Sort();}
lC
Rehash(bool
constantfolding){if(constantfolding)ConstantFolding(*this);else
Sort();data
xB
nU2
eI2
cM{cP3
l63
cQ3
yV1=0;
#if 0
long
double
value=Value;e9=crc32::calc(e11
xL3
char*)&eL3
cL3
eO3);key^=(key<<24);
#elif 0
union{eI2{xL3
char
filler1[16]l53
v;xL3
char
filler2[16];}
buf2;eI2{xL3
char
filler3[sizeof
tR2)+16-cL3
x91)];e9;}
buf1;}
data;memset(&data,0,cL3
data));data.buf2.v=Value;e9=data.buf1.key;
#else
int
xN3
l53
x32=std::frexp(Value,&y72;e9=xA2(xN3+0x8000)&0xFFFF
iR3
x32<0){x32=-x32;key=key^0xFFFF;}
else
key+=0x10000;x32-=l63(0.5);key<<=39;key|=nC1(x32+x32)*l63(1u<<31))<<8;
#endif
lQ
#ifdef FP_SUPPORT_COMPLEX_NUMBERS
nT1
T
e42
std::complex<T> >{cP3
std::complex<T>cQ3
cM<T>::nR3
eZ1,Value.real());nC
yD2
temp;cM<T>::nR3
temp,Value.imag());yV1^=temp.hash2;eZ1.hash2^=temp.hash1;}
}
;
#endif
#ifdef FP_SUPPORT_LONG_INT_TYPE
tT1
e42
long>{yG
long
Value){e9=Value;lQ
#endif
#ifdef FP_SUPPORT_GMP_INT_TYPE
tT1
e42
GmpInt>{cP3
GmpInt
cQ3
e9=Value.toInt();lQ
#endif
tB1
y92
xI::Recalculate_Hash_NoRecursion(){yD2
eZ1(nC1
Opcode)<<56,Opcode*l74(0x1131462E270012B));Depth=1;switch(Opcode
eS3
cImmed:{cM
xI::nR3
eZ1,Value
c62
case
lG3:{yV1|=nC1
cR1<<48
iF1((nC1
cR1)*11)^l74(0x3A83A83A83A83A0);break
xH3
cFCall:case
cPCall:{yV1|=nC1
cR1<<48
iF1((~nC1
cR1)*7)^3456789;}
yX3{size_t
t01=0;for
iD1
a=0;a<xI3;++a){if(lF3
x12>t01)t01=lF3
x12;yV1+=((lF3.iH2
hash1*(a+1))>>12)iF1
lF3.iH2
hash1
iF1(3)*l74(0x9ABCD801357);eZ1.hash2*=l74(0xECADB912345)iF1(~lF3.iH2
hash2)^4567890;}
Depth+=t01;}
}
if(Hash!=eZ1){Hash=eZ1;lK2=0;}
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
lR3
FUNCTIONPARSERTYPES;lR3{using
tF
yP1
bool
x11
xW2
tree,long
count,const
xI1::SequenceOpCode
xI&eU,xI1::e72&synth,size_t
max_bytecode_grow_length);static
const
eI2
SinCosTanDataType{OPCODE
whichopcode;OPCODE
inverse_opcode;enum{nominator,x42,inverse_nominator,lM1}
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
,{e52
l14
cCosh,e62,l14
cNop,{e52
cNop,cCosh}
}
,{cCosh,cNop,l14
e52
cNop}
}
,{cNop,cTanh,{cCosh,cSinh,e62,{cNop,cSinh,{cNop,cTanh,cCosh,cNop}
}
,{cNop,cCosh,{cTanh,cSinh,e62}
;}
tF{lC
SynthesizeByteCode(std
y33<xL3>&ByteCode,std
y33
xI&Immed,size_t&stacktop_max){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Making bytecode for:\n"
;iQ
#endif
while(RecreateInversionsAndNegations()){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"One change issued, produced:\n"
;iQ
#endif
FixIncompleteHashes();}
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Actually synthesizing, after recreating inv/neg:\n"
;iQ
#endif
xI1::e72
synth;SynthesizeByteCode(synth,false);synth.Pull(ByteCode,Immed,stacktop_max);}
lC
SynthesizeByteCode(xI1::e72&synth,bool
MustPopTemps)const{y21*this))yM;}
for
iD1
a=0;a<12;++a){const
SinCosTanDataType&data=SinCosTanData[a];if(data.whichopcode!=cNop)iI1!=data.whichopcode)continue;CodeTree
nS3;nS3.lS1
nS3
cI
data.inverse_opcode);nS3.xJ2;y21
nS3)){synth.l71
else
iI1!=cInv
eM2
GetParam(0)nD!=data.inverse_opcode)continue;y21
GetParam(0))){synth.l71
size_t
found[4];iC1
4;++b){CodeTree
tree;if(data.i53]==cNop){tZ2
cInv);CodeTree
nT3;nT3.lS1
nT3
cI
data.i53^2]);nT3.xJ2
iS
nT3);}
else{tree.lS1
tZ2
data.i53]);}
tree.xJ2;found[b]=synth.y23
tree);}
if(found[data.c32!=tS
x42]!=l31
c32)i12
x42
iY
cDiv
nM1
c32!=tS
lM1]!=l31
c32)i12
lM1
iY
cMul
nM1
n41!=tS
lM1]!=l31
n41)i12
lM1
iY
cRDiv
nM1
n41!=tS
x42]!=l31
n41)i12
x42
iY
cMul,2,1);synth.l71
size_t
n_subexpressions_synthesized=SynthCommonSubExpressions(synth);switch(l72{case
lG3:synth.PushVar(GetVar());lD
cImmed:t11
xQ1);lD
cAdd:case
cMul:case
cMin:case
cMax:case
cAnd:case
cOr:case
cAbsAnd:case
cAbsOr:iI1==cMul){bool
cR3
tK3
yI
n51
tK1&&isLongInteger(n51.xQ1)){long
value=makeLongInteger(n51.xQ1);CodeTree
tmp(*this,typename
CodeTree::CloneTag());tmp
nI2
a);tmp
nJ2
if(x11
tmp,eL3
xI1::tW1
xI::AddSequence,synth,MAX_MULI_BYTECODE_LENGTH)){cR3=true;yY3}
}
if(cR3)yY3
int
yG1=0;y42
done(GetParamCount(),false);CodeTree
iG;iG
cI
l72;for(;;){bool
found
tK3
yI
done[a]eM2
synth.IsStackTop(n51)){found=true;done[a]=true;n51.n8
iG
xM2
n51
iR3++yG1>1){yJ
2);iG.xJ2
eL1
iG);yG1=yG1-2+1;}
}
}
if(!found)yY3
yI
done[a])continue;n51.n8
iG
xM2
n51
iR3++yG1>1){yJ
2);iG.xJ2
eL1
iG);yG1=yG1-2+1;}
}
if(yG1==0
c13
l72{case
cAdd:case
cOr:case
cAbsOr:t11
0);lD
cMul:case
cAnd:case
cAbsAnd:t11
1);lD
cMin:case
cMax:t11
0);break;yX3
yY3++yG1;}
assert(n_stacked==1);break
xH3
cPow:{lS3
p0
xR2
0);lS3
p1
xR2
1
iR3!p1
tK1||!isLongInteger(p1.xQ1)||!x11
p0,makeLongInteger(p1.xQ1),xI1::tW1
xI::MulSequence,synth,MAX_POWI_BYTECODE_LENGTH)){p0.n8
p1.n8
yJ
2);tE1
cIf:case
cAbsIf:{typename
xI1::e72::IfData
ifdata;GetParam(0)iU3
SynthIfStep1(ifdata,l72;GetParam(1)iU3
SynthIfStep2(ifdata);GetParam(2)iU3
SynthIfStep3(ifdata
c62
case
cFCall:case
cPCall:{for
iD1
a=0;a<iR
n51.n8
yJ
xA2)GetParamCount());lW3
nY2|GetFuncNo(),0,0
c62
yX3{for
iD1
a=0;a<iR
n51.n8
yJ
xA2)GetParamCount()c62}
synth.StackTopIs(*this
iR3
MustPopTemps&&n_subexpressions_synthesized>0){size_t
top=synth.GetStackTop();synth.DoPopNMov(top-1-n_subexpressions_synthesized,top-1);}
}
}
lR3{yQ1
x11
xW2
tree,long
count,const
xI1::SequenceOpCode
xI&eU,xI1::e72&synth,size_t
max_bytecode_grow_length){if
eP3!=0){xI1::e72
backup=synth;tree.n8
size_t
bytecodesize_backup=synth.GetByteCodeSize();xI1::x11
count
y12
size_t
bytecode_grow_amount=synth.GetByteCodeSize()-bytecodesize_backup;if(bytecode_grow_amount>max_bytecode_grow_length){synth=backup;lY2
iB1
else{xI1::x11
count
y12
iB1}
}
#endif
#include <cmath>
#include <cassert>
#ifdef FP_SUPPORT_OPTIMIZER
using
lR3
FUNCTIONPARSERTYPES;lR3{using
tF;
#define FactorStack std y33
const
eI2
PowiMuliType{xL3
opcode_square;xL3
opcode_cumulate;xL3
opcode_invert;xL3
opcode_half;xL3
opcode_invhalf;}
iseq_powi={cSqr,cMul,cInv,cSqrt,cRSqrt}
,iseq_muli={l02
x2
cNeg,l02,l02}
yP1
l63
cS1
const
PowiMuliType&cS3,const
std
c72,nC2&stack
eV2
1);while(IP<limit){if(l82==cS3.opcode_square){if(!tU2
eW2
2;cA
opcode_invert){eH3=-eH3;cA
opcode_half){if(eH3>y51&&isEvenInteger(eW2
l63(0.5);cA
opcode_invhalf){if(eH3>y51&&isEvenInteger(eW2
l63(-0.5);++IP;continue;}
size_t
xN2=IP
l53
lhs(1
iR3
l82==cFetch){lJ1=nG2;if(index<y1||size_t(index-y1)>=lA2){IP=xN2;yY3
lhs=stack[index-y1];goto
c42;}
if(l82==cDup){lhs=eH3;goto
c42;c42:t21
eH3);++IP
l53
subexponent=cS1
cS3
iX
if(IP>=limit||l82!=cS3.opcode_cumulate){IP=xN2;yY3++IP;stack.pop_back()x83+=lhs*subexponent;continue;}
yY3
return
eH3;}
nU2
l63
ParsePowiSequence
e11
std
c72){nC2
stack;t21
l63(1))iP3
cS1
iseq_powi
iX}
nU2
l63
ParseMuliSequence
e11
std
c72){nC2
stack;t21
l63(1))iP3
cS1
iseq_muli
iX}
nU2
class
CodeTreeParserData{t83
lI3
CodeTreeParserData(bool
k_powi):stack(),clones(),keep_powi(k_powi){}
void
Eat
iD1
i33,OPCODE
opcode){eW
xR;xR
cI
opcode);eN
params=Pop(i33
yW1
iB2;if(!keep_powi)switch(opcode
eS3
cTanh:{eW
sinh,cosh;sinh
cI
cSinh);sinh
xM2
xR
l9
0));sinh
nJ2
cosh
cI
cCosh);cosh
l12
xR
l9
0));cosh
nJ2
eW
pow;pow
cI
cPow);pow
l12
cosh);pow
y2
l63(-1)));pow
nJ2
xR
tI3
xR.SetParamMove(0,sinh);xR
l12
pow
c62
case
cTan:{eW
sin,cos;sin
cI
cSin);sin
xM2
xR
l9
0));sin
nJ2
cos
cI
cCos);cos
l12
xR
l9
0));cos
nJ2
eW
pow;pow
cI
cPow);pow
l12
cos);pow
y2
l63(-1)));pow
nJ2
xR
tI3
xR.SetParamMove(0,sin);xR
l12
pow
c62
case
cPow:{xW2
p0=xR
l9
0);xW2
p1=xR
l9
1
iR3
p1
nD==cAdd){eN
mulgroup(p1
cZ1
cF2
0;a<p1.iR{eW
pow;pow
cI
cPow);pow
xM2
p0);pow
xM2
p1
l9
a));pow
nJ2
mulgroup[a
i63
pow);}
xR
cI
cMul
yW1
mulgroup);}
yY3
yX3
yY3
xR.Rehash(!keep_powi);l92,false);
#ifdef DEBUG_SUBSTITUTIONS
t41<<i33<<", "
<<cV3
opcode)<<"->"
<<cV3
xR
nD)<<": "
lH4
xR
yK
xR);
#endif
t21
eV3
EatFunc
iD1
i33,OPCODE
t93
xL3
funcno){eW
xR=CodeTreeFuncOp
xI(t93
funcno);eN
params=Pop(i33
yW1
iB2;xR.xJ2;
#ifdef DEBUG_SUBSTITUTIONS
t41<<i33<<", "
lH4
xR
yK
xR);
#endif
l92);t21
eV3
AddConst(yM1){eW
xR=CodeTreeImmed(eO3;l92);Push(eV3
AddVar
xA2
varno){eW
xR=CodeTreeVar
xI(varno);l92);Push(eV3
SwapLastTwoInStack(){t31
1
i63
t31
2])eW3
Dup(){Fetch(lA2-1)eW3
Fetch
iD1
which){Push(stack[which]);}
nT1
T>void
Push(T
tree){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<lH4
tree
yK
tree);
#endif
t21
tree)eW3
PopNMov
iD1
target,size_t
source){stack[target]=stack[source];stack
xV3
target+1);}
eW
c52{clones.clear();eW
eH3(stack.back());stack
xV3
lA2-1)iP3
eH3;}
eN
Pop
iD1
n_pop){eN
eH3(n_pop);for
xA2
n=0;n<n_pop;++n)eH3[n
i63
t31
n_pop+n]);
#ifdef DEBUG_SUBSTITUTIONS
for
iD1
n=n_pop;n-->0;){t41
yA3
eH3[n]yK
eH3[n]);}
#endif
stack
xV3
lA2-n_pop)iP3
eH3;}
size_t
GetStackTop(c61
lA2;}
private:void
FindClone(eW&,bool=true)yM;}
private:eN
stack;std::multimap<yD2,eW>clones;bool
keep_powi;private:CodeTreeParserData
e11
CodeTreeParserData&);CodeTreeParserData&eM1=e11
CodeTreeParserData&);}
yP1
eI2
IfInfo{eW
tE2;eW
thenbranch;size_t
endif_location;IfInfo():tE2(),thenbranch(),endif_location(){}
}
;}
tF{lC
GenerateFrom
e11
typename
FunctionParserBase
xI::Data&xF3,bool
keep_powi){eN
y62;y62.y03
xF3.mVariablesAmount);for
xA2
n=0;n<xF3.mVariablesAmount;++n){y62.push_back(CodeTreeVar
xI(n+lG3));}
GenerateFrom(xF3,y62,keep_powi);}
lC
GenerateFrom
e11
typename
FunctionParserBase
xI::Data&xF3,const
x5&y62,bool
keep_powi){const
std
y33<xL3>&ByteCode=xF3.mByteCode;const
std
y33
xI&Immed=xF3.mImmed;
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"ENTERS GenerateFrom()\n"
;
#endif
CodeTreeParserData
xI
sim(keep_powi);std
y33<IfInfo
xI>eM;for
iD1
IP=0,DP=0;;++IP){iI2:while(!eM
cT3&&(eM.eA==IP||(IP<c82&&l82==cJump&&eM.tP1.cO2)))){CodeTree
elsebranch=sim.c52
nE2
eM.back().tE2)nE2
eM.tP1)nE2
elsebranch)c3
3,cIf);eM.pop_back();}
if(IP>=c82)break;xL3
opcode=l82;if((opcode==cSqr||opcode==cDup||(opcode==cInv&&!IsIntType
xI::eH3)||opcode==cNeg||opcode==cSqrt||opcode==cRSqrt||opcode==cFetch)){size_t
was_ip=IP
l53
eY2
ParsePowiSequence
xI(ByteCode,IP,eM
cT3?c82:eM.eA,sim.xK
1
iR3
xN3!=l63(1.0)){xJ
y72
c3
2,cPow);goto
iI2;}
if(opcode==cDup||opcode==cFetch||opcode==cNeg
eU2
yJ2=ParseMuliSequence
xI(ByteCode,IP,eM
cT3?c82:eM.eA,sim.xK
1
iR3
yJ2!=l63(1.0)){xJ
yJ2)c3
2,cMul);goto
iI2;}
}
IP=was_ip;}
if(nF2>=lG3){lJ1=opcode-lG3
nE2
y62[index]);}
else{switch(nF2
eS3
cIf:case
cAbsIf:{eM
xV3
eM
tG3+1);CodeTree
res(sim.c52);eM.back().tE2.swap(res);eM.eA=c82;IP+=2;continue
xH3
cJump:{CodeTree
res(sim.c52);eM.tP1.swap(res);eM.eA=lQ3
IP+1]+1;IP+=2;continue
xH3
cImmed:xJ
Immed[DP++]);lD
cDup:sim.Dup();lD
cNop:lD
cFCall:{xL3
funcno=nG2;assert(funcno<fpdata.mFuncPtrs.size());xL3
params=xF3.mFuncPtrs[funcno].mParams;sim.EatFunc(params,nF2,funcno
c62
case
cPCall:{xL3
funcno=nG2;assert(funcno<fpdata.lB4.size());const
FunctionParserBase
xI&p=*xF3.lB4[funcno].mParserPtr;xL3
params=xF3.lB4[funcno].mParams;x5
paramlist=sim.Pop(iB2;CodeTree
iJ2;iJ2.GenerateFrom(*p.mData,paramlist)nE2
iJ2
c62
case
cInv:xJ
1
eX2
cDiv);lD
cNeg
nH2
cNeg);break;xJ
0
eX2
cSub);lD
cSqr:xJ
2)yH1
cSqrt:xJ
xO2
yH1
cRSqrt:xJ
l63(-0.5))yH1
cCbrt:xJ
l63(1)/l63(3))yH1
cDeg:xJ
fp_const_rad_to_deg
xI())yI1
cRad:xJ
fp_const_deg_to_rad
xI())yI1
cExp:iH)goto
i83;xJ
fp_const_e
xI()eX2
cPow);lD
cExp2:iH)goto
i83;xJ
2.0
eX2
cPow);lD
cCot
nH2
cTan);iH)x1
cCsc
nH2
cSin);iH)x1
cSec
nH2
cCos);iH)x1
cInt:
#ifndef __x86_64
iH)nU3
1,cInt
c62
#endif
xJ
xO2
iK2
c3
1,cFloor);lD
cLog10
nH2
cU3
fp_const_log10inv
xI())yI1
cLog2
nH2
cU3
fp_const_log2inv
xI())yI1
eC3:nV3
cU3
fp_const_log2inv
xI())c3
3,cMul);lD
cHypot:xJ
2)c3
2,cPow);i73
xJ
2)c3
2,cPow)iK2;xJ
xO2
yH1
cSinCos:sim.Dup()c3
1,cSin);nV3
cCos);lD
cSinhCosh:sim.Dup()c3
1,cSinh);nV3
cCosh);lD
cRSub:i73
case
cSub:iH)nU3
2,cSub
c62
xJ-1)c3
2,cMul)iK2;lD
cRDiv:i73
case
cDiv:iH||IsIntType
xI::eH3)nU3
2,cDiv
c62
xJ-1)c3
2,cPow)yI1
cAdd:case
cMul:case
cMod:case
cPow:case
cEqual:case
cLess:case
cGreater:case
i51:case
cLessOrEq:case
cGreaterOrEq:case
cAnd:case
cOr:case
cAbsAnd:case
cAbsOr:sim.Eat(2,y31
lD
cNot:case
cNotNot:case
cZ3:case
cAbsNotNot
nH2
y31
lD
cFetch:sim.Fetch(nG2);lD
cPopNMov:{xL3
stackOffs_target=nG2;xL3
stackOffs_source=nG2;sim.PopNMov(stackOffs_target,stackOffs_source
c62
#ifndef FP_DISABLE_EVAL
case
cEval:{size_t
paramcount=xF3.mVariablesAmount
c3
paramcount,y31
yY3
#endif
yX3
i83:;xL3
funcno=opcode-cAbs;assert(funcno<FUNC_AMOUNT);const
FuncDefinition&func=Functions[funcno]c3
func.params,y31
yY3}
}
Become(sim.c52);
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
lR3{using
lR3
FUNCTIONPARSERTYPES;using
tF
yP1
static
void
AdoptChildrenWithSameOpcode(eV{
#ifdef DEBUG_SUBSTITUTIONS
bool
x52
tK3
#endif
for
c5
if(iZ3
nD==tX2){
#ifdef DEBUG_SUBSTITUTIONS
if(!x52)cE2"Before assimilation: "
c4
x52=true;}
#endif
tree.AddParamsMove(iZ3.GetUniqueRef().lH2),a);}
#ifdef DEBUG_SUBSTITUTIONS
if(x52)cE2"After assimilation:   "
c4}
#endif
}
}
tF{tB1
ConstantFolding(eV{tree.Sort();
#ifdef DEBUG_SUBSTITUTIONS
void*cY3=0
xH1"["
<<(&cY3)<<"]Runs ConstantFolding for: "
c4
DumpHashes(tree)xH1
std::flush;
#endif
if(false){redo:;tree.Sort();
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"["
<<(&cY3)<<"]Re-runs ConstantFolding: "
c4
DumpHashes(tree);
#endif
}
if(tX2!=cImmed){yQ3
p=iO
tree
iR3
p
tH1
p
x62
l73==e23){xM
p
l73);nE}
if(false){ReplaceTreeWithOne:xM
l63(1));goto
do_return;ReplaceTreeWithZero:xM
xD1;goto
do_return;ReplaceTreeWithParam0:
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Before replace: "
xH1
std::hex<<'['tC1
hash1<<','tC1
hash2<<']'<<std::dec
c4
#endif
tree.t02
0));
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"After replace: "
xH1
std::hex<<'['tC1
hash1<<','tC1
hash2<<']'<<std::dec
c4
#endif
tG1
lK3
tX2
eS3
cImmed:lD
lG3:lD
cAnd:case
cAbsAnd:eP
bool
cG
tK3
for
c5{if(!c93
a)))cG=true;cD3
a),tX2==cAbsAnd)eS3
IsNever
tH
IsAlways:iZ);lD
n71
lK3
iU
eS3
0:i7
1:tZ2
tX2==cAnd?cNotNot:cAbsNotNot);tG1
yX3
t62
cAnd||!cG)if(ConstantFolding_AndLogic(tY2
tE1
cOr:case
cAbsOr:eP
bool
cG
tK3
for
c5{if(!c93
a)))cG=true;cD3
a),tX2==cAbsOr))lE2
i7
lN3
iZ);lD
n71
lK3
iU
eS3
0
tH
1:tZ2
tX2==cOr?cNotNot:cAbsNotNot);tG1
yX3
t62
cOr||!cG)if(ConstantFolding_OrLogic(tY2
tE1
cNot:case
cZ3:{xL3
nD1
0;switch(cB3
eS3
cEqual:nD1
i51;lD
i51:nD1
cEqual;lD
cLess:nD1
cGreaterOrEq;lD
cGreater:nD1
cLessOrEq;lD
cLessOrEq:nD1
cGreater;lD
cGreaterOrEq:nD1
cLess;lD
cNotNot:nD1
cNot;lD
cNot:nD1
cNotNot;lD
cZ3:nD1
cAbsNotNot;lD
cAbsNotNot:nD1
cZ3;break;yX3
yY3
if(opposite){tZ2
OPCODE(opposite));tree
iA1
xZ
0).GetUniqueRef().lH2));tG1
lK3
n61
0),tree
cT1
eS3
IsAlways
tH
lN3
i7
n71
t62
cNot&&GetPositivityInfo(xZ
0
yN3
tZ2
cZ3
iR3
cB3==cIf||cB3==tO3{eW
lL3=xZ
0);xW2
ifp1=lL3
l9
1);xW2
ifp2=lL3
l9
2
iR3
ifp1
nD==cNot||ifp1
cT1
e03
ifp1
nD==cNot?cNotNot:cAbsNotNot);t51
l9
0));t61
eW
p2;p2
iQ1
p2.tF1
tI
if(ifp2
nD==cNot||ifp2
cT1
e03
tX2);t51);t61
eW
p2;p2
cI
ifp2
nD==cNot?cNotNot:cAbsNotNot);p2.tF1
l9
0)tI
tE1
cNotNot:case
cAbsNotNot:{if(c93
0)))nW3
cD3
0),tX2==cAbsNotNot)eS3
IsNever
tH
IsAlways:i7
n71
t62
cNotNot&&GetPositivityInfo(xZ
0
yN3
tZ2
cAbsNotNot
iR3
cB3==cIf||cB3==tO3{eW
lL3=xZ
0);xW2
ifp1=lL3
l9
1);xW2
ifp2=lL3
l9
2
iR3
ifp1
nD==cNot||ifp1
cT1{tree
eR2
0,lL3
l9
0));xL2
ifp1);eW
p2;p2
iQ1
p2.tF1
tI
if(ifp2
nD==cNot||ifp2
cT1
e03
tX2);t51);t61
tree.tF1);tZ2
lL3
nD);tG1}
tE1
cIf:case
cAbsIf:{if(ConstantFolding_IfOperations(tY2
break
xH3
cMul:{NowWeAreMulGroup:;AdoptChildrenWithSameOpcode(tree)l53
nN1=l63(1);size_t
lB2=0;bool
nO1
tK3
lC1{if(!xZ
tJ1
continue
l53
immed=iZ3.xQ1;if(immed==xD1
goto
ReplaceTreeWithZero;nN1*=immed;++lB2;}
if(lB2>1||(lB2==1&&y83
nN1,l63(1))))nO1=true;if(nO1){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"cMul: Will add new "
lC4
nN1<<"\n"
;
#endif
for
c5
if(xZ
tJ1{
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<" - For that, deleting "
lC4
iZ3.xQ1
xH1"\n"
;
#endif
e13!y83
nN1,l63(1)))xL2
eA1
nN1));lK3
iU
eS3
0:i7
1:nW3
yX3
if(ConstantFolding_MulGrouping(tY2
if(ConstantFolding_MulLogicItems(tY2
tE1
cAdd:eP
l63
nM2=0.0;size_t
lB2=0;bool
nO1
tK3
lC1{if(!xZ
tJ1
continue
l53
immed=iZ3.xQ1;nM2+=immed;++lB2;}
if(lB2>1||(lB2==1&&nM2==xD1)nO1=true;if(nO1){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"cAdd: Will add new "
lC4
nM2<<"\n"
xH1"In: "
c4
#endif
for
c5
if(xZ
tJ1{
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<" - For that, deleting "
lC4
iZ3.xQ1
xH1"\n"
;
#endif
e13!(nM2==l63(0.0)))xL2
eA1
nM2));lK3
iU
eS3
0
tH
1:nW3
yX3
if(ConstantFolding_AddGrouping(tY2
if(ConstantFolding_AddLogicItems(tY2
tE1
cMin:eP
size_t
tF2
0;yQ3
e4;lC1{while(a+1<iU&&iZ3
xL
xZ
a+1)))iZ+1);yR3
max
xG3&&(!e4
c1
known||(e23)<e4
c1
val)){e4
c1
val=e23;e4
c1
known=true;tF2
a;}
}
if(e4
tO1
for
c5{yR3
min
xG3&&a!=preserve&&p
l73>=e4
c1
val)e13
iU==1){nW3
tE1
cMax:eP
size_t
tF2
0;yQ3
t5;lC1{while(a+1<iU&&iZ3
xL
xZ
a+1)))iZ+1);yR3
min
xG3&&(!t5
iL2||p
l73>t5
l73)){t5
l73=p
l73;t5
iL2=true;tF2
a;}
}
if(t5
iL2){for
c5{yR3
max
xG3&&a!=preserve&&(e23)<t5
l73){iZ);}
}
}
if(iU==1){nW3
tE1
cEqual:case
i51:case
cLess:case
cGreater:case
cLessOrEq:case
cGreaterOrEq:if(ConstantFolding_Comparison(tY2
lD
cAbs:{yQ3
p0
t7
0));if
n31
nW3
if(p0
c7{tZ2
cMul);tree
y2
l63(1)));goto
NowWeAreMulGroup;}
if(cB3==cMul){xW2
p=xZ
0);eN
nX3;eN
neg_set;for
iD1
a=0;a<p.iR{p0=iO
p
l9
a));if
n31{nX3.push_back(p
l9
a));}
if(p0
c7{neg_set.push_back(p
l9
a));}
}
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"Abs: mul group has "
<<nX3
tG3<<" pos, "
<<neg_set
tG3<<"neg\n"
;
#endif
if(!nX3
cT3||!neg_set
cT3){
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"AbsReplace-Before: "
yA3
tree)xH1"\n"
<<std::flush;DumpHashes(tree
e82;
#endif
eW
t53;t53
cI
cMul
cF2
0;a<p.iR{p0=iO
p
l9
a)iR3
n31||(p0
c7){}
else
t53
xM2
p
l9
a));}
t53
nJ2
eW
nY3;nY3
cI
cAbs);nY3
l12
t53);nY3
nJ2
eW
y91
cMul);mulgroup
l12
nY3);l22
AddParamsMove(nX3
iR3!neg_set
cT3){if(neg_set
tG3%2)mulgroup
y2
l63(-1)));l22
AddParamsMove(neg_set);}
tree.Become
l33);
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"AbsReplace-After: "
yA3
tree
e82
xH1"\n"
<<std::flush;DumpHashes(tree
e82;
#endif
goto
NowWeAreMulGroup;}
}
yY3
#define HANDLE_UNARY_CONST_FUNC(funcname) nT){xM funcname(lS));nE
case
cLog:l24(fp_log);if(cB3==cPow){eW
pow=xZ
0
iR3
GetPositivityInfo(pow
l9
0
yN3{pow.CopyOnWrite
yS
i93
if(GetEvennessInfo(pow
l9
1
yN3{pow.lD1
eW
abs;abs
cI
cAbs);abs
l12
pow
l9
0));abs.Rehash
yS
pow.SetParamMove(0,abs);i93}
iW1
cB3==cAbs){eW
pow=xZ
0)l9
0
iR3
pow
nD==cPow){pow.lD1
eW
abs;abs
cI
cAbs);abs
l12
pow
l9
0));abs.Rehash
yS
pow.SetParamMove(0,abs);i93}
lD
cAcosh:l24(fp_acosh);lD
cAsinh:l24(fp_asinh);lD
cAtanh:l24(fp_atanh);lD
cAcos:l24(fp_acos);lD
cAsin:l24(fp_asin);lD
cAtan:l24(fp_atan);lD
cCosh:l24(fp_cosh);lD
cSinh:l24(fp_sinh);lD
cTanh:l24(fp_tanh);lD
cSin:l24(fp_sin);lD
cCos:l24(fp_cos);lD
cTan:l24(fp_tan);lD
cCeil:lN4(fp_ceil);lD
cTrunc:lN4(fp_trunc);lD
cFloor:lN4(fp_floor);lD
cInt:lN4(fp_int);lD
cCbrt:l24(fp_cbrt);lD
cSqrt:l24(fp_sqrt);lD
cExp:l24(fp_exp);lD
cLog2:l24(fp_log2);lD
cLog10:l24(fp_log10);lD
eC3:if
lJ
fp_log2(lS)*xZ
nZ3
cArg:l24(fp_arg);lD
cConj:l24(fp_conj);lD
cImag:l24(fp_imag);lD
cReal:l24(fp_real);lD
cPolar:if
lJ
fp_polar(l42
lD
cMod:if
lJ
e63
l42
lD
cAtan2:{yQ3
p0
t7
0
yB2
p1
t7
1));nT&&y83
lS,xD1){if(p1
e92
p1
e33
xD1{xM
fp_const_pi
e43
if(p1
tH1
p1
l73>=tM1
xD1;nE}
if(tI1
y83
xZ
iV,xD1){if(p0
e92
p0
e33
xD1{xM-fp_const_pihalf
e43
if(p0
tH1
p0
l73>xD1{xM
fp_const_pihalf
e43}
if
lJ
fp_atan2(l42
if((p1
tH1
p1
l73>xD1||(p1
e92
p1
e33
fp_const_negativezero
xI()eO
cH2;cH2
cI
cPow);cH2
l12
xZ
1));cH2
y2
l63(-1)));cH2
nJ2
eW
cI2;cI2
tI3
cI2
l12
xZ
0));cI2
l12
cH2);cI2
nJ2
tZ2
cAtan);tJ
0,cI2
c71
1);tE1
cPow:{if(ConstantFolding_PowOperations(tY2
break
xH3
cDiv:nT&&tI1
xZ
iV!=tM1
lS/xZ
nZ3
cInv:nT&&lS!=tM1
l63(1)/lS);nE
lD
cSub:if
lJ
lS-xZ
nZ3
cNeg:nT){xM-lS);nE
lD
cRad:nT){xM
RadiansToDegrees
eS2
cDeg:nT){xM
DegreesToRadians
eS2
cSqr:nT){xM
lS*lS);nE
lD
cExp2:l24(fp_exp2);lD
cRSqrt:nT){xM
l63(1)/fp_sqrt
eS2
cCot:eT2
fp_tan(n0
cSec:eT2
fp_cos(n0
cCsc:eT2
fp_sin(n0
cHypot:if
lJ
fp_hypot(l42
lD
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
cFCall:case
cEval:yY3
do_return:;
#ifdef DEBUG_SUBSTITUTIONS
std::cout<<"["
<<(&cY3)<<"]Done ConstantFolding, result: "
c4
DumpHashes(tree);
#endif
}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
tF{tB1
yQ3::set_abs(nO
bool
has_negative=!min
xG3||min.val<l63();bool
has_positive=!max
xG3||max.val>l63();bool
crosses_axis=has_negative&&has_positive;rangehalf
xI
newmax;if(min
xG3&&max
xG3)newmax.set(fp_max(i61,i71
iR3
crosses_axis)min.set
tR2());e53
min
xG3&&max
xG3)min.set(fp_min(i61,i71);iW1
min
xG3)min.set(i61);else
min.set(i71;}
max=newmax;}
tB1
yQ3::set_neg(){std::swap(min,max);min.val=-min.val;max.val=-max.val;}
yQ1
IsLogicalTrueValue
e11
yQ3&p,bool
abs){if(nC
IsIntType
xI::eH3){if(p
tH1
p
l73>=l63(1
cX1
if(!abs&&p
x62
c1
val<=l63(-1
cX1}
e53
p
tH1
p
l73>=l63(0.5
cX1
if(!abs&&p
x62
c1
val<=l63(-0.5
cX1}
lY2
yQ1
IsLogicalFalseValue
e11
yQ3&p,bool
abs){if(nC
IsIntType
xI::eH3){if(abs)lM3
c1
known
lN1
1);else
lM3
tH1
p
x62
l73>l63(-1)lN1
1);}
e53
abs)lM3
c1
known
lN1
0.5);else
lM3
tH1
p
x62
l73>l63(-0.5)lN1
0.5);}
}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
using
lR3
FUNCTIONPARSERTYPES;using
tF;tF{nU2
yQ3
iO
const
eV
#ifdef DEBUG_SUBSTITUTIONS_extra_verbose
{using
lR3
FUNCTIONPARSERTYPES;yQ3
tmp=CalculateResultBoundaries_do(tree)xH1"Estimated boundaries: "
;if(tmp
iL2)std::cout<<tmp
l73;else
std::cout<<"-inf"
xH1" .. "
;if(tmp
tO1
std::cout<<tmp
c1
val;else
std::cout<<"+inf"
xH1": "
yA3
tree)xH1
std::endl
iP3
tmp;}
nU2
yQ3
eW::CalculateResultBoundaries_do
e11
eV
#endif
{iI
yJ1(-fp_const_pihalf
xI(),fp_const_pihalf
xI());iI
pi_limits(-fp_const_pi
xI(),fp_const_pi
xI());iI
abs_pi_limits(y51,fp_const_pi
xI());iI
plusminus1_limits
tR2(-cW3
using
lR3
std;switch(tX2
eS3
cImmed:nP
lK4,lK4);case
cAnd:case
cAbsAnd:case
cOr:case
cAbsOr:case
cNot:case
cZ3:case
cNotNot:case
cAbsNotNot:case
cEqual:case
i51:case
cLess:case
cLessOrEq:case
cGreater:case
cGreaterOrEq:{nP
y51,l63(1))xH3
cAbs:lE
m.set_abs();t9
cLog:lE
x03
fp_log);m
x13
fp_log);t9
cLog2:lE
x03
fp_log2);m
x13
fp_log2);t9
cLog10:lE
x03
fp_log10);m
x13
fp_log10);t9
cAcosh:lE
yO
nV2
cGreaterOrEq
tN1
fp_acosh
xS3
cGreaterOrEq
tN1
fp_acosh);t9
cAsinh:lE
yO
set(fp_asinh);m
c1
set(fp_asinh);t9
cAtanh:lE
yO
n4-1),fp_atanh
xS3
cLess
tN1
fp_atanh);t9
cAcos:lE
nP(m
e92
nW2)<l63(1))?fp_acos(nW2):y51,(cK2&&(cF1)>=l63(-1))?fp_acos(cF1):fp_const_pi
xI())xH3
cAsin:lE
yO
n4-1),fp_asin,yJ1
l73
xS3
cLess
tN1
fp_asin,yJ1
c1
val);t9
cAtan:lE
yO
set(fp_atan,yJ1
l73);m
c1
set(fp_atan,yJ1
c1
val);t9
cAtan2:{nT&&y83
lS,xD1)yM
abs_pi_limits;}
if(tI1
y83
xZ
iV,xD1)yM
yJ1;}
return
pi_limits
xH3
cSin:lE
bool
x21=!cK2||!m
c1
iT3-cF1)>=(yR
x21)eH
l63
min=e63
cF1,yR
min<xD1
min
yT
l63
max=e63
nW2,yR
max<xD1
max
yT
if(max<min)max
yT
bool
y41=(min<=fp_const_pihalf
xI()&&max>=fp_const_pihalf
xI());bool
nP1=(min<=cO&&max>=cO
iR3
y41&&nP1)eH
if(nP1)nP
l63(-1),x72
if(y41)nP
cJ2
l63(1));nP
cJ2
x72}
case
cCos:lE
if(cK2)cF1+=fp_const_pihalf
xI(iR3
m
tO1
nW2+=fp_const_pihalf
xI();bool
x21=!cK2||!m
c1
iT3-cF1)>=(yR
x21)eH
l63
min=e63
cF1,yR
min<xD1
min
yT
l63
max=e63
nW2,yR
max<xD1
max
yT
if(max<min)max
yT
bool
y41=(min<=fp_const_pihalf
xI()&&max>=fp_const_pihalf
xI());bool
nP1=(min<=cO&&max>=cO
iR3
y41&&nP1)eH
if(nP1)nP
l63(-1),x72
if(y41)nP
cJ2
l63(1));nP
cJ2
x72}
case
cTan:{nP)xH3
cCeil:lE
m
c1
i81
cFloor:lE
yO
nN2
t9
cTrunc:lE
yO
nN2
m
c1
i81
cInt:lE
yO
nN2
m
c1
i81
cSinh:lE
yO
set(fp_sinh);m
c1
set(fp_sinh);t9
cTanh:lE
yO
set(fp_tanh,plusminus1_limits.min);m
c1
set(fp_tanh,plusminus1_limits.max);t9
cCosh:lE
if(cK2){if(m
tO1{if(cF1>=y51&&nW2>=xD1{cF1
xW}
iW1(cF1)<y51&&nW2>=xD1{l63
tmp
xW
if(tmp>nW2)nW2=tmp;cF1=l63(1);}
else{cF1
xW
std::swap(cF1,nW2);}
}
e53
cF1>=xD1{m
t6
cF1=fp_cosh(cF1);}
else{m
t6
cF1=l63(1);}
}
}
else{cK2=true;cF1=l63(1
iR3
m
tO1{cF1=fp_cosh(nW2);m
t6}
else
m
t6}
t9
cIf:case
cAbsIf:{yQ3
res1
t7
1
yB2
res2
t7
2)iR3!res2
iL2)res1
iL2
tK3
iW1
res1
tH1(res2
l73)<res1
l73)res1
l73=res2
l73;if(!res2
tO1
res1
t6
iW1
res1
e92
res2
c1
val)>res1
c1
val)res1
c1
val=res2
c1
val
iP3
res1
xH3
cMin:{bool
iJ
tK3
bool
iK
tK3
yT3;x7
m
t7
e73
cK2)iJ
lA3
iL2||(cF1)<eJ3
iA3=cF1;if(!m
tO1
iK
lA3
c1
iT3)<eI3)eI3=nW2;}
if(iJ
lT2
known
tK3
if(iK)eH3
t6
return
eH3
xH3
cMax:{bool
iJ
tK3
bool
iK
tK3
yT3;x7
m
t7
e73
cK2)iJ
lA3
iL2||cF1>eJ3
iA3=cF1;if(!m
tO1
iK
lA3
c1
known||nW2>eI3)eI3=nW2;}
if(iJ
lT2
known
tK3
if(iK)eH3
t6
return
eH3
xH3
cAdd:{yT3(y51,xD1;x7
item
t7
a)iR3
item
iL2
iA3+=item
l73
yU3
iL2
tK3
if(item
tO1
eI3+=item
c1
val
yU3
t6
if(!eH3
tH1!eH3
tO1
yY3
if(eH3
tH1
eH3
c1
iB3
eJ3>eI3)std::swap(eJ3,eI3)iP3
eH3
xH3
cMul:{eI2
Value{enum
x33{e83,lC2,lB3}
;x33
tA
l53
value;Value(x33
t):tA(t),value(0){}
Value
tR2
v):tA(e83),value(v){}
bool
eA2
c61
tA==lC2||yC2
value<xD1
eW3
eM1*=e11
Value&rhs){if
yC2
rhs.tA==e83)value*=rhs.value;else
tA=(eA2)!=rhs.eA2)?lC2:lB3);}
iU2<e11
Value&rhs
c61(tA==lC2&&rhs.tA!=lC2)||yC2(rhs.tA==lB3||(rhs.tA==e83&&value<rhs.eO3));}
}
;eI2
yK1{Value
cL2,cM2;yK1():cL2(Value::lB3),cM2(Value::lC2){}
void
multiply(Value
e93,const
Value&value2){e93*=value2;if(e93<cL2)cL2=e93;if(cM2<e93)cM2=e93;}
}
;yT3
tR2(cW3
x7
item
t7
e73
item
tH1!item
tO1
nP
eY3
x43=eH3.min.lD2
eH3.min.n81
lC2
eY3
x53=eH3
c1
lD2
eH3
c1
n81
lB3
eY3
x63=item.min.lD2
item.min.n81
lC2
eY3
x73=item
c1
lD2
item
c1
n81
lB3);yK1
range
yS1
x43,x63)yS1
x43,x73)yS1
x53,x63)yS1
x53,x73
iR3
range.cL2.tA==Value::e83
iA3=range.cL2.value
yU3
iL2
tK3
if(range.cM2.tA==Value::e83)eI3=range.cM2.value
yU3
t6
if(!eH3
tH1!eH3
tO1
yY3
if(eH3
tH1
eH3
c1
iB3
eJ3>eI3)std::swap(eJ3,eI3)iP3
eH3
xH3
cMod:{yQ3
x
t7
0
yB2
y
t7
1)iR3
y
tO1{if(y
c1
val>=xD1{if(!x
iL2||(x
l73)<xD1
nP-y
c1
val,y
c1
val);iC3
y51,y
c1
val);}
e53!x
c1
known||(x
c1
val)>=xD1
nP
y
c1
val,-y
c1
val);iC3
y
c1
val,fp_const_negativezero
xI());}
}
iC3)xH3
cPow:{if(tI1
xZ
iV==xD1{nP
l63(cW3}
nT&&lS==xD1{nP
y51,xD1;}
nT&&y83
lS
y22
nP
l63(cW3}
if(tI1
xZ
iV>y51&&GetEvennessInfo(xZ
1))==IsAlways
eU2
eY2
xZ
iV;yQ3
tmp
t7
0
yB2
eH3
x83
iL2=true
x83
l73=0;if(tmp
tH1
tmp
l73>=xD1
eJ3
tM3
tmp
l73,y72;iW1
tmp
c1
iB3
tmp
c1
val<=xD1
eJ3
tM3
tmp
c1
val,y72
x83
t6
if(tmp
tH1
tmp
tO1{eH3
c1
known=true
x83
c1
val=fp_max(fp_abs(tmp
l73),fp_abs(tmp
c1
val))x83
c1
val
tM3
eI3,y72;}
return
eH3;}
yQ3
p0
t7
0
yB2
p1
t7
1));TriTruthValue
p0_positivity=(p0
tH1(p0
l73)>=xD1?IsAlways:(p0
e92
p0
e33
y51?lN3
iD3);TriTruthValue
eB2=GetEvennessInfo(xZ
1));TriTruthValue
tB=iD3;switch(p0_positivity)lE2
tB=IsAlways;lD
lN3{tB=eB2;yY3
yX3
switch(eB2)lE2
tB=IsAlways;lD
lN3
lD
iD3:{if(tI1!tU2
xZ
iV)&&xZ
iV>=xD1{tB=IsAlways;}
yY3}
lK3
tB)lE2{l63
min=y51;if(p0
tH1
p1
iL2){min
tM3
p0
l73,p1
l73
iR3
p0
l73<y51&&(!p1
c1
known||p1
c1
val>=xD1&&min>=xD1
min=y51;}
if(p0
tH1
p0
l73>=y51&&p0
c1
iB3
p1
tO1{l63
max
tM3
p0
c1
val,p1
c1
val
iR3
min>max)std::swap(min,max);nP
min,max);}
nP
min,false)xH3
lN3{nP
false,fp_const_negativezero
xI());}
yX3{yY3
tE1
cNeg:lE
m.set_neg();t9
cSub:{yP
cNeg);iE3
1));tmp
cI
cAdd);tmp.iF3
tmp.cG1
xB1
cInv:{eC2-1)));xB1
cDiv:{yP
cInv);iE3
1));tmp
cI
y71.cG1
xB1
cRad:c8
y71
y2
fp_const_rad_to_deg
xI()));xB1
cDeg:c8
y71
y2
fp_const_deg_to_rad
xI()));xB1
cSqr:{eC2
2)));xB1
cExp:c8
cPow);tmp
y2
fp_const_e
xI()));tmp.iF3
xB1
cExp2:c8
cPow);tmp
y2
c23
tmp.iF3
xB1
cCbrt:lE
yO
set(fp_cbrt);m
c1
set(fp_cbrt);t9
cSqrt:lE
if(cK2)cF1=(cF1)<y51?0:fp_sqrt(cF1
iR3
m
tO1
nW2=(nW2)<y51?0:fp_sqrt(nW2);t9
cRSqrt:{eC2-0.5)));xB1
cHypot:{eW
xsqr,ysqr,add,sqrt;xsqr.iF3
xsqr
y2
c23
ysqr.nM
1));ysqr
y2
c23
xsqr
cI
cPow);ysqr
cI
cPow);add
l12
xsqr);add
l12
ysqr);add
cI
cAdd);sqrt
l12
add);sqrt
cI
cSqrt)iP3
iO
sqrt)xH3
eC3:{yP
cLog2);iE3
0));tmp
tI3
tmp.cG1
tmp.nM
1));xB1
cCot:{yP
cTan)x8
lI
cSec:{yP
cCos)x8
lI
cCsc:{yP
cSin)x8
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
lG3:lD
cArg:case
cConj:case
cImag:case
cReal:case
cPolar:lD
cPCall:lD
cFCall:lD
cEval:yY3
nP);}
nU2
TriTruthValue
GetIntegerInfo
e11
eV{switch(tX2
eS3
cImmed:return
tU2
lK4)?IsAlways:IsNever;case
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
i51:case
cLess:case
cLessOrEq:case
cGreater:case
cGreaterOrEq:return
IsAlways;case
cIf:{TriTruthValue
a=GetIntegerInfo(xZ
1));TriTruthValue
b=GetIntegerInfo(xZ
2)iR3
a==b)return
a;yY1}
case
cAdd:case
cMul:{for
c5
if(GetIntegerInfo(iZ3)!=IsAlways)yY1
return
IsAlways;}
yX3
yY3
yY1}
yQ1
IsLogicalValue
e11
eV{switch(tX2
eS3
cImmed:return
y83
lK4,xD1||y83
lK4,l63(1));case
cAnd:case
cOr:case
cNot:case
cNotNot:case
cAbsAnd:case
cAbsOr:case
cZ3:case
cAbsNotNot:case
cEqual:case
i51:case
cLess:case
cLessOrEq:case
cGreater:case
cGreaterOrEq:nY
cMul:{for
c5
if(!c93
a))cJ
iB1
case
cIf:case
cAbsIf:yM
c93
1))&&c93
2));}
yX3
yY3
lY2}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
using
lR3
FUNCTIONPARSERTYPES;
#if defined(__x86_64) || !defined(FP_SUPPORT_CBRT)
# define CBRT_IS_SLOW
#endif
#if defined(DEBUG_POWI) || defined(DEBUG_SUBSTITUTIONS)
#include <cstdio>
#endif
lR3
xI1{extern
const
xL3
char
powi_table[256];}
lR3{using
tF
yP1
bool
IsOptimizableUsingPowi(long
immed,long
penalty=0){xI1::e72
synth;synth.PushVar(lG3);size_t
bytecodesize_backup=synth.GetByteCodeSize();xI1::x11
immed,xI1::tW1
xI::MulSequence,synth);size_t
bytecode_grow_amount=synth.GetByteCodeSize()-bytecodesize_backup
iP3
bytecode_grow_amount<size_t(MAX_POWI_BYTECODE_LENGTH-penalty);}
tB1
ChangeIntoRootChain(eW&tree,bool
lO3,long
iM2,long
iN2){while(iN2>0)c8
cCbrt);lT1
tmp
nJ2
tree.iO2--iN2;}
while(iM2>0)c8
cSqrt
iR3
lO3){tmp
cI
cRSqrt);lO3
tK3}
lT1
tmp
nJ2
tree.iO2--iM2;}
if(lO3)c8
cInv);lT1
tree.iO2}
}
nU2
eI2
RootPowerTable{static
const
l63
RootPowers[(1+4)*(1+3)];}
yP1
const
l63
tK(1+4)*(1+3)]={l63(1)lT
iH3
iH3
2*iH3
iH1)lT
3*2)lT
3*2*2)lT
3*iH1*2*iH1*3)lT
3*3*tG2
2*tG2
iH1*3*2*iH1*3*3)lT
3*3*3*tG2
3*2*tG2
3*iH1*3*3*2*2*2*2)}
;eI2
PowiResolver{static
const
xL3
MaxSep=4;static
xC3
iI3=5;typedef
int
eA3;typedef
long
xJ3;typedef
long
tM;eI2
cN2{cN2():n_int_sqrt(0),n_int_cbrt(0),sep_list(),n91(0){}
int
n_int_sqrt;int
n_int_cbrt;int
tV1
MaxSep];tM
n91;}
yP1
static
cN2
CreatePowiResult
tR2
y72{cN2
eH3;eA3
tL=FindIntegerFactor(y72;if(tL==0){
#ifdef DEBUG_POWI
iP2"no factor found for %Lg\n"
,lU1
y72;
#endif
return
eH3;}
eH3.n91=y81
xN3,tL);xJ3
tH2=EvaluateFactorCost(tL,0,0,0)+cN
eH3.n91);int
iJ3=0;int
iK3=0;int
xB3=0;
#ifdef DEBUG_POWI
iP2"orig = %Lg\n"
,lU1
y72;iP2"plain factor = "
lE4"%ld\n"
,(int)tL,(long)tH2);
#endif
for
xA2
n_s=0;n_s<MaxSep;++n_s){int
xG=0;xJ3
yL1=tH2;eA3
c31=tL;for(int
s=1;s<iI3*4;++s){
#ifdef CBRT_IS_SLOW
if(s>=iI3)break;
#endif
int
n_sqrt=s%iI3;int
n_cbrt=s/iI3;if(n_sqrt+n_cbrt>4)continue
l53
lO1=xN3;lO1-=tK
s];i91=FindIntegerFactor(lO1
iR3
yJ2!=0){tM
xS=y81
lO1,yJ2);xJ3
cost=EvaluateFactorCost(yJ2,iJ3+n_sqrt,iK3+n_cbrt,xB3+1)+cN
xS);
#ifdef DEBUG_POWI
iP2"Candidate sep %u (%d*sqrt %d*cbrt)factor = "
lE4"%ld (for %Lg to %ld)\n"
,s,n_sqrt,n_cbrt,yJ2,(long)cost,lU1
lO1,(long)xS);
#endif
if(cost<yL1){xG=s;c31=yJ2;yL1=cost;}
}
}
if(!xG)break;
#ifdef DEBUG_POWI
iP2"CHOSEN sep %u (%d*sqrt %d*cbrt)factor = "
lE4"%ld, exponent %Lg->%Lg\n"
,xG,xG%iI3,xG/iI3,c31,yL1,lU1(y72,lU1(xN3-tK
xG]));
#endif
eH3.tV1
n_s]=xG;xN3-=tK
xG];iJ3+=xG%iI3;iK3+=xG/iI3;tH2=yL1;tL=c31;xB3+=1;}
eH3.n91=y81
xN3,tL);
#ifdef DEBUG_POWI
iP2"resulting exponent is %ld (from exponent=%Lg, best_factor=%Lg)\n"
,eH3.n91,lU1
xN3,lU1
tL);
#endif
while(tL%2==0){++eH3
eE2;tL/=2;}
while(tL%3==0){++eH3.n_int_cbrt;tL/=3;}
return
eH3;}
private:static
xJ3
cN
tM
xS){static
std::map
eD2
iB;if(xS<0){xJ3
cost=22
iP3
cost+cN-xS);}
std::map
eD2::yL3
i=iB.yK2
xS
iR3
i!=iB.e61
xS)return
i
eQ2;std::pair
eD2
eH3(xS,0.0);xJ3&cost=eH3
n23;while(xS>1){int
yJ2=0;if(xS<256){yJ2=xI1::powi_table[xS];if(yJ2&128)yJ2&=127;else
yJ2=0;if(yJ2&64)yJ2=-(yJ2&63)-1;}
if(yJ2){cost+=cN
yJ2);xS/=yJ2;continue;}
if(!(xS&1)){xS/=2;cost+=6;}
else{cost+=7;xS-=1;}
}
iB.yH3,eH3)iP3
cost;}
cH1
tM
y81
yM1,i91)yM
makeLongInteger(value*l63(yJ2));}
cH1
bool
yN1
yM1,i91
eU2
v=value*l63(yJ2)iP3
isLongInteger(v);}
cH1
eA3
FindIntegerFactor(yM1){i91=(2*2*2*2);
#ifdef CBRT_IS_SLOW
#else
yJ2*=(3*3*3);
#endif
eA3
eH3=0;if(yN1
iL3)){n33;while((yJ2%2)==0&&yN1
iL3/2))n33/=2;while((yJ2%3)==0&&yN1
iL3/3))n33/=3;}
#ifdef CBRT_IS_SLOW
if(eH3==0){if(yN1
eL3
3
cA3
3;}
#endif
return
eH3;}
static
int
EvaluateFactorCost(int
yJ2,int
s,int
c,int
nmuls){xC3
xD3=6;
#ifdef CBRT_IS_SLOW
xC3
tI2=25;
#else
xC3
tI2=8;
#endif
int
eH3=s*xD3+c*tI2;while(yJ2%2==0){yJ2/=2
x83+=xD3;}
while(yJ2%3==0){yJ2/=3
x83+=tI2;}
eH3+=nmuls
iP3
eH3;}
}
;}
tF{yQ1
eW::RecreateInversionsAndNegations(bool
prefer_base2){bool
changed
tK3
for
iD1
a=0;a<iR
if(n51.RecreateInversionsAndNegations(prefer_base2))xQ2
if(changed){exit_changed:Mark_Incompletely_Hashed();iB1
switch(l72{case
cMul:{eN
nO2;eW
nP2,cU1;if(true){bool
nQ1=false
l53
xP2=0;for
iP
nV
nD==eB3
0)nD==iM3
tT
1)tK1){nQ1=true;xP2=tT
iV;yY3}
if(nQ1
eU2
immeds=1.0;for
iP
nV
tK1){immeds*=powgroup.xQ1;yO1}
for
iP-->0;){eW&powgroup=n51;if(powgroup
nD==eB3
0)nD==iM3
tT
1).IsImmed(eO&log2=tT
0);log2.lD1
log2
cI
eC3);log2
y2
fp_pow(immeds,l63(1)/xP2)));log2
nJ2
yY3}
}
}
for
iP
nV
nD==eB3
1)tK1){xW2
exp_param=tT
1)l53
eY2
exp_param.xQ1;if(e81,l63(-1))){lD1
nO2.push_back(n51
l9
0));yO1
iW1
xN3<y51&&tU2
xN3
eO
iL;iL
cI
cPow);iL
xM2
tT
0));iL
y2-y72);iL
nJ2
nO2.push_back(iL);lD1
yO1}
iW1
powgroup
nD==iM3!nP2.cO2)){nP2=tT
0);lD1
yO1
iW1
powgroup
nD==eC3&&!cU1.cO2)){cU1=powgroup;lD1
yO1}
if(!nO2
cT3){xQ2
eW
iX1;iX1
tI3
iX1
iA1
nO2);iX1
nJ2
eW
y91
cMul);l22
SetParamsMove
eR
if
l33
tK1&&y83
l22
xQ1
y22
nQ2
cInv
eS
iX1);}
e53
l22
GetDepth()>=iX1
x12){nQ2
cDiv
eS
mulgroup
eS
iX1);}
else{nQ2
cRDiv
eS
iX1
eS
mulgroup);}
}
}
if(nP2.cO2
eO
y91
l72;l22
SetParamsMove
eR
while(l22
RecreateInversionsAndNegations(prefer_base2))l22
FixIncompleteHashes();nQ2
eC3
eS
nP2
eS
mulgroup);xQ2}
if(cU1.cO2
eO
y91
cMul);mulgroup
l12
cU1
l9
1));l22
AddParamsMove
eR
while(l22
RecreateInversionsAndNegations(prefer_base2))l22
FixIncompleteHashes();DelParams();nQ2
eC3
eS
cU1
l9
0)eS
mulgroup);xQ2
tE1
cAdd:{eN
iQ2;for
iP-->0;)if(eD3
cMul){nR2
yA1:;eW&mulgroup
eE3
for
iD1
b=mulgroup
cZ1;b-->0;){if
l33
l9
b).n11
yJ2=mulgroup
l9
b).xQ1;if
y73
yJ2
tQ1
yA1;}
l22
lD1
l22
DelParam(b);lP3
iW1
y83
yJ2,l63(-2)))xO
yA1;}
l22
lD1
l22
DelParam(b);mulgroup
y2
c23
lP3}
}
if(tC){l22
tN
mulgroup);yO1}
iW1
eD3
cDiv&&!IsIntType
xI::eH3){nR2
yB1:;eW&iX1
eE3
if(iX1
l9
0)iN3
y73
iX1
l9
0).xQ1
tQ1
yB1;}
iX1.lD1
iX1
nI2
0);iX1
cI
cInv);lP3}
if(tC)xO
yB1;}
iX1.tN
iX1);yO1}
iW1
eD3
cRDiv&&!IsIntType
xI::eH3){nR2
x81:;eW&iX1
eE3
if(iX1
l9
1)iN3
y73
iX1
l9
iV
tQ1
x81;}
iX1.lD1
iX1
nI2
1);iX1
cI
cInv);lP3}
if(tC)xO
x81;}
iX1.tN
iX1);yO1}
if(!iQ2
cT3){
#ifdef DEBUG_SUBSTITUTIONS
iP2"Will make a Sub conversion in:\n"
);fflush(stdout);iQ
#endif
eW
cP2;cP2
cI
cAdd);cP2
iA1
iQ2);cP2
nJ2
eW
cV1;cV1
cI
cAdd);cV1
iA1
lH2));cV1
nJ2
if(cV1
tK1&&y83
cV1.xQ1,xD1){nQ2
cNeg);eB);}
e53
cV1
x12==1){nQ2
cRSub);eB
eF3}
iW1
cP2
nD==cAdd){nQ2
cSub
eF3
eB
l9
0)cF2
1;a<cP2.iR{eW
tJ2;tJ2
cI
cSub);tJ2
iA1
lH2));tJ2.Rehash(false
eS
tJ2);eB
l9
a));}
}
else{nQ2
cSub
eF3
eB);}
}
#ifdef DEBUG_SUBSTITUTIONS
iP2"After Sub conversion:\n"
);fflush(stdout);iQ
#endif
tE1
cPow:{xW2
p0
xR2
0);xW2
p1
xR2
1
iR3
p1
iN3(p1.xQ1!=y51&&!tU2
p1.xQ1)){eJ
cN2
r=eJ
CreatePowiResult(fp_abs(p1.xQ1)iR3
r.n91!=0){bool
lF2
tK3
if(p1.xQ1<y51&&r.tV1
0]==0&&r
eE2>0){lF2=true;}
#ifdef DEBUG_POWI
iP2"Will resolve powi %Lg as powi(chain(%d,%d),%ld)"
,lU1
fp_abs(p1.xQ1),r
eE2,r.n_int_cbrt,r.n91);for
xA2
n=0;n<eJ
MaxSep;++n){if(r
yZ3==0)break;int
n_sqrt=r
yZ3%eJ
iI3;int
n_cbrt=r
yZ3/eJ
iI3;iP2"*chain(%d,%d)"
,n_sqrt,n_cbrt);}
iP2"\n"
);
#endif
eW
eF2
xR2
0);eW
cQ2=eF2;cQ2.lD1
ChangeIntoRootChain(cQ2,lF2,r
eE2,r.n_int_cbrt);cQ2
nJ2
eW
pow;if(r.n91!=1){pow
cI
cPow);pow
l12
cQ2);pow
y2
l63(r.n91)));}
else
pow.swap(cQ2);eW
mul;mul
tI3
mul
l12
pow);for
xA2
n=0;n<eJ
MaxSep;++n){if(r
yZ3==0)break;int
n_sqrt=r
yZ3%eJ
iI3;int
n_cbrt=r
yZ3/eJ
iI3;eW
tK2=eF2;tK2.lD1
ChangeIntoRootChain(tK2,false,n_sqrt,n_cbrt);tK2
nJ2
mul
l12
tK2);}
if(p1.xQ1<y51&&!lF2){mul
nJ2
nQ2
cInv);SetParamMove(0,mul);DelParam(1);}
else{nQ2
cMul);SetParamsMove(mul.lH2));}
#ifdef DEBUG_POWI
iQ
#endif
xQ2
yY3}
}
if(GetOpcode()==cPow&&(!p1
tK1||!isLongInteger(p1.xQ1)||!IsOptimizableUsingPowi
xI(makeLongInteger(p1.xQ1)))){if(p0
tK1&&p0.xQ1>l63(0.0)){if(prefer_base2
eU2
cR2=fp_log2(p0.xQ1);if
y73
cR2
y22
DelParam(0);}
else{n1
eA1
cR2))yT1
eG2
p1
yS3
yU1}
nQ2
cExp2);xQ2}
else{l63
cR2=fp_log(p0.xQ1);if
y73
cR2
y22
DelParam(0);}
else{n1
eA1
cR2))yT1
eG2
p1
yS3
yU1}
nQ2
cExp);xQ2}
}
iW1
GetPositivityInfo(p0)==IsAlways){if(prefer_base2){eW
log;log
cI
cLog2);log
xM2
p0);log
nJ2
n1
p1)yT1
l32
log
yS3;nQ2
cExp2)yU1
xQ2}
else{eW
log;log
cI
cLog);log
xM2
p0);log
nJ2
n1
p1)yT1
l32
log
yS3;nQ2
cExp)yU1
xQ2}
}
tE1
cDiv:{if(GetParam(0)tK1&&y83
GetParam(0).xQ1
y22
nQ2
cInv);DelParam(0);}
yY3
yX3
yY3
if(changed)goto
exit_changed
iP3
changed;}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
using
lR3
FUNCTIONPARSERTYPES;lR3{using
tF;class
iZ1{size_t
nR1;size_t
eK;size_t
eL;size_t
lP1;size_t
tD;size_t
tE;size_t
nE1;t83
iZ1():nR1(0),eK(0),eL(0),lP1(0),tD(0),tE(0),nE1(0){}
void
tN3
OPCODE
op){nR1+=1
iY1
cCos)++eK
iY1
cSin)++eL
iY1
cSec)++eK
iY1
cCsc)++eL
iY1
cTan)++lP1
iY1
cCot)++lP1
iY1
cSinh)++tE
iY1
cCosh)++tD
iY1
cTanh)++nE1;}
size_t
GetCSEscore
c03
size_t
eH3=nR1
iP3
eH3;}
int
NeedsSinCos
c03
bool
yC1=(nR1==(eK+eL+lP1)iR3(lP1&&(eL||eK))||(eL&&eK)){if(yC1)return
1
iP3
2;}
return
0;}
int
NeedsSinhCosh
c03
bool
yC1=(nR1==(tD+tE+nE1)iR3(nE1&&(tE||tD))||(tE&&tD)){if(yC1)return
1
iP3
2;}
return
0;}
size_t
MinimumDepth
c03
size_t
n_sincos=std::min(eK,eL);size_t
n_sinhcosh=std::min(tD,tE
iR3
n_sincos==0&&n_sinhcosh==0)return
2
iP3
1;}
}
yP1
class
TreeCountType:public
std::multimap<yD2,std::pair<iZ1,eW> >{}
xX3
FindTreeCounts(tR1&iE1,xW2
tree,OPCODE
xT2,bool
skip_root=false){e0
i=iE1.yK2
tree.GetHash()iR3!skip_root){bool
found
tK3
for(;i!=iE1.e61
tree.GetHash();++i){if(tree
xL
i
eQ2
n23)){i
eQ2.first.tN3
xT2);found=true;yY3}
if(!found){iZ1
count;count.tN3
xT2);iE1.yH3,std::make_pair(tree.GetHash(),std::make_pair
eP3,tree)));}
}
lC1
FindTreeCounts(iE1,iZ3,tX2);}
eI2
yV{bool
BalanceGood;bool
FoundChild;}
yP1
yV
lQ1
xW2
root,xW2
child){if(root
xL
child)){yV
eH3={true,true}
iP3
eH3;}
yV
eH3={true,false}
;if(root
nD==cIf||root
nD==tO3{yV
cond=lQ1
root
l9
0
tL2
yV
xY=lQ1
root
l9
1
tL2
yV
y3=lQ1
root
l9
2
tL2
if(cond
yW||xY
yW||y3
yW){eH3
yW=true;}
eH3
eC=((xY
yW==y3
yW)||t71&&(cond
eC||(xY
yW&&y3
yW))&&(xY
eC||t71&&(y3
eC||t71;}
else{bool
iJ1
tK3
bool
nS1
tK3
for
iD1
b=root
cZ1,a=0;a<b;++a){yV
tmp=lQ1
root
l9
a
tL2
if(tmp
yW)eH3
yW=true;if(tmp
eC==false)iJ1=true;iW1
tmp
yW)nS1=true;}
if(iJ1&&!nS1)eH3
eC
tK3}
return
eH3;}
yQ1
nA3
lZ3
iQ3
xW2
tree,const
xI1::e72&synth,const
tR1&iE1){for
iD1
b=iU,a=0;a<b;++a){xW2
leaf=iZ3;e0
synth_it;yN2
tR1::const_iterator
i=iE1.yK3
i!=iE1.end();++i){if(i->first!=leaf.GetHash())continue;const
iZ1&occ
xS2
first;size_t
score=occ.GetCSEscore();xW2
candidate
xS2
second;nT2
candidate)eM2
leaf
x12<occ.MinimumDepth()eM2
score<2
eM2
lQ1
iQ3
leaf)eC==false)continue;iB1
if(nA3(iQ3
leaf,synth,iE1
cX1}
lY2
yQ1
nS2
lZ3
yI3,xW2
expr){for
iD1
a
y8
yI3
n93
expr
cX1
for
iD1
a
y8
nS2(yI3
l9
a),expr
cX1
lY2
yQ1
GoodMomentForCSE
lZ3
yI3,xW2
expr){if(yI3
nD==cIf)return
true;for
iD1
a
y8
yI3
n93
expr
cX1
size_t
iR2=0;for
iD1
a
y8
nS2(yI3
l9
a),expr))++iR2
iP3
iR2!=1;}
}
tF{nU2
size_t
eW::SynthCommonSubExpressions(xI1::cD1
const{if(GetParamCount()==0)return
0;size_t
stacktop_before=synth.GetStackTop();tR1
iE1;FindTreeCounts(iE1,*this,GetOpcode(),true);for(;;){size_t
cS2=0;
#ifdef DEBUG_SUBSTITUTIONS_CSE
std::cout<<"Finding a CSE candidate, root is:"
<<std::endl;DumpHashes(*this);
#endif
e0
cs_it(iE1.end());for(e0
j=iE1.yK3
j!=iE1.end();){e0
i(j++);const
iZ1&occ
xS2
first;size_t
score=occ.GetCSEscore();xW2
tree
xS2
second;
#ifdef DEBUG_SUBSTITUTIONS_CSE
std::cout<<"Score "
<<score<<":\n"
<<std::flush;DumpTreeWithIndent(tree);
#endif
nT2
tree))y9
if(tree
x12<occ.MinimumDepth())y9
if(score<2)y9
if(lQ1*this,tree)eC==false)y9
if(nA3(*this,tree,synth,iE1)){continue;}
if(!GoodMomentForCSE(*this,tree))y9
score*=tree
x12;if(score>cS2){cS2=score;cs_it=i;}
}
if(cS2<=0){
#ifdef DEBUG_SUBSTITUTIONS_CSE
std::cout<<"No more CSE candidates.\n"
<<std::flush;
#endif
yY3
xW2
tree=cs_it
eQ2
n23;
#ifdef DEBUG_SUBSTITUTIONS_CSE
std::cout<<lJ4"Common Subexpression:"
;DumpTree
xI(tree)xH1
std::endl;
#endif
#if 0
int
nA1=occ.NeedsSinCos();int
i2=occ.NeedsSinhCosh();eW
iS2,iT2,cT2,cU2;if(nA1){iS2.c41
iS2
cI
cSin);iS2
nJ2
iT2.c41
iT2
cI
cCos);iT2
nJ2
nT2
iS2)||synth.Find(iT2)eZ3==2){cY1
continue;}
nA1=0;}
}
if(i2){cT2.c41
cT2
cI
cSinh);cT2
nJ2
cU2.c41
cU2
cI
cCosh);cU2
nJ2
nT2
cT2)||synth.Find(cU2)){if(i2==2){cY1
continue;}
i2=0;}
}
#endif
tree.SynthesizeByteCode(synth,false);cY1
#ifdef DEBUG_SUBSTITUTIONS_CSE
synth.xM3
Dump<0>()xH1"Done with Common Subexpression:"
;DumpTree
xI(tree)xH1
std::endl;
#endif
#if 0
if(nA1
eZ3==2||i2){synth.eK1}
lW3
cSinCos,1,2)eL1
iS2,1)eL1
iT2,0);}
if(i2
eZ3)synth.eK1
if(i2==2){synth.eK1}
lW3
cSinhCosh,1,2)eL1
cT2,1)eL1
cU2,0);}
#endif
}
return
synth.xK
stacktop_before;}
}
#endif
#ifdef FP_SUPPORT_OPTIMIZER
tB1
FunctionParserBase
xI::Optimize(){using
tF;lD1
eW
tree;tree.GenerateFrom(*mData);FPoptimizer_Optimize::ApplyGrammars(tree);std
y33<xL3>eG3;std
y33
xI
immed;size_t
stacktop_max=0;tree.SynthesizeByteCode(eG3,immed,stacktop_max
iR3
mData->mStackSize!=stacktop_max){mData->mStackSize=xL3(stacktop_max);
#if !defined(FP_USE_THREAD_SAFE_EVAL) && \
    !defined(FP_USE_THREAD_SAFE_EVAL_WITH_ALLOCA)
mData->mStack
xV3
stacktop_max);
#endif
}
mData->mByteCode.swap(eG3);mData->mImmed.swap(immed);}
#ifdef FP_SUPPORT_MPFR_FLOAT_TYPE
xF
MpfrFloat
nF1
#endif
#ifdef FP_SUPPORT_GMP_INT_TYPE
xF
GmpInt
nF1
#endif
#ifdef FP_SUPPORT_COMPLEX_DOUBLE_TYPE
xF
std::complex<double>nF1
#endif
#ifdef FP_SUPPORT_COMPLEX_FLOAT_TYPE
xF
std::complex<float>nF1
#endif
#ifdef FP_SUPPORT_COMPLEX_LONG_DOUBLE_TYPE
xF
std::complex<long
double>nF1
#endif
FUNCTIONPARSER_INSTANTIATE_TYPES
#endif

#endif
