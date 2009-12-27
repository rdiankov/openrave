#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCg_param.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCg_param::create(DAE& dae)
{
	domCg_paramRef ref = new domCg_param(dae);
	return ref;
}


daeMetaElement *
domCg_param::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cg_param" );
	meta->registerClass(domCg_param::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool) );
	mea->setElementType( domCg_param::domBool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool2) );
	mea->setElementType( domCg_param::domBool2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool3) );
	mea->setElementType( domCg_param::domBool3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool4) );
	mea->setElementType( domCg_param::domBool4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool2x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool2x1) );
	mea->setElementType( domCg_param::domBool2x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool2x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool2x2) );
	mea->setElementType( domCg_param::domBool2x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool2x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool2x3) );
	mea->setElementType( domCg_param::domBool2x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool2x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool2x4) );
	mea->setElementType( domCg_param::domBool2x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool3x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool3x1) );
	mea->setElementType( domCg_param::domBool3x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool3x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool3x2) );
	mea->setElementType( domCg_param::domBool3x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool3x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool3x3) );
	mea->setElementType( domCg_param::domBool3x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool3x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool3x4) );
	mea->setElementType( domCg_param::domBool3x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool4x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool4x1) );
	mea->setElementType( domCg_param::domBool4x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool4x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool4x2) );
	mea->setElementType( domCg_param::domBool4x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool4x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool4x3) );
	mea->setElementType( domCg_param::domBool4x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool4x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemBool4x4) );
	mea->setElementType( domCg_param::domBool4x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat) );
	mea->setElementType( domCg_param::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat2) );
	mea->setElementType( domCg_param::domFloat2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat3) );
	mea->setElementType( domCg_param::domFloat3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat4) );
	mea->setElementType( domCg_param::domFloat4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat2x1) );
	mea->setElementType( domCg_param::domFloat2x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat2x2) );
	mea->setElementType( domCg_param::domFloat2x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat2x3) );
	mea->setElementType( domCg_param::domFloat2x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat2x4) );
	mea->setElementType( domCg_param::domFloat2x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat3x1) );
	mea->setElementType( domCg_param::domFloat3x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat3x2) );
	mea->setElementType( domCg_param::domFloat3x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat3x3) );
	mea->setElementType( domCg_param::domFloat3x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat3x4) );
	mea->setElementType( domCg_param::domFloat3x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat4x1) );
	mea->setElementType( domCg_param::domFloat4x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat4x2) );
	mea->setElementType( domCg_param::domFloat4x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat4x3) );
	mea->setElementType( domCg_param::domFloat4x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFloat4x4) );
	mea->setElementType( domCg_param::domFloat4x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt) );
	mea->setElementType( domCg_param::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt2) );
	mea->setElementType( domCg_param::domInt2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt3) );
	mea->setElementType( domCg_param::domInt3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt4) );
	mea->setElementType( domCg_param::domInt4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int2x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt2x1) );
	mea->setElementType( domCg_param::domInt2x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int2x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt2x2) );
	mea->setElementType( domCg_param::domInt2x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int2x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt2x3) );
	mea->setElementType( domCg_param::domInt2x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int2x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt2x4) );
	mea->setElementType( domCg_param::domInt2x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int3x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt3x1) );
	mea->setElementType( domCg_param::domInt3x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int3x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt3x2) );
	mea->setElementType( domCg_param::domInt3x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int3x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt3x3) );
	mea->setElementType( domCg_param::domInt3x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int3x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt3x4) );
	mea->setElementType( domCg_param::domInt3x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int4x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt4x1) );
	mea->setElementType( domCg_param::domInt4x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int4x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt4x2) );
	mea->setElementType( domCg_param::domInt4x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int4x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt4x3) );
	mea->setElementType( domCg_param::domInt4x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int4x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemInt4x4) );
	mea->setElementType( domCg_param::domInt4x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf) );
	mea->setElementType( domCg_param::domHalf::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf2) );
	mea->setElementType( domCg_param::domHalf2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf3) );
	mea->setElementType( domCg_param::domHalf3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf4) );
	mea->setElementType( domCg_param::domHalf4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half2x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf2x1) );
	mea->setElementType( domCg_param::domHalf2x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half2x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf2x2) );
	mea->setElementType( domCg_param::domHalf2x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half2x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf2x3) );
	mea->setElementType( domCg_param::domHalf2x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half2x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf2x4) );
	mea->setElementType( domCg_param::domHalf2x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half3x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf3x1) );
	mea->setElementType( domCg_param::domHalf3x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half3x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf3x2) );
	mea->setElementType( domCg_param::domHalf3x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half3x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf3x3) );
	mea->setElementType( domCg_param::domHalf3x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half3x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf3x4) );
	mea->setElementType( domCg_param::domHalf3x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half4x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf4x1) );
	mea->setElementType( domCg_param::domHalf4x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half4x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf4x2) );
	mea->setElementType( domCg_param::domHalf4x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half4x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf4x3) );
	mea->setElementType( domCg_param::domHalf4x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "half4x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemHalf4x4) );
	mea->setElementType( domCg_param::domHalf4x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed) );
	mea->setElementType( domCg_param::domFixed::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed2) );
	mea->setElementType( domCg_param::domFixed2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed3) );
	mea->setElementType( domCg_param::domFixed3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed4) );
	mea->setElementType( domCg_param::domFixed4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed2x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed2x1) );
	mea->setElementType( domCg_param::domFixed2x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed2x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed2x2) );
	mea->setElementType( domCg_param::domFixed2x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed2x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed2x3) );
	mea->setElementType( domCg_param::domFixed2x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed2x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed2x4) );
	mea->setElementType( domCg_param::domFixed2x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed3x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed3x1) );
	mea->setElementType( domCg_param::domFixed3x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed3x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed3x2) );
	mea->setElementType( domCg_param::domFixed3x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed3x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed3x3) );
	mea->setElementType( domCg_param::domFixed3x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed3x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed3x4) );
	mea->setElementType( domCg_param::domFixed3x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed4x1" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed4x1) );
	mea->setElementType( domCg_param::domFixed4x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed4x2" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed4x2) );
	mea->setElementType( domCg_param::domFixed4x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed4x3" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed4x3) );
	mea->setElementType( domCg_param::domFixed4x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fixed4x4" );
	mea->setOffset( daeOffsetOf(domCg_param,elemFixed4x4) );
	mea->setElementType( domCg_param::domFixed4x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler1D" );
	mea->setOffset( daeOffsetOf(domCg_param,elemSampler1D) );
	mea->setElementType( domFx_sampler1D::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler2D" );
	mea->setOffset( daeOffsetOf(domCg_param,elemSampler2D) );
	mea->setElementType( domFx_sampler2D::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler3D" );
	mea->setOffset( daeOffsetOf(domCg_param,elemSampler3D) );
	mea->setElementType( domFx_sampler3D::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "samplerRECT" );
	mea->setOffset( daeOffsetOf(domCg_param,elemSamplerRECT) );
	mea->setElementType( domFx_samplerRECT::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "samplerCUBE" );
	mea->setOffset( daeOffsetOf(domCg_param,elemSamplerCUBE) );
	mea->setElementType( domFx_samplerCUBE::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "samplerDEPTH" );
	mea->setOffset( daeOffsetOf(domCg_param,elemSamplerDEPTH) );
	mea->setElementType( domFx_samplerDEPTH::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "string" );
	mea->setOffset( daeOffsetOf(domCg_param,elemString) );
	mea->setElementType( domCg_param::domString::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "enum" );
	mea->setOffset( daeOffsetOf(domCg_param,elemEnum) );
	mea->setElementType( domCg_param::domEnum::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domCg_param,elemArray) );
	mea->setElementType( domCg_array::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "usertype" );
	mea->setOffset( daeOffsetOf(domCg_param,elemUsertype) );
	mea->setElementType( domCg_user::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domCg_param,_contents));
	meta->addContentsOrder(daeOffsetOf(domCg_param,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domCg_param,_CMData), 1);
	meta->setElementSize(sizeof(domCg_param));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool::create(DAE& dae)
{
	domCg_param::domBoolRef ref = new domCg_param::domBool(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domCg_param::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool2::create(DAE& dae)
{
	domCg_param::domBool2Ref ref = new domCg_param::domBool2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool2" );
	meta->registerClass(domCg_param::domBool2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool3::create(DAE& dae)
{
	domCg_param::domBool3Ref ref = new domCg_param::domBool3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool3" );
	meta->registerClass(domCg_param::domBool3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool4::create(DAE& dae)
{
	domCg_param::domBool4Ref ref = new domCg_param::domBool4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool4" );
	meta->registerClass(domCg_param::domBool4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool2x1::create(DAE& dae)
{
	domCg_param::domBool2x1Ref ref = new domCg_param::domBool2x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool2x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool2x1" );
	meta->registerClass(domCg_param::domBool2x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool2x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool2x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool2x2::create(DAE& dae)
{
	domCg_param::domBool2x2Ref ref = new domCg_param::domBool2x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool2x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool2x2" );
	meta->registerClass(domCg_param::domBool2x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool2x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool2x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool2x3::create(DAE& dae)
{
	domCg_param::domBool2x3Ref ref = new domCg_param::domBool2x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool2x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool2x3" );
	meta->registerClass(domCg_param::domBool2x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool2x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool2x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool2x4::create(DAE& dae)
{
	domCg_param::domBool2x4Ref ref = new domCg_param::domBool2x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool2x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool2x4" );
	meta->registerClass(domCg_param::domBool2x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool2x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool2x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool3x1::create(DAE& dae)
{
	domCg_param::domBool3x1Ref ref = new domCg_param::domBool3x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool3x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool3x1" );
	meta->registerClass(domCg_param::domBool3x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool3x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool3x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool3x2::create(DAE& dae)
{
	domCg_param::domBool3x2Ref ref = new domCg_param::domBool3x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool3x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool3x2" );
	meta->registerClass(domCg_param::domBool3x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool3x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool3x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool3x3::create(DAE& dae)
{
	domCg_param::domBool3x3Ref ref = new domCg_param::domBool3x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool3x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool3x3" );
	meta->registerClass(domCg_param::domBool3x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool3x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool3x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool3x4::create(DAE& dae)
{
	domCg_param::domBool3x4Ref ref = new domCg_param::domBool3x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool3x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool3x4" );
	meta->registerClass(domCg_param::domBool3x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool3x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool3x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool4x1::create(DAE& dae)
{
	domCg_param::domBool4x1Ref ref = new domCg_param::domBool4x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool4x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool4x1" );
	meta->registerClass(domCg_param::domBool4x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool4x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool4x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool4x2::create(DAE& dae)
{
	domCg_param::domBool4x2Ref ref = new domCg_param::domBool4x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool4x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool4x2" );
	meta->registerClass(domCg_param::domBool4x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool4x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool4x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool4x3::create(DAE& dae)
{
	domCg_param::domBool4x3Ref ref = new domCg_param::domBool4x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool4x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool4x3" );
	meta->registerClass(domCg_param::domBool4x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool4x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool4x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domBool4x4::create(DAE& dae)
{
	domCg_param::domBool4x4Ref ref = new domCg_param::domBool4x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domBool4x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool4x4" );
	meta->registerClass(domCg_param::domBool4x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domBool4x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domBool4x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat::create(DAE& dae)
{
	domCg_param::domFloatRef ref = new domCg_param::domFloat(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domCg_param::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat2::create(DAE& dae)
{
	domCg_param::domFloat2Ref ref = new domCg_param::domFloat2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2" );
	meta->registerClass(domCg_param::domFloat2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat3::create(DAE& dae)
{
	domCg_param::domFloat3Ref ref = new domCg_param::domFloat3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3" );
	meta->registerClass(domCg_param::domFloat3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat4::create(DAE& dae)
{
	domCg_param::domFloat4Ref ref = new domCg_param::domFloat4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4" );
	meta->registerClass(domCg_param::domFloat4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat2x1::create(DAE& dae)
{
	domCg_param::domFloat2x1Ref ref = new domCg_param::domFloat2x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat2x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x1" );
	meta->registerClass(domCg_param::domFloat2x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat2x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat2x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat2x2::create(DAE& dae)
{
	domCg_param::domFloat2x2Ref ref = new domCg_param::domFloat2x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat2x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x2" );
	meta->registerClass(domCg_param::domFloat2x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat2x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat2x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat2x3::create(DAE& dae)
{
	domCg_param::domFloat2x3Ref ref = new domCg_param::domFloat2x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat2x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x3" );
	meta->registerClass(domCg_param::domFloat2x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat2x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat2x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat2x4::create(DAE& dae)
{
	domCg_param::domFloat2x4Ref ref = new domCg_param::domFloat2x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat2x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x4" );
	meta->registerClass(domCg_param::domFloat2x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat2x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat2x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat3x1::create(DAE& dae)
{
	domCg_param::domFloat3x1Ref ref = new domCg_param::domFloat3x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat3x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x1" );
	meta->registerClass(domCg_param::domFloat3x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat3x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat3x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat3x2::create(DAE& dae)
{
	domCg_param::domFloat3x2Ref ref = new domCg_param::domFloat3x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat3x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x2" );
	meta->registerClass(domCg_param::domFloat3x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat3x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat3x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat3x3::create(DAE& dae)
{
	domCg_param::domFloat3x3Ref ref = new domCg_param::domFloat3x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat3x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x3" );
	meta->registerClass(domCg_param::domFloat3x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat3x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat3x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat3x4::create(DAE& dae)
{
	domCg_param::domFloat3x4Ref ref = new domCg_param::domFloat3x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat3x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x4" );
	meta->registerClass(domCg_param::domFloat3x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat3x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat3x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat4x1::create(DAE& dae)
{
	domCg_param::domFloat4x1Ref ref = new domCg_param::domFloat4x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat4x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x1" );
	meta->registerClass(domCg_param::domFloat4x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat4x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat4x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat4x2::create(DAE& dae)
{
	domCg_param::domFloat4x2Ref ref = new domCg_param::domFloat4x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat4x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x2" );
	meta->registerClass(domCg_param::domFloat4x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat4x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat4x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat4x3::create(DAE& dae)
{
	domCg_param::domFloat4x3Ref ref = new domCg_param::domFloat4x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat4x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x3" );
	meta->registerClass(domCg_param::domFloat4x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat4x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat4x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFloat4x4::create(DAE& dae)
{
	domCg_param::domFloat4x4Ref ref = new domCg_param::domFloat4x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFloat4x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x4" );
	meta->registerClass(domCg_param::domFloat4x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFloat4x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFloat4x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt::create(DAE& dae)
{
	domCg_param::domIntRef ref = new domCg_param::domInt(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domCg_param::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt2::create(DAE& dae)
{
	domCg_param::domInt2Ref ref = new domCg_param::domInt2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int2" );
	meta->registerClass(domCg_param::domInt2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt3::create(DAE& dae)
{
	domCg_param::domInt3Ref ref = new domCg_param::domInt3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int3" );
	meta->registerClass(domCg_param::domInt3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt4::create(DAE& dae)
{
	domCg_param::domInt4Ref ref = new domCg_param::domInt4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int4" );
	meta->registerClass(domCg_param::domInt4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt2x1::create(DAE& dae)
{
	domCg_param::domInt2x1Ref ref = new domCg_param::domInt2x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt2x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int2x1" );
	meta->registerClass(domCg_param::domInt2x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt2x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt2x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt2x2::create(DAE& dae)
{
	domCg_param::domInt2x2Ref ref = new domCg_param::domInt2x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt2x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int2x2" );
	meta->registerClass(domCg_param::domInt2x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt2x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt2x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt2x3::create(DAE& dae)
{
	domCg_param::domInt2x3Ref ref = new domCg_param::domInt2x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt2x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int2x3" );
	meta->registerClass(domCg_param::domInt2x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt2x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt2x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt2x4::create(DAE& dae)
{
	domCg_param::domInt2x4Ref ref = new domCg_param::domInt2x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt2x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int2x4" );
	meta->registerClass(domCg_param::domInt2x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt2x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt2x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt3x1::create(DAE& dae)
{
	domCg_param::domInt3x1Ref ref = new domCg_param::domInt3x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt3x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int3x1" );
	meta->registerClass(domCg_param::domInt3x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt3x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt3x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt3x2::create(DAE& dae)
{
	domCg_param::domInt3x2Ref ref = new domCg_param::domInt3x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt3x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int3x2" );
	meta->registerClass(domCg_param::domInt3x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt3x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt3x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt3x3::create(DAE& dae)
{
	domCg_param::domInt3x3Ref ref = new domCg_param::domInt3x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt3x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int3x3" );
	meta->registerClass(domCg_param::domInt3x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt3x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt3x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt3x4::create(DAE& dae)
{
	domCg_param::domInt3x4Ref ref = new domCg_param::domInt3x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt3x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int3x4" );
	meta->registerClass(domCg_param::domInt3x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt3x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt3x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt4x1::create(DAE& dae)
{
	domCg_param::domInt4x1Ref ref = new domCg_param::domInt4x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt4x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int4x1" );
	meta->registerClass(domCg_param::domInt4x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt4x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt4x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt4x2::create(DAE& dae)
{
	domCg_param::domInt4x2Ref ref = new domCg_param::domInt4x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt4x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int4x2" );
	meta->registerClass(domCg_param::domInt4x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt4x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt4x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt4x3::create(DAE& dae)
{
	domCg_param::domInt4x3Ref ref = new domCg_param::domInt4x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt4x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int4x3" );
	meta->registerClass(domCg_param::domInt4x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt4x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt4x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domInt4x4::create(DAE& dae)
{
	domCg_param::domInt4x4Ref ref = new domCg_param::domInt4x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domInt4x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int4x4" );
	meta->registerClass(domCg_param::domInt4x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domInt4x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domInt4x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf::create(DAE& dae)
{
	domCg_param::domHalfRef ref = new domCg_param::domHalf(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half" );
	meta->registerClass(domCg_param::domHalf::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf2::create(DAE& dae)
{
	domCg_param::domHalf2Ref ref = new domCg_param::domHalf2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half2" );
	meta->registerClass(domCg_param::domHalf2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf3::create(DAE& dae)
{
	domCg_param::domHalf3Ref ref = new domCg_param::domHalf3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half3" );
	meta->registerClass(domCg_param::domHalf3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf4::create(DAE& dae)
{
	domCg_param::domHalf4Ref ref = new domCg_param::domHalf4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half4" );
	meta->registerClass(domCg_param::domHalf4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf2x1::create(DAE& dae)
{
	domCg_param::domHalf2x1Ref ref = new domCg_param::domHalf2x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf2x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half2x1" );
	meta->registerClass(domCg_param::domHalf2x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf2x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf2x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf2x2::create(DAE& dae)
{
	domCg_param::domHalf2x2Ref ref = new domCg_param::domHalf2x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf2x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half2x2" );
	meta->registerClass(domCg_param::domHalf2x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf2x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf2x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf2x3::create(DAE& dae)
{
	domCg_param::domHalf2x3Ref ref = new domCg_param::domHalf2x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf2x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half2x3" );
	meta->registerClass(domCg_param::domHalf2x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf2x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf2x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf2x4::create(DAE& dae)
{
	domCg_param::domHalf2x4Ref ref = new domCg_param::domHalf2x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf2x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half2x4" );
	meta->registerClass(domCg_param::domHalf2x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf2x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf2x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf3x1::create(DAE& dae)
{
	domCg_param::domHalf3x1Ref ref = new domCg_param::domHalf3x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf3x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half3x1" );
	meta->registerClass(domCg_param::domHalf3x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf3x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf3x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf3x2::create(DAE& dae)
{
	domCg_param::domHalf3x2Ref ref = new domCg_param::domHalf3x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf3x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half3x2" );
	meta->registerClass(domCg_param::domHalf3x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf3x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf3x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf3x3::create(DAE& dae)
{
	domCg_param::domHalf3x3Ref ref = new domCg_param::domHalf3x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf3x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half3x3" );
	meta->registerClass(domCg_param::domHalf3x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf3x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf3x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf3x4::create(DAE& dae)
{
	domCg_param::domHalf3x4Ref ref = new domCg_param::domHalf3x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf3x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half3x4" );
	meta->registerClass(domCg_param::domHalf3x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf3x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf3x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf4x1::create(DAE& dae)
{
	domCg_param::domHalf4x1Ref ref = new domCg_param::domHalf4x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf4x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half4x1" );
	meta->registerClass(domCg_param::domHalf4x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf4x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf4x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf4x2::create(DAE& dae)
{
	domCg_param::domHalf4x2Ref ref = new domCg_param::domHalf4x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf4x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half4x2" );
	meta->registerClass(domCg_param::domHalf4x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf4x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf4x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf4x3::create(DAE& dae)
{
	domCg_param::domHalf4x3Ref ref = new domCg_param::domHalf4x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf4x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half4x3" );
	meta->registerClass(domCg_param::domHalf4x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf4x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf4x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domHalf4x4::create(DAE& dae)
{
	domCg_param::domHalf4x4Ref ref = new domCg_param::domHalf4x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domHalf4x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "half4x4" );
	meta->registerClass(domCg_param::domHalf4x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domHalf4x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domHalf4x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed::create(DAE& dae)
{
	domCg_param::domFixedRef ref = new domCg_param::domFixed(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed" );
	meta->registerClass(domCg_param::domFixed::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed2::create(DAE& dae)
{
	domCg_param::domFixed2Ref ref = new domCg_param::domFixed2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed2" );
	meta->registerClass(domCg_param::domFixed2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed3::create(DAE& dae)
{
	domCg_param::domFixed3Ref ref = new domCg_param::domFixed3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed3" );
	meta->registerClass(domCg_param::domFixed3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed4::create(DAE& dae)
{
	domCg_param::domFixed4Ref ref = new domCg_param::domFixed4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed4" );
	meta->registerClass(domCg_param::domFixed4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed2x1::create(DAE& dae)
{
	domCg_param::domFixed2x1Ref ref = new domCg_param::domFixed2x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed2x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed2x1" );
	meta->registerClass(domCg_param::domFixed2x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed2x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed2x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed2x2::create(DAE& dae)
{
	domCg_param::domFixed2x2Ref ref = new domCg_param::domFixed2x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed2x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed2x2" );
	meta->registerClass(domCg_param::domFixed2x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed2x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed2x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed2x3::create(DAE& dae)
{
	domCg_param::domFixed2x3Ref ref = new domCg_param::domFixed2x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed2x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed2x3" );
	meta->registerClass(domCg_param::domFixed2x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed2x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed2x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed2x4::create(DAE& dae)
{
	domCg_param::domFixed2x4Ref ref = new domCg_param::domFixed2x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed2x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed2x4" );
	meta->registerClass(domCg_param::domFixed2x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed2x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed2x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed3x1::create(DAE& dae)
{
	domCg_param::domFixed3x1Ref ref = new domCg_param::domFixed3x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed3x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed3x1" );
	meta->registerClass(domCg_param::domFixed3x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed3x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed3x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed3x2::create(DAE& dae)
{
	domCg_param::domFixed3x2Ref ref = new domCg_param::domFixed3x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed3x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed3x2" );
	meta->registerClass(domCg_param::domFixed3x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed3x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed3x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed3x3::create(DAE& dae)
{
	domCg_param::domFixed3x3Ref ref = new domCg_param::domFixed3x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed3x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed3x3" );
	meta->registerClass(domCg_param::domFixed3x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed3x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed3x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed3x4::create(DAE& dae)
{
	domCg_param::domFixed3x4Ref ref = new domCg_param::domFixed3x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed3x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed3x4" );
	meta->registerClass(domCg_param::domFixed3x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed3x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed3x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed4x1::create(DAE& dae)
{
	domCg_param::domFixed4x1Ref ref = new domCg_param::domFixed4x1(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed4x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed4x1" );
	meta->registerClass(domCg_param::domFixed4x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed4x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed4x1));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed4x2::create(DAE& dae)
{
	domCg_param::domFixed4x2Ref ref = new domCg_param::domFixed4x2(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed4x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed4x2" );
	meta->registerClass(domCg_param::domFixed4x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x2"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed4x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed4x2));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed4x3::create(DAE& dae)
{
	domCg_param::domFixed4x3Ref ref = new domCg_param::domFixed4x3(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed4x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed4x3" );
	meta->registerClass(domCg_param::domFixed4x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x3"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed4x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed4x3));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domFixed4x4::create(DAE& dae)
{
	domCg_param::domFixed4x4Ref ref = new domCg_param::domFixed4x4(dae);
	return ref;
}


daeMetaElement *
domCg_param::domFixed4x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fixed4x4" );
	meta->registerClass(domCg_param::domFixed4x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x4"));
		ma->setOffset( daeOffsetOf( domCg_param::domFixed4x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domFixed4x4));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domString::create(DAE& dae)
{
	domCg_param::domStringRef ref = new domCg_param::domString(dae);
	return ref;
}


daeMetaElement *
domCg_param::domString::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "string" );
	meta->registerClass(domCg_param::domString::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domCg_param::domString , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domString));
	meta->validate();

	return meta;
}

daeElementRef
domCg_param::domEnum::create(DAE& dae)
{
	domCg_param::domEnumRef ref = new domCg_param::domEnum(dae);
	return ref;
}


daeMetaElement *
domCg_param::domEnum::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "enum" );
	meta->registerClass(domCg_param::domEnum::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Gl_enumeration"));
		ma->setOffset( daeOffsetOf( domCg_param::domEnum , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_param::domEnum));
	meta->validate();

	return meta;
}

