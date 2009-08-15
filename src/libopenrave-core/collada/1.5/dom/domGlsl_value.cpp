#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGlsl_value.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGlsl_value::create(DAE& dae)
{
	domGlsl_valueRef ref = new domGlsl_value(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "glsl_value" );
	meta->registerClass(domGlsl_value::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemBool) );
	mea->setElementType( domGlsl_value::domBool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool2" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemBool2) );
	mea->setElementType( domGlsl_value::domBool2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool3" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemBool3) );
	mea->setElementType( domGlsl_value::domBool3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool4" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemBool4) );
	mea->setElementType( domGlsl_value::domBool4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemFloat) );
	mea->setElementType( domGlsl_value::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemFloat2) );
	mea->setElementType( domGlsl_value::domFloat2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemFloat3) );
	mea->setElementType( domGlsl_value::domFloat3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemFloat4) );
	mea->setElementType( domGlsl_value::domFloat4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x2" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemFloat2x2) );
	mea->setElementType( domGlsl_value::domFloat2x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x3" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemFloat3x3) );
	mea->setElementType( domGlsl_value::domFloat3x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x4" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemFloat4x4) );
	mea->setElementType( domGlsl_value::domFloat4x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemInt) );
	mea->setElementType( domGlsl_value::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int2" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemInt2) );
	mea->setElementType( domGlsl_value::domInt2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int3" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemInt3) );
	mea->setElementType( domGlsl_value::domInt3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int4" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemInt4) );
	mea->setElementType( domGlsl_value::domInt4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler1D" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemSampler1D) );
	mea->setElementType( domFx_sampler1D::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler2D" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemSampler2D) );
	mea->setElementType( domFx_sampler2D::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler3D" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemSampler3D) );
	mea->setElementType( domFx_sampler3D::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "samplerCUBE" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemSamplerCUBE) );
	mea->setElementType( domFx_samplerCUBE::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "samplerRECT" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemSamplerRECT) );
	mea->setElementType( domFx_samplerRECT::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "samplerDEPTH" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemSamplerDEPTH) );
	mea->setElementType( domFx_samplerDEPTH::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "enum" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemEnum) );
	mea->setElementType( domGlsl_value::domEnum::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domGlsl_value,elemArray) );
	mea->setElementType( domGlsl_array::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGlsl_value,_contents));
	meta->addContentsOrder(daeOffsetOf(domGlsl_value,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGlsl_value,_CMData), 1);
	meta->setElementSize(sizeof(domGlsl_value));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domBool::create(DAE& dae)
{
	domGlsl_value::domBoolRef ref = new domGlsl_value::domBool(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domGlsl_value::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domBool));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domBool2::create(DAE& dae)
{
	domGlsl_value::domBool2Ref ref = new domGlsl_value::domBool2(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domBool2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool2" );
	meta->registerClass(domGlsl_value::domBool2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domBool2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domBool2));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domBool3::create(DAE& dae)
{
	domGlsl_value::domBool3Ref ref = new domGlsl_value::domBool3(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domBool3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool3" );
	meta->registerClass(domGlsl_value::domBool3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domBool3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domBool3));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domBool4::create(DAE& dae)
{
	domGlsl_value::domBool4Ref ref = new domGlsl_value::domBool4(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domBool4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool4" );
	meta->registerClass(domGlsl_value::domBool4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domBool4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domBool4));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domFloat::create(DAE& dae)
{
	domGlsl_value::domFloatRef ref = new domGlsl_value::domFloat(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domGlsl_value::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domFloat2::create(DAE& dae)
{
	domGlsl_value::domFloat2Ref ref = new domGlsl_value::domFloat2(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domFloat2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2" );
	meta->registerClass(domGlsl_value::domFloat2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domFloat2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domFloat2));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domFloat3::create(DAE& dae)
{
	domGlsl_value::domFloat3Ref ref = new domGlsl_value::domFloat3(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domFloat3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3" );
	meta->registerClass(domGlsl_value::domFloat3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domFloat3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domFloat3));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domFloat4::create(DAE& dae)
{
	domGlsl_value::domFloat4Ref ref = new domGlsl_value::domFloat4(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domFloat4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4" );
	meta->registerClass(domGlsl_value::domFloat4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domFloat4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domFloat4));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domFloat2x2::create(DAE& dae)
{
	domGlsl_value::domFloat2x2Ref ref = new domGlsl_value::domFloat2x2(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domFloat2x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x2" );
	meta->registerClass(domGlsl_value::domFloat2x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x2"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domFloat2x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domFloat2x2));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domFloat3x3::create(DAE& dae)
{
	domGlsl_value::domFloat3x3Ref ref = new domGlsl_value::domFloat3x3(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domFloat3x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x3" );
	meta->registerClass(domGlsl_value::domFloat3x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x3"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domFloat3x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domFloat3x3));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domFloat4x4::create(DAE& dae)
{
	domGlsl_value::domFloat4x4Ref ref = new domGlsl_value::domFloat4x4(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domFloat4x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x4" );
	meta->registerClass(domGlsl_value::domFloat4x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x4"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domFloat4x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domFloat4x4));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domInt::create(DAE& dae)
{
	domGlsl_value::domIntRef ref = new domGlsl_value::domInt(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domGlsl_value::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domInt2::create(DAE& dae)
{
	domGlsl_value::domInt2Ref ref = new domGlsl_value::domInt2(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domInt2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int2" );
	meta->registerClass(domGlsl_value::domInt2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domInt2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domInt2));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domInt3::create(DAE& dae)
{
	domGlsl_value::domInt3Ref ref = new domGlsl_value::domInt3(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domInt3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int3" );
	meta->registerClass(domGlsl_value::domInt3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domInt3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domInt3));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domInt4::create(DAE& dae)
{
	domGlsl_value::domInt4Ref ref = new domGlsl_value::domInt4(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domInt4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int4" );
	meta->registerClass(domGlsl_value::domInt4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domInt4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domInt4));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_value::domEnum::create(DAE& dae)
{
	domGlsl_value::domEnumRef ref = new domGlsl_value::domEnum(dae);
	return ref;
}


daeMetaElement *
domGlsl_value::domEnum::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "enum" );
	meta->registerClass(domGlsl_value::domEnum::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Gl_enumeration"));
		ma->setOffset( daeOffsetOf( domGlsl_value::domEnum , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_value::domEnum));
	meta->validate();

	return meta;
}

