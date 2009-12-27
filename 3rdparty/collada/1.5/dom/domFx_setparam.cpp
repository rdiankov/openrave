#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_setparam.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_setparam::create(DAE& dae)
{
	domFx_setparamRef ref = new domFx_setparam(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_setparam" );
	meta->registerClass(domFx_setparam::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemBool) );
	mea->setElementType( domFx_setparam::domBool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool2" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemBool2) );
	mea->setElementType( domFx_setparam::domBool2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool3" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemBool3) );
	mea->setElementType( domFx_setparam::domBool3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool4" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemBool4) );
	mea->setElementType( domFx_setparam::domBool4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemInt) );
	mea->setElementType( domFx_setparam::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int2" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemInt2) );
	mea->setElementType( domFx_setparam::domInt2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int3" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemInt3) );
	mea->setElementType( domFx_setparam::domInt3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int4" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemInt4) );
	mea->setElementType( domFx_setparam::domInt4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat) );
	mea->setElementType( domFx_setparam::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat2) );
	mea->setElementType( domFx_setparam::domFloat2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat3) );
	mea->setElementType( domFx_setparam::domFloat3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat4) );
	mea->setElementType( domFx_setparam::domFloat4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x1" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat2x1) );
	mea->setElementType( domFx_setparam::domFloat2x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x2" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat2x2) );
	mea->setElementType( domFx_setparam::domFloat2x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x3" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat2x3) );
	mea->setElementType( domFx_setparam::domFloat2x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x4" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat2x4) );
	mea->setElementType( domFx_setparam::domFloat2x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x1" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat3x1) );
	mea->setElementType( domFx_setparam::domFloat3x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x2" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat3x2) );
	mea->setElementType( domFx_setparam::domFloat3x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x3" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat3x3) );
	mea->setElementType( domFx_setparam::domFloat3x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x4" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat3x4) );
	mea->setElementType( domFx_setparam::domFloat3x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x1" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat4x1) );
	mea->setElementType( domFx_setparam::domFloat4x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x2" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat4x2) );
	mea->setElementType( domFx_setparam::domFloat4x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x3" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat4x3) );
	mea->setElementType( domFx_setparam::domFloat4x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x4" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemFloat4x4) );
	mea->setElementType( domFx_setparam::domFloat4x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "enum" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemEnum) );
	mea->setElementType( domFx_setparam::domEnum::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler_image" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemSampler_image) );
	mea->setElementType( domInstance_image::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler_states" );
	mea->setOffset( daeOffsetOf(domFx_setparam,elemSampler_states) );
	mea->setElementType( domFx_setparam::domSampler_states::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFx_setparam,_contents));
	meta->addContentsOrder(daeOffsetOf(domFx_setparam,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFx_setparam,_CMData), 1);
	meta->setElementSize(sizeof(domFx_setparam));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domBool::create(DAE& dae)
{
	domFx_setparam::domBoolRef ref = new domFx_setparam::domBool(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domFx_setparam::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domBool));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domBool2::create(DAE& dae)
{
	domFx_setparam::domBool2Ref ref = new domFx_setparam::domBool2(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domBool2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool2" );
	meta->registerClass(domFx_setparam::domBool2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domBool2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domBool2));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domBool3::create(DAE& dae)
{
	domFx_setparam::domBool3Ref ref = new domFx_setparam::domBool3(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domBool3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool3" );
	meta->registerClass(domFx_setparam::domBool3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domBool3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domBool3));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domBool4::create(DAE& dae)
{
	domFx_setparam::domBool4Ref ref = new domFx_setparam::domBool4(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domBool4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool4" );
	meta->registerClass(domFx_setparam::domBool4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domBool4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domBool4));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domInt::create(DAE& dae)
{
	domFx_setparam::domIntRef ref = new domFx_setparam::domInt(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domFx_setparam::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domInt2::create(DAE& dae)
{
	domFx_setparam::domInt2Ref ref = new domFx_setparam::domInt2(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domInt2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int2" );
	meta->registerClass(domFx_setparam::domInt2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domInt2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domInt2));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domInt3::create(DAE& dae)
{
	domFx_setparam::domInt3Ref ref = new domFx_setparam::domInt3(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domInt3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int3" );
	meta->registerClass(domFx_setparam::domInt3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domInt3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domInt3));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domInt4::create(DAE& dae)
{
	domFx_setparam::domInt4Ref ref = new domFx_setparam::domInt4(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domInt4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int4" );
	meta->registerClass(domFx_setparam::domInt4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domInt4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domInt4));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat::create(DAE& dae)
{
	domFx_setparam::domFloatRef ref = new domFx_setparam::domFloat(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domFx_setparam::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat2::create(DAE& dae)
{
	domFx_setparam::domFloat2Ref ref = new domFx_setparam::domFloat2(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2" );
	meta->registerClass(domFx_setparam::domFloat2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat2));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat3::create(DAE& dae)
{
	domFx_setparam::domFloat3Ref ref = new domFx_setparam::domFloat3(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3" );
	meta->registerClass(domFx_setparam::domFloat3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat3));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat4::create(DAE& dae)
{
	domFx_setparam::domFloat4Ref ref = new domFx_setparam::domFloat4(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4" );
	meta->registerClass(domFx_setparam::domFloat4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat4));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat2x1::create(DAE& dae)
{
	domFx_setparam::domFloat2x1Ref ref = new domFx_setparam::domFloat2x1(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat2x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x1" );
	meta->registerClass(domFx_setparam::domFloat2x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat2x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat2x1));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat2x2::create(DAE& dae)
{
	domFx_setparam::domFloat2x2Ref ref = new domFx_setparam::domFloat2x2(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat2x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x2" );
	meta->registerClass(domFx_setparam::domFloat2x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x2"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat2x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat2x2));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat2x3::create(DAE& dae)
{
	domFx_setparam::domFloat2x3Ref ref = new domFx_setparam::domFloat2x3(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat2x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x3" );
	meta->registerClass(domFx_setparam::domFloat2x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x3"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat2x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat2x3));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat2x4::create(DAE& dae)
{
	domFx_setparam::domFloat2x4Ref ref = new domFx_setparam::domFloat2x4(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat2x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x4" );
	meta->registerClass(domFx_setparam::domFloat2x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x4"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat2x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat2x4));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat3x1::create(DAE& dae)
{
	domFx_setparam::domFloat3x1Ref ref = new domFx_setparam::domFloat3x1(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat3x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x1" );
	meta->registerClass(domFx_setparam::domFloat3x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat3x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat3x1));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat3x2::create(DAE& dae)
{
	domFx_setparam::domFloat3x2Ref ref = new domFx_setparam::domFloat3x2(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat3x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x2" );
	meta->registerClass(domFx_setparam::domFloat3x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x2"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat3x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat3x2));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat3x3::create(DAE& dae)
{
	domFx_setparam::domFloat3x3Ref ref = new domFx_setparam::domFloat3x3(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat3x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x3" );
	meta->registerClass(domFx_setparam::domFloat3x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x3"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat3x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat3x3));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat3x4::create(DAE& dae)
{
	domFx_setparam::domFloat3x4Ref ref = new domFx_setparam::domFloat3x4(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat3x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x4" );
	meta->registerClass(domFx_setparam::domFloat3x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x4"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat3x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat3x4));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat4x1::create(DAE& dae)
{
	domFx_setparam::domFloat4x1Ref ref = new domFx_setparam::domFloat4x1(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat4x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x1" );
	meta->registerClass(domFx_setparam::domFloat4x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat4x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat4x1));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat4x2::create(DAE& dae)
{
	domFx_setparam::domFloat4x2Ref ref = new domFx_setparam::domFloat4x2(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat4x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x2" );
	meta->registerClass(domFx_setparam::domFloat4x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x2"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat4x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat4x2));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat4x3::create(DAE& dae)
{
	domFx_setparam::domFloat4x3Ref ref = new domFx_setparam::domFloat4x3(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat4x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x3" );
	meta->registerClass(domFx_setparam::domFloat4x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x3"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat4x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat4x3));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domFloat4x4::create(DAE& dae)
{
	domFx_setparam::domFloat4x4Ref ref = new domFx_setparam::domFloat4x4(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domFloat4x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x4" );
	meta->registerClass(domFx_setparam::domFloat4x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x4"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domFloat4x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domFloat4x4));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domEnum::create(DAE& dae)
{
	domFx_setparam::domEnumRef ref = new domFx_setparam::domEnum(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domEnum::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "enum" );
	meta->registerClass(domFx_setparam::domEnum::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domFx_setparam::domEnum , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_setparam::domEnum));
	meta->validate();

	return meta;
}

daeElementRef
domFx_setparam::domSampler_states::create(DAE& dae)
{
	domFx_setparam::domSampler_statesRef ref = new domFx_setparam::domSampler_states(dae);
	return ref;
}


daeMetaElement *
domFx_setparam::domSampler_states::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "sampler_states" );
	meta->registerClass(domFx_setparam::domSampler_states::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fx_sampler_states" );
	mea->setOffset( daeOffsetOf(domFx_setparam::domSampler_states,elemFx_sampler_states) );
	mea->setElementType( domFx_sampler_states::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domFx_setparam::domSampler_states));
	meta->validate();

	return meta;
}

