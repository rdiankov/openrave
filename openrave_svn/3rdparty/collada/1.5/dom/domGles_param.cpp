#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_param.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_param::create(DAE& dae)
{
	domGles_paramRef ref = new domGles_param(dae);
	return ref;
}


daeMetaElement *
domGles_param::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_param" );
	meta->registerClass(domGles_param::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domGles_param,elemBool) );
	mea->setElementType( domGles_param::domBool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool2" );
	mea->setOffset( daeOffsetOf(domGles_param,elemBool2) );
	mea->setElementType( domGles_param::domBool2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool3" );
	mea->setOffset( daeOffsetOf(domGles_param,elemBool3) );
	mea->setElementType( domGles_param::domBool3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool4" );
	mea->setOffset( daeOffsetOf(domGles_param,elemBool4) );
	mea->setElementType( domGles_param::domBool4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domGles_param,elemInt) );
	mea->setElementType( domGles_param::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int2" );
	mea->setOffset( daeOffsetOf(domGles_param,elemInt2) );
	mea->setElementType( domGles_param::domInt2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int3" );
	mea->setOffset( daeOffsetOf(domGles_param,elemInt3) );
	mea->setElementType( domGles_param::domInt3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int4" );
	mea->setOffset( daeOffsetOf(domGles_param,elemInt4) );
	mea->setElementType( domGles_param::domInt4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat) );
	mea->setElementType( domGles_param::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat2) );
	mea->setElementType( domGles_param::domFloat2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat3) );
	mea->setElementType( domGles_param::domFloat3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat4) );
	mea->setElementType( domGles_param::domFloat4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float1x1" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat1x1) );
	mea->setElementType( domGles_param::domFloat1x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float1x2" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat1x2) );
	mea->setElementType( domGles_param::domFloat1x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float1x3" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat1x3) );
	mea->setElementType( domGles_param::domFloat1x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float1x4" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat1x4) );
	mea->setElementType( domGles_param::domFloat1x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x1" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat2x1) );
	mea->setElementType( domGles_param::domFloat2x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x2" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat2x2) );
	mea->setElementType( domGles_param::domFloat2x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x3" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat2x3) );
	mea->setElementType( domGles_param::domFloat2x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2x4" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat2x4) );
	mea->setElementType( domGles_param::domFloat2x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x1" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat3x1) );
	mea->setElementType( domGles_param::domFloat3x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x2" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat3x2) );
	mea->setElementType( domGles_param::domFloat3x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x3" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat3x3) );
	mea->setElementType( domGles_param::domFloat3x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3x4" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat3x4) );
	mea->setElementType( domGles_param::domFloat3x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x1" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat4x1) );
	mea->setElementType( domGles_param::domFloat4x1::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x2" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat4x2) );
	mea->setElementType( domGles_param::domFloat4x2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x3" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat4x3) );
	mea->setElementType( domGles_param::domFloat4x3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4x4" );
	mea->setOffset( daeOffsetOf(domGles_param,elemFloat4x4) );
	mea->setElementType( domGles_param::domFloat4x4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler2D" );
	mea->setOffset( daeOffsetOf(domGles_param,elemSampler2D) );
	mea->setElementType( domGles_sampler::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "enum" );
	mea->setOffset( daeOffsetOf(domGles_param,elemEnum) );
	mea->setElementType( domGles_param::domEnum::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGles_param,_contents));
	meta->addContentsOrder(daeOffsetOf(domGles_param,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGles_param,_CMData), 1);
	meta->setElementSize(sizeof(domGles_param));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domBool::create(DAE& dae)
{
	domGles_param::domBoolRef ref = new domGles_param::domBool(dae);
	return ref;
}


daeMetaElement *
domGles_param::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domGles_param::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles_param::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domBool));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domBool2::create(DAE& dae)
{
	domGles_param::domBool2Ref ref = new domGles_param::domBool2(dae);
	return ref;
}


daeMetaElement *
domGles_param::domBool2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool2" );
	meta->registerClass(domGles_param::domBool2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2"));
		ma->setOffset( daeOffsetOf( domGles_param::domBool2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domBool2));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domBool3::create(DAE& dae)
{
	domGles_param::domBool3Ref ref = new domGles_param::domBool3(dae);
	return ref;
}


daeMetaElement *
domGles_param::domBool3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool3" );
	meta->registerClass(domGles_param::domBool3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3"));
		ma->setOffset( daeOffsetOf( domGles_param::domBool3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domBool3));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domBool4::create(DAE& dae)
{
	domGles_param::domBool4Ref ref = new domGles_param::domBool4(dae);
	return ref;
}


daeMetaElement *
domGles_param::domBool4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool4" );
	meta->registerClass(domGles_param::domBool4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4"));
		ma->setOffset( daeOffsetOf( domGles_param::domBool4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domBool4));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domInt::create(DAE& dae)
{
	domGles_param::domIntRef ref = new domGles_param::domInt(dae);
	return ref;
}


daeMetaElement *
domGles_param::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domGles_param::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domGles_param::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domInt2::create(DAE& dae)
{
	domGles_param::domInt2Ref ref = new domGles_param::domInt2(dae);
	return ref;
}


daeMetaElement *
domGles_param::domInt2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int2" );
	meta->registerClass(domGles_param::domInt2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2"));
		ma->setOffset( daeOffsetOf( domGles_param::domInt2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domInt2));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domInt3::create(DAE& dae)
{
	domGles_param::domInt3Ref ref = new domGles_param::domInt3(dae);
	return ref;
}


daeMetaElement *
domGles_param::domInt3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int3" );
	meta->registerClass(domGles_param::domInt3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3"));
		ma->setOffset( daeOffsetOf( domGles_param::domInt3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domInt3));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domInt4::create(DAE& dae)
{
	domGles_param::domInt4Ref ref = new domGles_param::domInt4(dae);
	return ref;
}


daeMetaElement *
domGles_param::domInt4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int4" );
	meta->registerClass(domGles_param::domInt4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4"));
		ma->setOffset( daeOffsetOf( domGles_param::domInt4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domInt4));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat::create(DAE& dae)
{
	domGles_param::domFloatRef ref = new domGles_param::domFloat(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domGles_param::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat2::create(DAE& dae)
{
	domGles_param::domFloat2Ref ref = new domGles_param::domFloat2(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2" );
	meta->registerClass(domGles_param::domFloat2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat2));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat3::create(DAE& dae)
{
	domGles_param::domFloat3Ref ref = new domGles_param::domFloat3(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3" );
	meta->registerClass(domGles_param::domFloat3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat3));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat4::create(DAE& dae)
{
	domGles_param::domFloat4Ref ref = new domGles_param::domFloat4(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4" );
	meta->registerClass(domGles_param::domFloat4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat4));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat1x1::create(DAE& dae)
{
	domGles_param::domFloat1x1Ref ref = new domGles_param::domFloat1x1(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat1x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float1x1" );
	meta->registerClass(domGles_param::domFloat1x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat1x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat1x1));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat1x2::create(DAE& dae)
{
	domGles_param::domFloat1x2Ref ref = new domGles_param::domFloat1x2(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat1x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float1x2" );
	meta->registerClass(domGles_param::domFloat1x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat1x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat1x2));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat1x3::create(DAE& dae)
{
	domGles_param::domFloat1x3Ref ref = new domGles_param::domFloat1x3(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat1x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float1x3" );
	meta->registerClass(domGles_param::domFloat1x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat1x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat1x3));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat1x4::create(DAE& dae)
{
	domGles_param::domFloat1x4Ref ref = new domGles_param::domFloat1x4(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat1x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float1x4" );
	meta->registerClass(domGles_param::domFloat1x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat1x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat1x4));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat2x1::create(DAE& dae)
{
	domGles_param::domFloat2x1Ref ref = new domGles_param::domFloat2x1(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat2x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x1" );
	meta->registerClass(domGles_param::domFloat2x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat2x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat2x1));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat2x2::create(DAE& dae)
{
	domGles_param::domFloat2x2Ref ref = new domGles_param::domFloat2x2(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat2x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x2" );
	meta->registerClass(domGles_param::domFloat2x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x2"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat2x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat2x2));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat2x3::create(DAE& dae)
{
	domGles_param::domFloat2x3Ref ref = new domGles_param::domFloat2x3(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat2x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x3" );
	meta->registerClass(domGles_param::domFloat2x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x3"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat2x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat2x3));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat2x4::create(DAE& dae)
{
	domGles_param::domFloat2x4Ref ref = new domGles_param::domFloat2x4(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat2x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2x4" );
	meta->registerClass(domGles_param::domFloat2x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x4"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat2x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat2x4));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat3x1::create(DAE& dae)
{
	domGles_param::domFloat3x1Ref ref = new domGles_param::domFloat3x1(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat3x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x1" );
	meta->registerClass(domGles_param::domFloat3x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat3x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat3x1));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat3x2::create(DAE& dae)
{
	domGles_param::domFloat3x2Ref ref = new domGles_param::domFloat3x2(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat3x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x2" );
	meta->registerClass(domGles_param::domFloat3x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x2"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat3x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat3x2));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat3x3::create(DAE& dae)
{
	domGles_param::domFloat3x3Ref ref = new domGles_param::domFloat3x3(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat3x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x3" );
	meta->registerClass(domGles_param::domFloat3x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x3"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat3x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat3x3));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat3x4::create(DAE& dae)
{
	domGles_param::domFloat3x4Ref ref = new domGles_param::domFloat3x4(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat3x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3x4" );
	meta->registerClass(domGles_param::domFloat3x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x4"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat3x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat3x4));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat4x1::create(DAE& dae)
{
	domGles_param::domFloat4x1Ref ref = new domGles_param::domFloat4x1(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat4x1::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x1" );
	meta->registerClass(domGles_param::domFloat4x1::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat4x1 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat4x1));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat4x2::create(DAE& dae)
{
	domGles_param::domFloat4x2Ref ref = new domGles_param::domFloat4x2(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat4x2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x2" );
	meta->registerClass(domGles_param::domFloat4x2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x2"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat4x2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat4x2));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat4x3::create(DAE& dae)
{
	domGles_param::domFloat4x3Ref ref = new domGles_param::domFloat4x3(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat4x3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x3" );
	meta->registerClass(domGles_param::domFloat4x3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x3"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat4x3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat4x3));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domFloat4x4::create(DAE& dae)
{
	domGles_param::domFloat4x4Ref ref = new domGles_param::domFloat4x4(dae);
	return ref;
}


daeMetaElement *
domGles_param::domFloat4x4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4x4" );
	meta->registerClass(domGles_param::domFloat4x4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x4"));
		ma->setOffset( daeOffsetOf( domGles_param::domFloat4x4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domFloat4x4));
	meta->validate();

	return meta;
}

daeElementRef
domGles_param::domEnum::create(DAE& dae)
{
	domGles_param::domEnumRef ref = new domGles_param::domEnum(dae);
	return ref;
}


daeMetaElement *
domGles_param::domEnum::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "enum" );
	meta->registerClass(domGles_param::domEnum::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Gles_enumeration"));
		ma->setOffset( daeOffsetOf( domGles_param::domEnum , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_param::domEnum));
	meta->validate();

	return meta;
}

