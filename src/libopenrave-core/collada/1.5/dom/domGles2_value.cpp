#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles2_value.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles2_value::create(DAE& dae)
{
	domGles2_valueRef ref = new domGles2_value(dae);
	return ref;
}


daeMetaElement *
domGles2_value::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles2_value" );
	meta->registerClass(domGles2_value::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemBool) );
	mea->setElementType( domGles2_value::domBool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bvec2" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemBvec2) );
	mea->setElementType( domGles2_value::domBvec2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bvec3" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemBvec3) );
	mea->setElementType( domGles2_value::domBvec3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bvec4" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemBvec4) );
	mea->setElementType( domGles2_value::domBvec4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemFloat) );
	mea->setElementType( domGles2_value::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "vec2" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemVec2) );
	mea->setElementType( domGles2_value::domVec2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "vec3" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemVec3) );
	mea->setElementType( domGles2_value::domVec3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "vec4" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemVec4) );
	mea->setElementType( domGles2_value::domVec4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "mat2" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemMat2) );
	mea->setElementType( domGles2_value::domMat2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "mat3" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemMat3) );
	mea->setElementType( domGles2_value::domMat3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "mat4" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemMat4) );
	mea->setElementType( domGles2_value::domMat4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemInt) );
	mea->setElementType( domGles2_value::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ivec2" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemIvec2) );
	mea->setElementType( domGles2_value::domIvec2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ivec3" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemIvec3) );
	mea->setElementType( domGles2_value::domIvec3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ivec4" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemIvec4) );
	mea->setElementType( domGles2_value::domIvec4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "enum" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemEnum) );
	mea->setElementType( domGles2_value::domEnum::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler2D" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemSampler2D) );
	mea->setElementType( domFx_sampler2D::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler3D" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemSampler3D) );
	mea->setElementType( domFx_sampler3D::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "samplerCUBE" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemSamplerCUBE) );
	mea->setElementType( domFx_samplerCUBE::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "samplerDEPTH" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemSamplerDEPTH) );
	mea->setElementType( domFx_samplerDEPTH::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "usertype" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemUsertype) );
	mea->setElementType( domGles2_value::domUsertype::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domGles2_value,elemArray) );
	mea->setElementType( domGles2_value::domArray::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGles2_value,_contents));
	meta->addContentsOrder(daeOffsetOf(domGles2_value,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGles2_value,_CMData), 1);
	meta->setElementSize(sizeof(domGles2_value));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domBool::create(DAE& dae)
{
	domGles2_value::domBoolRef ref = new domGles2_value::domBool(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domGles2_value::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_value::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domBool));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domBvec2::create(DAE& dae)
{
	domGles2_value::domBvec2Ref ref = new domGles2_value::domBvec2(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domBvec2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bvec2" );
	meta->registerClass(domGles2_value::domBvec2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool2"));
		ma->setOffset( daeOffsetOf( domGles2_value::domBvec2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domBvec2));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domBvec3::create(DAE& dae)
{
	domGles2_value::domBvec3Ref ref = new domGles2_value::domBvec3(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domBvec3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bvec3" );
	meta->registerClass(domGles2_value::domBvec3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool3"));
		ma->setOffset( daeOffsetOf( domGles2_value::domBvec3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domBvec3));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domBvec4::create(DAE& dae)
{
	domGles2_value::domBvec4Ref ref = new domGles2_value::domBvec4(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domBvec4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bvec4" );
	meta->registerClass(domGles2_value::domBvec4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Bool4"));
		ma->setOffset( daeOffsetOf( domGles2_value::domBvec4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domBvec4));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domFloat::create(DAE& dae)
{
	domGles2_value::domFloatRef ref = new domGles2_value::domFloat(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domGles2_value::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domGles2_value::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domVec2::create(DAE& dae)
{
	domGles2_value::domVec2Ref ref = new domGles2_value::domVec2(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domVec2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "vec2" );
	meta->registerClass(domGles2_value::domVec2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domGles2_value::domVec2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domVec2));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domVec3::create(DAE& dae)
{
	domGles2_value::domVec3Ref ref = new domGles2_value::domVec3(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domVec3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "vec3" );
	meta->registerClass(domGles2_value::domVec3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domGles2_value::domVec3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domVec3));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domVec4::create(DAE& dae)
{
	domGles2_value::domVec4Ref ref = new domGles2_value::domVec4(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domVec4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "vec4" );
	meta->registerClass(domGles2_value::domVec4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domGles2_value::domVec4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domVec4));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domMat2::create(DAE& dae)
{
	domGles2_value::domMat2Ref ref = new domGles2_value::domMat2(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domMat2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mat2" );
	meta->registerClass(domGles2_value::domMat2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2x2"));
		ma->setOffset( daeOffsetOf( domGles2_value::domMat2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domMat2));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domMat3::create(DAE& dae)
{
	domGles2_value::domMat3Ref ref = new domGles2_value::domMat3(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domMat3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mat3" );
	meta->registerClass(domGles2_value::domMat3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3x3"));
		ma->setOffset( daeOffsetOf( domGles2_value::domMat3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domMat3));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domMat4::create(DAE& dae)
{
	domGles2_value::domMat4Ref ref = new domGles2_value::domMat4(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domMat4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mat4" );
	meta->registerClass(domGles2_value::domMat4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4x4"));
		ma->setOffset( daeOffsetOf( domGles2_value::domMat4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domMat4));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domInt::create(DAE& dae)
{
	domGles2_value::domIntRef ref = new domGles2_value::domInt(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domGles2_value::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domGles2_value::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domIvec2::create(DAE& dae)
{
	domGles2_value::domIvec2Ref ref = new domGles2_value::domIvec2(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domIvec2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "ivec2" );
	meta->registerClass(domGles2_value::domIvec2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int2"));
		ma->setOffset( daeOffsetOf( domGles2_value::domIvec2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domIvec2));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domIvec3::create(DAE& dae)
{
	domGles2_value::domIvec3Ref ref = new domGles2_value::domIvec3(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domIvec3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "ivec3" );
	meta->registerClass(domGles2_value::domIvec3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int3"));
		ma->setOffset( daeOffsetOf( domGles2_value::domIvec3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domIvec3));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domIvec4::create(DAE& dae)
{
	domGles2_value::domIvec4Ref ref = new domGles2_value::domIvec4(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domIvec4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "ivec4" );
	meta->registerClass(domGles2_value::domIvec4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int4"));
		ma->setOffset( daeOffsetOf( domGles2_value::domIvec4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domIvec4));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domEnum::create(DAE& dae)
{
	domGles2_value::domEnumRef ref = new domGles2_value::domEnum(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domEnum::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "enum" );
	meta->registerClass(domGles2_value::domEnum::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Gl_enumeration"));
		ma->setOffset( daeOffsetOf( domGles2_value::domEnum , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domEnum));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domUsertype::create(DAE& dae)
{
	domGles2_value::domUsertypeRef ref = new domGles2_value::domUsertype(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domUsertype::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "usertype" );
	meta->registerClass(domGles2_value::domUsertype::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domGles2_value::domUsertype,elemSetparam_array) );
	mea->setElementType( domGles2_value::domUsertype::domSetparam::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: typename
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "typename" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_value::domUsertype , attrTypename ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domUsertype));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domUsertype::domSetparam::create(DAE& dae)
{
	domGles2_value::domUsertype::domSetparamRef ref = new domGles2_value::domUsertype::domSetparam(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domUsertype::domSetparam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "setparam" );
	meta->registerClass(domGles2_value::domUsertype::domSetparam::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "gles2_value" );
	mea->setOffset( daeOffsetOf(domGles2_value::domUsertype::domSetparam,elemGles2_value_array) );
	mea->setElementType( domGles2_value::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 0, -1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_value::domUsertype::domSetparam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domUsertype::domSetparam));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_value::domArray::create(DAE& dae)
{
	domGles2_value::domArrayRef ref = new domGles2_value::domArray(dae);
	return ref;
}


daeMetaElement *
domGles2_value::domArray::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "array" );
	meta->registerClass(domGles2_value::domArray::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "gles2_value" );
	mea->setOffset( daeOffsetOf(domGles2_value::domArray,elemGles2_value_array) );
	mea->setElementType( domGles2_value::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 1, -1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: length
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "length" );
		ma->setType( dae.getAtomicTypes().get("xsPositiveInteger"));
		ma->setOffset( daeOffsetOf( domGles2_value::domArray , attrLength ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_value::domArray));
	meta->validate();

	return meta;
}

