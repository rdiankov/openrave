#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_common_newparam.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_common_newparam::create(DAE& dae)
{
	domFx_common_newparamRef ref = new domFx_common_newparam(dae);
	return ref;
}


daeMetaElement *
domFx_common_newparam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_common_newparam" );
	meta->registerClass(domFx_common_newparam::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "semantic" );
	mea->setOffset( daeOffsetOf(domFx_common_newparam,elemSemantic) );
	mea->setElementType( domFx_common_newparam::domSemantic::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaChoice( meta, cm, 0, 1, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domFx_common_newparam,elemFloat) );
	mea->setElementType( domFx_common_newparam::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float2" );
	mea->setOffset( daeOffsetOf(domFx_common_newparam,elemFloat2) );
	mea->setElementType( domFx_common_newparam::domFloat2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float3" );
	mea->setOffset( daeOffsetOf(domFx_common_newparam,elemFloat3) );
	mea->setElementType( domFx_common_newparam::domFloat3::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float4" );
	mea->setOffset( daeOffsetOf(domFx_common_newparam,elemFloat4) );
	mea->setElementType( domFx_common_newparam::domFloat4::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sampler2D" );
	mea->setOffset( daeOffsetOf(domFx_common_newparam,elemSampler2D) );
	mea->setElementType( domFx_sampler2D::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFx_common_newparam,_contents));
	meta->addContentsOrder(daeOffsetOf(domFx_common_newparam,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFx_common_newparam,_CMData), 1);
	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domFx_common_newparam , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_newparam));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_newparam::domSemantic::create(DAE& dae)
{
	domFx_common_newparam::domSemanticRef ref = new domFx_common_newparam::domSemantic(dae);
	return ref;
}


daeMetaElement *
domFx_common_newparam::domSemantic::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "semantic" );
	meta->registerClass(domFx_common_newparam::domSemantic::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_common_newparam::domSemantic , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_newparam::domSemantic));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_newparam::domFloat::create(DAE& dae)
{
	domFx_common_newparam::domFloatRef ref = new domFx_common_newparam::domFloat(dae);
	return ref;
}


daeMetaElement *
domFx_common_newparam::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domFx_common_newparam::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domFx_common_newparam::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_newparam::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_newparam::domFloat2::create(DAE& dae)
{
	domFx_common_newparam::domFloat2Ref ref = new domFx_common_newparam::domFloat2(dae);
	return ref;
}


daeMetaElement *
domFx_common_newparam::domFloat2::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float2" );
	meta->registerClass(domFx_common_newparam::domFloat2::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domFx_common_newparam::domFloat2 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_newparam::domFloat2));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_newparam::domFloat3::create(DAE& dae)
{
	domFx_common_newparam::domFloat3Ref ref = new domFx_common_newparam::domFloat3(dae);
	return ref;
}


daeMetaElement *
domFx_common_newparam::domFloat3::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float3" );
	meta->registerClass(domFx_common_newparam::domFloat3::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domFx_common_newparam::domFloat3 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_newparam::domFloat3));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_newparam::domFloat4::create(DAE& dae)
{
	domFx_common_newparam::domFloat4Ref ref = new domFx_common_newparam::domFloat4(dae);
	return ref;
}


daeMetaElement *
domFx_common_newparam::domFloat4::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float4" );
	meta->registerClass(domFx_common_newparam::domFloat4::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domFx_common_newparam::domFloat4 , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_newparam::domFloat4));
	meta->validate();

	return meta;
}

