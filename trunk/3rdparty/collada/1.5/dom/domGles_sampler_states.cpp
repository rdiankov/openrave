#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_sampler_states.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_sampler_states::create(DAE& dae)
{
	domGles_sampler_statesRef ref = new domGles_sampler_states(dae);
	return ref;
}


daeMetaElement *
domGles_sampler_states::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_sampler_states" );
	meta->registerClass(domGles_sampler_states::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "wrap_s" );
	mea->setOffset( daeOffsetOf(domGles_sampler_states,elemWrap_s) );
	mea->setElementType( domGles_sampler_states::domWrap_s::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "wrap_t" );
	mea->setOffset( daeOffsetOf(domGles_sampler_states,elemWrap_t) );
	mea->setElementType( domGles_sampler_states::domWrap_t::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "minfilter" );
	mea->setOffset( daeOffsetOf(domGles_sampler_states,elemMinfilter) );
	mea->setElementType( domGles_sampler_states::domMinfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "magfilter" );
	mea->setOffset( daeOffsetOf(domGles_sampler_states,elemMagfilter) );
	mea->setElementType( domGles_sampler_states::domMagfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "mipfilter" );
	mea->setOffset( daeOffsetOf(domGles_sampler_states,elemMipfilter) );
	mea->setElementType( domGles_sampler_states::domMipfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "mip_max_level" );
	mea->setOffset( daeOffsetOf(domGles_sampler_states,elemMip_max_level) );
	mea->setElementType( domGles_sampler_states::domMip_max_level::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "mip_bias" );
	mea->setOffset( daeOffsetOf(domGles_sampler_states,elemMip_bias) );
	mea->setElementType( domGles_sampler_states::domMip_bias::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 7, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domGles_sampler_states,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 7 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles_sampler_states));
	meta->validate();

	return meta;
}

daeElementRef
domGles_sampler_states::domWrap_s::create(DAE& dae)
{
	domGles_sampler_states::domWrap_sRef ref = new domGles_sampler_states::domWrap_s(dae);
	return ref;
}


daeMetaElement *
domGles_sampler_states::domWrap_s::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "wrap_s" );
	meta->registerClass(domGles_sampler_states::domWrap_s::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Gles_sampler_wrap"));
		ma->setOffset( daeOffsetOf( domGles_sampler_states::domWrap_s , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_sampler_states::domWrap_s));
	meta->validate();

	return meta;
}

daeElementRef
domGles_sampler_states::domWrap_t::create(DAE& dae)
{
	domGles_sampler_states::domWrap_tRef ref = new domGles_sampler_states::domWrap_t(dae);
	return ref;
}


daeMetaElement *
domGles_sampler_states::domWrap_t::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "wrap_t" );
	meta->registerClass(domGles_sampler_states::domWrap_t::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Gles_sampler_wrap"));
		ma->setOffset( daeOffsetOf( domGles_sampler_states::domWrap_t , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_sampler_states::domWrap_t));
	meta->validate();

	return meta;
}

daeElementRef
domGles_sampler_states::domMinfilter::create(DAE& dae)
{
	domGles_sampler_states::domMinfilterRef ref = new domGles_sampler_states::domMinfilter(dae);
	return ref;
}


daeMetaElement *
domGles_sampler_states::domMinfilter::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "minfilter" );
	meta->registerClass(domGles_sampler_states::domMinfilter::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_min_filter"));
		ma->setOffset( daeOffsetOf( domGles_sampler_states::domMinfilter , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_sampler_states::domMinfilter));
	meta->validate();

	return meta;
}

daeElementRef
domGles_sampler_states::domMagfilter::create(DAE& dae)
{
	domGles_sampler_states::domMagfilterRef ref = new domGles_sampler_states::domMagfilter(dae);
	return ref;
}


daeMetaElement *
domGles_sampler_states::domMagfilter::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "magfilter" );
	meta->registerClass(domGles_sampler_states::domMagfilter::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_mag_filter"));
		ma->setOffset( daeOffsetOf( domGles_sampler_states::domMagfilter , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_sampler_states::domMagfilter));
	meta->validate();

	return meta;
}

daeElementRef
domGles_sampler_states::domMipfilter::create(DAE& dae)
{
	domGles_sampler_states::domMipfilterRef ref = new domGles_sampler_states::domMipfilter(dae);
	return ref;
}


daeMetaElement *
domGles_sampler_states::domMipfilter::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mipfilter" );
	meta->registerClass(domGles_sampler_states::domMipfilter::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_mip_filter"));
		ma->setOffset( daeOffsetOf( domGles_sampler_states::domMipfilter , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_sampler_states::domMipfilter));
	meta->validate();

	return meta;
}

daeElementRef
domGles_sampler_states::domMip_max_level::create(DAE& dae)
{
	domGles_sampler_states::domMip_max_levelRef ref = new domGles_sampler_states::domMip_max_level(dae);
	return ref;
}


daeMetaElement *
domGles_sampler_states::domMip_max_level::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mip_max_level" );
	meta->registerClass(domGles_sampler_states::domMip_max_level::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles_sampler_states::domMip_max_level , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_sampler_states::domMip_max_level));
	meta->validate();

	return meta;
}

daeElementRef
domGles_sampler_states::domMip_bias::create(DAE& dae)
{
	domGles_sampler_states::domMip_biasRef ref = new domGles_sampler_states::domMip_bias(dae);
	return ref;
}


daeMetaElement *
domGles_sampler_states::domMip_bias::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mip_bias" );
	meta->registerClass(domGles_sampler_states::domMip_bias::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domGles_sampler_states::domMip_bias , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_sampler_states::domMip_bias));
	meta->validate();

	return meta;
}

