#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_sampler_states.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_sampler_states::create(DAE& dae)
{
	domFx_sampler_statesRef ref = new domFx_sampler_states(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_sampler_states" );
	meta->registerClass(domFx_sampler_states::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "wrap_s" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemWrap_s) );
	mea->setElementType( domFx_sampler_states::domWrap_s::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "wrap_t" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemWrap_t) );
	mea->setElementType( domFx_sampler_states::domWrap_t::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "wrap_p" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemWrap_p) );
	mea->setElementType( domFx_sampler_states::domWrap_p::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "minfilter" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemMinfilter) );
	mea->setElementType( domFx_sampler_states::domMinfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "magfilter" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemMagfilter) );
	mea->setElementType( domFx_sampler_states::domMagfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "mipfilter" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemMipfilter) );
	mea->setElementType( domFx_sampler_states::domMipfilter::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "border_color" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemBorder_color) );
	mea->setElementType( domFx_sampler_states::domBorder_color::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 7, 0, 1 );
	mea->setName( "mip_max_level" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemMip_max_level) );
	mea->setElementType( domFx_sampler_states::domMip_max_level::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 8, 0, 1 );
	mea->setName( "mip_min_level" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemMip_min_level) );
	mea->setElementType( domFx_sampler_states::domMip_min_level::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 9, 0, 1 );
	mea->setName( "mip_bias" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemMip_bias) );
	mea->setElementType( domFx_sampler_states::domMip_bias::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 10, 0, 1 );
	mea->setName( "max_anisotropy" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemMax_anisotropy) );
	mea->setElementType( domFx_sampler_states::domMax_anisotropy::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 11, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domFx_sampler_states,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 11 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domFx_sampler_states));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domWrap_s::create(DAE& dae)
{
	domFx_sampler_states::domWrap_sRef ref = new domFx_sampler_states::domWrap_s(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domWrap_s::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "wrap_s" );
	meta->registerClass(domFx_sampler_states::domWrap_s::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_wrap"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domWrap_s , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domWrap_s));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domWrap_t::create(DAE& dae)
{
	domFx_sampler_states::domWrap_tRef ref = new domFx_sampler_states::domWrap_t(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domWrap_t::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "wrap_t" );
	meta->registerClass(domFx_sampler_states::domWrap_t::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_wrap"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domWrap_t , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domWrap_t));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domWrap_p::create(DAE& dae)
{
	domFx_sampler_states::domWrap_pRef ref = new domFx_sampler_states::domWrap_p(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domWrap_p::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "wrap_p" );
	meta->registerClass(domFx_sampler_states::domWrap_p::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_wrap"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domWrap_p , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domWrap_p));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domMinfilter::create(DAE& dae)
{
	domFx_sampler_states::domMinfilterRef ref = new domFx_sampler_states::domMinfilter(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domMinfilter::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "minfilter" );
	meta->registerClass(domFx_sampler_states::domMinfilter::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_min_filter"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domMinfilter , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domMinfilter));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domMagfilter::create(DAE& dae)
{
	domFx_sampler_states::domMagfilterRef ref = new domFx_sampler_states::domMagfilter(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domMagfilter::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "magfilter" );
	meta->registerClass(domFx_sampler_states::domMagfilter::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_mag_filter"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domMagfilter , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domMagfilter));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domMipfilter::create(DAE& dae)
{
	domFx_sampler_states::domMipfilterRef ref = new domFx_sampler_states::domMipfilter(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domMipfilter::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mipfilter" );
	meta->registerClass(domFx_sampler_states::domMipfilter::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_sampler_mip_filter"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domMipfilter , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domMipfilter));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domBorder_color::create(DAE& dae)
{
	domFx_sampler_states::domBorder_colorRef ref = new domFx_sampler_states::domBorder_color(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domBorder_color::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "border_color" );
	meta->registerClass(domFx_sampler_states::domBorder_color::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_color"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domBorder_color , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domBorder_color));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domMip_max_level::create(DAE& dae)
{
	domFx_sampler_states::domMip_max_levelRef ref = new domFx_sampler_states::domMip_max_level(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domMip_max_level::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mip_max_level" );
	meta->registerClass(domFx_sampler_states::domMip_max_level::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domMip_max_level , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domMip_max_level));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domMip_min_level::create(DAE& dae)
{
	domFx_sampler_states::domMip_min_levelRef ref = new domFx_sampler_states::domMip_min_level(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domMip_min_level::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mip_min_level" );
	meta->registerClass(domFx_sampler_states::domMip_min_level::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domMip_min_level , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domMip_min_level));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domMip_bias::create(DAE& dae)
{
	domFx_sampler_states::domMip_biasRef ref = new domFx_sampler_states::domMip_bias(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domMip_bias::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mip_bias" );
	meta->registerClass(domFx_sampler_states::domMip_bias::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domMip_bias , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domMip_bias));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sampler_states::domMax_anisotropy::create(DAE& dae)
{
	domFx_sampler_states::domMax_anisotropyRef ref = new domFx_sampler_states::domMax_anisotropy(dae);
	return ref;
}


daeMetaElement *
domFx_sampler_states::domMax_anisotropy::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "max_anisotropy" );
	meta->registerClass(domFx_sampler_states::domMax_anisotropy::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_sampler_states::domMax_anisotropy , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sampler_states::domMax_anisotropy));
	meta->validate();

	return meta;
}

