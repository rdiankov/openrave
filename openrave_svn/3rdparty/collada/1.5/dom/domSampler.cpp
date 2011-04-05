#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domSampler.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSampler::create(DAE& dae)
{
	domSamplerRef ref = new domSampler(dae);
	return ref;
}


daeMetaElement *
domSampler::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "sampler" );
	meta->registerClass(domSampler::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domSampler,elemInput_array) );
	mea->setElementType( domInput_local::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domSampler , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: pre_behavior
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "pre_behavior" );
		ma->setType( dae.getAtomicTypes().get("Sampler_behavior"));
		ma->setOffset( daeOffsetOf( domSampler , attrPre_behavior ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: post_behavior
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "post_behavior" );
		ma->setType( dae.getAtomicTypes().get("Sampler_behavior"));
		ma->setOffset( daeOffsetOf( domSampler , attrPost_behavior ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domSampler));
	meta->validate();

	return meta;
}

