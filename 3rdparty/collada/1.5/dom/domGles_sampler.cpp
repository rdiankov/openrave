#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_sampler.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_sampler::create(DAE& dae)
{
	domGles_samplerRef ref = new domGles_sampler(dae);
	return ref;
}


daeMetaElement *
domGles_sampler::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_sampler" );
	meta->registerClass(domGles_sampler::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "instance_image" );
	mea->setOffset( daeOffsetOf(domGles_sampler,elemInstance_image) );
	mea->setElementType( domInstance_image::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "texcoord" );
	mea->setOffset( daeOffsetOf(domGles_sampler,elemTexcoord) );
	mea->setElementType( domGles_sampler::domTexcoord::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "gles_sampler_states" );
	mea->setOffset( daeOffsetOf(domGles_sampler,elemGles_sampler_states) );
	mea->setElementType( domGles_sampler_states::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 2, 1, 1 ) );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles_sampler));
	meta->validate();

	return meta;
}

daeElementRef
domGles_sampler::domTexcoord::create(DAE& dae)
{
	domGles_sampler::domTexcoordRef ref = new domGles_sampler::domTexcoord(dae);
	return ref;
}


daeMetaElement *
domGles_sampler::domTexcoord::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "texcoord" );
	meta->registerClass(domGles_sampler::domTexcoord::create);

	meta->setIsInnerClass( true );

	//	Add attribute: semantic
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "semantic" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_sampler::domTexcoord , attrSemantic ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_sampler::domTexcoord));
	meta->validate();

	return meta;
}

