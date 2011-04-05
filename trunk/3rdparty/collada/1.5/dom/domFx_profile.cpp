#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_profile.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_profile::create(DAE& dae)
{
	domFx_profileRef ref = new domFx_profile(dae);
	return ref;
}


daeMetaElement *
domFx_profile::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_profile" );
	meta->registerClass(domFx_profile::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "profile_COMMON" );
	mea->setOffset( daeOffsetOf(domFx_profile,elemProfile_COMMON) );
	mea->setElementType( domProfile_common::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "profile_BRIDGE" );
	mea->setOffset( daeOffsetOf(domFx_profile,elemProfile_BRIDGE) );
	mea->setElementType( domProfile_bridge::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "profile_GLES2" );
	mea->setOffset( daeOffsetOf(domFx_profile,elemProfile_GLES2) );
	mea->setElementType( domProfile_gles2::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "profile_GLSL" );
	mea->setOffset( daeOffsetOf(domFx_profile,elemProfile_GLSL) );
	mea->setElementType( domProfile_glsl::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "profile_CG" );
	mea->setOffset( daeOffsetOf(domFx_profile,elemProfile_CG) );
	mea->setElementType( domProfile_cg::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "profile_GLES" );
	mea->setOffset( daeOffsetOf(domFx_profile,elemProfile_GLES) );
	mea->setElementType( domProfile_gles::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFx_profile,_contents));
	meta->addContentsOrder(daeOffsetOf(domFx_profile,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFx_profile,_CMData), 1);
	meta->setElementSize(sizeof(domFx_profile));
	meta->validate();

	return meta;
}

