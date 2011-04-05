#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domSurface_curves.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSurface_curves::create(DAE& dae)
{
	domSurface_curvesRef ref = new domSurface_curves(dae);
	return ref;
}


daeMetaElement *
domSurface_curves::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "surface_curves" );
	meta->registerClass(domSurface_curves::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "curve" );
	mea->setOffset( daeOffsetOf(domSurface_curves,elemCurve_array) );
	mea->setElementType( domCurve::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSurface_curves,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domSurface_curves));
	meta->validate();

	return meta;
}

