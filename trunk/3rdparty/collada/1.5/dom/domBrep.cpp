#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domBrep.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domBrep::create(DAE& dae)
{
	domBrepRef ref = new domBrep(dae);
	return ref;
}


daeMetaElement *
domBrep::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "brep" );
	meta->registerClass(domBrep::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "curves" );
	mea->setOffset( daeOffsetOf(domBrep,elemCurves) );
	mea->setElementType( domCurves::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "surface_curves" );
	mea->setOffset( daeOffsetOf(domBrep,elemSurface_curves) );
	mea->setElementType( domSurface_curves::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "surfaces" );
	mea->setOffset( daeOffsetOf(domBrep,elemSurfaces) );
	mea->setElementType( domSurfaces::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 1, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domBrep,elemSource_array) );
	mea->setElementType( domSource::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 1, 1 );
	mea->setName( "vertices" );
	mea->setOffset( daeOffsetOf(domBrep,elemVertices) );
	mea->setElementType( domVertices::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "edges" );
	mea->setOffset( daeOffsetOf(domBrep,elemEdges) );
	mea->setElementType( domEdges::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "wires" );
	mea->setOffset( daeOffsetOf(domBrep,elemWires) );
	mea->setElementType( domWires::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 7, 0, 1 );
	mea->setName( "faces" );
	mea->setOffset( daeOffsetOf(domBrep,elemFaces) );
	mea->setElementType( domFaces::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 8, 0, 1 );
	mea->setName( "pcurves" );
	mea->setOffset( daeOffsetOf(domBrep,elemPcurves) );
	mea->setElementType( domPcurves::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 9, 0, 1 );
	mea->setName( "shells" );
	mea->setOffset( daeOffsetOf(domBrep,elemShells) );
	mea->setElementType( domShells::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 10, 0, 1 );
	mea->setName( "solids" );
	mea->setOffset( daeOffsetOf(domBrep,elemSolids) );
	mea->setElementType( domSolids::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 11, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domBrep,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 11 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domBrep));
	meta->validate();

	return meta;
}

