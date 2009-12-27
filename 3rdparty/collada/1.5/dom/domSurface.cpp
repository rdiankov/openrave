#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domSurface.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSurface::create(DAE& dae)
{
	domSurfaceRef ref = new domSurface(dae);
	return ref;
}


daeMetaElement *
domSurface::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "surface" );
	meta->registerClass(domSurface::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "plane" );
	mea->setOffset( daeOffsetOf(domSurface,elemPlane) );
	mea->setElementType( domPlane::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sphere" );
	mea->setOffset( daeOffsetOf(domSurface,elemSphere) );
	mea->setElementType( domSphere::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "torus" );
	mea->setOffset( daeOffsetOf(domSurface,elemTorus) );
	mea->setElementType( domTorus::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "swept_surface" );
	mea->setOffset( daeOffsetOf(domSurface,elemSwept_surface) );
	mea->setElementType( domSwept_surface::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "nurbs_surface" );
	mea->setOffset( daeOffsetOf(domSurface,elemNurbs_surface) );
	mea->setElementType( domNurbs_surface::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "cone" );
	mea->setOffset( daeOffsetOf(domSurface,elemCone) );
	mea->setElementType( domCone::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "cylinder" );
	mea->setOffset( daeOffsetOf(domSurface,elemCylinder) );
	mea->setElementType( domSurface::domCylinder::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "orient" );
	mea->setOffset( daeOffsetOf(domSurface,elemOrient_array) );
	mea->setElementType( domOrient::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "origin" );
	mea->setOffset( daeOffsetOf(domSurface,elemOrigin) );
	mea->setElementType( domOrigin::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domSurface,_contents));
	meta->addContentsOrder(daeOffsetOf(domSurface,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domSurface,_CMData), 1);
	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domSurface , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domSurface , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domSurface));
	meta->validate();

	return meta;
}

daeElementRef
domSurface::domCylinder::create(DAE& dae)
{
	domSurface::domCylinderRef ref = new domSurface::domCylinder(dae);
	return ref;
}


daeMetaElement *
domSurface::domCylinder::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cylinder" );
	meta->registerClass(domSurface::domCylinder::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "radius" );
	mea->setOffset( daeOffsetOf(domSurface::domCylinder,elemRadius) );
	mea->setElementType( domSurface::domCylinder::domRadius::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSurface::domCylinder,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domSurface::domCylinder));
	meta->validate();

	return meta;
}

daeElementRef
domSurface::domCylinder::domRadius::create(DAE& dae)
{
	domSurface::domCylinder::domRadiusRef ref = new domSurface::domCylinder::domRadius(dae);
	return ref;
}


daeMetaElement *
domSurface::domCylinder::domRadius::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "radius" );
	meta->registerClass(domSurface::domCylinder::domRadius::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domSurface::domCylinder::domRadius , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domSurface::domCylinder::domRadius));
	meta->validate();

	return meta;
}

