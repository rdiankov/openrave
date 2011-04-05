#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCurve.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCurve::create(DAE& dae)
{
	domCurveRef ref = new domCurve(dae);
	return ref;
}


daeMetaElement *
domCurve::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "curve" );
	meta->registerClass(domCurve::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "line" );
	mea->setOffset( daeOffsetOf(domCurve,elemLine) );
	mea->setElementType( domLine::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "circle" );
	mea->setOffset( daeOffsetOf(domCurve,elemCircle) );
	mea->setElementType( domCircle::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ellipse" );
	mea->setOffset( daeOffsetOf(domCurve,elemEllipse) );
	mea->setElementType( domEllipse::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "parabola" );
	mea->setOffset( daeOffsetOf(domCurve,elemParabola) );
	mea->setElementType( domParabola::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hyperbola" );
	mea->setOffset( daeOffsetOf(domCurve,elemHyperbola) );
	mea->setElementType( domHyperbola::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "nurbs" );
	mea->setOffset( daeOffsetOf(domCurve,elemNurbs) );
	mea->setElementType( domNurbs::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "orient" );
	mea->setOffset( daeOffsetOf(domCurve,elemOrient_array) );
	mea->setElementType( domOrient::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "origin" );
	mea->setOffset( daeOffsetOf(domCurve,elemOrigin) );
	mea->setElementType( domOrigin::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domCurve,_contents));
	meta->addContentsOrder(daeOffsetOf(domCurve,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domCurve,_CMData), 1);
	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domCurve , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domCurve , attrName ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCurve));
	meta->validate();

	return meta;
}

