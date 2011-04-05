#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domNurbs.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domNurbs::create(DAE& dae)
{
	domNurbsRef ref = new domNurbs(dae);
	return ref;
}


daeMetaElement *
domNurbs::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "nurbs" );
	meta->registerClass(domNurbs::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domNurbs,elemSource_array) );
	mea->setElementType( domSource::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "control_vertices" );
	mea->setOffset( daeOffsetOf(domNurbs,elemControl_vertices) );
	mea->setElementType( domNurbs::domControl_vertices::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domNurbs,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: degree
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "degree" );
		ma->setType( dae.getAtomicTypes().get("Uint"));
		ma->setOffset( daeOffsetOf( domNurbs , attrDegree ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: closed
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "closed" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domNurbs , attrClosed ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domNurbs));
	meta->validate();

	return meta;
}

daeElementRef
domNurbs::domControl_vertices::create(DAE& dae)
{
	domNurbs::domControl_verticesRef ref = new domNurbs::domControl_vertices(dae);
	return ref;
}


daeMetaElement *
domNurbs::domControl_vertices::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "control_vertices" );
	meta->registerClass(domNurbs::domControl_vertices::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domNurbs::domControl_vertices,elemInput_array) );
	mea->setElementType( domInput_local::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domNurbs::domControl_vertices,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domNurbs::domControl_vertices));
	meta->validate();

	return meta;
}

