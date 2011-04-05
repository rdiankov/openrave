#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domNurbs_surface.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domNurbs_surface::create(DAE& dae)
{
	domNurbs_surfaceRef ref = new domNurbs_surface(dae);
	return ref;
}


daeMetaElement *
domNurbs_surface::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "nurbs_surface" );
	meta->registerClass(domNurbs_surface::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domNurbs_surface,elemSource_array) );
	mea->setElementType( domSource::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "control_vertices" );
	mea->setOffset( daeOffsetOf(domNurbs_surface,elemControl_vertices) );
	mea->setElementType( domNurbs_surface::domControl_vertices::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domNurbs_surface,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: degree_u
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "degree_u" );
		ma->setType( dae.getAtomicTypes().get("Uint"));
		ma->setOffset( daeOffsetOf( domNurbs_surface , attrDegree_u ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: closed_u
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "closed_u" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domNurbs_surface , attrClosed_u ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: degree_v
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "degree_v" );
		ma->setType( dae.getAtomicTypes().get("Uint"));
		ma->setOffset( daeOffsetOf( domNurbs_surface , attrDegree_v ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: closed_v
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "closed_v" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domNurbs_surface , attrClosed_v ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domNurbs_surface));
	meta->validate();

	return meta;
}

daeElementRef
domNurbs_surface::domControl_vertices::create(DAE& dae)
{
	domNurbs_surface::domControl_verticesRef ref = new domNurbs_surface::domControl_vertices(dae);
	return ref;
}


daeMetaElement *
domNurbs_surface::domControl_vertices::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "control_vertices" );
	meta->registerClass(domNurbs_surface::domControl_vertices::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domNurbs_surface::domControl_vertices,elemInput_array) );
	mea->setElementType( domInput_local::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domNurbs_surface::domControl_vertices,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domNurbs_surface::domControl_vertices));
	meta->validate();

	return meta;
}

