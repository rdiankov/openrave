#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domKinematics_frame.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domKinematics_frame::create(DAE& dae)
{
	domKinematics_frameRef ref = new domKinematics_frame(dae);
	return ref;
}


daeMetaElement *
domKinematics_frame::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "kinematics_frame" );
	meta->registerClass(domKinematics_frame::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domKinematics_frame,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domKinematics_frame,elemRotate_array) );
	mea->setElementType( domRotate::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3000 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domKinematics_frame,_contents));
	meta->addContentsOrder(daeOffsetOf(domKinematics_frame,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domKinematics_frame,_CMData), 1);
	//	Add attribute: link
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "link" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domKinematics_frame , attrLink ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_frame));
	meta->validate();

	return meta;
}

