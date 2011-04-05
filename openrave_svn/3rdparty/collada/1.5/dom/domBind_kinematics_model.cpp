#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domBind_kinematics_model.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domBind_kinematics_model::create(DAE& dae)
{
	domBind_kinematics_modelRef ref = new domBind_kinematics_model(dae);
	return ref;
}


daeMetaElement *
domBind_kinematics_model::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_kinematics_model" );
	meta->registerClass(domBind_kinematics_model::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "SIDREF" );
	mea->setOffset( daeOffsetOf(domBind_kinematics_model,elemSIDREF) );
	mea->setElementType( domSIDREF::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domBind_kinematics_model,elemParam) );
	mea->setElementType( domCommon_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domBind_kinematics_model,_contents));
	meta->addContentsOrder(daeOffsetOf(domBind_kinematics_model,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domBind_kinematics_model,_CMData), 1);
	//	Add attribute: node
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "node" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domBind_kinematics_model , attrNode ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domBind_kinematics_model));
	meta->validate();

	return meta;
}

