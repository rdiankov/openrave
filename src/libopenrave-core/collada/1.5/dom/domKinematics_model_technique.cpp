#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domKinematics_model_technique.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domKinematics_model_technique::create(DAE& dae)
{
	domKinematics_model_techniqueRef ref = new domKinematics_model_technique(dae);
	return ref;
}


daeMetaElement *
domKinematics_model_technique::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "kinematics_model_technique" );
	meta->registerClass(domKinematics_model_technique::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domKinematics_model_technique,elemNewparam_array) );
	mea->setElementType( domKinematics_newparam::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaChoice( meta, cm, 0, 1, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "instance_joint" );
	mea->setOffset( daeOffsetOf(domKinematics_model_technique,elemInstance_joint_array) );
	mea->setElementType( domInstance_joint::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "joint" );
	mea->setOffset( daeOffsetOf(domKinematics_model_technique,elemJoint_array) );
	mea->setElementType( domJoint::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementArrayAttribute( meta, cm, 3002, 1, -1 );
	mea->setName( "link" );
	mea->setOffset( daeOffsetOf(domKinematics_model_technique,elemLink_array) );
	mea->setElementType( domLink::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaChoice( meta, cm, 1, 3003, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "formula" );
	mea->setOffset( daeOffsetOf(domKinematics_model_technique,elemFormula_array) );
	mea->setElementType( domFormula::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "instance_formula" );
	mea->setOffset( daeOffsetOf(domKinematics_model_technique,elemInstance_formula_array) );
	mea->setElementType( domInstance_formula::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 6003 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domKinematics_model_technique,_contents));
	meta->addContentsOrder(daeOffsetOf(domKinematics_model_technique,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domKinematics_model_technique,_CMData), 2);
	meta->setElementSize(sizeof(domKinematics_model_technique));
	meta->validate();

	return meta;
}

