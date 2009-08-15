#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFormula_technique.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFormula_technique::create(DAE& dae)
{
	domFormula_techniqueRef ref = new domFormula_technique(dae);
	return ref;
}


daeMetaElement *
domFormula_technique::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "formula_technique" );
	meta->registerClass(domFormula_technique::create);

	daeMetaCMPolicy *cm = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaAny( meta, cm, 0, 0, -1 );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	meta->setAllowsAny( true );
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFormula_technique,_contents));
	meta->addContentsOrder(daeOffsetOf(domFormula_technique,_contentsOrder));


	meta->setElementSize(sizeof(domFormula_technique));
	meta->validate();

	return meta;
}

