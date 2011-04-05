#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFormula.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFormula::create(DAE& dae)
{
	domFormulaRef ref = new domFormula(dae);
	return ref;
}


daeMetaElement *
domFormula::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "formula" );
	meta->registerClass(domFormula::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domFormula,elemNewparam_array) );
	mea->setElementType( domFormula_newparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "target" );
	mea->setOffset( daeOffsetOf(domFormula,elemTarget) );
	mea->setElementType( domCommon_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "technique_common" );
	mea->setOffset( daeOffsetOf(domFormula,elemTechnique_common) );
	mea->setElementType( domFormula_technique::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domFormula,elemTechnique_array) );
	mea->setElementType( domTechnique::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domFormula , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domFormula , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domFormula , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula));
	meta->validate();

	return meta;
}

