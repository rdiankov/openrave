#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domInstance_formula.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domInstance_formula::create(DAE& dae)
{
	domInstance_formulaRef ref = new domInstance_formula(dae);
	ref->attrUrl.setContainer( (domInstance_formula*)ref );
	return ref;
}


daeMetaElement *
domInstance_formula::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "instance_formula" );
	meta->registerClass(domInstance_formula::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domInstance_formula,elemSetparam_array) );
	mea->setElementType( domFormula_setparam::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domInstance_formula , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domInstance_formula , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: url
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_formula , attrUrl ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_formula));
	meta->validate();

	return meta;
}

