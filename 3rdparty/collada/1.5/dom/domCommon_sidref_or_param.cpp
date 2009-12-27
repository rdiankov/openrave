#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCommon_sidref_or_param.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCommon_sidref_or_param::create(DAE& dae)
{
	domCommon_sidref_or_paramRef ref = new domCommon_sidref_or_param(dae);
	return ref;
}


daeMetaElement *
domCommon_sidref_or_param::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "common_sidref_or_param" );
	meta->registerClass(domCommon_sidref_or_param::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "SIDREF" );
	mea->setOffset( daeOffsetOf(domCommon_sidref_or_param,elemSIDREF) );
	mea->setElementType( domCommon_sidref_or_param::domSIDREF::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domCommon_sidref_or_param,elemParam) );
	mea->setElementType( domCommon_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domCommon_sidref_or_param,_contents));
	meta->addContentsOrder(daeOffsetOf(domCommon_sidref_or_param,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domCommon_sidref_or_param,_CMData), 1);
	meta->setElementSize(sizeof(domCommon_sidref_or_param));
	meta->validate();

	return meta;
}

daeElementRef
domCommon_sidref_or_param::domSIDREF::create(DAE& dae)
{
	domCommon_sidref_or_param::domSIDREFRef ref = new domCommon_sidref_or_param::domSIDREF(dae);
	return ref;
}


daeMetaElement *
domCommon_sidref_or_param::domSIDREF::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "SIDREF" );
	meta->registerClass(domCommon_sidref_or_param::domSIDREF::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Sidref"));
		ma->setOffset( daeOffsetOf( domCommon_sidref_or_param::domSIDREF , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCommon_sidref_or_param::domSIDREF));
	meta->validate();

	return meta;
}

