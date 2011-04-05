#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFormula_newparam.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFormula_newparam::create(DAE& dae)
{
	domFormula_newparamRef ref = new domFormula_newparam(dae);
	return ref;
}


daeMetaElement *
domFormula_newparam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "formula_newparam" );
	meta->registerClass(domFormula_newparam::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domFormula_newparam,elemFloat) );
	mea->setElementType( domFormula_newparam::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domFormula_newparam,elemInt) );
	mea->setElementType( domFormula_newparam::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "SIDREF" );
	mea->setOffset( daeOffsetOf(domFormula_newparam,elemSIDREF) );
	mea->setElementType( domFormula_newparam::domSIDREF::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domFormula_newparam,elemBool) );
	mea->setElementType( domFormula_newparam::domBool::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFormula_newparam,_contents));
	meta->addContentsOrder(daeOffsetOf(domFormula_newparam,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFormula_newparam,_CMData), 1);
	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domFormula_newparam , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_newparam));
	meta->validate();

	return meta;
}

daeElementRef
domFormula_newparam::domFloat::create(DAE& dae)
{
	domFormula_newparam::domFloatRef ref = new domFormula_newparam::domFloat(dae);
	return ref;
}


daeMetaElement *
domFormula_newparam::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domFormula_newparam::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domFormula_newparam::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_newparam::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domFormula_newparam::domInt::create(DAE& dae)
{
	domFormula_newparam::domIntRef ref = new domFormula_newparam::domInt(dae);
	return ref;
}


daeMetaElement *
domFormula_newparam::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domFormula_newparam::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domFormula_newparam::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_newparam::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domFormula_newparam::domSIDREF::create(DAE& dae)
{
	domFormula_newparam::domSIDREFRef ref = new domFormula_newparam::domSIDREF(dae);
	return ref;
}


daeMetaElement *
domFormula_newparam::domSIDREF::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "SIDREF" );
	meta->registerClass(domFormula_newparam::domSIDREF::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Sidref"));
		ma->setOffset( daeOffsetOf( domFormula_newparam::domSIDREF , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_newparam::domSIDREF));
	meta->validate();

	return meta;
}

daeElementRef
domFormula_newparam::domBool::create(DAE& dae)
{
	domFormula_newparam::domBoolRef ref = new domFormula_newparam::domBool(dae);
	return ref;
}


daeMetaElement *
domFormula_newparam::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domFormula_newparam::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domFormula_newparam::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_newparam::domBool));
	meta->validate();

	return meta;
}

