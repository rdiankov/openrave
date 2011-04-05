#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFormula_setparam.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFormula_setparam::create(DAE& dae)
{
	domFormula_setparamRef ref = new domFormula_setparam(dae);
	return ref;
}


daeMetaElement *
domFormula_setparam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "formula_setparam" );
	meta->registerClass(domFormula_setparam::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domFormula_setparam,elemFloat) );
	mea->setElementType( domFormula_setparam::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domFormula_setparam,elemInt) );
	mea->setElementType( domFormula_setparam::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "SIDREF" );
	mea->setOffset( daeOffsetOf(domFormula_setparam,elemSIDREF) );
	mea->setElementType( domFormula_setparam::domSIDREF::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domFormula_setparam,elemBool) );
	mea->setElementType( domFormula_setparam::domBool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "connect_param" );
	mea->setOffset( daeOffsetOf(domFormula_setparam,elemConnect_param) );
	mea->setElementType( domKinematics_connect_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFormula_setparam,_contents));
	meta->addContentsOrder(daeOffsetOf(domFormula_setparam,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFormula_setparam,_CMData), 1);
	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domFormula_setparam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_setparam));
	meta->validate();

	return meta;
}

daeElementRef
domFormula_setparam::domFloat::create(DAE& dae)
{
	domFormula_setparam::domFloatRef ref = new domFormula_setparam::domFloat(dae);
	return ref;
}


daeMetaElement *
domFormula_setparam::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domFormula_setparam::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domFormula_setparam::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_setparam::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domFormula_setparam::domInt::create(DAE& dae)
{
	domFormula_setparam::domIntRef ref = new domFormula_setparam::domInt(dae);
	return ref;
}


daeMetaElement *
domFormula_setparam::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domFormula_setparam::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domFormula_setparam::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_setparam::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domFormula_setparam::domSIDREF::create(DAE& dae)
{
	domFormula_setparam::domSIDREFRef ref = new domFormula_setparam::domSIDREF(dae);
	return ref;
}


daeMetaElement *
domFormula_setparam::domSIDREF::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "SIDREF" );
	meta->registerClass(domFormula_setparam::domSIDREF::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Sidref"));
		ma->setOffset( daeOffsetOf( domFormula_setparam::domSIDREF , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_setparam::domSIDREF));
	meta->validate();

	return meta;
}

daeElementRef
domFormula_setparam::domBool::create(DAE& dae)
{
	domFormula_setparam::domBoolRef ref = new domFormula_setparam::domBool(dae);
	return ref;
}


daeMetaElement *
domFormula_setparam::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domFormula_setparam::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domFormula_setparam::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFormula_setparam::domBool));
	meta->validate();

	return meta;
}

