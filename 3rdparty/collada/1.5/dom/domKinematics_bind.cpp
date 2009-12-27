#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domKinematics_bind.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domKinematics_bind::create(DAE& dae)
{
	domKinematics_bindRef ref = new domKinematics_bind(dae);
	return ref;
}


daeMetaElement *
domKinematics_bind::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "kinematics_bind" );
	meta->registerClass(domKinematics_bind::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domKinematics_bind,elemParam) );
	mea->setElementType( domKinematics_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domKinematics_bind,elemBool) );
	mea->setElementType( domKinematics_bind::domBool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domKinematics_bind,elemFloat) );
	mea->setElementType( domKinematics_bind::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domKinematics_bind,elemInt) );
	mea->setElementType( domKinematics_bind::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "SIDREF" );
	mea->setOffset( daeOffsetOf(domKinematics_bind,elemSIDREF) );
	mea->setElementType( domKinematics_bind::domSIDREF::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domKinematics_bind,_contents));
	meta->addContentsOrder(daeOffsetOf(domKinematics_bind,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domKinematics_bind,_CMData), 1);
	//	Add attribute: symbol
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domKinematics_bind , attrSymbol ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_bind));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_bind::domBool::create(DAE& dae)
{
	domKinematics_bind::domBoolRef ref = new domKinematics_bind::domBool(dae);
	return ref;
}


daeMetaElement *
domKinematics_bind::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domKinematics_bind::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domKinematics_bind::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_bind::domBool));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_bind::domFloat::create(DAE& dae)
{
	domKinematics_bind::domFloatRef ref = new domKinematics_bind::domFloat(dae);
	return ref;
}


daeMetaElement *
domKinematics_bind::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domKinematics_bind::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domKinematics_bind::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_bind::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_bind::domInt::create(DAE& dae)
{
	domKinematics_bind::domIntRef ref = new domKinematics_bind::domInt(dae);
	return ref;
}


daeMetaElement *
domKinematics_bind::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domKinematics_bind::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domKinematics_bind::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_bind::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_bind::domSIDREF::create(DAE& dae)
{
	domKinematics_bind::domSIDREFRef ref = new domKinematics_bind::domSIDREF(dae);
	return ref;
}


daeMetaElement *
domKinematics_bind::domSIDREF::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "SIDREF" );
	meta->registerClass(domKinematics_bind::domSIDREF::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Sidref"));
		ma->setOffset( daeOffsetOf( domKinematics_bind::domSIDREF , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_bind::domSIDREF));
	meta->validate();

	return meta;
}

