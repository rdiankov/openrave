#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domKinematics_setparam.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domKinematics_setparam::create(DAE& dae)
{
	domKinematics_setparamRef ref = new domKinematics_setparam(dae);
	return ref;
}


daeMetaElement *
domKinematics_setparam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "kinematics_setparam" );
	meta->registerClass(domKinematics_setparam::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domKinematics_setparam,elemFloat) );
	mea->setElementType( domKinematics_setparam::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domKinematics_setparam,elemInt) );
	mea->setElementType( domKinematics_setparam::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "SIDREF" );
	mea->setOffset( daeOffsetOf(domKinematics_setparam,elemSIDREF) );
	mea->setElementType( domKinematics_setparam::domSIDREF::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domKinematics_setparam,elemBool) );
	mea->setElementType( domKinematics_setparam::domBool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "connect_param" );
	mea->setOffset( daeOffsetOf(domKinematics_setparam,elemConnect_param) );
	mea->setElementType( domKinematics_connect_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domKinematics_setparam,_contents));
	meta->addContentsOrder(daeOffsetOf(domKinematics_setparam,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domKinematics_setparam,_CMData), 1);
	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domKinematics_setparam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_setparam));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_setparam::domFloat::create(DAE& dae)
{
	domKinematics_setparam::domFloatRef ref = new domKinematics_setparam::domFloat(dae);
	return ref;
}


daeMetaElement *
domKinematics_setparam::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domKinematics_setparam::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domKinematics_setparam::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_setparam::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_setparam::domInt::create(DAE& dae)
{
	domKinematics_setparam::domIntRef ref = new domKinematics_setparam::domInt(dae);
	return ref;
}


daeMetaElement *
domKinematics_setparam::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domKinematics_setparam::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domKinematics_setparam::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_setparam::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_setparam::domSIDREF::create(DAE& dae)
{
	domKinematics_setparam::domSIDREFRef ref = new domKinematics_setparam::domSIDREF(dae);
	return ref;
}


daeMetaElement *
domKinematics_setparam::domSIDREF::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "SIDREF" );
	meta->registerClass(domKinematics_setparam::domSIDREF::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Sidref"));
		ma->setOffset( daeOffsetOf( domKinematics_setparam::domSIDREF , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_setparam::domSIDREF));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_setparam::domBool::create(DAE& dae)
{
	domKinematics_setparam::domBoolRef ref = new domKinematics_setparam::domBool(dae);
	return ref;
}


daeMetaElement *
domKinematics_setparam::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domKinematics_setparam::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domKinematics_setparam::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_setparam::domBool));
	meta->validate();

	return meta;
}

