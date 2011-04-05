#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domKinematics_newparam.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domKinematics_newparam::create(DAE& dae)
{
	domKinematics_newparamRef ref = new domKinematics_newparam(dae);
	return ref;
}


daeMetaElement *
domKinematics_newparam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "kinematics_newparam" );
	meta->registerClass(domKinematics_newparam::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domKinematics_newparam,elemFloat) );
	mea->setElementType( domKinematics_newparam::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domKinematics_newparam,elemInt) );
	mea->setElementType( domKinematics_newparam::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "SIDREF" );
	mea->setOffset( daeOffsetOf(domKinematics_newparam,elemSIDREF) );
	mea->setElementType( domKinematics_newparam::domSIDREF::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domKinematics_newparam,elemBool) );
	mea->setElementType( domKinematics_newparam::domBool::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domKinematics_newparam,_contents));
	meta->addContentsOrder(daeOffsetOf(domKinematics_newparam,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domKinematics_newparam,_CMData), 1);
	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domKinematics_newparam , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_newparam));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_newparam::domFloat::create(DAE& dae)
{
	domKinematics_newparam::domFloatRef ref = new domKinematics_newparam::domFloat(dae);
	return ref;
}


daeMetaElement *
domKinematics_newparam::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domKinematics_newparam::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domKinematics_newparam::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_newparam::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_newparam::domInt::create(DAE& dae)
{
	domKinematics_newparam::domIntRef ref = new domKinematics_newparam::domInt(dae);
	return ref;
}


daeMetaElement *
domKinematics_newparam::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domKinematics_newparam::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domKinematics_newparam::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_newparam::domInt));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_newparam::domSIDREF::create(DAE& dae)
{
	domKinematics_newparam::domSIDREFRef ref = new domKinematics_newparam::domSIDREF(dae);
	return ref;
}


daeMetaElement *
domKinematics_newparam::domSIDREF::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "SIDREF" );
	meta->registerClass(domKinematics_newparam::domSIDREF::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Sidref"));
		ma->setOffset( daeOffsetOf( domKinematics_newparam::domSIDREF , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_newparam::domSIDREF));
	meta->validate();

	return meta;
}

daeElementRef
domKinematics_newparam::domBool::create(DAE& dae)
{
	domKinematics_newparam::domBoolRef ref = new domKinematics_newparam::domBool(dae);
	return ref;
}


daeMetaElement *
domKinematics_newparam::domBool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bool" );
	meta->registerClass(domKinematics_newparam::domBool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domKinematics_newparam::domBool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_newparam::domBool));
	meta->validate();

	return meta;
}

