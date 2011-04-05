#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domEffect.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domEffect::create(DAE& dae)
{
	domEffectRef ref = new domEffect(dae);
	return ref;
}


daeMetaElement *
domEffect::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "effect" );
	meta->registerClass(domEffect::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domEffect,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domEffect,elemAnnotate_array) );
	mea->setElementType( domFx_annotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domEffect,elemNewparam_array) );
	mea->setElementType( domFx_newparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 1, -1 );
	mea->setName( "fx_profile" );
	mea->setOffset( daeOffsetOf(domEffect,elemFx_profile_array) );
	mea->setElementType( domFx_profile::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 3, 1, -1 ) );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domEffect,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 4 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domEffect , attrId ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domEffect , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domEffect));
	meta->validate();

	return meta;
}

