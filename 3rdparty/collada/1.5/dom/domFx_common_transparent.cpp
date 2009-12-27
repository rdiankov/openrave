#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_common_transparent.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_common_transparent::create(DAE& dae)
{
	domFx_common_transparentRef ref = new domFx_common_transparent(dae);
	return ref;
}


daeMetaElement *
domFx_common_transparent::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_common_transparent" );
	meta->registerClass(domFx_common_transparent::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "color" );
	mea->setOffset( daeOffsetOf(domFx_common_transparent,elemColor) );
	mea->setElementType( domColor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domFx_common_transparent,elemParam) );
	mea->setElementType( domParam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "texture" );
	mea->setOffset( daeOffsetOf(domFx_common_transparent,elemTexture) );
	mea->setElementType( domTexture::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFx_common_transparent,_contents));
	meta->addContentsOrder(daeOffsetOf(domFx_common_transparent,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFx_common_transparent,_CMData), 1);
	//	Add attribute: opaque
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "opaque" );
		ma->setType( dae.getAtomicTypes().get("Fx_opaque"));
		ma->setOffset( daeOffsetOf( domFx_common_transparent , attrOpaque ));
		ma->setContainer( meta );
		ma->setDefaultString( "A_ONE");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_transparent));
	meta->validate();

	return meta;
}

