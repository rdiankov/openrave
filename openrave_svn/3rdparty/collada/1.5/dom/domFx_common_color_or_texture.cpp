#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_common_color_or_texture.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_common_color_or_texture::create(DAE& dae)
{
	domFx_common_color_or_textureRef ref = new domFx_common_color_or_texture(dae);
	return ref;
}


daeMetaElement *
domFx_common_color_or_texture::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_common_color_or_texture" );
	meta->registerClass(domFx_common_color_or_texture::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "color" );
	mea->setOffset( daeOffsetOf(domFx_common_color_or_texture,elemColor) );
	mea->setElementType( domFx_common_color_or_texture::domColor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domFx_common_color_or_texture,elemParam) );
	mea->setElementType( domFx_common_color_or_texture::domParam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "texture" );
	mea->setOffset( daeOffsetOf(domFx_common_color_or_texture,elemTexture) );
	mea->setElementType( domFx_common_color_or_texture::domTexture::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFx_common_color_or_texture,_contents));
	meta->addContentsOrder(daeOffsetOf(domFx_common_color_or_texture,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFx_common_color_or_texture,_CMData), 1);
	meta->setElementSize(sizeof(domFx_common_color_or_texture));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_color_or_texture::domColor::create(DAE& dae)
{
	domFx_common_color_or_texture::domColorRef ref = new domFx_common_color_or_texture::domColor(dae);
	return ref;
}


daeMetaElement *
domFx_common_color_or_texture::domColor::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "color" );
	meta->registerClass(domFx_common_color_or_texture::domColor::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_color"));
		ma->setOffset( daeOffsetOf( domFx_common_color_or_texture::domColor , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domFx_common_color_or_texture::domColor , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_color_or_texture::domColor));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_color_or_texture::domParam::create(DAE& dae)
{
	domFx_common_color_or_texture::domParamRef ref = new domFx_common_color_or_texture::domParam(dae);
	return ref;
}


daeMetaElement *
domFx_common_color_or_texture::domParam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "param" );
	meta->registerClass(domFx_common_color_or_texture::domParam::create);

	meta->setIsInnerClass( true );

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_common_color_or_texture::domParam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_color_or_texture::domParam));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_color_or_texture::domTexture::create(DAE& dae)
{
	domFx_common_color_or_texture::domTextureRef ref = new domFx_common_color_or_texture::domTexture(dae);
	return ref;
}


daeMetaElement *
domFx_common_color_or_texture::domTexture::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "texture" );
	meta->registerClass(domFx_common_color_or_texture::domTexture::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domFx_common_color_or_texture::domTexture,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: texture
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "texture" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_common_color_or_texture::domTexture , attrTexture ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: texcoord
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "texcoord" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_common_color_or_texture::domTexture , attrTexcoord ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_color_or_texture::domTexture));
	meta->validate();

	return meta;
}

