#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domImage.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domImage::create(DAE& dae)
{
	domImageRef ref = new domImage(dae);
	return ref;
}


daeMetaElement *
domImage::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "image" );
	meta->registerClass(domImage::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domImage,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "renderable" );
	mea->setOffset( daeOffsetOf(domImage,elemRenderable) );
	mea->setElementType( domImage::domRenderable::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaChoice( meta, cm, 0, 2, 0, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "init_from" );
	mea->setOffset( daeOffsetOf(domImage,elemInit_from) );
	mea->setElementType( domImage::domInit_from::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "create_2d" );
	mea->setOffset( daeOffsetOf(domImage,elemCreate_2d) );
	mea->setElementType( domImage::domCreate_2d::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "create_3d" );
	mea->setOffset( daeOffsetOf(domImage,elemCreate_3d) );
	mea->setElementType( domImage::domCreate_3d::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "create_cube" );
	mea->setOffset( daeOffsetOf(domImage,elemCreate_cube) );
	mea->setElementType( domImage::domCreate_cube::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domImage,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domImage,_contents));
	meta->addContentsOrder(daeOffsetOf(domImage,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domImage,_CMData), 1);
	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domImage , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domImage , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domRenderable::create(DAE& dae)
{
	domImage::domRenderableRef ref = new domImage::domRenderable(dae);
	return ref;
}


daeMetaElement *
domImage::domRenderable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "renderable" );
	meta->registerClass(domImage::domRenderable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: share
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "share" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domImage::domRenderable , attrShare ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domRenderable));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domInit_from::create(DAE& dae)
{
	domImage::domInit_fromRef ref = new domImage::domInit_from(dae);
	return ref;
}


daeMetaElement *
domImage::domInit_from::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "init_from" );
	meta->registerClass(domImage::domInit_from::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domImage::domInit_from,elemRef) );
	mea->setElementType( domRef::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hex" );
	mea->setOffset( daeOffsetOf(domImage::domInit_from,elemHex) );
	mea->setElementType( domHex::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domImage::domInit_from,_contents));
	meta->addContentsOrder(daeOffsetOf(domImage::domInit_from,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domImage::domInit_from,_CMData), 1);
	//	Add attribute: mips_generate
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "mips_generate" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domImage::domInit_from , attrMips_generate ));
		ma->setContainer( meta );
		ma->setDefaultString( "true");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domInit_from));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::create(DAE& dae)
{
	domImage::domCreate_2dRef ref = new domImage::domCreate_2d(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "create_2d" );
	meta->registerClass(domImage::domCreate_2d::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "size_exact" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d,elemSize_exact) );
	mea->setElementType( domImage::domCreate_2d::domSize_exact::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "size_ratio" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d,elemSize_ratio) );
	mea->setElementType( domImage::domCreate_2d::domSize_ratio::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm = new daeMetaChoice( meta, cm, 1, 1, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "mips" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d,elemMips) );
	mea->setElementType( domImage_mips::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "unnormalized" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d,elemUnnormalized) );
	mea->setElementType( domImage::domCreate_2d::domUnnormalized::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d,elemArray) );
	mea->setElementType( domImage::domCreate_2d::domArray::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "format" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d,elemFormat) );
	mea->setElementType( domImage::domCreate_2d::domFormat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "init_from" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d,elemInit_from_array) );
	mea->setElementType( domImage::domCreate_2d::domInit_from::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 4 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domImage::domCreate_2d,_contents));
	meta->addContentsOrder(daeOffsetOf(domImage::domCreate_2d,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domImage::domCreate_2d,_CMData), 2);
	meta->setElementSize(sizeof(domImage::domCreate_2d));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::domSize_exact::create(DAE& dae)
{
	domImage::domCreate_2d::domSize_exactRef ref = new domImage::domCreate_2d::domSize_exact(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::domSize_exact::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "size_exact" );
	meta->registerClass(domImage::domCreate_2d::domSize_exact::create);

	meta->setIsInnerClass( true );

	//	Add attribute: width
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "width" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domSize_exact , attrWidth ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: height
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "height" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domSize_exact , attrHeight ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_2d::domSize_exact));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::domSize_ratio::create(DAE& dae)
{
	domImage::domCreate_2d::domSize_ratioRef ref = new domImage::domCreate_2d::domSize_ratio(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::domSize_ratio::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "size_ratio" );
	meta->registerClass(domImage::domCreate_2d::domSize_ratio::create);

	meta->setIsInnerClass( true );

	//	Add attribute: width
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "width" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domSize_ratio , attrWidth ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: height
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "height" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domSize_ratio , attrHeight ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_2d::domSize_ratio));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::domUnnormalized::create(DAE& dae)
{
	domImage::domCreate_2d::domUnnormalizedRef ref = new domImage::domCreate_2d::domUnnormalized(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::domUnnormalized::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "unnormalized" );
	meta->registerClass(domImage::domCreate_2d::domUnnormalized::create);

	meta->setIsInnerClass( true );

	meta->setElementSize(sizeof(domImage::domCreate_2d::domUnnormalized));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::domArray::create(DAE& dae)
{
	domImage::domCreate_2d::domArrayRef ref = new domImage::domCreate_2d::domArray(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::domArray::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "array" );
	meta->registerClass(domImage::domCreate_2d::domArray::create);

	meta->setIsInnerClass( true );

	//	Add attribute: length
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "length" );
		ma->setType( dae.getAtomicTypes().get("xsPositiveInteger"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domArray , attrLength ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_2d::domArray));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::domFormat::create(DAE& dae)
{
	domImage::domCreate_2d::domFormatRef ref = new domImage::domCreate_2d::domFormat(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::domFormat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "format" );
	meta->registerClass(domImage::domCreate_2d::domFormat::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hint" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d::domFormat,elemHint) );
	mea->setElementType( domImage::domCreate_2d::domFormat::domHint::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "exact" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d::domFormat,elemExact) );
	mea->setElementType( domImage::domCreate_2d::domFormat::domExact::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domImage::domCreate_2d::domFormat));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::domFormat::domHint::create(DAE& dae)
{
	domImage::domCreate_2d::domFormat::domHintRef ref = new domImage::domCreate_2d::domFormat::domHint(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::domFormat::domHint::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "hint" );
	meta->registerClass(domImage::domCreate_2d::domFormat::domHint::create);

	meta->setIsInnerClass( true );

	//	Add attribute: channels
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "channels" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_channels"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domFormat::domHint , attrChannels ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: range
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "range" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_range"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domFormat::domHint , attrRange ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: precision
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "precision" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_precision"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domFormat::domHint , attrPrecision ));
		ma->setContainer( meta );
		ma->setDefaultString( "DEFAULT");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: space
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "space" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domFormat::domHint , attrSpace ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_2d::domFormat::domHint));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::domFormat::domExact::create(DAE& dae)
{
	domImage::domCreate_2d::domFormat::domExactRef ref = new domImage::domCreate_2d::domFormat::domExact(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::domFormat::domExact::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "exact" );
	meta->registerClass(domImage::domCreate_2d::domFormat::domExact::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domFormat::domExact , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_2d::domFormat::domExact));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_2d::domInit_from::create(DAE& dae)
{
	domImage::domCreate_2d::domInit_fromRef ref = new domImage::domCreate_2d::domInit_from(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_2d::domInit_from::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "init_from" );
	meta->registerClass(domImage::domCreate_2d::domInit_from::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d::domInit_from,elemRef) );
	mea->setElementType( domRef::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hex" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_2d::domInit_from,elemHex) );
	mea->setElementType( domHex::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domImage::domCreate_2d::domInit_from,_contents));
	meta->addContentsOrder(daeOffsetOf(domImage::domCreate_2d::domInit_from,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domImage::domCreate_2d::domInit_from,_CMData), 1);
	//	Add attribute: mip_index
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "mip_index" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domInit_from , attrMip_index ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: array_index
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "array_index" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_2d::domInit_from , attrArray_index ));
		ma->setContainer( meta );
		ma->setDefaultString( "0");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_2d::domInit_from));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_3d::create(DAE& dae)
{
	domImage::domCreate_3dRef ref = new domImage::domCreate_3d(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_3d::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "create_3d" );
	meta->registerClass(domImage::domCreate_3d::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "size" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d,elemSize) );
	mea->setElementType( domImage::domCreate_3d::domSize::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "mips" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d,elemMips) );
	mea->setElementType( domImage_mips::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d,elemArray) );
	mea->setElementType( domImage::domCreate_3d::domArray::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "format" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d,elemFormat) );
	mea->setElementType( domImage::domCreate_3d::domFormat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "init_from" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d,elemInit_from_array) );
	mea->setElementType( domImage::domCreate_3d::domInit_from::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 4 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domImage::domCreate_3d));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_3d::domSize::create(DAE& dae)
{
	domImage::domCreate_3d::domSizeRef ref = new domImage::domCreate_3d::domSize(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_3d::domSize::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "size" );
	meta->registerClass(domImage::domCreate_3d::domSize::create);

	meta->setIsInnerClass( true );

	//	Add attribute: width
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "width" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domSize , attrWidth ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: height
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "height" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domSize , attrHeight ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: depth
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "depth" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domSize , attrDepth ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_3d::domSize));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_3d::domArray::create(DAE& dae)
{
	domImage::domCreate_3d::domArrayRef ref = new domImage::domCreate_3d::domArray(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_3d::domArray::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "array" );
	meta->registerClass(domImage::domCreate_3d::domArray::create);

	meta->setIsInnerClass( true );

	//	Add attribute: length
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "length" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domArray , attrLength ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_3d::domArray));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_3d::domFormat::create(DAE& dae)
{
	domImage::domCreate_3d::domFormatRef ref = new domImage::domCreate_3d::domFormat(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_3d::domFormat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "format" );
	meta->registerClass(domImage::domCreate_3d::domFormat::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hint" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d::domFormat,elemHint) );
	mea->setElementType( domImage::domCreate_3d::domFormat::domHint::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "exact" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d::domFormat,elemExact) );
	mea->setElementType( domImage::domCreate_3d::domFormat::domExact::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domImage::domCreate_3d::domFormat));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_3d::domFormat::domHint::create(DAE& dae)
{
	domImage::domCreate_3d::domFormat::domHintRef ref = new domImage::domCreate_3d::domFormat::domHint(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_3d::domFormat::domHint::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "hint" );
	meta->registerClass(domImage::domCreate_3d::domFormat::domHint::create);

	meta->setIsInnerClass( true );

	//	Add attribute: channels
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "channels" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_channels"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domFormat::domHint , attrChannels ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: range
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "range" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_range"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domFormat::domHint , attrRange ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: precision
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "precision" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_precision"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domFormat::domHint , attrPrecision ));
		ma->setContainer( meta );
		ma->setDefaultString( "DEFAULT");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: space
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "space" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domFormat::domHint , attrSpace ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_3d::domFormat::domHint));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_3d::domFormat::domExact::create(DAE& dae)
{
	domImage::domCreate_3d::domFormat::domExactRef ref = new domImage::domCreate_3d::domFormat::domExact(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_3d::domFormat::domExact::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "exact" );
	meta->registerClass(domImage::domCreate_3d::domFormat::domExact::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domFormat::domExact , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_3d::domFormat::domExact));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_3d::domInit_from::create(DAE& dae)
{
	domImage::domCreate_3d::domInit_fromRef ref = new domImage::domCreate_3d::domInit_from(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_3d::domInit_from::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "init_from" );
	meta->registerClass(domImage::domCreate_3d::domInit_from::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d::domInit_from,elemRef) );
	mea->setElementType( domRef::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hex" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_3d::domInit_from,elemHex) );
	mea->setElementType( domHex::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domImage::domCreate_3d::domInit_from,_contents));
	meta->addContentsOrder(daeOffsetOf(domImage::domCreate_3d::domInit_from,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domImage::domCreate_3d::domInit_from,_CMData), 1);
	//	Add attribute: depth
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "depth" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domInit_from , attrDepth ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: mip_index
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "mip_index" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domInit_from , attrMip_index ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: array_index
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "array_index" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_3d::domInit_from , attrArray_index ));
		ma->setContainer( meta );
		ma->setDefaultString( "0");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_3d::domInit_from));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_cube::create(DAE& dae)
{
	domImage::domCreate_cubeRef ref = new domImage::domCreate_cube(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_cube::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "create_cube" );
	meta->registerClass(domImage::domCreate_cube::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "size" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube,elemSize) );
	mea->setElementType( domImage::domCreate_cube::domSize::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "mips" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube,elemMips) );
	mea->setElementType( domImage_mips::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube,elemArray) );
	mea->setElementType( domImage::domCreate_cube::domArray::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "format" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube,elemFormat) );
	mea->setElementType( domImage::domCreate_cube::domFormat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "init_from" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube,elemInit_from_array) );
	mea->setElementType( domImage::domCreate_cube::domInit_from::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 4 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domImage::domCreate_cube));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_cube::domSize::create(DAE& dae)
{
	domImage::domCreate_cube::domSizeRef ref = new domImage::domCreate_cube::domSize(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_cube::domSize::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "size" );
	meta->registerClass(domImage::domCreate_cube::domSize::create);

	meta->setIsInnerClass( true );

	//	Add attribute: width
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "width" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domSize , attrWidth ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_cube::domSize));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_cube::domArray::create(DAE& dae)
{
	domImage::domCreate_cube::domArrayRef ref = new domImage::domCreate_cube::domArray(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_cube::domArray::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "array" );
	meta->registerClass(domImage::domCreate_cube::domArray::create);

	meta->setIsInnerClass( true );

	//	Add attribute: length
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "length" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domArray , attrLength ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_cube::domArray));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_cube::domFormat::create(DAE& dae)
{
	domImage::domCreate_cube::domFormatRef ref = new domImage::domCreate_cube::domFormat(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_cube::domFormat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "format" );
	meta->registerClass(domImage::domCreate_cube::domFormat::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hint" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube::domFormat,elemHint) );
	mea->setElementType( domImage::domCreate_cube::domFormat::domHint::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "exact" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube::domFormat,elemExact) );
	mea->setElementType( domImage::domCreate_cube::domFormat::domExact::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domImage::domCreate_cube::domFormat));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_cube::domFormat::domHint::create(DAE& dae)
{
	domImage::domCreate_cube::domFormat::domHintRef ref = new domImage::domCreate_cube::domFormat::domHint(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_cube::domFormat::domHint::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "hint" );
	meta->registerClass(domImage::domCreate_cube::domFormat::domHint::create);

	meta->setIsInnerClass( true );

	//	Add attribute: channels
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "channels" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_channels"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domFormat::domHint , attrChannels ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: range
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "range" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_range"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domFormat::domHint , attrRange ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: precision
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "precision" );
		ma->setType( dae.getAtomicTypes().get("Image_format_hint_precision"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domFormat::domHint , attrPrecision ));
		ma->setContainer( meta );
		ma->setDefaultString( "DEFAULT");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: space
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "space" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domFormat::domHint , attrSpace ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_cube::domFormat::domHint));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_cube::domFormat::domExact::create(DAE& dae)
{
	domImage::domCreate_cube::domFormat::domExactRef ref = new domImage::domCreate_cube::domFormat::domExact(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_cube::domFormat::domExact::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "exact" );
	meta->registerClass(domImage::domCreate_cube::domFormat::domExact::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domFormat::domExact , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_cube::domFormat::domExact));
	meta->validate();

	return meta;
}

daeElementRef
domImage::domCreate_cube::domInit_from::create(DAE& dae)
{
	domImage::domCreate_cube::domInit_fromRef ref = new domImage::domCreate_cube::domInit_from(dae);
	return ref;
}


daeMetaElement *
domImage::domCreate_cube::domInit_from::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "init_from" );
	meta->registerClass(domImage::domCreate_cube::domInit_from::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube::domInit_from,elemRef) );
	mea->setElementType( domRef::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hex" );
	mea->setOffset( daeOffsetOf(domImage::domCreate_cube::domInit_from,elemHex) );
	mea->setElementType( domHex::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domImage::domCreate_cube::domInit_from,_contents));
	meta->addContentsOrder(daeOffsetOf(domImage::domCreate_cube::domInit_from,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domImage::domCreate_cube::domInit_from,_CMData), 1);
	//	Add attribute: face
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "face" );
		ma->setType( dae.getAtomicTypes().get("Image_face"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domInit_from , attrFace ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: mip_index
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "mip_index" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domInit_from , attrMip_index ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: array_index
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "array_index" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage::domCreate_cube::domInit_from , attrArray_index ));
		ma->setContainer( meta );
		ma->setDefaultString( "0");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage::domCreate_cube::domInit_from));
	meta->validate();

	return meta;
}

