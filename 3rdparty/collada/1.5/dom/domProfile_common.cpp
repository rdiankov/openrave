#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domProfile_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domProfile_common::create(DAE& dae)
{
	domProfile_commonRef ref = new domProfile_common(dae);
	return ref;
}


daeMetaElement *
domProfile_common::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "profile_common" );
	meta->registerClass(domProfile_common::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_common,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_common,elemNewparam_array) );
	mea->setElementType( domFx_common_newparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domProfile_common,elemTechnique) );
	mea->setElementType( domProfile_common::domTechnique::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_common,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_common , attrId ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_common));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_common::domTechnique::create(DAE& dae)
{
	domProfile_common::domTechniqueRef ref = new domProfile_common::domTechnique(dae);
	return ref;
}


daeMetaElement *
domProfile_common::domTechnique::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "technique" );
	meta->registerClass(domProfile_common::domTechnique::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaChoice( meta, cm, 0, 1, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "constant" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique,elemConstant) );
	mea->setElementType( domProfile_common::domTechnique::domConstant::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "lambert" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique,elemLambert) );
	mea->setElementType( domProfile_common::domTechnique::domLambert::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "phong" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique,elemPhong) );
	mea->setElementType( domProfile_common::domTechnique::domPhong::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "blinn" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique,elemBlinn) );
	mea->setElementType( domProfile_common::domTechnique::domBlinn::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domProfile_common::domTechnique,_contents));
	meta->addContentsOrder(daeOffsetOf(domProfile_common::domTechnique,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domProfile_common::domTechnique,_CMData), 1);
	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_common::domTechnique , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domProfile_common::domTechnique , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_common::domTechnique));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_common::domTechnique::domConstant::create(DAE& dae)
{
	domProfile_common::domTechnique::domConstantRef ref = new domProfile_common::domTechnique::domConstant(dae);
	return ref;
}


daeMetaElement *
domProfile_common::domTechnique::domConstant::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "constant" );
	meta->registerClass(domProfile_common::domTechnique::domConstant::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "emission" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domConstant,elemEmission) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "reflective" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domConstant,elemReflective) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "reflectivity" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domConstant,elemReflectivity) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "transparent" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domConstant,elemTransparent) );
	mea->setElementType( domFx_common_transparent::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "transparency" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domConstant,elemTransparency) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "index_of_refraction" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domConstant,elemIndex_of_refraction) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 5 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domProfile_common::domTechnique::domConstant));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_common::domTechnique::domLambert::create(DAE& dae)
{
	domProfile_common::domTechnique::domLambertRef ref = new domProfile_common::domTechnique::domLambert(dae);
	return ref;
}


daeMetaElement *
domProfile_common::domTechnique::domLambert::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "lambert" );
	meta->registerClass(domProfile_common::domTechnique::domLambert::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "emission" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domLambert,elemEmission) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "ambient" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domLambert,elemAmbient) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "diffuse" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domLambert,elemDiffuse) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "reflective" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domLambert,elemReflective) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "reflectivity" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domLambert,elemReflectivity) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "transparent" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domLambert,elemTransparent) );
	mea->setElementType( domFx_common_transparent::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "transparency" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domLambert,elemTransparency) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 7, 0, 1 );
	mea->setName( "index_of_refraction" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domLambert,elemIndex_of_refraction) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 7 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domProfile_common::domTechnique::domLambert));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_common::domTechnique::domPhong::create(DAE& dae)
{
	domProfile_common::domTechnique::domPhongRef ref = new domProfile_common::domTechnique::domPhong(dae);
	return ref;
}


daeMetaElement *
domProfile_common::domTechnique::domPhong::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "phong" );
	meta->registerClass(domProfile_common::domTechnique::domPhong::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "emission" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemEmission) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "ambient" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemAmbient) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "diffuse" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemDiffuse) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "specular" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemSpecular) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "shininess" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemShininess) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "reflective" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemReflective) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "reflectivity" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemReflectivity) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 7, 0, 1 );
	mea->setName( "transparent" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemTransparent) );
	mea->setElementType( domFx_common_transparent::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 8, 0, 1 );
	mea->setName( "transparency" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemTransparency) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 9, 0, 1 );
	mea->setName( "index_of_refraction" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domPhong,elemIndex_of_refraction) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 9 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domProfile_common::domTechnique::domPhong));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_common::domTechnique::domBlinn::create(DAE& dae)
{
	domProfile_common::domTechnique::domBlinnRef ref = new domProfile_common::domTechnique::domBlinn(dae);
	return ref;
}


daeMetaElement *
domProfile_common::domTechnique::domBlinn::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "blinn" );
	meta->registerClass(domProfile_common::domTechnique::domBlinn::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "emission" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemEmission) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "ambient" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemAmbient) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "diffuse" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemDiffuse) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "specular" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemSpecular) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "shininess" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemShininess) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "reflective" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemReflective) );
	mea->setElementType( domFx_common_color_or_texture::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "reflectivity" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemReflectivity) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 7, 0, 1 );
	mea->setName( "transparent" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemTransparent) );
	mea->setElementType( domFx_common_transparent::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 8, 0, 1 );
	mea->setName( "transparency" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemTransparency) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 9, 0, 1 );
	mea->setName( "index_of_refraction" );
	mea->setOffset( daeOffsetOf(domProfile_common::domTechnique::domBlinn,elemIndex_of_refraction) );
	mea->setElementType( domFx_common_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 9 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domProfile_common::domTechnique::domBlinn));
	meta->validate();

	return meta;
}

