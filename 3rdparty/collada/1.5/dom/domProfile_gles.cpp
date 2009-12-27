#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domProfile_gles.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domProfile_gles::create(DAE& dae)
{
	domProfile_glesRef ref = new domProfile_gles(dae);
	return ref;
}


daeMetaElement *
domProfile_gles::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "profile_gles" );
	meta->registerClass(domProfile_gles::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_gles,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_gles,elemNewparam_array) );
	mea->setElementType( domGles_newparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 1, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domProfile_gles,elemTechnique_array) );
	mea->setElementType( domProfile_gles::domTechnique::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_gles,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_gles , attrId ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: platform
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_gles , attrPlatform ));
		ma->setContainer( meta );
		ma->setDefaultString( "PC");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_gles));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_gles::domTechnique::create(DAE& dae)
{
	domProfile_gles::domTechniqueRef ref = new domProfile_gles::domTechnique(dae);
	return ref;
}


daeMetaElement *
domProfile_gles::domTechnique::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "technique" );
	meta->registerClass(domProfile_gles::domTechnique::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique,elemAnnotate_array) );
	mea->setElementType( domFx_annotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 1, -1 );
	mea->setName( "pass" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique,elemPass_array) );
	mea->setElementType( domProfile_gles::domTechnique::domPass::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_gles::domTechnique , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domProfile_gles::domTechnique , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_gles::domTechnique));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_gles::domTechnique::domPass::create(DAE& dae)
{
	domProfile_gles::domTechnique::domPassRef ref = new domProfile_gles::domTechnique::domPass(dae);
	return ref;
}


daeMetaElement *
domProfile_gles::domTechnique::domPass::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "pass" );
	meta->registerClass(domProfile_gles::domTechnique::domPass::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass,elemAnnotate_array) );
	mea->setElementType( domFx_annotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "states" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass,elemStates) );
	mea->setElementType( domProfile_gles::domTechnique::domPass::domStates::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "evaluate" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass,elemEvaluate) );
	mea->setElementType( domProfile_gles::domTechnique::domPass::domEvaluate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domProfile_gles::domTechnique::domPass , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_gles::domTechnique::domPass));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_gles::domTechnique::domPass::domStates::create(DAE& dae)
{
	domProfile_gles::domTechnique::domPass::domStatesRef ref = new domProfile_gles::domTechnique::domPass::domStates(dae);
	return ref;
}


daeMetaElement *
domProfile_gles::domTechnique::domPass::domStates::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "states" );
	meta->registerClass(domProfile_gles::domTechnique::domPass::domStates::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "gles_pipeline_settings" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass::domStates,elemGles_pipeline_settings_array) );
	mea->setElementType( domGles_pipeline_settings::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 0, -1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domProfile_gles::domTechnique::domPass::domStates));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_gles::domTechnique::domPass::domEvaluate::create(DAE& dae)
{
	domProfile_gles::domTechnique::domPass::domEvaluateRef ref = new domProfile_gles::domTechnique::domPass::domEvaluate(dae);
	return ref;
}


daeMetaElement *
domProfile_gles::domTechnique::domPass::domEvaluate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "evaluate" );
	meta->registerClass(domProfile_gles::domTechnique::domPass::domEvaluate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "color_target" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass::domEvaluate,elemColor_target_array) );
	mea->setElementType( domFx_colortarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "depth_target" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass::domEvaluate,elemDepth_target_array) );
	mea->setElementType( domFx_depthtarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "stencil_target" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass::domEvaluate,elemStencil_target_array) );
	mea->setElementType( domFx_stenciltarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "color_clear" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass::domEvaluate,elemColor_clear_array) );
	mea->setElementType( domFx_clearcolor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "depth_clear" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass::domEvaluate,elemDepth_clear_array) );
	mea->setElementType( domFx_cleardepth::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 5, 0, -1 );
	mea->setName( "stencil_clear" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass::domEvaluate,elemStencil_clear_array) );
	mea->setElementType( domFx_clearstencil::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "draw" );
	mea->setOffset( daeOffsetOf(domProfile_gles::domTechnique::domPass::domEvaluate,elemDraw) );
	mea->setElementType( domProfile_gles::domTechnique::domPass::domEvaluate::domDraw::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 6 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domProfile_gles::domTechnique::domPass::domEvaluate));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_gles::domTechnique::domPass::domEvaluate::domDraw::create(DAE& dae)
{
	domProfile_gles::domTechnique::domPass::domEvaluate::domDrawRef ref = new domProfile_gles::domTechnique::domPass::domEvaluate::domDraw(dae);
	return ref;
}


daeMetaElement *
domProfile_gles::domTechnique::domPass::domEvaluate::domDraw::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "draw" );
	meta->registerClass(domProfile_gles::domTechnique::domPass::domEvaluate::domDraw::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_draw"));
		ma->setOffset( daeOffsetOf( domProfile_gles::domTechnique::domPass::domEvaluate::domDraw , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_gles::domTechnique::domPass::domEvaluate::domDraw));
	meta->validate();

	return meta;
}

