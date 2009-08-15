#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domProfile_glsl.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domProfile_glsl::create(DAE& dae)
{
	domProfile_glslRef ref = new domProfile_glsl(dae);
	return ref;
}


daeMetaElement *
domProfile_glsl::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "profile_glsl" );
	meta->registerClass(domProfile_glsl::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_glsl,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaChoice( meta, cm, 0, 1, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "code" );
	mea->setOffset( daeOffsetOf(domProfile_glsl,elemCode_array) );
	mea->setElementType( domFx_code::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "include" );
	mea->setOffset( daeOffsetOf(domProfile_glsl,elemInclude_array) );
	mea->setElementType( domFx_include::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementArrayAttribute( meta, cm, 3002, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domProfile_glsl,elemNewparam_array) );
	mea->setElementType( domGlsl_newparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3003, 1, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domProfile_glsl,elemTechnique_array) );
	mea->setElementType( domProfile_glsl::domTechnique::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3004, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_glsl,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3004 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domProfile_glsl,_contents));
	meta->addContentsOrder(daeOffsetOf(domProfile_glsl,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domProfile_glsl,_CMData), 1);
	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_glsl , attrId ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: platform
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_glsl , attrPlatform ));
		ma->setContainer( meta );
		ma->setDefaultString( "PC");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_glsl));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_glsl::domTechnique::create(DAE& dae)
{
	domProfile_glsl::domTechniqueRef ref = new domProfile_glsl::domTechnique(dae);
	return ref;
}


daeMetaElement *
domProfile_glsl::domTechnique::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "technique" );
	meta->registerClass(domProfile_glsl::domTechnique::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique,elemAnnotate_array) );
	mea->setElementType( domFx_annotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 1, -1 );
	mea->setName( "pass" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique,elemPass_array) );
	mea->setElementType( domProfile_glsl::domTechnique::domPass::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_glsl::domTechnique , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domProfile_glsl::domTechnique , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_glsl::domTechnique));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_glsl::domTechnique::domPass::create(DAE& dae)
{
	domProfile_glsl::domTechnique::domPassRef ref = new domProfile_glsl::domTechnique::domPass(dae);
	return ref;
}


daeMetaElement *
domProfile_glsl::domTechnique::domPass::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "pass" );
	meta->registerClass(domProfile_glsl::domTechnique::domPass::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass,elemAnnotate_array) );
	mea->setElementType( domFx_annotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "states" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass,elemStates) );
	mea->setElementType( domProfile_glsl::domTechnique::domPass::domStates::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "program" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass,elemProgram) );
	mea->setElementType( domGlsl_program::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "evaluate" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass,elemEvaluate) );
	mea->setElementType( domProfile_glsl::domTechnique::domPass::domEvaluate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 4 );
	meta->setCMRoot( cm );	

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domProfile_glsl::domTechnique::domPass , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_glsl::domTechnique::domPass));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_glsl::domTechnique::domPass::domStates::create(DAE& dae)
{
	domProfile_glsl::domTechnique::domPass::domStatesRef ref = new domProfile_glsl::domTechnique::domPass::domStates(dae);
	return ref;
}


daeMetaElement *
domProfile_glsl::domTechnique::domPass::domStates::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "states" );
	meta->registerClass(domProfile_glsl::domTechnique::domPass::domStates::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "gl_pipeline_settings" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass::domStates,elemGl_pipeline_settings_array) );
	mea->setElementType( domGl_pipeline_settings::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 0, -1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domProfile_glsl::domTechnique::domPass::domStates));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_glsl::domTechnique::domPass::domEvaluate::create(DAE& dae)
{
	domProfile_glsl::domTechnique::domPass::domEvaluateRef ref = new domProfile_glsl::domTechnique::domPass::domEvaluate(dae);
	return ref;
}


daeMetaElement *
domProfile_glsl::domTechnique::domPass::domEvaluate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "evaluate" );
	meta->registerClass(domProfile_glsl::domTechnique::domPass::domEvaluate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "color_target" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass::domEvaluate,elemColor_target_array) );
	mea->setElementType( domFx_colortarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "depth_target" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass::domEvaluate,elemDepth_target_array) );
	mea->setElementType( domFx_depthtarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "stencil_target" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass::domEvaluate,elemStencil_target_array) );
	mea->setElementType( domFx_stenciltarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "color_clear" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass::domEvaluate,elemColor_clear_array) );
	mea->setElementType( domFx_clearcolor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "depth_clear" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass::domEvaluate,elemDepth_clear_array) );
	mea->setElementType( domFx_cleardepth::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 5, 0, -1 );
	mea->setName( "stencil_clear" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass::domEvaluate,elemStencil_clear_array) );
	mea->setElementType( domFx_clearstencil::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "draw" );
	mea->setOffset( daeOffsetOf(domProfile_glsl::domTechnique::domPass::domEvaluate,elemDraw) );
	mea->setElementType( domProfile_glsl::domTechnique::domPass::domEvaluate::domDraw::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 6 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domProfile_glsl::domTechnique::domPass::domEvaluate));
	meta->validate();

	return meta;
}

daeElementRef
domProfile_glsl::domTechnique::domPass::domEvaluate::domDraw::create(DAE& dae)
{
	domProfile_glsl::domTechnique::domPass::domEvaluate::domDrawRef ref = new domProfile_glsl::domTechnique::domPass::domEvaluate::domDraw(dae);
	return ref;
}


daeMetaElement *
domProfile_glsl::domTechnique::domPass::domEvaluate::domDraw::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "draw" );
	meta->registerClass(domProfile_glsl::domTechnique::domPass::domEvaluate::domDraw::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_draw"));
		ma->setOffset( daeOffsetOf( domProfile_glsl::domTechnique::domPass::domEvaluate::domDraw , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domProfile_glsl::domTechnique::domPass::domEvaluate::domDraw));
	meta->validate();

	return meta;
}

