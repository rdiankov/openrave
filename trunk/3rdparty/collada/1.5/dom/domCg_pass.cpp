#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCg_pass.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCg_pass::create(DAE& dae)
{
	domCg_passRef ref = new domCg_pass(dae);
	return ref;
}


daeMetaElement *
domCg_pass::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cg_pass" );
	meta->registerClass(domCg_pass::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domCg_pass,elemAnnotate_array) );
	mea->setElementType( domFx_annotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "states" );
	mea->setOffset( daeOffsetOf(domCg_pass,elemStates) );
	mea->setElementType( domCg_pass::domStates::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "program" );
	mea->setOffset( daeOffsetOf(domCg_pass,elemProgram) );
	mea->setElementType( domCg_pass::domProgram::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "evaluate" );
	mea->setOffset( daeOffsetOf(domCg_pass,elemEvaluate) );
	mea->setElementType( domCg_pass::domEvaluate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCg_pass,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 4 );
	meta->setCMRoot( cm );	

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domCg_pass , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_pass));
	meta->validate();

	return meta;
}

daeElementRef
domCg_pass::domStates::create(DAE& dae)
{
	domCg_pass::domStatesRef ref = new domCg_pass::domStates(dae);
	return ref;
}


daeMetaElement *
domCg_pass::domStates::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "states" );
	meta->registerClass(domCg_pass::domStates::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "gl_pipeline_settings" );
	mea->setOffset( daeOffsetOf(domCg_pass::domStates,elemGl_pipeline_settings_array) );
	mea->setElementType( domGl_pipeline_settings::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 0, -1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domCg_pass::domStates));
	meta->validate();

	return meta;
}

daeElementRef
domCg_pass::domProgram::create(DAE& dae)
{
	domCg_pass::domProgramRef ref = new domCg_pass::domProgram(dae);
	return ref;
}


daeMetaElement *
domCg_pass::domProgram::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "program" );
	meta->registerClass(domCg_pass::domProgram::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "shader" );
	mea->setOffset( daeOffsetOf(domCg_pass::domProgram,elemShader_array) );
	mea->setElementType( domCg_pass::domProgram::domShader::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domCg_pass::domProgram));
	meta->validate();

	return meta;
}

daeElementRef
domCg_pass::domProgram::domShader::create(DAE& dae)
{
	domCg_pass::domProgram::domShaderRef ref = new domCg_pass::domProgram::domShader(dae);
	return ref;
}


daeMetaElement *
domCg_pass::domProgram::domShader::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "shader" );
	meta->registerClass(domCg_pass::domProgram::domShader::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sources" );
	mea->setOffset( daeOffsetOf(domCg_pass::domProgram::domShader,elemSources) );
	mea->setElementType( domCg_pass::domProgram::domShader::domSources::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "compiler" );
	mea->setOffset( daeOffsetOf(domCg_pass::domProgram::domShader,elemCompiler_array) );
	mea->setElementType( domFx_target::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "bind_uniform" );
	mea->setOffset( daeOffsetOf(domCg_pass::domProgram::domShader,elemBind_uniform_array) );
	mea->setElementType( domCg_pass::domProgram::domShader::domBind_uniform::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: stage
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "stage" );
		ma->setType( dae.getAtomicTypes().get("Fx_pipeline_stage"));
		ma->setOffset( daeOffsetOf( domCg_pass::domProgram::domShader , attrStage ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_pass::domProgram::domShader));
	meta->validate();

	return meta;
}

daeElementRef
domCg_pass::domProgram::domShader::domSources::create(DAE& dae)
{
	domCg_pass::domProgram::domShader::domSourcesRef ref = new domCg_pass::domProgram::domShader::domSources(dae);
	return ref;
}


daeMetaElement *
domCg_pass::domProgram::domShader::domSources::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "sources" );
	meta->registerClass(domCg_pass::domProgram::domShader::domSources::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "inline" );
	mea->setOffset( daeOffsetOf(domCg_pass::domProgram::domShader::domSources,elemInline_array) );
	mea->setElementType( domInline::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "import" );
	mea->setOffset( daeOffsetOf(domCg_pass::domProgram::domShader::domSources,elemImport_array) );
	mea->setElementType( domImport::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 3000 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 3000 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domCg_pass::domProgram::domShader::domSources,_contents));
	meta->addContentsOrder(daeOffsetOf(domCg_pass::domProgram::domShader::domSources,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domCg_pass::domProgram::domShader::domSources,_CMData), 1);
	//	Add attribute: entry
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "entry" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domCg_pass::domProgram::domShader::domSources , attrEntry ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_pass::domProgram::domShader::domSources));
	meta->validate();

	return meta;
}

daeElementRef
domCg_pass::domProgram::domShader::domBind_uniform::create(DAE& dae)
{
	domCg_pass::domProgram::domShader::domBind_uniformRef ref = new domCg_pass::domProgram::domShader::domBind_uniform(dae);
	return ref;
}


daeMetaElement *
domCg_pass::domProgram::domShader::domBind_uniform::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_uniform" );
	meta->registerClass(domCg_pass::domProgram::domShader::domBind_uniform::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domCg_pass::domProgram::domShader::domBind_uniform,elemParam) );
	mea->setElementType( domCg_pass::domProgram::domShader::domBind_uniform::domParam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "cg_param" );
	mea->setOffset( daeOffsetOf(domCg_pass::domProgram::domShader::domBind_uniform,elemCg_param) );
	mea->setElementType( domCg_param::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 1, 1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domCg_pass::domProgram::domShader::domBind_uniform,_contents));
	meta->addContentsOrder(daeOffsetOf(domCg_pass::domProgram::domShader::domBind_uniform,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domCg_pass::domProgram::domShader::domBind_uniform,_CMData), 1);
	//	Add attribute: symbol
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCg_pass::domProgram::domShader::domBind_uniform , attrSymbol ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_pass::domProgram::domShader::domBind_uniform));
	meta->validate();

	return meta;
}

daeElementRef
domCg_pass::domProgram::domShader::domBind_uniform::domParam::create(DAE& dae)
{
	domCg_pass::domProgram::domShader::domBind_uniform::domParamRef ref = new domCg_pass::domProgram::domShader::domBind_uniform::domParam(dae);
	return ref;
}


daeMetaElement *
domCg_pass::domProgram::domShader::domBind_uniform::domParam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "param" );
	meta->registerClass(domCg_pass::domProgram::domShader::domBind_uniform::domParam::create);

	meta->setIsInnerClass( true );

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCg_pass::domProgram::domShader::domBind_uniform::domParam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_pass::domProgram::domShader::domBind_uniform::domParam));
	meta->validate();

	return meta;
}

daeElementRef
domCg_pass::domEvaluate::create(DAE& dae)
{
	domCg_pass::domEvaluateRef ref = new domCg_pass::domEvaluate(dae);
	return ref;
}


daeMetaElement *
domCg_pass::domEvaluate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "evaluate" );
	meta->registerClass(domCg_pass::domEvaluate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "color_target" );
	mea->setOffset( daeOffsetOf(domCg_pass::domEvaluate,elemColor_target_array) );
	mea->setElementType( domFx_colortarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "depth_target" );
	mea->setOffset( daeOffsetOf(domCg_pass::domEvaluate,elemDepth_target_array) );
	mea->setElementType( domFx_depthtarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "stencil_target" );
	mea->setOffset( daeOffsetOf(domCg_pass::domEvaluate,elemStencil_target_array) );
	mea->setElementType( domFx_stenciltarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "color_clear" );
	mea->setOffset( daeOffsetOf(domCg_pass::domEvaluate,elemColor_clear_array) );
	mea->setElementType( domFx_clearcolor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "depth_clear" );
	mea->setOffset( daeOffsetOf(domCg_pass::domEvaluate,elemDepth_clear_array) );
	mea->setElementType( domFx_cleardepth::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 5, 0, -1 );
	mea->setName( "stencil_clear" );
	mea->setOffset( daeOffsetOf(domCg_pass::domEvaluate,elemStencil_clear_array) );
	mea->setElementType( domFx_clearstencil::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "draw" );
	mea->setOffset( daeOffsetOf(domCg_pass::domEvaluate,elemDraw) );
	mea->setElementType( domCg_pass::domEvaluate::domDraw::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 6 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domCg_pass::domEvaluate));
	meta->validate();

	return meta;
}

daeElementRef
domCg_pass::domEvaluate::domDraw::create(DAE& dae)
{
	domCg_pass::domEvaluate::domDrawRef ref = new domCg_pass::domEvaluate::domDraw(dae);
	return ref;
}


daeMetaElement *
domCg_pass::domEvaluate::domDraw::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "draw" );
	meta->registerClass(domCg_pass::domEvaluate::domDraw::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_draw"));
		ma->setOffset( daeOffsetOf( domCg_pass::domEvaluate::domDraw , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_pass::domEvaluate::domDraw));
	meta->validate();

	return meta;
}

