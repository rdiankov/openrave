#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles2_pass.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles2_pass::create(DAE& dae)
{
	domGles2_passRef ref = new domGles2_pass(dae);
	return ref;
}


daeMetaElement *
domGles2_pass::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles2_pass" );
	meta->registerClass(domGles2_pass::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domGles2_pass,elemAnnotate_array) );
	mea->setElementType( domFx_annotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "states" );
	mea->setOffset( daeOffsetOf(domGles2_pass,elemStates) );
	mea->setElementType( domGles2_pass::domStates::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "program" );
	mea->setOffset( daeOffsetOf(domGles2_pass,elemProgram) );
	mea->setElementType( domGles2_program::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "evaluate" );
	mea->setOffset( daeOffsetOf(domGles2_pass,elemEvaluate) );
	mea->setElementType( domGles2_pass::domEvaluate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domGles2_pass,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 4 );
	meta->setCMRoot( cm );	

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domGles2_pass , attrSid ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pass));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pass::domStates::create(DAE& dae)
{
	domGles2_pass::domStatesRef ref = new domGles2_pass::domStates(dae);
	return ref;
}


daeMetaElement *
domGles2_pass::domStates::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "states" );
	meta->registerClass(domGles2_pass::domStates::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "gles2_pipeline_settings" );
	mea->setOffset( daeOffsetOf(domGles2_pass::domStates,elemGles2_pipeline_settings_array) );
	mea->setElementType( domGles2_pipeline_settings::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 1, 1 ) );

	cm->setMaxOrdinal( 2999 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pass::domStates));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pass::domEvaluate::create(DAE& dae)
{
	domGles2_pass::domEvaluateRef ref = new domGles2_pass::domEvaluate(dae);
	return ref;
}


daeMetaElement *
domGles2_pass::domEvaluate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "evaluate" );
	meta->registerClass(domGles2_pass::domEvaluate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "color_target" );
	mea->setOffset( daeOffsetOf(domGles2_pass::domEvaluate,elemColor_target_array) );
	mea->setElementType( domFx_colortarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "depth_target" );
	mea->setOffset( daeOffsetOf(domGles2_pass::domEvaluate,elemDepth_target_array) );
	mea->setElementType( domFx_depthtarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "stencil_target" );
	mea->setOffset( daeOffsetOf(domGles2_pass::domEvaluate,elemStencil_target_array) );
	mea->setElementType( domFx_stenciltarget::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "color_clear" );
	mea->setOffset( daeOffsetOf(domGles2_pass::domEvaluate,elemColor_clear_array) );
	mea->setElementType( domFx_clearcolor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 4, 0, -1 );
	mea->setName( "stencil_clear" );
	mea->setOffset( daeOffsetOf(domGles2_pass::domEvaluate,elemStencil_clear_array) );
	mea->setElementType( domFx_clearstencil::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 5, 0, -1 );
	mea->setName( "depth_clear" );
	mea->setOffset( daeOffsetOf(domGles2_pass::domEvaluate,elemDepth_clear_array) );
	mea->setElementType( domFx_cleardepth::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "draw" );
	mea->setOffset( daeOffsetOf(domGles2_pass::domEvaluate,elemDraw) );
	mea->setElementType( domGles2_pass::domEvaluate::domDraw::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 6 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pass::domEvaluate));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pass::domEvaluate::domDraw::create(DAE& dae)
{
	domGles2_pass::domEvaluate::domDrawRef ref = new domGles2_pass::domEvaluate::domDraw(dae);
	return ref;
}


daeMetaElement *
domGles2_pass::domEvaluate::domDraw::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "draw" );
	meta->registerClass(domGles2_pass::domEvaluate::domDraw::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Fx_draw"));
		ma->setOffset( daeOffsetOf( domGles2_pass::domEvaluate::domDraw , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pass::domEvaluate::domDraw));
	meta->validate();

	return meta;
}

