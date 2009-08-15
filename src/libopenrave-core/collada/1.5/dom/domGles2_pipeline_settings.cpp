#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles2_pipeline_settings.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles2_pipeline_settings::create(DAE& dae)
{
	domGles2_pipeline_settingsRef ref = new domGles2_pipeline_settings(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles2_pipeline_settings" );
	meta->registerClass(domGles2_pipeline_settings::create);

	meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "blend_color" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemBlend_color) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_color::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "blend_equation" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemBlend_equation) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_equation::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "blend_equation_separate" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemBlend_equation_separate) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_equation_separate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "blend_func" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemBlend_func) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_func::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "blend_func_separate" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemBlend_func_separate) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_func_separate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "color_mask" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemColor_mask) );
	mea->setElementType( domGles2_pipeline_settings::domColor_mask::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "cull_face" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemCull_face) );
	mea->setElementType( domGles2_pipeline_settings::domCull_face::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "depth_func" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemDepth_func) );
	mea->setElementType( domGles2_pipeline_settings::domDepth_func::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "depth_mask" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemDepth_mask) );
	mea->setElementType( domGles2_pipeline_settings::domDepth_mask::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "depth_range" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemDepth_range) );
	mea->setElementType( domGles2_pipeline_settings::domDepth_range::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "front_face" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemFront_face) );
	mea->setElementType( domGles2_pipeline_settings::domFront_face::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "line_width" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemLine_width) );
	mea->setElementType( domGles2_pipeline_settings::domLine_width::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "polygon_offset" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemPolygon_offset) );
	mea->setElementType( domGles2_pipeline_settings::domPolygon_offset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "point_size" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemPoint_size) );
	mea->setElementType( domGles2_pipeline_settings::domPoint_size::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sample_coverage" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemSample_coverage) );
	mea->setElementType( domGles2_pipeline_settings::domSample_coverage::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "scissor" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemScissor) );
	mea->setElementType( domGles2_pipeline_settings::domScissor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "stencil_func" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemStencil_func) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "stencil_func_separate" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemStencil_func_separate) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func_separate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "stencil_mask" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemStencil_mask) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_mask::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "stencil_mask_separate" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemStencil_mask_separate) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_mask_separate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "stencil_op" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemStencil_op) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "stencil_op_separate" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemStencil_op_separate) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op_separate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "blend_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemBlend_enable) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "cull_face_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemCull_face_enable) );
	mea->setElementType( domGles2_pipeline_settings::domCull_face_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "depth_test_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemDepth_test_enable) );
	mea->setElementType( domGles2_pipeline_settings::domDepth_test_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "dither_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemDither_enable) );
	mea->setElementType( domGles2_pipeline_settings::domDither_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "polygon_offset_fill_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemPolygon_offset_fill_enable) );
	mea->setElementType( domGles2_pipeline_settings::domPolygon_offset_fill_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "point_size_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemPoint_size_enable) );
	mea->setElementType( domGles2_pipeline_settings::domPoint_size_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sample_alpha_to_coverage_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemSample_alpha_to_coverage_enable) );
	mea->setElementType( domGles2_pipeline_settings::domSample_alpha_to_coverage_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sample_coverage_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemSample_coverage_enable) );
	mea->setElementType( domGles2_pipeline_settings::domSample_coverage_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "scissor_test_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemScissor_test_enable) );
	mea->setElementType( domGles2_pipeline_settings::domScissor_test_enable::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "stencil_test_enable" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings,elemStencil_test_enable) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_test_enable::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGles2_pipeline_settings,_contents));
	meta->addContentsOrder(daeOffsetOf(domGles2_pipeline_settings,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGles2_pipeline_settings,_CMData), 1);
	meta->setElementSize(sizeof(domGles2_pipeline_settings));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_color::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_colorRef ref = new domGles2_pipeline_settings::domBlend_color(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_color::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "blend_color" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_color::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_color , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "0 0 0 0");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_color , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_color));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_equation::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_equationRef ref = new domGles2_pipeline_settings::domBlend_equation(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_equation::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "blend_equation" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_equation::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend_equation"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_equation , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "FUNC_ADD");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_equation , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_equation));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_equation_separate::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_equation_separateRef ref = new domGles2_pipeline_settings::domBlend_equation_separate(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_equation_separate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "blend_equation_separate" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_equation_separate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "rgb" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domBlend_equation_separate,elemRgb) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_equation_separate::domRgb::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "alpha" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domBlend_equation_separate,elemAlpha) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_equation_separate::domAlpha::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_equation_separate));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_equation_separate::domRgb::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_equation_separate::domRgbRef ref = new domGles2_pipeline_settings::domBlend_equation_separate::domRgb(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_equation_separate::domRgb::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "rgb" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_equation_separate::domRgb::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend_equation"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_equation_separate::domRgb , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "FUNC_ADD");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_equation_separate::domRgb , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_equation_separate::domRgb));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_equation_separate::domAlpha::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_equation_separate::domAlphaRef ref = new domGles2_pipeline_settings::domBlend_equation_separate::domAlpha(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_equation_separate::domAlpha::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "alpha" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_equation_separate::domAlpha::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend_equation"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_equation_separate::domAlpha , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "FUNC_ADD");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_equation_separate::domAlpha , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_equation_separate::domAlpha));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_func::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_funcRef ref = new domGles2_pipeline_settings::domBlend_func(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_func::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "blend_func" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_func::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "src" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domBlend_func,elemSrc) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_func::domSrc::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "dest" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domBlend_func,elemDest) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_func::domDest::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_func));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_func::domSrc::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_func::domSrcRef ref = new domGles2_pipeline_settings::domBlend_func::domSrc(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_func::domSrc::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "src" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_func::domSrc::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func::domSrc , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ONE");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func::domSrc , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_func::domSrc));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_func::domDest::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_func::domDestRef ref = new domGles2_pipeline_settings::domBlend_func::domDest(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_func::domDest::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "dest" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_func::domDest::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func::domDest , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ZERO");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func::domDest , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_func::domDest));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_func_separate::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_func_separateRef ref = new domGles2_pipeline_settings::domBlend_func_separate(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_func_separate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "blend_func_separate" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_func_separate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "src_rgb" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domBlend_func_separate,elemSrc_rgb) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgb::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "dest_rgb" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domBlend_func_separate,elemDest_rgb) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_func_separate::domDest_rgb::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "src_alpha" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domBlend_func_separate,elemSrc_alpha) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_func_separate::domSrc_alpha::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 1, 1 );
	mea->setName( "dest_alpha" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domBlend_func_separate,elemDest_alpha) );
	mea->setElementType( domGles2_pipeline_settings::domBlend_func_separate::domDest_alpha::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_func_separate));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgb::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgbRef ref = new domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgb(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgb::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "src_rgb" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgb::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgb , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ONE");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgb , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_func_separate::domSrc_rgb));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_func_separate::domDest_rgb::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_func_separate::domDest_rgbRef ref = new domGles2_pipeline_settings::domBlend_func_separate::domDest_rgb(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_func_separate::domDest_rgb::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "dest_rgb" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_func_separate::domDest_rgb::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func_separate::domDest_rgb , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ZERO");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func_separate::domDest_rgb , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_func_separate::domDest_rgb));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_func_separate::domSrc_alpha::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_func_separate::domSrc_alphaRef ref = new domGles2_pipeline_settings::domBlend_func_separate::domSrc_alpha(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_func_separate::domSrc_alpha::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "src_alpha" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_func_separate::domSrc_alpha::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func_separate::domSrc_alpha , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ONE");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func_separate::domSrc_alpha , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_func_separate::domSrc_alpha));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_func_separate::domDest_alpha::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_func_separate::domDest_alphaRef ref = new domGles2_pipeline_settings::domBlend_func_separate::domDest_alpha(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_func_separate::domDest_alpha::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "dest_alpha" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_func_separate::domDest_alpha::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_blend"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func_separate::domDest_alpha , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ZERO");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_func_separate::domDest_alpha , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_func_separate::domDest_alpha));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domColor_mask::create(DAE& dae)
{
	domGles2_pipeline_settings::domColor_maskRef ref = new domGles2_pipeline_settings::domColor_mask(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domColor_mask::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "color_mask" );
	meta->registerClass(domGles2_pipeline_settings::domColor_mask::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Bool4"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domColor_mask , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "true true true true");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domColor_mask , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domColor_mask));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domCull_face::create(DAE& dae)
{
	domGles2_pipeline_settings::domCull_faceRef ref = new domGles2_pipeline_settings::domCull_face(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domCull_face::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cull_face" );
	meta->registerClass(domGles2_pipeline_settings::domCull_face::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_face"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domCull_face , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "BACK");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domCull_face , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domCull_face));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domDepth_func::create(DAE& dae)
{
	domGles2_pipeline_settings::domDepth_funcRef ref = new domGles2_pipeline_settings::domDepth_func(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domDepth_func::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "depth_func" );
	meta->registerClass(domGles2_pipeline_settings::domDepth_func::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_func"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDepth_func , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ALWAYS");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDepth_func , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domDepth_func));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domDepth_mask::create(DAE& dae)
{
	domGles2_pipeline_settings::domDepth_maskRef ref = new domGles2_pipeline_settings::domDepth_mask(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domDepth_mask::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "depth_mask" );
	meta->registerClass(domGles2_pipeline_settings::domDepth_mask::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDepth_mask , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "true");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDepth_mask , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domDepth_mask));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domDepth_range::create(DAE& dae)
{
	domGles2_pipeline_settings::domDepth_rangeRef ref = new domGles2_pipeline_settings::domDepth_range(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domDepth_range::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "depth_range" );
	meta->registerClass(domGles2_pipeline_settings::domDepth_range::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDepth_range , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "0 1");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDepth_range , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domDepth_range));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domFront_face::create(DAE& dae)
{
	domGles2_pipeline_settings::domFront_faceRef ref = new domGles2_pipeline_settings::domFront_face(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domFront_face::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "front_face" );
	meta->registerClass(domGles2_pipeline_settings::domFront_face::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_front_face"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domFront_face , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "CCW");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domFront_face , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domFront_face));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domLine_width::create(DAE& dae)
{
	domGles2_pipeline_settings::domLine_widthRef ref = new domGles2_pipeline_settings::domLine_width(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domLine_width::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "line_width" );
	meta->registerClass(domGles2_pipeline_settings::domLine_width::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domLine_width , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "1");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domLine_width , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domLine_width));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domPolygon_offset::create(DAE& dae)
{
	domGles2_pipeline_settings::domPolygon_offsetRef ref = new domGles2_pipeline_settings::domPolygon_offset(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domPolygon_offset::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "polygon_offset" );
	meta->registerClass(domGles2_pipeline_settings::domPolygon_offset::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Float2"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domPolygon_offset , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "0 0");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domPolygon_offset , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domPolygon_offset));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domPoint_size::create(DAE& dae)
{
	domGles2_pipeline_settings::domPoint_sizeRef ref = new domGles2_pipeline_settings::domPoint_size(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domPoint_size::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "point_size" );
	meta->registerClass(domGles2_pipeline_settings::domPoint_size::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domPoint_size , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "1");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domPoint_size , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domPoint_size));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domSample_coverage::create(DAE& dae)
{
	domGles2_pipeline_settings::domSample_coverageRef ref = new domGles2_pipeline_settings::domSample_coverage(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domSample_coverage::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "sample_coverage" );
	meta->registerClass(domGles2_pipeline_settings::domSample_coverage::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "value" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domSample_coverage,elemValue) );
	mea->setElementType( domGles2_pipeline_settings::domSample_coverage::domValue::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "invert" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domSample_coverage,elemInvert) );
	mea->setElementType( domGles2_pipeline_settings::domSample_coverage::domInvert::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domSample_coverage));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domSample_coverage::domValue::create(DAE& dae)
{
	domGles2_pipeline_settings::domSample_coverage::domValueRef ref = new domGles2_pipeline_settings::domSample_coverage::domValue(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domSample_coverage::domValue::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "value" );
	meta->registerClass(domGles2_pipeline_settings::domSample_coverage::domValue::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domSample_coverage::domValue , attrValue ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domSample_coverage::domValue , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domSample_coverage::domValue));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domSample_coverage::domInvert::create(DAE& dae)
{
	domGles2_pipeline_settings::domSample_coverage::domInvertRef ref = new domGles2_pipeline_settings::domSample_coverage::domInvert(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domSample_coverage::domInvert::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "invert" );
	meta->registerClass(domGles2_pipeline_settings::domSample_coverage::domInvert::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domSample_coverage::domInvert , attrValue ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domSample_coverage::domInvert , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domSample_coverage::domInvert));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domScissor::create(DAE& dae)
{
	domGles2_pipeline_settings::domScissorRef ref = new domGles2_pipeline_settings::domScissor(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domScissor::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "scissor" );
	meta->registerClass(domGles2_pipeline_settings::domScissor::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Int4"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domScissor , attrValue ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domScissor , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domScissor));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_funcRef ref = new domGles2_pipeline_settings::domStencil_func(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "stencil_func" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "func" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_func,elemFunc) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func::domFunc::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_func,elemRef) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func::domRef::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "mask" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_func,elemMask) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func::domMask::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func::domFunc::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_func::domFuncRef ref = new domGles2_pipeline_settings::domStencil_func::domFunc(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func::domFunc::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "func" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func::domFunc::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_func"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func::domFunc , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ALWAYS");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func::domFunc , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func::domFunc));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func::domRef::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_func::domRefRef ref = new domGles2_pipeline_settings::domStencil_func::domRef(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func::domRef::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "ref" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func::domRef::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func::domRef , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "0");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func::domRef , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func::domRef));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func::domMask::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_func::domMaskRef ref = new domGles2_pipeline_settings::domStencil_func::domMask(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func::domMask::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mask" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func::domMask::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func::domMask , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "255");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func::domMask , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func::domMask));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func_separate::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_func_separateRef ref = new domGles2_pipeline_settings::domStencil_func_separate(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func_separate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "stencil_func_separate" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func_separate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "front" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_func_separate,elemFront) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func_separate::domFront::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "back" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_func_separate,elemBack) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func_separate::domBack::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_func_separate,elemRef) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func_separate::domRef::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 1, 1 );
	mea->setName( "mask" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_func_separate,elemMask) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_func_separate::domMask::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func_separate));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func_separate::domFront::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_func_separate::domFrontRef ref = new domGles2_pipeline_settings::domStencil_func_separate::domFront(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func_separate::domFront::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "front" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func_separate::domFront::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_func"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func_separate::domFront , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ALWAYS");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func_separate::domFront , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func_separate::domFront));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func_separate::domBack::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_func_separate::domBackRef ref = new domGles2_pipeline_settings::domStencil_func_separate::domBack(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func_separate::domBack::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "back" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func_separate::domBack::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_func"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func_separate::domBack , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "ALWAYS");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func_separate::domBack , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func_separate::domBack));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func_separate::domRef::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_func_separate::domRefRef ref = new domGles2_pipeline_settings::domStencil_func_separate::domRef(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func_separate::domRef::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "ref" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func_separate::domRef::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func_separate::domRef , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "0");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func_separate::domRef , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func_separate::domRef));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_func_separate::domMask::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_func_separate::domMaskRef ref = new domGles2_pipeline_settings::domStencil_func_separate::domMask(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_func_separate::domMask::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mask" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_func_separate::domMask::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func_separate::domMask , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "255");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_func_separate::domMask , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_func_separate::domMask));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_mask::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_maskRef ref = new domGles2_pipeline_settings::domStencil_mask(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_mask::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "stencil_mask" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_mask::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_mask , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "4294967295");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_mask , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_mask));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_mask_separate::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_mask_separateRef ref = new domGles2_pipeline_settings::domStencil_mask_separate(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_mask_separate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "stencil_mask_separate" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_mask_separate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "face" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_mask_separate,elemFace) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_mask_separate::domFace::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "mask" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_mask_separate,elemMask) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_mask_separate::domMask::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_mask_separate));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_mask_separate::domFace::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_mask_separate::domFaceRef ref = new domGles2_pipeline_settings::domStencil_mask_separate::domFace(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_mask_separate::domFace::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "face" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_mask_separate::domFace::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_face"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_mask_separate::domFace , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "FRONT_AND_BACK");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_mask_separate::domFace , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_mask_separate::domFace));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_mask_separate::domMask::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_mask_separate::domMaskRef ref = new domGles2_pipeline_settings::domStencil_mask_separate::domMask(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_mask_separate::domMask::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "mask" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_mask_separate::domMask::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_mask_separate::domMask , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "255");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_mask_separate::domMask , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_mask_separate::domMask));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_opRef ref = new domGles2_pipeline_settings::domStencil_op(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "stencil_op" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "fail" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_op,elemFail) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op::domFail::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "zfail" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_op,elemZfail) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op::domZfail::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "zpass" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_op,elemZpass) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op::domZpass::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op::domFail::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_op::domFailRef ref = new domGles2_pipeline_settings::domStencil_op::domFail(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op::domFail::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fail" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op::domFail::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_stencil_op"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op::domFail , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "KEEP");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op::domFail , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op::domFail));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op::domZfail::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_op::domZfailRef ref = new domGles2_pipeline_settings::domStencil_op::domZfail(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op::domZfail::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "zfail" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op::domZfail::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_stencil_op"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op::domZfail , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "KEEP");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op::domZfail , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op::domZfail));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op::domZpass::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_op::domZpassRef ref = new domGles2_pipeline_settings::domStencil_op::domZpass(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op::domZpass::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "zpass" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op::domZpass::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_stencil_op"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op::domZpass , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "KEEP");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op::domZpass , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op::domZpass));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op_separate::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_op_separateRef ref = new domGles2_pipeline_settings::domStencil_op_separate(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op_separate::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "stencil_op_separate" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op_separate::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "face" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_op_separate,elemFace) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op_separate::domFace::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "fail" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_op_separate,elemFail) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op_separate::domFail::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "zfail" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_op_separate,elemZfail) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op_separate::domZfail::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 1, 1 );
	mea->setName( "zpass" );
	mea->setOffset( daeOffsetOf(domGles2_pipeline_settings::domStencil_op_separate,elemZpass) );
	mea->setElementType( domGles2_pipeline_settings::domStencil_op_separate::domZpass::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op_separate));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op_separate::domFace::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_op_separate::domFaceRef ref = new domGles2_pipeline_settings::domStencil_op_separate::domFace(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op_separate::domFace::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "face" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op_separate::domFace::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_face"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op_separate::domFace , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "FRONT_AND_BACK");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op_separate::domFace , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op_separate::domFace));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op_separate::domFail::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_op_separate::domFailRef ref = new domGles2_pipeline_settings::domStencil_op_separate::domFail(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op_separate::domFail::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fail" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op_separate::domFail::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_stencil_op"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op_separate::domFail , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "KEEP");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op_separate::domFail , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op_separate::domFail));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op_separate::domZfail::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_op_separate::domZfailRef ref = new domGles2_pipeline_settings::domStencil_op_separate::domZfail(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op_separate::domZfail::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "zfail" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op_separate::domZfail::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_stencil_op"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op_separate::domZfail , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "KEEP");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op_separate::domZfail , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op_separate::domZfail));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_op_separate::domZpass::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_op_separate::domZpassRef ref = new domGles2_pipeline_settings::domStencil_op_separate::domZpass(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_op_separate::domZpass::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "zpass" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_op_separate::domZpass::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Gl_stencil_op"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op_separate::domZpass , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "KEEP");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_op_separate::domZpass , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_op_separate::domZpass));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domBlend_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domBlend_enableRef ref = new domGles2_pipeline_settings::domBlend_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domBlend_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "blend_enable" );
	meta->registerClass(domGles2_pipeline_settings::domBlend_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domBlend_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domBlend_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domCull_face_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domCull_face_enableRef ref = new domGles2_pipeline_settings::domCull_face_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domCull_face_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cull_face_enable" );
	meta->registerClass(domGles2_pipeline_settings::domCull_face_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domCull_face_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domCull_face_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domCull_face_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domDepth_test_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domDepth_test_enableRef ref = new domGles2_pipeline_settings::domDepth_test_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domDepth_test_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "depth_test_enable" );
	meta->registerClass(domGles2_pipeline_settings::domDepth_test_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDepth_test_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDepth_test_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domDepth_test_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domDither_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domDither_enableRef ref = new domGles2_pipeline_settings::domDither_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domDither_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "dither_enable" );
	meta->registerClass(domGles2_pipeline_settings::domDither_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDither_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "true");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domDither_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domDither_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domPolygon_offset_fill_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domPolygon_offset_fill_enableRef ref = new domGles2_pipeline_settings::domPolygon_offset_fill_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domPolygon_offset_fill_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "polygon_offset_fill_enable" );
	meta->registerClass(domGles2_pipeline_settings::domPolygon_offset_fill_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domPolygon_offset_fill_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domPolygon_offset_fill_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domPolygon_offset_fill_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domPoint_size_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domPoint_size_enableRef ref = new domGles2_pipeline_settings::domPoint_size_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domPoint_size_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "point_size_enable" );
	meta->registerClass(domGles2_pipeline_settings::domPoint_size_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domPoint_size_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domPoint_size_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domPoint_size_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domSample_alpha_to_coverage_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domSample_alpha_to_coverage_enableRef ref = new domGles2_pipeline_settings::domSample_alpha_to_coverage_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domSample_alpha_to_coverage_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "sample_alpha_to_coverage_enable" );
	meta->registerClass(domGles2_pipeline_settings::domSample_alpha_to_coverage_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domSample_alpha_to_coverage_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domSample_alpha_to_coverage_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domSample_alpha_to_coverage_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domSample_coverage_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domSample_coverage_enableRef ref = new domGles2_pipeline_settings::domSample_coverage_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domSample_coverage_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "sample_coverage_enable" );
	meta->registerClass(domGles2_pipeline_settings::domSample_coverage_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domSample_coverage_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domSample_coverage_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domSample_coverage_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domScissor_test_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domScissor_test_enableRef ref = new domGles2_pipeline_settings::domScissor_test_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domScissor_test_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "scissor_test_enable" );
	meta->registerClass(domGles2_pipeline_settings::domScissor_test_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domScissor_test_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domScissor_test_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domScissor_test_enable));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_pipeline_settings::domStencil_test_enable::create(DAE& dae)
{
	domGles2_pipeline_settings::domStencil_test_enableRef ref = new domGles2_pipeline_settings::domStencil_test_enable(dae);
	return ref;
}


daeMetaElement *
domGles2_pipeline_settings::domStencil_test_enable::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "stencil_test_enable" );
	meta->registerClass(domGles2_pipeline_settings::domStencil_test_enable::create);

	meta->setIsInnerClass( true );

	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_test_enable , attrValue ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_pipeline_settings::domStencil_test_enable , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_pipeline_settings::domStencil_test_enable));
	meta->validate();

	return meta;
}

