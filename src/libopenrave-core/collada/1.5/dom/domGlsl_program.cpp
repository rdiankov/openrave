#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGlsl_program.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGlsl_program::create(DAE& dae)
{
	domGlsl_programRef ref = new domGlsl_program(dae);
	return ref;
}


daeMetaElement *
domGlsl_program::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "glsl_program" );
	meta->registerClass(domGlsl_program::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "shader" );
	mea->setOffset( daeOffsetOf(domGlsl_program,elemShader_array) );
	mea->setElementType( domGlsl_shader::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "bind_attribute" );
	mea->setOffset( daeOffsetOf(domGlsl_program,elemBind_attribute_array) );
	mea->setElementType( domGlsl_program::domBind_attribute::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "bind_uniform" );
	mea->setOffset( daeOffsetOf(domGlsl_program,elemBind_uniform_array) );
	mea->setElementType( domGlsl_program::domBind_uniform::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGlsl_program));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_program::domBind_attribute::create(DAE& dae)
{
	domGlsl_program::domBind_attributeRef ref = new domGlsl_program::domBind_attribute(dae);
	return ref;
}


daeMetaElement *
domGlsl_program::domBind_attribute::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_attribute" );
	meta->registerClass(domGlsl_program::domBind_attribute::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "semantic" );
	mea->setOffset( daeOffsetOf(domGlsl_program::domBind_attribute,elemSemantic) );
	mea->setElementType( domGlsl_program::domBind_attribute::domSemantic::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGlsl_program::domBind_attribute,_contents));
	meta->addContentsOrder(daeOffsetOf(domGlsl_program::domBind_attribute,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGlsl_program::domBind_attribute,_CMData), 1);
	//	Add attribute: symbol
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGlsl_program::domBind_attribute , attrSymbol ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_program::domBind_attribute));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_program::domBind_attribute::domSemantic::create(DAE& dae)
{
	domGlsl_program::domBind_attribute::domSemanticRef ref = new domGlsl_program::domBind_attribute::domSemantic(dae);
	return ref;
}


daeMetaElement *
domGlsl_program::domBind_attribute::domSemantic::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "semantic" );
	meta->registerClass(domGlsl_program::domBind_attribute::domSemantic::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGlsl_program::domBind_attribute::domSemantic , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_program::domBind_attribute::domSemantic));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_program::domBind_uniform::create(DAE& dae)
{
	domGlsl_program::domBind_uniformRef ref = new domGlsl_program::domBind_uniform(dae);
	return ref;
}


daeMetaElement *
domGlsl_program::domBind_uniform::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_uniform" );
	meta->registerClass(domGlsl_program::domBind_uniform::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domGlsl_program::domBind_uniform,elemParam) );
	mea->setElementType( domGlsl_program::domBind_uniform::domParam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "glsl_value" );
	mea->setOffset( daeOffsetOf(domGlsl_program::domBind_uniform,elemGlsl_value) );
	mea->setElementType( domGlsl_value::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 1, 1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGlsl_program::domBind_uniform,_contents));
	meta->addContentsOrder(daeOffsetOf(domGlsl_program::domBind_uniform,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGlsl_program::domBind_uniform,_CMData), 1);
	//	Add attribute: symbol
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGlsl_program::domBind_uniform , attrSymbol ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_program::domBind_uniform));
	meta->validate();

	return meta;
}

daeElementRef
domGlsl_program::domBind_uniform::domParam::create(DAE& dae)
{
	domGlsl_program::domBind_uniform::domParamRef ref = new domGlsl_program::domBind_uniform::domParam(dae);
	return ref;
}


daeMetaElement *
domGlsl_program::domBind_uniform::domParam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "param" );
	meta->registerClass(domGlsl_program::domBind_uniform::domParam::create);

	meta->setIsInnerClass( true );

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGlsl_program::domBind_uniform::domParam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGlsl_program::domBind_uniform::domParam));
	meta->validate();

	return meta;
}

