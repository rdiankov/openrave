#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles2_program.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles2_program::create(DAE& dae)
{
	domGles2_programRef ref = new domGles2_program(dae);
	return ref;
}


daeMetaElement *
domGles2_program::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles2_program" );
	meta->registerClass(domGles2_program::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "shader" );
	mea->setOffset( daeOffsetOf(domGles2_program,elemShader_array) );
	mea->setElementType( domGles2_shader::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "linker" );
	mea->setOffset( daeOffsetOf(domGles2_program,elemLinker_array) );
	mea->setElementType( domFx_target::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "bind_attribute" );
	mea->setOffset( daeOffsetOf(domGles2_program,elemBind_attribute_array) );
	mea->setElementType( domGles2_program::domBind_attribute::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "bind_uniform" );
	mea->setOffset( daeOffsetOf(domGles2_program,elemBind_uniform_array) );
	mea->setElementType( domGles2_program::domBind_uniform::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles2_program));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_program::domBind_attribute::create(DAE& dae)
{
	domGles2_program::domBind_attributeRef ref = new domGles2_program::domBind_attribute(dae);
	return ref;
}


daeMetaElement *
domGles2_program::domBind_attribute::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_attribute" );
	meta->registerClass(domGles2_program::domBind_attribute::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "semantic" );
	mea->setOffset( daeOffsetOf(domGles2_program::domBind_attribute,elemSemantic) );
	mea->setElementType( domGles2_program::domBind_attribute::domSemantic::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGles2_program::domBind_attribute,_contents));
	meta->addContentsOrder(daeOffsetOf(domGles2_program::domBind_attribute,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGles2_program::domBind_attribute,_CMData), 1);
	//	Add attribute: symbol
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_program::domBind_attribute , attrSymbol ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_program::domBind_attribute));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_program::domBind_attribute::domSemantic::create(DAE& dae)
{
	domGles2_program::domBind_attribute::domSemanticRef ref = new domGles2_program::domBind_attribute::domSemantic(dae);
	return ref;
}


daeMetaElement *
domGles2_program::domBind_attribute::domSemantic::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "semantic" );
	meta->registerClass(domGles2_program::domBind_attribute::domSemantic::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_program::domBind_attribute::domSemantic , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_program::domBind_attribute::domSemantic));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_program::domBind_uniform::create(DAE& dae)
{
	domGles2_program::domBind_uniformRef ref = new domGles2_program::domBind_uniform(dae);
	return ref;
}


daeMetaElement *
domGles2_program::domBind_uniform::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_uniform" );
	meta->registerClass(domGles2_program::domBind_uniform::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domGles2_program::domBind_uniform,elemParam) );
	mea->setElementType( domGles2_program::domBind_uniform::domParam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "gles2_value" );
	mea->setOffset( daeOffsetOf(domGles2_program::domBind_uniform,elemGles2_value) );
	mea->setElementType( domGles2_value::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 1, 1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domGles2_program::domBind_uniform,_contents));
	meta->addContentsOrder(daeOffsetOf(domGles2_program::domBind_uniform,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGles2_program::domBind_uniform,_CMData), 1);
	//	Add attribute: symbol
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_program::domBind_uniform , attrSymbol ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_program::domBind_uniform));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_program::domBind_uniform::domParam::create(DAE& dae)
{
	domGles2_program::domBind_uniform::domParamRef ref = new domGles2_program::domBind_uniform::domParam(dae);
	return ref;
}


daeMetaElement *
domGles2_program::domBind_uniform::domParam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "param" );
	meta->registerClass(domGles2_program::domBind_uniform::domParam::create);

	meta->setIsInnerClass( true );

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles2_program::domBind_uniform::domParam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_program::domBind_uniform::domParam));
	meta->validate();

	return meta;
}

