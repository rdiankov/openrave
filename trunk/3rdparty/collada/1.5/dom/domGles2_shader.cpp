#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles2_shader.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles2_shader::create(DAE& dae)
{
	domGles2_shaderRef ref = new domGles2_shader(dae);
	return ref;
}


daeMetaElement *
domGles2_shader::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles2_shader" );
	meta->registerClass(domGles2_shader::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "sources" );
	mea->setOffset( daeOffsetOf(domGles2_shader,elemSources) );
	mea->setElementType( domGles2_shader::domSources::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "compiler" );
	mea->setOffset( daeOffsetOf(domGles2_shader,elemCompiler_array) );
	mea->setElementType( domFx_target::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domGles2_shader,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: stage
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "stage" );
		ma->setType( dae.getAtomicTypes().get("Fx_pipeline_stage"));
		ma->setOffset( daeOffsetOf( domGles2_shader , attrStage ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_shader));
	meta->validate();

	return meta;
}

daeElementRef
domGles2_shader::domSources::create(DAE& dae)
{
	domGles2_shader::domSourcesRef ref = new domGles2_shader::domSources(dae);
	return ref;
}


daeMetaElement *
domGles2_shader::domSources::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "sources" );
	meta->registerClass(domGles2_shader::domSources::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "inline" );
	mea->setOffset( daeOffsetOf(domGles2_shader::domSources,elemInline_array) );
	mea->setElementType( domInline::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "import" );
	mea->setOffset( daeOffsetOf(domGles2_shader::domSources,elemImport_array) );
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
	meta->addContents(daeOffsetOf(domGles2_shader::domSources,_contents));
	meta->addContentsOrder(daeOffsetOf(domGles2_shader::domSources,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domGles2_shader::domSources,_CMData), 1);
	//	Add attribute: entry
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "entry" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domGles2_shader::domSources , attrEntry ));
		ma->setContainer( meta );
		ma->setDefaultString( "main");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles2_shader::domSources));
	meta->validate();

	return meta;
}

