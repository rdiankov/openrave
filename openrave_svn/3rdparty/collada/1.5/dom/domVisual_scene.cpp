#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domVisual_scene.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domVisual_scene::create(DAE& dae)
{
	domVisual_sceneRef ref = new domVisual_scene(dae);
	return ref;
}


daeMetaElement *
domVisual_scene::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "visual_scene" );
	meta->registerClass(domVisual_scene::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domVisual_scene,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 1, -1 );
	mea->setName( "node" );
	mea->setOffset( daeOffsetOf(domVisual_scene,elemNode_array) );
	mea->setElementType( domNode::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "evaluate_scene" );
	mea->setOffset( daeOffsetOf(domVisual_scene,elemEvaluate_scene_array) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domVisual_scene,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domVisual_scene , attrId ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domVisual_scene , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domVisual_scene));
	meta->validate();

	return meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::create(DAE& dae)
{
	domVisual_scene::domEvaluate_sceneRef ref = new domVisual_scene::domEvaluate_scene(dae);
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "evaluate_scene" );
	meta->registerClass(domVisual_scene::domEvaluate_scene::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene,elemAsset) );
	mea->setElementType( domAsset::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "render" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene,elemRender_array) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::domRender::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene , attrId ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: enable
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "enable" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene , attrEnable ));
		ma->setContainer( meta );
		ma->setDefaultString( "true");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene));
	meta->validate();

	return meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::domRender::create(DAE& dae)
{
	domVisual_scene::domEvaluate_scene::domRenderRef ref = new domVisual_scene::domEvaluate_scene::domRender(dae);
	ref->attrCamera_node.setContainer( (domVisual_scene::domEvaluate_scene::domRender*)ref );
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::domRender::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "render" );
	meta->registerClass(domVisual_scene::domEvaluate_scene::domRender::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "layer" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene::domRender,elemLayer_array) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::domRender::domLayer::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "instance_material" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene::domRender,elemInstance_material) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::domRender::domInstance_material::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene::domRender,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: camera_node
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "camera_node" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender , attrCamera_node ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene::domRender));
	meta->validate();

	return meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::domRender::domLayer::create(DAE& dae)
{
	domVisual_scene::domEvaluate_scene::domRender::domLayerRef ref = new domVisual_scene::domEvaluate_scene::domRender::domLayer(dae);
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::domRender::domLayer::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "layer" );
	meta->registerClass(domVisual_scene::domEvaluate_scene::domRender::domLayer::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender::domLayer , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene::domRender::domLayer));
	meta->validate();

	return meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::domRender::domInstance_material::create(DAE& dae)
{
	domVisual_scene::domEvaluate_scene::domRender::domInstance_materialRef ref = new domVisual_scene::domEvaluate_scene::domRender::domInstance_material(dae);
	ref->attrUrl.setContainer( (domVisual_scene::domEvaluate_scene::domRender::domInstance_material*)ref );
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::domRender::domInstance_material::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "instance_material" );
	meta->registerClass(domVisual_scene::domEvaluate_scene::domRender::domInstance_material::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "technique_override" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene::domRender::domInstance_material,elemTechnique_override) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_override::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "bind" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene::domRender::domInstance_material,elemBind_array) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBind::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene::domRender::domInstance_material,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	//	Add attribute: url
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender::domInstance_material , attrUrl ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene::domRender::domInstance_material));
	meta->validate();

	return meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_override::create(DAE& dae)
{
	domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_overrideRef ref = new domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_override(dae);
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_override::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "technique_override" );
	meta->registerClass(domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_override::create);

	meta->setIsInnerClass( true );

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_override , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: pass
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "pass" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_override , attrPass ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domTechnique_override));
	meta->validate();

	return meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBind::create(DAE& dae)
{
	domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBindRef ref = new domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBind(dae);
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBind::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind" );
	meta->registerClass(domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBind::create);

	meta->setIsInnerClass( true );

	//	Add attribute: semantic
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "semantic" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBind , attrSemantic ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: target
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "target" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBind , attrTarget ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene::domRender::domInstance_material::domBind));
	meta->validate();

	return meta;
}

