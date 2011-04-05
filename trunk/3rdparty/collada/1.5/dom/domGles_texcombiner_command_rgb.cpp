#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_texcombiner_command_rgb.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_texcombiner_command_rgb::create(DAE& dae)
{
	domGles_texcombiner_command_rgbRef ref = new domGles_texcombiner_command_rgb(dae);
	return ref;
}


daeMetaElement *
domGles_texcombiner_command_rgb::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_texcombiner_command_rgb" );
	meta->registerClass(domGles_texcombiner_command_rgb::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 3 );
	mea->setName( "argument" );
	mea->setOffset( daeOffsetOf(domGles_texcombiner_command_rgb,elemArgument_array) );
	mea->setElementType( domGles_texcombiner_argument_rgb::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: operator
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "operator" );
		ma->setType( dae.getAtomicTypes().get("Gles_texcombiner_operator_rgb"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_command_rgb , attrOperator ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: scale
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "scale" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_command_rgb , attrScale ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_texcombiner_command_rgb));
	meta->validate();

	return meta;
}

