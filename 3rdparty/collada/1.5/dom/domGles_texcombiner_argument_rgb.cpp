#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_texcombiner_argument_rgb.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_texcombiner_argument_rgb::create(DAE& dae)
{
	domGles_texcombiner_argument_rgbRef ref = new domGles_texcombiner_argument_rgb(dae);
	return ref;
}


daeMetaElement *
domGles_texcombiner_argument_rgb::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_texcombiner_argument_rgb" );
	meta->registerClass(domGles_texcombiner_argument_rgb::create);


	//	Add attribute: source
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( dae.getAtomicTypes().get("Gles_texcombiner_source"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argument_rgb , attrSource ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: operand
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "operand" );
		ma->setType( dae.getAtomicTypes().get("Gles_texcombiner_operand_rgb"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argument_rgb , attrOperand ));
		ma->setContainer( meta );
		ma->setDefaultString( "SRC_COLOR");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: sampler
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sampler" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texcombiner_argument_rgb , attrSampler ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_texcombiner_argument_rgb));
	meta->validate();

	return meta;
}

