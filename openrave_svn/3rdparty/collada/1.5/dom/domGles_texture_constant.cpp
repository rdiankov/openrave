#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_texture_constant.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_texture_constant::create(DAE& dae)
{
	domGles_texture_constantRef ref = new domGles_texture_constant(dae);
	return ref;
}


daeMetaElement *
domGles_texture_constant::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_texture_constant" );
	meta->registerClass(domGles_texture_constant::create);


	//	Add attribute: value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "value" );
		ma->setType( dae.getAtomicTypes().get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_texture_constant , attrValue ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: param
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_constant , attrParam ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domGles_texture_constant));
	meta->validate();

	return meta;
}

