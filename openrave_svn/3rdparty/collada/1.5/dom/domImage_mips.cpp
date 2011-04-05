#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domImage_mips.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domImage_mips::create(DAE& dae)
{
	domImage_mipsRef ref = new domImage_mips(dae);
	return ref;
}


daeMetaElement *
domImage_mips::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "image_mips" );
	meta->registerClass(domImage_mips::create);


	//	Add attribute: levels
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "levels" );
		ma->setType( dae.getAtomicTypes().get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domImage_mips , attrLevels ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: auto_generate
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "auto_generate" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domImage_mips , attrAuto_generate ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage_mips));
	meta->validate();

	return meta;
}

