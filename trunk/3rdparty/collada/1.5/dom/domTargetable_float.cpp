#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domTargetable_float.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domTargetable_float::create(DAE& dae)
{
	domTargetable_floatRef ref = new domTargetable_float(dae);
	return ref;
}


daeMetaElement *
domTargetable_float::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "targetable_float" );
	meta->registerClass(domTargetable_float::create);

	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domTargetable_float , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domTargetable_float , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domTargetable_float));
	meta->validate();

	return meta;
}

