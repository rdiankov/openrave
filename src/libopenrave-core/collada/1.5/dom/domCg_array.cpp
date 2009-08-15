#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCg_array.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCg_array::create(DAE& dae)
{
	domCg_arrayRef ref = new domCg_array(dae);
	return ref;
}


daeMetaElement *
domCg_array::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cg_array" );
	meta->registerClass(domCg_array::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, -1 );
	mea->setName( "cg_param" );
	mea->setOffset( daeOffsetOf(domCg_array,elemCg_param_array) );
	mea->setElementType( domCg_param::registerElement(dae) );
	cm->appendChild( new daeMetaGroup( mea, meta, cm, 0, 1, -1 ) );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: length
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "length" );
		ma->setType( dae.getAtomicTypes().get("xsPositiveInteger"));
		ma->setOffset( daeOffsetOf( domCg_array , attrLength ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: resizable
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "resizable" );
		ma->setType( dae.getAtomicTypes().get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domCg_array , attrResizable ));
		ma->setContainer( meta );
		ma->setDefaultString( "false");
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCg_array));
	meta->validate();

	return meta;
}

