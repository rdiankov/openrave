#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domGles_texcombiner_command.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_texcombiner_command::create(DAE& dae)
{
	domGles_texcombiner_commandRef ref = new domGles_texcombiner_command(dae);
	return ref;
}


daeMetaElement *
domGles_texcombiner_command::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "gles_texcombiner_command" );
	meta->registerClass(domGles_texcombiner_command::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "constant" );
	mea->setOffset( daeOffsetOf(domGles_texcombiner_command,elemConstant) );
	mea->setElementType( domGles_texture_constant::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "RGB" );
	mea->setOffset( daeOffsetOf(domGles_texcombiner_command,elemRGB) );
	mea->setElementType( domGles_texcombiner_command_rgb::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "alpha" );
	mea->setOffset( daeOffsetOf(domGles_texcombiner_command,elemAlpha) );
	mea->setElementType( domGles_texcombiner_command_alpha::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domGles_texcombiner_command));
	meta->validate();

	return meta;
}

