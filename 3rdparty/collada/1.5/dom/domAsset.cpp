#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domAsset.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domAsset::create(DAE& dae)
{
	domAssetRef ref = new domAsset(dae);
	return ref;
}


daeMetaElement *
domAsset::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "asset" );
	meta->registerClass(domAsset::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "contributor" );
	mea->setOffset( daeOffsetOf(domAsset,elemContributor_array) );
	mea->setElementType( domAsset::domContributor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "coverage" );
	mea->setOffset( daeOffsetOf(domAsset,elemCoverage) );
	mea->setElementType( domAsset::domCoverage::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "created" );
	mea->setOffset( daeOffsetOf(domAsset,elemCreated) );
	mea->setElementType( domAsset::domCreated::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "keywords" );
	mea->setOffset( daeOffsetOf(domAsset,elemKeywords) );
	mea->setElementType( domAsset::domKeywords::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 1, 1 );
	mea->setName( "modified" );
	mea->setOffset( daeOffsetOf(domAsset,elemModified) );
	mea->setElementType( domAsset::domModified::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "revision" );
	mea->setOffset( daeOffsetOf(domAsset,elemRevision) );
	mea->setElementType( domAsset::domRevision::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "subject" );
	mea->setOffset( daeOffsetOf(domAsset,elemSubject) );
	mea->setElementType( domAsset::domSubject::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 7, 0, 1 );
	mea->setName( "title" );
	mea->setOffset( daeOffsetOf(domAsset,elemTitle) );
	mea->setElementType( domAsset::domTitle::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 8, 0, 1 );
	mea->setName( "unit" );
	mea->setOffset( daeOffsetOf(domAsset,elemUnit) );
	mea->setElementType( domAsset::domUnit::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 9, 0, 1 );
	mea->setName( "up_axis" );
	mea->setOffset( daeOffsetOf(domAsset,elemUp_axis) );
	mea->setElementType( domAsset::domUp_axis::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 10, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domAsset,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 10 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domAsset));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domContributor::create(DAE& dae)
{
	domAsset::domContributorRef ref = new domAsset::domContributor(dae);
	return ref;
}


daeMetaElement *
domAsset::domContributor::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "contributor" );
	meta->registerClass(domAsset::domContributor::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "author" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemAuthor) );
	mea->setElementType( domAsset::domContributor::domAuthor::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "author_email" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemAuthor_email) );
	mea->setElementType( domAsset::domContributor::domAuthor_email::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "author_website" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemAuthor_website) );
	mea->setElementType( domAsset::domContributor::domAuthor_website::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "authoring_tool" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemAuthoring_tool) );
	mea->setElementType( domAsset::domContributor::domAuthoring_tool::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "comments" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemComments) );
	mea->setElementType( domAsset::domContributor::domComments::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "copyright" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemCopyright) );
	mea->setElementType( domAsset::domContributor::domCopyright::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "source_data" );
	mea->setOffset( daeOffsetOf(domAsset::domContributor,elemSource_data) );
	mea->setElementType( domAsset::domContributor::domSource_data::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 6 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domAsset::domContributor));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domContributor::domAuthor::create(DAE& dae)
{
	domAsset::domContributor::domAuthorRef ref = new domAsset::domContributor::domAuthor(dae);
	return ref;
}


daeMetaElement *
domAsset::domContributor::domAuthor::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "author" );
	meta->registerClass(domAsset::domContributor::domAuthor::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domAuthor , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domContributor::domAuthor));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domContributor::domAuthor_email::create(DAE& dae)
{
	domAsset::domContributor::domAuthor_emailRef ref = new domAsset::domContributor::domAuthor_email(dae);
	return ref;
}


daeMetaElement *
domAsset::domContributor::domAuthor_email::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "author_email" );
	meta->registerClass(domAsset::domContributor::domAuthor_email::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domAuthor_email , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domContributor::domAuthor_email));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domContributor::domAuthor_website::create(DAE& dae)
{
	domAsset::domContributor::domAuthor_websiteRef ref = new domAsset::domContributor::domAuthor_website(dae);
	ref->_value.setContainer( (domAsset::domContributor::domAuthor_website*)ref );
	return ref;
}


daeMetaElement *
domAsset::domContributor::domAuthor_website::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "author_website" );
	meta->registerClass(domAsset::domContributor::domAuthor_website::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domAuthor_website , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domContributor::domAuthor_website));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domContributor::domAuthoring_tool::create(DAE& dae)
{
	domAsset::domContributor::domAuthoring_toolRef ref = new domAsset::domContributor::domAuthoring_tool(dae);
	return ref;
}


daeMetaElement *
domAsset::domContributor::domAuthoring_tool::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "authoring_tool" );
	meta->registerClass(domAsset::domContributor::domAuthoring_tool::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domAuthoring_tool , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domContributor::domAuthoring_tool));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domContributor::domComments::create(DAE& dae)
{
	domAsset::domContributor::domCommentsRef ref = new domAsset::domContributor::domComments(dae);
	return ref;
}


daeMetaElement *
domAsset::domContributor::domComments::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "comments" );
	meta->registerClass(domAsset::domContributor::domComments::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domComments , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domContributor::domComments));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domContributor::domCopyright::create(DAE& dae)
{
	domAsset::domContributor::domCopyrightRef ref = new domAsset::domContributor::domCopyright(dae);
	return ref;
}


daeMetaElement *
domAsset::domContributor::domCopyright::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "copyright" );
	meta->registerClass(domAsset::domContributor::domCopyright::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domCopyright , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domContributor::domCopyright));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domContributor::domSource_data::create(DAE& dae)
{
	domAsset::domContributor::domSource_dataRef ref = new domAsset::domContributor::domSource_data(dae);
	ref->_value.setContainer( (domAsset::domContributor::domSource_data*)ref );
	return ref;
}


daeMetaElement *
domAsset::domContributor::domSource_data::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "source_data" );
	meta->registerClass(domAsset::domContributor::domSource_data::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domAsset::domContributor::domSource_data , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domContributor::domSource_data));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domCoverage::create(DAE& dae)
{
	domAsset::domCoverageRef ref = new domAsset::domCoverage(dae);
	return ref;
}


daeMetaElement *
domAsset::domCoverage::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "coverage" );
	meta->registerClass(domAsset::domCoverage::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "geographic_location" );
	mea->setOffset( daeOffsetOf(domAsset::domCoverage,elemGeographic_location) );
	mea->setElementType( domAsset::domCoverage::domGeographic_location::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domAsset::domCoverage));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domCoverage::domGeographic_location::create(DAE& dae)
{
	domAsset::domCoverage::domGeographic_locationRef ref = new domAsset::domCoverage::domGeographic_location(dae);
	return ref;
}


daeMetaElement *
domAsset::domCoverage::domGeographic_location::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "geographic_location" );
	meta->registerClass(domAsset::domCoverage::domGeographic_location::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "longitude" );
	mea->setOffset( daeOffsetOf(domAsset::domCoverage::domGeographic_location,elemLongitude) );
	mea->setElementType( domAsset::domCoverage::domGeographic_location::domLongitude::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "latitude" );
	mea->setOffset( daeOffsetOf(domAsset::domCoverage::domGeographic_location,elemLatitude) );
	mea->setElementType( domAsset::domCoverage::domGeographic_location::domLatitude::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "altitude" );
	mea->setOffset( daeOffsetOf(domAsset::domCoverage::domGeographic_location,elemAltitude) );
	mea->setElementType( domAsset::domCoverage::domGeographic_location::domAltitude::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domAsset::domCoverage::domGeographic_location));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domCoverage::domGeographic_location::domLongitude::create(DAE& dae)
{
	domAsset::domCoverage::domGeographic_location::domLongitudeRef ref = new domAsset::domCoverage::domGeographic_location::domLongitude(dae);
	return ref;
}


daeMetaElement *
domAsset::domCoverage::domGeographic_location::domLongitude::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "longitude" );
	meta->registerClass(domAsset::domCoverage::domGeographic_location::domLongitude::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domAsset::domCoverage::domGeographic_location::domLongitude , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domCoverage::domGeographic_location::domLongitude));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domCoverage::domGeographic_location::domLatitude::create(DAE& dae)
{
	domAsset::domCoverage::domGeographic_location::domLatitudeRef ref = new domAsset::domCoverage::domGeographic_location::domLatitude(dae);
	return ref;
}


daeMetaElement *
domAsset::domCoverage::domGeographic_location::domLatitude::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "latitude" );
	meta->registerClass(domAsset::domCoverage::domGeographic_location::domLatitude::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domAsset::domCoverage::domGeographic_location::domLatitude , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domCoverage::domGeographic_location::domLatitude));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domCoverage::domGeographic_location::domAltitude::create(DAE& dae)
{
	domAsset::domCoverage::domGeographic_location::domAltitudeRef ref = new domAsset::domCoverage::domGeographic_location::domAltitude(dae);
	return ref;
}


daeMetaElement *
domAsset::domCoverage::domGeographic_location::domAltitude::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "altitude" );
	meta->registerClass(domAsset::domCoverage::domGeographic_location::domAltitude::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domAsset::domCoverage::domGeographic_location::domAltitude , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: mode
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "mode" );
		ma->setType( dae.getAtomicTypes().get("Altitude_mode"));
		ma->setOffset( daeOffsetOf( domAsset::domCoverage::domGeographic_location::domAltitude , attrMode ));
		ma->setContainer( meta );
		ma->setDefaultString( "relativeToGround");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domCoverage::domGeographic_location::domAltitude));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domCreated::create(DAE& dae)
{
	domAsset::domCreatedRef ref = new domAsset::domCreated(dae);
	return ref;
}


daeMetaElement *
domAsset::domCreated::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "created" );
	meta->registerClass(domAsset::domCreated::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsDateTime"));
		ma->setOffset( daeOffsetOf( domAsset::domCreated , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domCreated));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domKeywords::create(DAE& dae)
{
	domAsset::domKeywordsRef ref = new domAsset::domKeywords(dae);
	return ref;
}


daeMetaElement *
domAsset::domKeywords::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "keywords" );
	meta->registerClass(domAsset::domKeywords::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domAsset::domKeywords , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domKeywords));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domModified::create(DAE& dae)
{
	domAsset::domModifiedRef ref = new domAsset::domModified(dae);
	return ref;
}


daeMetaElement *
domAsset::domModified::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "modified" );
	meta->registerClass(domAsset::domModified::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsDateTime"));
		ma->setOffset( daeOffsetOf( domAsset::domModified , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domModified));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domRevision::create(DAE& dae)
{
	domAsset::domRevisionRef ref = new domAsset::domRevision(dae);
	return ref;
}


daeMetaElement *
domAsset::domRevision::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "revision" );
	meta->registerClass(domAsset::domRevision::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domRevision , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domRevision));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domSubject::create(DAE& dae)
{
	domAsset::domSubjectRef ref = new domAsset::domSubject(dae);
	return ref;
}


daeMetaElement *
domAsset::domSubject::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "subject" );
	meta->registerClass(domAsset::domSubject::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domSubject , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domSubject));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domTitle::create(DAE& dae)
{
	domAsset::domTitleRef ref = new domAsset::domTitle(dae);
	return ref;
}


daeMetaElement *
domAsset::domTitle::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "title" );
	meta->registerClass(domAsset::domTitle::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domAsset::domTitle , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domTitle));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domUnit::create(DAE& dae)
{
	domAsset::domUnitRef ref = new domAsset::domUnit(dae);
	return ref;
}


daeMetaElement *
domAsset::domUnit::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "unit" );
	meta->registerClass(domAsset::domUnit::create);

	meta->setIsInnerClass( true );

	//	Add attribute: meter
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "meter" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domAsset::domUnit , attrMeter ));
		ma->setContainer( meta );
		ma->setDefaultString( "1.0");
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsNMTOKEN"));
		ma->setOffset( daeOffsetOf( domAsset::domUnit , attrName ));
		ma->setContainer( meta );
		ma->setDefaultString( "meter");
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domUnit));
	meta->validate();

	return meta;
}

daeElementRef
domAsset::domUp_axis::create(DAE& dae)
{
	domAsset::domUp_axisRef ref = new domAsset::domUp_axis(dae);
	return ref;
}


daeMetaElement *
domAsset::domUp_axis::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "up_axis" );
	meta->registerClass(domAsset::domUp_axis::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Up_axis"));
		ma->setOffset( daeOffsetOf( domAsset::domUp_axis , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAsset::domUp_axis));
	meta->validate();

	return meta;
}

