/*
 * Copywrite 2009 Universitat Jaume I
 *
 * This file is part of OpenRAVE.
 * OpenRAVE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __domInstance_sensor_h__
#define __domInstance_sensor_h__

#include <dae/daeDocument.h>
#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domTranslate.h>
#include <dom/domRotate.h>

class DAE;

/*
 * @class domInstance_sensor
 * @brief Instance of sensor defined in Library_sensors and attached to a Robot
 *
 * This class was added to COLLADA to improve it with sensor information
 * This info is used by OpenRAVE to attach sensors to links
 *
 * @author Gustavo Puche Rodríguez
 */
class domInstance_sensor : public daeElement
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INSTANCE_SENSOR; }
	static daeInt ID() { return 970; } //	TODO : Update the number
	virtual daeInt typeID() const { return ID(); }

protected:  // Elements and attributes
	xsID		attrId;
	xsToken	attrName;
	xsToken	attrLink;
	xsAnyURI attrUrl;

	domTranslateRef	elemTranslate;
	domRotateRef		elemRotate;

public:	//Accessors and Mutators
	/**
	 * Gets the id attribute.
	 * @return Returns a xsID of the id attribute.
	 */
	xsID getId() const { return attrId; }
	/**
	 * Sets the id attribute.
	 * @param atId The new value for the id attribute.
	 */
	void setId( xsID atId ) { *(daeStringRef*)&attrId = atId;
		if( _document != NULL ) _document->changeElementID( this, attrId );
	}

	/**
	 * Gets the name attribute.
	 * @return Returns a xsToken of the name attribute.
	 */
	xsToken getName() const { return attrName; }
	/**
	 * Sets the name attribute.
	 * @param atName The new value for the name attribute.
	 */
	void setName( xsToken atName ) { *(daeStringRef*)&attrName = atName;}
	/**
	 * Gets the link attribute.
	 * @return Returns a xsToken of the link attribute.
	 */
	xsToken getLink() const { return attrLink; }
	/**
	 * Sets the link attribute.
	 * @param atLink The new value for the link attribute.
	 */
	void setLink( xsToken atLink ) { *(daeStringRef*)&attrLink = atLink;}
	/**
	 * Gets the url attribute.
	 * @return Returns a xsAnyURI reference of the url attribute.
	 */
	xsAnyURI &getUrl() { return attrUrl; }
	/**
	 * Gets the url attribute.
	 * @return Returns a constant xsAnyURI reference of the url attribute.
	 */
	const xsAnyURI &getUrl() const { return attrUrl; }
	/**
	 * Sets the url attribute.
	 * @param atUrl The new value for the url attribute.
	 */
	void setUrl( const xsAnyURI &atUrl ) { attrUrl = atUrl; }
	/**
	 * Sets the url attribute.
	 * @param atUrl The new value for the url attribute.
	 */
	void setUrl( xsString atUrl ) { attrUrl = atUrl; }
	/**
	 * Gets the translate element.
	 * @return Returns a reference to the translate element.
	 */
	domTranslateRef getTranslate() { return elemTranslate; }

	/**
	 * Gets the translate element.
	 * @return Returns a constant reference to the translate element.
	 */
	const domTranslateRef getTranslate() const { return elemTranslate; }
	/**
	 * Gets rotate element.
	 * @return Returns a reference to rotate element.
	 */
	domRotateRef getRotate() { return elemRotate; }
	/**
	 * Gets rotate element.
	 * @return Returns a constant reference to rotate element.
	 */
	const domRotateRef getRotate() const { return elemRotate; }
protected:
	/**
	 * Constructor
	 */
	domInstance_sensor(DAE& dae) : daeElement(dae), attrId(), attrName(), attrLink(), attrUrl(dae, *this), elemTranslate(), elemRotate() {}
	/**
	 * Destructor
	 */
	virtual ~domInstance_sensor() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domInstance_sensor &operator=( const domInstance_sensor &cpy ) { (void)cpy; return *this; }

public: // STATIC METHODS
	/**
	 * Creates an instance of this class and returns a daeElementRef referencing it.
	 * @return a daeElementRef referencing an instance of this object.
	 */
	static DLLSPEC daeElementRef create(DAE& dae);
	/**
	 * Creates a daeMetaElement object that describes this element in the meta object reflection framework.
	 * If a daeMetaElement already exists it will return that instead of creating a new one.
	 * @return A daeMetaElement describing this COLLADA element.
	 */
	static DLLSPEC daeMetaElement* registerElement(DAE& dae);
};


#endif
