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
#ifndef __domLibrary_sensors_h__
#define __domLibrary_sensors_h__

#include <dae/daeDocument.h>
#include <dom/domTypes.h>
#include <dom/domElements.h>

#include <dom/domAsset.h>
#include <dom/domSensor.h>
#include <dom/domExtra.h>

class DAE;

/*
 * @class domLibrary_sensors
 * @brief Store list of sensors in COLLADA's library of sensors
 *
 * This class was added to COLLADA to improve it with sensor information
 * This info is used by OpenRAVE to read sensors info
 *
 * @author Gustavo Puche Rodríguez
 */
class domLibrary_sensors : public daeElement
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::LIBRARY_SENSORS; }
	//	TODO : Search this integer
	static daeInt ID() { return 968; }
	virtual daeInt typeID() const { return ID(); }
protected:  // Attributes
	xsID attrId;
	xsToken attrName;

protected:  // Elements
	domAssetRef			elemAsset;
	domSensor_Array	elemSensor_array;
	domExtra_Array	elemExtra_array;

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
	 * Gets the asset element.
	 * @return a daeSmartRef to the asset element.
	 */
	const domAssetRef getAsset() const { return elemAsset; }
	/**
	 * Gets the Sensor element array.
	 * @return Returns a reference to the array of Sensor elements.
	 */
	domSensor_Array &getSensor_array() { return elemSensor_array; }
	/**
	 * Gets the Sensor element array.
	 * @return Returns a constant reference to the array of Sensor elements.
	 */
	const domSensor_Array &getSensor_array() const { return elemSensor_array; }
	/**
	 * Gets the extra element array.
	 * @return Returns a reference to the array of extra elements.
	 */
	domExtra_Array &getExtra_array() { return elemExtra_array; }
	/**
	 * Gets the extra element array.
	 * @return Returns a constant reference to the array of extra elements.
	 */
	const domExtra_Array &getExtra_array() const { return elemExtra_array; }
protected:
	/**
	 * Constructor
	 */
	domLibrary_sensors(DAE& dae) : daeElement(dae), attrId(), attrName(), elemAsset(), elemSensor_array(),elemExtra_array() {}
	/**
	 * Destructor
	 */
	virtual ~domLibrary_sensors() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domLibrary_sensors &operator=( const domLibrary_sensors &cpy ) { (void)cpy; return *this; }

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
