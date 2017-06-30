#ifndef __dom150Joint_limits_h__
#define __dom150Joint_limits_h__

#include <dae/daeDocument.h>
#include <1.5/dom/domTypes.h>
#include <1.5/dom/domElements.h>

#include <1.5/dom/domMinmax.h>

class DAE;
namespace ColladaDOM150 {

class domJoint_limits : public daeElement
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::JOINT_LIMITS; }
	static daeInt ID() { return 459; }
	virtual daeInt typeID() const { return ID(); }

protected:  // Elements
	domMinmaxRef elemMin;
	domMinmaxRef elemMax;

public:	//Accessors and Mutators
	/**
	 * Gets the min element.
	 * @return a daeSmartRef to the min element.
	 */
	const domMinmaxRef getMin() const { return elemMin; }
	/**
	 * Gets the max element.
	 * @return a daeSmartRef to the max element.
	 */
	const domMinmaxRef getMax() const { return elemMax; }
protected:
	/**
	 * Constructor
	 */
	domJoint_limits(DAE& dae) : daeElement(dae), elemMin(), elemMax() {}
	/**
	 * Destructor
	 */
	virtual ~domJoint_limits() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domJoint_limits &operator=( const domJoint_limits &cpy ) { (void)cpy; return *this; }

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


} // ColladaDOM150
#endif
