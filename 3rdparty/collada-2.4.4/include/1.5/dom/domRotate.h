#ifndef __dom150Rotate_h__
#define __dom150Rotate_h__

#include <dae/daeDocument.h>
#include <1.5/dom/domTypes.h>
#include <1.5/dom/domElements.h>

#include <1.5/dom/domTargetable_float4.h>

class DAE;
namespace ColladaDOM150 {

#include <1.5/dom/domTargetable_float4.h>
/**
 * The rotate element contains an angle and a mathematical vector that represents
 * the axis of rotation.
 */
class domRotate : public domTargetable_float4
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::ROTATE; }
	static daeInt ID() { return 369; }
	virtual daeInt typeID() const { return ID(); }


public:	//Accessors and Mutators
protected:
	/**
	 * Constructor
	 */
	domRotate(DAE& dae) : domTargetable_float4(dae) {}
	/**
	 * Destructor
	 */
	virtual ~domRotate() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domRotate &operator=( const domRotate &cpy ) { (void)cpy; return *this; }

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
