#ifndef __dom150Translate_h__
#define __dom150Translate_h__

#include <dae/daeDocument.h>
#include <1.5/dom/domTypes.h>
#include <1.5/dom/domElements.h>

#include <1.5/dom/domTargetable_float3.h>

class DAE;
namespace ColladaDOM150 {

#include <1.5/dom/domTargetable_float3.h>
/**
 * The translate element contains a mathematical vector that represents the
 * distance along the  X, Y and Z-axes.
 */
class domTranslate : public domTargetable_float3
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::TRANSLATE; }
	static daeInt ID() { return 18; }
	virtual daeInt typeID() const { return ID(); }


public:	//Accessors and Mutators
protected:
	/**
	 * Constructor
	 */
	domTranslate(DAE& dae) : domTargetable_float3(dae) {}
	/**
	 * Destructor
	 */
	virtual ~domTranslate() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domTranslate &operator=( const domTranslate &cpy ) { (void)cpy; return *this; }

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
