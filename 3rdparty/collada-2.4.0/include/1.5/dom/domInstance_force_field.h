#ifndef __dom150Instance_force_field_h__
#define __dom150Instance_force_field_h__

#include <dae/daeDocument.h>
#include <1.5/dom/domTypes.h>
#include <1.5/dom/domElements.h>

#include <1.5/dom/domInstance_with_extra.h>

class DAE;
namespace ColladaDOM150 {

#include <1.5/dom/domInstance_with_extra.h>
/**
 * The instance_force_field element declares the instantiation of a COLLADA
 * force_field resource.
 */
class domInstance_force_field : public domInstance_with_extra
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INSTANCE_FORCE_FIELD; }
	static daeInt ID() { return 101; }
	virtual daeInt typeID() const { return ID(); }

protected:
	/**
	 * Constructor
	 */
	domInstance_force_field(DAE& dae) : domInstance_with_extra(dae) {}
	/**
	 * Destructor
	 */
	virtual ~domInstance_force_field() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domInstance_force_field &operator=( const domInstance_force_field &cpy ) { (void)cpy; return *this; }

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
