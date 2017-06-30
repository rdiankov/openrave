#ifndef __dom150Instance_camera_h__
#define __dom150Instance_camera_h__

#include <dae/daeDocument.h>
#include <1.5/dom/domTypes.h>
#include <1.5/dom/domElements.h>

#include <1.5/dom/domInstance_with_extra.h>

class DAE;
namespace ColladaDOM150 {

#include <1.5/dom/domInstance_with_extra.h>
/**
 * The instance_camera element declares the instantiation of a COLLADA camera
 * resource.
 */
class domInstance_camera : public domInstance_with_extra
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::INSTANCE_CAMERA; }
	static daeInt ID() { return 100; }
	virtual daeInt typeID() const { return ID(); }

protected:
	/**
	 * Constructor
	 */
	domInstance_camera(DAE& dae) : domInstance_with_extra(dae) {}
	/**
	 * Destructor
	 */
	virtual ~domInstance_camera() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domInstance_camera &operator=( const domInstance_camera &cpy ) { (void)cpy; return *this; }

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
