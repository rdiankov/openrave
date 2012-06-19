#ifndef __dom150Fx_sampler3D_h__
#define __dom150Fx_sampler3D_h__

#include <dae/daeDocument.h>
#include <1.5/dom/domTypes.h>
#include <1.5/dom/domElements.h>

#include <1.5/dom/domFx_sampler.h>

class DAE;
namespace ColladaDOM150 {

#include <1.5/dom/domFx_sampler.h>
/**
 * A three-dimensional texture sampler.
 */
class domFx_sampler3D : public domFx_sampler
{
public:
	virtual COLLADA_TYPE::TypeEnum getElementType() const { return COLLADA_TYPE::FX_SAMPLER3D; }
	static daeInt ID() { return 125; }
	virtual daeInt typeID() const { return ID(); }

protected:
	/**
	 * Constructor
	 */
	domFx_sampler3D(DAE& dae) : domFx_sampler(dae) {}
	/**
	 * Destructor
	 */
	virtual ~domFx_sampler3D() {}
	/**
	 * Overloaded assignment operator
	 */
	virtual domFx_sampler3D &operator=( const domFx_sampler3D &cpy ) { (void)cpy; return *this; }

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
