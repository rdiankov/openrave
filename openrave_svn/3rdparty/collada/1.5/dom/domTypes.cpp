#include <dae.h>
#include <dom/domTypes.h>
#include <dae/daeDom.h>
#include <dom/domCOLLADA.h>


void registerDomTypes(DAE& dae)
{
	daeAtomicType* type = NULL;
	daeAtomicTypeList& atomicTypes = dae.getAtomicTypes();

	// TYPEDEF: Float	//check if this type has an existing base
	type = atomicTypes.get("xsDouble");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float");
	}
	
	// TYPEDEF: Int	//check if this type has an existing base
	type = atomicTypes.get("xsLong");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int");
	}
	
	// TYPEDEF: Uint	//check if this type has an existing base
	type = atomicTypes.get("xsUnsignedLong");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Uint");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Uint");
	}
	
	// TYPEDEF: Sidref	//check if this type has an existing base
	type = atomicTypes.get("xsString");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Sidref");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Sidref");
	}
	
	// TYPEDEF: Sid	//check if this type has an existing base
	type = atomicTypes.get("xsNCName");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Sid");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Sid");
	}
	
	// TYPEDEF: List_of_bools	//check if this type has an existing base
	type = atomicTypes.get("xsBoolean");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_bools");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_bools");
	}
	
	// TYPEDEF: List_of_floats	//check if this type has an existing base
	type = atomicTypes.get("Float");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_floats");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_floats");
	}
	
	// TYPEDEF: List_of_hex_binary	//check if this type has an existing base
	type = atomicTypes.get("xsHexBinary");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_hex_binary");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_hex_binary");
	}
	
	// TYPEDEF: List_of_ints	//check if this type has an existing base
	type = atomicTypes.get("Int");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_ints");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_ints");
	}
	
	// TYPEDEF: List_of_names	//check if this type has an existing base
	type = atomicTypes.get("xsName");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_names");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_names");
	}
	
	// TYPEDEF: List_of_idrefs	//check if this type has an existing base
	type = atomicTypes.get("xsName");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_idrefs");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_idrefs");
	}
	
	// TYPEDEF: List_of_sidrefs	//check if this type has an existing base
	type = atomicTypes.get("Sidref");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_sidrefs");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_sidrefs");
	}
	
	// TYPEDEF: List_of_tokens	//check if this type has an existing base
	type = atomicTypes.get("xsToken");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_tokens");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_tokens");
	}
	
	// TYPEDEF: List_of_uints	//check if this type has an existing base
	type = atomicTypes.get("Uint");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("List_of_uints");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("List_of_uints");
	}
	
	// TYPEDEF: Bool2	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool2");
	}
	
	// TYPEDEF: Bool3	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool3");
	}
	
	// TYPEDEF: Bool4	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool4");
	}
	
	// TYPEDEF: Bool2x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool2x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool2x2");
	}
	
	// TYPEDEF: Bool2x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool2x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool2x3");
	}
	
	// TYPEDEF: Bool2x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool2x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool2x4");
	}
	
	// TYPEDEF: Bool3x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool3x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool3x2");
	}
	
	// TYPEDEF: Bool3x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool3x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool3x3");
	}
	
	// TYPEDEF: Bool3x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool3x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool3x4");
	}
	
	// TYPEDEF: Bool4x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool4x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool4x2");
	}
	
	// TYPEDEF: Bool4x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool4x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool4x3");
	}
	
	// TYPEDEF: Bool4x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_bools");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Bool4x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Bool4x4");
	}
	
	// TYPEDEF: Float2	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float2");
	}
	
	// TYPEDEF: Float3	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float3");
	}
	
	// TYPEDEF: Float4	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float4");
	}
	
	// TYPEDEF: Float7	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float7");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float7");
	}
	
	// TYPEDEF: Float2x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float2x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float2x2");
	}
	
	// TYPEDEF: Float2x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float2x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float2x3");
	}
	
	// TYPEDEF: Float2x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float2x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float2x4");
	}
	
	// TYPEDEF: Float3x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float3x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float3x2");
	}
	
	// TYPEDEF: Float3x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float3x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float3x3");
	}
	
	// TYPEDEF: Float3x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float3x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float3x4");
	}
	
	// TYPEDEF: Float4x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float4x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float4x2");
	}
	
	// TYPEDEF: Float4x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float4x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float4x3");
	}
	
	// TYPEDEF: Float4x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_floats");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Float4x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Float4x4");
	}
	
	// TYPEDEF: Int2	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int2");
	}
	
	// TYPEDEF: Int3	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int3");
	}
	
	// TYPEDEF: Int4	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int4");
	}
	
	// TYPEDEF: Int2x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int2x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int2x2");
	}
	
	// TYPEDEF: Int2x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int2x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int2x3");
	}
	
	// TYPEDEF: Int2x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int2x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int2x4");
	}
	
	// TYPEDEF: Int3x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int3x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int3x2");
	}
	
	// TYPEDEF: Int3x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int3x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int3x3");
	}
	
	// TYPEDEF: Int3x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int3x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int3x4");
	}
	
	// TYPEDEF: Int4x2	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int4x2");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int4x2");
	}
	
	// TYPEDEF: Int4x3	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int4x3");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int4x3");
	}
	
	// TYPEDEF: Int4x4	//check if this type has an existing base
	type = atomicTypes.get("List_of_ints");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Int4x4");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Int4x4");
	}
	
	// TYPEDEF: Digits	//check if this type has an existing base
	type = atomicTypes.get("xsUnsignedByte");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Digits");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Digits");
	}
	
	// TYPEDEF: Magnitude	//check if this type has an existing base
	type = atomicTypes.get("xsShort");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Magnitude");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Magnitude");
	}
	
	// ENUM: Morph_method
	type = new daeEnumType(dae);
	type->_nameBindings.append("Morph_method");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NORMALIZED");
	((daeEnumType*)type)->_values->append(MORPH_METHOD_NORMALIZED);
	((daeEnumType*)type)->_strings->append("RELATIVE");
	((daeEnumType*)type)->_values->append(MORPH_METHOD_RELATIVE);
	atomicTypes.append( type );

	// ENUM: Node_enum
	type = new daeEnumType(dae);
	type->_nameBindings.append("Node_enum");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("JOINT");
	((daeEnumType*)type)->_values->append(NODE_ENUM_JOINT);
	((daeEnumType*)type)->_strings->append("NODE");
	((daeEnumType*)type)->_values->append(NODE_ENUM_NODE);
	atomicTypes.append( type );

	// ENUM: Sampler_behavior
	type = new daeEnumType(dae);
	type->_nameBindings.append("Sampler_behavior");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("CONSTANT");
	((daeEnumType*)type)->_values->append(SAMPLER_BEHAVIOR_CONSTANT);
	((daeEnumType*)type)->_strings->append("CYCLE");
	((daeEnumType*)type)->_values->append(SAMPLER_BEHAVIOR_CYCLE);
	((daeEnumType*)type)->_strings->append("CYCLE_RELATIVE");
	((daeEnumType*)type)->_values->append(SAMPLER_BEHAVIOR_CYCLE_RELATIVE);
	((daeEnumType*)type)->_strings->append("GRADIENT");
	((daeEnumType*)type)->_values->append(SAMPLER_BEHAVIOR_GRADIENT);
	((daeEnumType*)type)->_strings->append("OSCILLATE");
	((daeEnumType*)type)->_values->append(SAMPLER_BEHAVIOR_OSCILLATE);
	((daeEnumType*)type)->_strings->append("UNDEFINED");
	((daeEnumType*)type)->_values->append(SAMPLER_BEHAVIOR_UNDEFINED);
	atomicTypes.append( type );

	// TYPEDEF: Urifragment	//check if this type has an existing base
	type = atomicTypes.get("xsAnyURI");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Urifragment");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Urifragment");
	}
	
	// ENUM: Up_axis
	type = new daeEnumType(dae);
	type->_nameBindings.append("Up_axis");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("X_UP");
	((daeEnumType*)type)->_values->append(UP_AXIS_X_UP);
	((daeEnumType*)type)->_strings->append("Y_UP");
	((daeEnumType*)type)->_values->append(UP_AXIS_Y_UP);
	((daeEnumType*)type)->_strings->append("Z_UP");
	((daeEnumType*)type)->_values->append(UP_AXIS_Z_UP);
	atomicTypes.append( type );

	// ENUM: Version
	type = new daeEnumType(dae);
	type->_nameBindings.append("Version");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("1.5.0");
	((daeEnumType*)type)->_values->append(VERSION_1_5_0);
	atomicTypes.append( type );

	// ENUM: Image_face
	type = new daeEnumType(dae);
	type->_nameBindings.append("Image_face");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("POSITIVE_X");
	((daeEnumType*)type)->_values->append(IMAGE_FACE_POSITIVE_X);
	((daeEnumType*)type)->_strings->append("NEGATIVE_X");
	((daeEnumType*)type)->_values->append(IMAGE_FACE_NEGATIVE_X);
	((daeEnumType*)type)->_strings->append("POSITIVE_Y");
	((daeEnumType*)type)->_values->append(IMAGE_FACE_POSITIVE_Y);
	((daeEnumType*)type)->_strings->append("NEGATIVE_Y");
	((daeEnumType*)type)->_values->append(IMAGE_FACE_NEGATIVE_Y);
	((daeEnumType*)type)->_strings->append("POSITIVE_Z");
	((daeEnumType*)type)->_values->append(IMAGE_FACE_POSITIVE_Z);
	((daeEnumType*)type)->_strings->append("NEGATIVE_Z");
	((daeEnumType*)type)->_values->append(IMAGE_FACE_NEGATIVE_Z);
	atomicTypes.append( type );

	// ENUM: Image_format_hint_channels
	type = new daeEnumType(dae);
	type->_nameBindings.append("Image_format_hint_channels");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("RGB");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_CHANNELS_RGB);
	((daeEnumType*)type)->_strings->append("RGBA");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_CHANNELS_RGBA);
	((daeEnumType*)type)->_strings->append("RGBE");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_CHANNELS_RGBE);
	((daeEnumType*)type)->_strings->append("L");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_CHANNELS_L);
	((daeEnumType*)type)->_strings->append("LA");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_CHANNELS_LA);
	((daeEnumType*)type)->_strings->append("D");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_CHANNELS_D);
	atomicTypes.append( type );

	// ENUM: Image_format_hint_precision
	type = new daeEnumType(dae);
	type->_nameBindings.append("Image_format_hint_precision");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("DEFAULT");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_PRECISION_DEFAULT);
	((daeEnumType*)type)->_strings->append("LOW");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_PRECISION_LOW);
	((daeEnumType*)type)->_strings->append("MID");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_PRECISION_MID);
	((daeEnumType*)type)->_strings->append("HIGH");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_PRECISION_HIGH);
	((daeEnumType*)type)->_strings->append("MAX");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_PRECISION_MAX);
	atomicTypes.append( type );

	// ENUM: Image_format_hint_range
	type = new daeEnumType(dae);
	type->_nameBindings.append("Image_format_hint_range");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("SNORM");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_RANGE_SNORM);
	((daeEnumType*)type)->_strings->append("UNORM");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_RANGE_UNORM);
	((daeEnumType*)type)->_strings->append("SINT");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_RANGE_SINT);
	((daeEnumType*)type)->_strings->append("UINT");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_RANGE_UINT);
	((daeEnumType*)type)->_strings->append("FLOAT");
	((daeEnumType*)type)->_values->append(IMAGE_FORMAT_HINT_RANGE_FLOAT);
	atomicTypes.append( type );

	// ENUM: Altitude_mode
	type = new daeEnumType(dae);
	type->_nameBindings.append("Altitude_mode");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("absolute");
	((daeEnumType*)type)->_values->append(ALTITUDE_MODE_absolute);
	((daeEnumType*)type)->_strings->append("relativeToGround");
	((daeEnumType*)type)->_values->append(ALTITUDE_MODE_relativeToGround);
	atomicTypes.append( type );

	// TYPEDEF: Fx_color	//check if this type has an existing base
	type = atomicTypes.get("Float4");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Fx_color");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Fx_color");
	}
	
	// ENUM: Fx_opaque
	type = new daeEnumType(dae);
	type->_nameBindings.append("Fx_opaque");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("A_ONE");
	((daeEnumType*)type)->_values->append(FX_OPAQUE_A_ONE);
	((daeEnumType*)type)->_strings->append("A_ZERO");
	((daeEnumType*)type)->_values->append(FX_OPAQUE_A_ZERO);
	((daeEnumType*)type)->_strings->append("RGB_ONE");
	((daeEnumType*)type)->_values->append(FX_OPAQUE_RGB_ONE);
	((daeEnumType*)type)->_strings->append("RGB_ZERO");
	((daeEnumType*)type)->_values->append(FX_OPAQUE_RGB_ZERO);
	atomicTypes.append( type );

	// ENUM: Fx_sampler_wrap
	type = new daeEnumType(dae);
	type->_nameBindings.append("Fx_sampler_wrap");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("WRAP");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_WRAP);
	((daeEnumType*)type)->_strings->append("CLAMP");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_CLAMP);
	((daeEnumType*)type)->_strings->append("BORDER");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_BORDER);
	((daeEnumType*)type)->_strings->append("MIRROR");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_MIRROR);
	((daeEnumType*)type)->_strings->append("MIRROR_ONCE");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_WRAP_MIRROR_ONCE);
	atomicTypes.append( type );

	// ENUM: Fx_sampler_min_filter
	type = new daeEnumType(dae);
	type->_nameBindings.append("Fx_sampler_min_filter");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NEAREST");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_MIN_FILTER_NEAREST);
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_MIN_FILTER_LINEAR);
	((daeEnumType*)type)->_strings->append("ANISOTROPIC");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_MIN_FILTER_ANISOTROPIC);
	atomicTypes.append( type );

	// ENUM: Fx_sampler_mag_filter
	type = new daeEnumType(dae);
	type->_nameBindings.append("Fx_sampler_mag_filter");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NEAREST");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_MAG_FILTER_NEAREST);
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_MAG_FILTER_LINEAR);
	atomicTypes.append( type );

	// ENUM: Fx_sampler_mip_filter
	type = new daeEnumType(dae);
	type->_nameBindings.append("Fx_sampler_mip_filter");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NONE");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_MIP_FILTER_NONE);
	((daeEnumType*)type)->_strings->append("NEAREST");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_MIP_FILTER_NEAREST);
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(FX_SAMPLER_MIP_FILTER_LINEAR);
	atomicTypes.append( type );

	// ENUM: Fx_modifier
	type = new daeEnumType(dae);
	type->_nameBindings.append("Fx_modifier");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("CONST");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_CONST);
	((daeEnumType*)type)->_strings->append("UNIFORM");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_UNIFORM);
	((daeEnumType*)type)->_strings->append("VARYING");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_VARYING);
	((daeEnumType*)type)->_strings->append("STATIC");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_STATIC);
	((daeEnumType*)type)->_strings->append("VOLATILE");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_VOLATILE);
	((daeEnumType*)type)->_strings->append("EXTERN");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_EXTERN);
	((daeEnumType*)type)->_strings->append("SHARED");
	((daeEnumType*)type)->_values->append(FX_MODIFIER_SHARED);
	atomicTypes.append( type );

	// TYPEDEF: Fx_draw	//check if this type has an existing base
	type = atomicTypes.get("xsString");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Fx_draw");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Fx_draw");
	}
	
	// ENUM: Fx_pipeline_stage
	type = new daeEnumType(dae);
	type->_nameBindings.append("Fx_pipeline_stage");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("TESSELLATION");
	((daeEnumType*)type)->_values->append(FX_PIPELINE_STAGE_TESSELLATION);
	((daeEnumType*)type)->_strings->append("VERTEX");
	((daeEnumType*)type)->_values->append(FX_PIPELINE_STAGE_VERTEX);
	((daeEnumType*)type)->_strings->append("GEOMETRY");
	((daeEnumType*)type)->_values->append(FX_PIPELINE_STAGE_GEOMETRY);
	((daeEnumType*)type)->_strings->append("FRAGMENT");
	((daeEnumType*)type)->_values->append(FX_PIPELINE_STAGE_FRAGMENT);
	atomicTypes.append( type );

	// TYPEDEF: Gl_max_lights_index	//check if this type has an existing base
	type = atomicTypes.get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gl_max_lights_index");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gl_max_lights_index");
	}
	
	// TYPEDEF: Gl_max_clip_planes_index	//check if this type has an existing base
	type = atomicTypes.get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gl_max_clip_planes_index");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gl_max_clip_planes_index");
	}
	
	// TYPEDEF: Gl_max_texture_image_units_index	//check if this type has an existing base
	type = atomicTypes.get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gl_max_texture_image_units_index");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gl_max_texture_image_units_index");
	}
	
	// ENUM: Gl_blend
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_blend");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GL_BLEND_ZERO);
	((daeEnumType*)type)->_strings->append("ONE");
	((daeEnumType*)type)->_values->append(GL_BLEND_ONE);
	((daeEnumType*)type)->_strings->append("SRC_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_SRC_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_ONE_MINUS_SRC_COLOR);
	((daeEnumType*)type)->_strings->append("DEST_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_DEST_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DEST_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_ONE_MINUS_DEST_COLOR);
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_SRC_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_ONE_MINUS_SRC_ALPHA);
	((daeEnumType*)type)->_strings->append("DST_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_DST_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DST_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_ONE_MINUS_DST_ALPHA);
	((daeEnumType*)type)->_strings->append("CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_CONSTANT_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GL_BLEND_ONE_MINUS_CONSTANT_COLOR);
	((daeEnumType*)type)->_strings->append("CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_CONSTANT_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GL_BLEND_ONE_MINUS_CONSTANT_ALPHA);
	((daeEnumType*)type)->_strings->append("SRC_ALPHA_SATURATE");
	((daeEnumType*)type)->_values->append(GL_BLEND_SRC_ALPHA_SATURATE);
	atomicTypes.append( type );

	// ENUM: Gl_face
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_face");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("FRONT");
	((daeEnumType*)type)->_values->append(GL_FACE_FRONT);
	((daeEnumType*)type)->_strings->append("BACK");
	((daeEnumType*)type)->_values->append(GL_FACE_BACK);
	((daeEnumType*)type)->_strings->append("FRONT_AND_BACK");
	((daeEnumType*)type)->_values->append(GL_FACE_FRONT_AND_BACK);
	atomicTypes.append( type );

	// ENUM: Gl_blend_equation
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_blend_equation");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("FUNC_ADD");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_FUNC_ADD);
	((daeEnumType*)type)->_strings->append("FUNC_SUBTRACT");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_FUNC_SUBTRACT);
	((daeEnumType*)type)->_strings->append("FUNC_REVERSE_SUBTRACT");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_FUNC_REVERSE_SUBTRACT);
	((daeEnumType*)type)->_strings->append("MIN");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_MIN);
	((daeEnumType*)type)->_strings->append("MAX");
	((daeEnumType*)type)->_values->append(GL_BLEND_EQUATION_MAX);
	atomicTypes.append( type );

	// ENUM: Gl_func
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_func");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("NEVER");
	((daeEnumType*)type)->_values->append(GL_FUNC_NEVER);
	((daeEnumType*)type)->_strings->append("LESS");
	((daeEnumType*)type)->_values->append(GL_FUNC_LESS);
	((daeEnumType*)type)->_strings->append("LEQUAL");
	((daeEnumType*)type)->_values->append(GL_FUNC_LEQUAL);
	((daeEnumType*)type)->_strings->append("EQUAL");
	((daeEnumType*)type)->_values->append(GL_FUNC_EQUAL);
	((daeEnumType*)type)->_strings->append("GREATER");
	((daeEnumType*)type)->_values->append(GL_FUNC_GREATER);
	((daeEnumType*)type)->_strings->append("NOTEQUAL");
	((daeEnumType*)type)->_values->append(GL_FUNC_NOTEQUAL);
	((daeEnumType*)type)->_strings->append("GEQUAL");
	((daeEnumType*)type)->_values->append(GL_FUNC_GEQUAL);
	((daeEnumType*)type)->_strings->append("ALWAYS");
	((daeEnumType*)type)->_values->append(GL_FUNC_ALWAYS);
	atomicTypes.append( type );

	// ENUM: Gl_stencil_op
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_stencil_op");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("KEEP");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_KEEP);
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_ZERO);
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_REPLACE);
	((daeEnumType*)type)->_strings->append("INCR");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_INCR);
	((daeEnumType*)type)->_strings->append("DECR");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_DECR);
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_INVERT);
	((daeEnumType*)type)->_strings->append("INCR_WRAP");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_INCR_WRAP);
	((daeEnumType*)type)->_strings->append("DECR_WRAP");
	((daeEnumType*)type)->_values->append(GL_STENCIL_OP_DECR_WRAP);
	atomicTypes.append( type );

	// ENUM: Gl_material
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_material");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("EMISSION");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_EMISSION);
	((daeEnumType*)type)->_strings->append("AMBIENT");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_AMBIENT);
	((daeEnumType*)type)->_strings->append("DIFFUSE");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_DIFFUSE);
	((daeEnumType*)type)->_strings->append("SPECULAR");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_SPECULAR);
	((daeEnumType*)type)->_strings->append("AMBIENT_AND_DIFFUSE");
	((daeEnumType*)type)->_values->append(GL_MATERIAL_AMBIENT_AND_DIFFUSE);
	atomicTypes.append( type );

	// ENUM: Gl_fog
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_fog");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(GL_FOG_LINEAR);
	((daeEnumType*)type)->_strings->append("EXP");
	((daeEnumType*)type)->_values->append(GL_FOG_EXP);
	((daeEnumType*)type)->_strings->append("EXP2");
	((daeEnumType*)type)->_values->append(GL_FOG_EXP2);
	atomicTypes.append( type );

	// ENUM: Gl_fog_coord_src
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_fog_coord_src");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("FOG_COORDINATE");
	((daeEnumType*)type)->_values->append(GL_FOG_COORD_SRC_FOG_COORDINATE);
	((daeEnumType*)type)->_strings->append("FRAGMENT_DEPTH");
	((daeEnumType*)type)->_values->append(GL_FOG_COORD_SRC_FRAGMENT_DEPTH);
	atomicTypes.append( type );

	// ENUM: Gl_front_face
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_front_face");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("CW");
	((daeEnumType*)type)->_values->append(GL_FRONT_FACE_CW);
	((daeEnumType*)type)->_strings->append("CCW");
	((daeEnumType*)type)->_values->append(GL_FRONT_FACE_CCW);
	atomicTypes.append( type );

	// ENUM: Gl_light_model_color_control
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_light_model_color_control");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("SINGLE_COLOR");
	((daeEnumType*)type)->_values->append(GL_LIGHT_MODEL_COLOR_CONTROL_SINGLE_COLOR);
	((daeEnumType*)type)->_strings->append("SEPARATE_SPECULAR_COLOR");
	((daeEnumType*)type)->_values->append(GL_LIGHT_MODEL_COLOR_CONTROL_SEPARATE_SPECULAR_COLOR);
	atomicTypes.append( type );

	// ENUM: Gl_logic_op
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_logic_op");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("CLEAR");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_CLEAR);
	((daeEnumType*)type)->_strings->append("AND");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_AND);
	((daeEnumType*)type)->_strings->append("AND_REVERSE");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_AND_REVERSE);
	((daeEnumType*)type)->_strings->append("COPY");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_COPY);
	((daeEnumType*)type)->_strings->append("AND_INVERTED");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_AND_INVERTED);
	((daeEnumType*)type)->_strings->append("NOOP");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_NOOP);
	((daeEnumType*)type)->_strings->append("XOR");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_XOR);
	((daeEnumType*)type)->_strings->append("OR");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_OR);
	((daeEnumType*)type)->_strings->append("NOR");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_NOR);
	((daeEnumType*)type)->_strings->append("EQUIV");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_EQUIV);
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_INVERT);
	((daeEnumType*)type)->_strings->append("OR_REVERSE");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_OR_REVERSE);
	((daeEnumType*)type)->_strings->append("COPY_INVERTED");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_COPY_INVERTED);
	((daeEnumType*)type)->_strings->append("NAND");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_NAND);
	((daeEnumType*)type)->_strings->append("SET");
	((daeEnumType*)type)->_values->append(GL_LOGIC_OP_SET);
	atomicTypes.append( type );

	// ENUM: Gl_polygon_mode
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_polygon_mode");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("POINT");
	((daeEnumType*)type)->_values->append(GL_POLYGON_MODE_POINT);
	((daeEnumType*)type)->_strings->append("LINE");
	((daeEnumType*)type)->_values->append(GL_POLYGON_MODE_LINE);
	((daeEnumType*)type)->_strings->append("FILL");
	((daeEnumType*)type)->_values->append(GL_POLYGON_MODE_FILL);
	atomicTypes.append( type );

	// ENUM: Gl_shade_model
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_shade_model");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("FLAT");
	((daeEnumType*)type)->_values->append(GL_SHADE_MODEL_FLAT);
	((daeEnumType*)type)->_strings->append("SMOOTH");
	((daeEnumType*)type)->_values->append(GL_SHADE_MODEL_SMOOTH);
	atomicTypes.append( type );

	// TYPEDEF: Gl_alpha_value	//check if this type has an existing base
	type = atomicTypes.get("xsFloat");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gl_alpha_value");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gl_alpha_value");
	}
	
	// ENUM: Gl_enumeration
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gl_enumeration");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ZERO);
	((daeEnumType*)type)->_strings->append("ONE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE);
	((daeEnumType*)type)->_strings->append("SRC_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SRC_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_SRC_COLOR);
	((daeEnumType*)type)->_strings->append("DEST_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DEST_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DEST_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_DEST_COLOR);
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SRC_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_SRC_ALPHA);
	((daeEnumType*)type)->_strings->append("DST_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DST_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DST_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_DST_ALPHA);
	((daeEnumType*)type)->_strings->append("CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CONSTANT_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_CONSTANT_COLOR);
	((daeEnumType*)type)->_strings->append("CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CONSTANT_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ONE_MINUS_CONSTANT_ALPHA);
	((daeEnumType*)type)->_strings->append("SRC_ALPHA_SATURATE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SRC_ALPHA_SATURATE);
	((daeEnumType*)type)->_strings->append("FRONT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FRONT);
	((daeEnumType*)type)->_strings->append("BACK");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_BACK);
	((daeEnumType*)type)->_strings->append("FRONT_AND_BACK");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FRONT_AND_BACK);
	((daeEnumType*)type)->_strings->append("FUNC_ADD");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FUNC_ADD);
	((daeEnumType*)type)->_strings->append("FUNC_SUBTRACT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FUNC_SUBTRACT);
	((daeEnumType*)type)->_strings->append("FUNC_REVERSE_SUBTRACT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FUNC_REVERSE_SUBTRACT);
	((daeEnumType*)type)->_strings->append("MIN");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_MIN);
	((daeEnumType*)type)->_strings->append("MAX");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_MAX);
	((daeEnumType*)type)->_strings->append("NEVER");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NEVER);
	((daeEnumType*)type)->_strings->append("LESS");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_LESS);
	((daeEnumType*)type)->_strings->append("LEQUAL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_LEQUAL);
	((daeEnumType*)type)->_strings->append("EQUAL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EQUAL);
	((daeEnumType*)type)->_strings->append("GREATER");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_GREATER);
	((daeEnumType*)type)->_strings->append("NOTEQUAL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NOTEQUAL);
	((daeEnumType*)type)->_strings->append("GEQUAL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_GEQUAL);
	((daeEnumType*)type)->_strings->append("ALWAYS");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_ALWAYS);
	((daeEnumType*)type)->_strings->append("KEEP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_KEEP);
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_REPLACE);
	((daeEnumType*)type)->_strings->append("INCR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_INCR);
	((daeEnumType*)type)->_strings->append("DECR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DECR);
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_INVERT);
	((daeEnumType*)type)->_strings->append("INCR_WRAP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_INCR_WRAP);
	((daeEnumType*)type)->_strings->append("DECR_WRAP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DECR_WRAP);
	((daeEnumType*)type)->_strings->append("EMISSION");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EMISSION);
	((daeEnumType*)type)->_strings->append("AMBIENT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AMBIENT);
	((daeEnumType*)type)->_strings->append("DIFFUSE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_DIFFUSE);
	((daeEnumType*)type)->_strings->append("SPECULAR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SPECULAR);
	((daeEnumType*)type)->_strings->append("AMBIENT_AND_DIFFUSE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AMBIENT_AND_DIFFUSE);
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_LINEAR);
	((daeEnumType*)type)->_strings->append("EXP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EXP);
	((daeEnumType*)type)->_strings->append("EXP2");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EXP2);
	((daeEnumType*)type)->_strings->append("FOG_COORDINATE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FOG_COORDINATE);
	((daeEnumType*)type)->_strings->append("FRAGMENT_DEPTH");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FRAGMENT_DEPTH);
	((daeEnumType*)type)->_strings->append("CW");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CW);
	((daeEnumType*)type)->_strings->append("CCW");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CCW);
	((daeEnumType*)type)->_strings->append("SINGLE_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SINGLE_COLOR);
	((daeEnumType*)type)->_strings->append("SEPARATE_SPECULAR_COLOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SEPARATE_SPECULAR_COLOR);
	((daeEnumType*)type)->_strings->append("CLEAR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_CLEAR);
	((daeEnumType*)type)->_strings->append("AND");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AND);
	((daeEnumType*)type)->_strings->append("AND_REVERSE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AND_REVERSE);
	((daeEnumType*)type)->_strings->append("COPY");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_COPY);
	((daeEnumType*)type)->_strings->append("AND_INVERTED");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_AND_INVERTED);
	((daeEnumType*)type)->_strings->append("NOOP");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NOOP);
	((daeEnumType*)type)->_strings->append("XOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_XOR);
	((daeEnumType*)type)->_strings->append("OR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_OR);
	((daeEnumType*)type)->_strings->append("NOR");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NOR);
	((daeEnumType*)type)->_strings->append("EQUIV");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_EQUIV);
	((daeEnumType*)type)->_strings->append("OR_REVERSE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_OR_REVERSE);
	((daeEnumType*)type)->_strings->append("COPY_INVERTED");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_COPY_INVERTED);
	((daeEnumType*)type)->_strings->append("NAND");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_NAND);
	((daeEnumType*)type)->_strings->append("SET");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SET);
	((daeEnumType*)type)->_strings->append("POINT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_POINT);
	((daeEnumType*)type)->_strings->append("LINE");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_LINE);
	((daeEnumType*)type)->_strings->append("FILL");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FILL);
	((daeEnumType*)type)->_strings->append("FLAT");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_FLAT);
	((daeEnumType*)type)->_strings->append("SMOOTH");
	((daeEnumType*)type)->_values->append(GL_ENUMERATION_SMOOTH);
	atomicTypes.append( type );

	// TYPEDEF: Gles_max_lights_index	//check if this type has an existing base
	type = atomicTypes.get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gles_max_lights_index");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gles_max_lights_index");
	}
	
	// TYPEDEF: Gles_max_clip_planes_index	//check if this type has an existing base
	type = atomicTypes.get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gles_max_clip_planes_index");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gles_max_clip_planes_index");
	}
	
	// TYPEDEF: Gles_max_texture_coords_index	//check if this type has an existing base
	type = atomicTypes.get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gles_max_texture_coords_index");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gles_max_texture_coords_index");
	}
	
	// TYPEDEF: Gles_max_texture_image_units_index	//check if this type has an existing base
	type = atomicTypes.get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gles_max_texture_image_units_index");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gles_max_texture_image_units_index");
	}
	
	// ENUM: Gles_texenv_mode
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_texenv_mode");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_REPLACE);
	((daeEnumType*)type)->_strings->append("MODULATE");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_MODULATE);
	((daeEnumType*)type)->_strings->append("DECAL");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_DECAL);
	((daeEnumType*)type)->_strings->append("BLEND");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_BLEND);
	((daeEnumType*)type)->_strings->append("ADD");
	((daeEnumType*)type)->_values->append(GLES_TEXENV_MODE_ADD);
	atomicTypes.append( type );

	// ENUM: Gles_texcombiner_operator_rgb
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_texcombiner_operator_rgb");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_RGB_REPLACE);
	((daeEnumType*)type)->_strings->append("MODULATE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_RGB_MODULATE);
	((daeEnumType*)type)->_strings->append("ADD");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_RGB_ADD);
	((daeEnumType*)type)->_strings->append("ADD_SIGNED");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_RGB_ADD_SIGNED);
	((daeEnumType*)type)->_strings->append("INTERPOLATE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_RGB_INTERPOLATE);
	((daeEnumType*)type)->_strings->append("SUBTRACT");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_RGB_SUBTRACT);
	((daeEnumType*)type)->_strings->append("DOT3_RGB");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_RGB_DOT3_RGB);
	((daeEnumType*)type)->_strings->append("DOT3_RGBA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_RGB_DOT3_RGBA);
	atomicTypes.append( type );

	// ENUM: Gles_texcombiner_operator_alpha
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_texcombiner_operator_alpha");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_ALPHA_REPLACE);
	((daeEnumType*)type)->_strings->append("MODULATE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_ALPHA_MODULATE);
	((daeEnumType*)type)->_strings->append("ADD");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_ALPHA_ADD);
	((daeEnumType*)type)->_strings->append("ADD_SIGNED");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_ALPHA_ADD_SIGNED);
	((daeEnumType*)type)->_strings->append("INTERPOLATE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_ALPHA_INTERPOLATE);
	((daeEnumType*)type)->_strings->append("SUBTRACT");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERATOR_ALPHA_SUBTRACT);
	atomicTypes.append( type );

	// ENUM: Gles_texcombiner_source
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_texcombiner_source");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("TEXTURE");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_SOURCE_TEXTURE);
	((daeEnumType*)type)->_strings->append("CONSTANT");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_SOURCE_CONSTANT);
	((daeEnumType*)type)->_strings->append("PRIMARY");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_SOURCE_PRIMARY);
	((daeEnumType*)type)->_strings->append("PREVIOUS");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_SOURCE_PREVIOUS);
	atomicTypes.append( type );

	// ENUM: Gles_texcombiner_operand_rgb
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_texcombiner_operand_rgb");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("SRC_COLOR");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERAND_RGB_SRC_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_COLOR");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERAND_RGB_ONE_MINUS_SRC_COLOR);
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERAND_RGB_SRC_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERAND_RGB_ONE_MINUS_SRC_ALPHA);
	atomicTypes.append( type );

	// ENUM: Gles_texcombiner_operand_alpha
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_texcombiner_operand_alpha");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERAND_ALPHA_SRC_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_TEXCOMBINER_OPERAND_ALPHA_ONE_MINUS_SRC_ALPHA);
	atomicTypes.append( type );

	// TYPEDEF: Gles_texcombiner_argument_index	//check if this type has an existing base
	type = atomicTypes.get("xsNonNegativeInteger");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Gles_texcombiner_argument_index");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Gles_texcombiner_argument_index");
	}
	
	// ENUM: Gles_sampler_wrap
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_sampler_wrap");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("REPEAT");
	((daeEnumType*)type)->_values->append(GLES_SAMPLER_WRAP_REPEAT);
	((daeEnumType*)type)->_strings->append("CLAMP");
	((daeEnumType*)type)->_values->append(GLES_SAMPLER_WRAP_CLAMP);
	((daeEnumType*)type)->_strings->append("CLAMP_TO_EDGE");
	((daeEnumType*)type)->_values->append(GLES_SAMPLER_WRAP_CLAMP_TO_EDGE);
	((daeEnumType*)type)->_strings->append("MIRRORED_REPEAT");
	((daeEnumType*)type)->_values->append(GLES_SAMPLER_WRAP_MIRRORED_REPEAT);
	atomicTypes.append( type );

	// ENUM: Gles_stencil_op
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_stencil_op");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("KEEP");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_KEEP);
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_ZERO);
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_REPLACE);
	((daeEnumType*)type)->_strings->append("INCR");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_INCR);
	((daeEnumType*)type)->_strings->append("DECR");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_DECR);
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GLES_STENCIL_OP_INVERT);
	atomicTypes.append( type );

	// ENUM: Gles_enumeration
	type = new daeEnumType(dae);
	type->_nameBindings.append("Gles_enumeration");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("ZERO");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ZERO);
	((daeEnumType*)type)->_strings->append("ONE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE);
	((daeEnumType*)type)->_strings->append("SRC_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SRC_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_SRC_COLOR);
	((daeEnumType*)type)->_strings->append("DEST_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DEST_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DEST_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_DEST_COLOR);
	((daeEnumType*)type)->_strings->append("SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SRC_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_SRC_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_SRC_ALPHA);
	((daeEnumType*)type)->_strings->append("DST_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DST_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_DST_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_DST_ALPHA);
	((daeEnumType*)type)->_strings->append("CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CONSTANT_COLOR);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_CONSTANT_COLOR);
	((daeEnumType*)type)->_strings->append("CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CONSTANT_ALPHA);
	((daeEnumType*)type)->_strings->append("ONE_MINUS_CONSTANT_ALPHA");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ONE_MINUS_CONSTANT_ALPHA);
	((daeEnumType*)type)->_strings->append("SRC_ALPHA_SATURATE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SRC_ALPHA_SATURATE);
	((daeEnumType*)type)->_strings->append("FRONT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_FRONT);
	((daeEnumType*)type)->_strings->append("BACK");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_BACK);
	((daeEnumType*)type)->_strings->append("FRONT_AND_BACK");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_FRONT_AND_BACK);
	((daeEnumType*)type)->_strings->append("NEVER");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NEVER);
	((daeEnumType*)type)->_strings->append("LESS");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_LESS);
	((daeEnumType*)type)->_strings->append("LEQUAL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_LEQUAL);
	((daeEnumType*)type)->_strings->append("EQUAL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EQUAL);
	((daeEnumType*)type)->_strings->append("GREATER");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_GREATER);
	((daeEnumType*)type)->_strings->append("NOTEQUAL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NOTEQUAL);
	((daeEnumType*)type)->_strings->append("GEQUAL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_GEQUAL);
	((daeEnumType*)type)->_strings->append("ALWAYS");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_ALWAYS);
	((daeEnumType*)type)->_strings->append("KEEP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_KEEP);
	((daeEnumType*)type)->_strings->append("REPLACE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_REPLACE);
	((daeEnumType*)type)->_strings->append("INCR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_INCR);
	((daeEnumType*)type)->_strings->append("DECR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DECR);
	((daeEnumType*)type)->_strings->append("INVERT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_INVERT);
	((daeEnumType*)type)->_strings->append("INCR_WRAP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_INCR_WRAP);
	((daeEnumType*)type)->_strings->append("DECR_WRAP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DECR_WRAP);
	((daeEnumType*)type)->_strings->append("EMISSION");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EMISSION);
	((daeEnumType*)type)->_strings->append("AMBIENT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AMBIENT);
	((daeEnumType*)type)->_strings->append("DIFFUSE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_DIFFUSE);
	((daeEnumType*)type)->_strings->append("SPECULAR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SPECULAR);
	((daeEnumType*)type)->_strings->append("AMBIENT_AND_DIFFUSE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AMBIENT_AND_DIFFUSE);
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_LINEAR);
	((daeEnumType*)type)->_strings->append("EXP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EXP);
	((daeEnumType*)type)->_strings->append("EXP2");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EXP2);
	((daeEnumType*)type)->_strings->append("CW");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CW);
	((daeEnumType*)type)->_strings->append("CCW");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CCW);
	((daeEnumType*)type)->_strings->append("SINGLE_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SINGLE_COLOR);
	((daeEnumType*)type)->_strings->append("SEPARATE_SPECULAR_COLOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SEPARATE_SPECULAR_COLOR);
	((daeEnumType*)type)->_strings->append("CLEAR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_CLEAR);
	((daeEnumType*)type)->_strings->append("AND");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AND);
	((daeEnumType*)type)->_strings->append("AND_REVERSE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AND_REVERSE);
	((daeEnumType*)type)->_strings->append("COPY");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_COPY);
	((daeEnumType*)type)->_strings->append("AND_INVERTED");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_AND_INVERTED);
	((daeEnumType*)type)->_strings->append("NOOP");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NOOP);
	((daeEnumType*)type)->_strings->append("XOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_XOR);
	((daeEnumType*)type)->_strings->append("OR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_OR);
	((daeEnumType*)type)->_strings->append("NOR");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NOR);
	((daeEnumType*)type)->_strings->append("EQUIV");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_EQUIV);
	((daeEnumType*)type)->_strings->append("OR_REVERSE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_OR_REVERSE);
	((daeEnumType*)type)->_strings->append("COPY_INVERTED");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_COPY_INVERTED);
	((daeEnumType*)type)->_strings->append("NAND");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_NAND);
	((daeEnumType*)type)->_strings->append("SET");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SET);
	((daeEnumType*)type)->_strings->append("POINT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_POINT);
	((daeEnumType*)type)->_strings->append("LINE");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_LINE);
	((daeEnumType*)type)->_strings->append("FILL");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_FILL);
	((daeEnumType*)type)->_strings->append("FLAT");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_FLAT);
	((daeEnumType*)type)->_strings->append("SMOOTH");
	((daeEnumType*)type)->_values->append(GLES_ENUMERATION_SMOOTH);
	atomicTypes.append( type );

	// ENUM: Spring
	type = new daeEnumType(dae);
	type->_nameBindings.append("Spring");
	((daeEnumType*)type)->_strings = new daeStringRefArray;
	((daeEnumType*)type)->_values = new daeEnumArray;
	((daeEnumType*)type)->_strings->append("LINEAR");
	((daeEnumType*)type)->_values->append(SPRING_LINEAR);
	((daeEnumType*)type)->_strings->append("ANGULAR");
	((daeEnumType*)type)->_values->append(SPRING_ANGULAR);
	atomicTypes.append( type );

	// TYPEDEF: Dynamic_limit	//check if this type has an existing base
	type = atomicTypes.get("Float2");
	if ( type == NULL ) { //register as a raw type
		type = new daeRawRefType(dae);
		type->_nameBindings.append("Dynamic_limit");
		atomicTypes.append( type );
	}
	else { //add binding to existing type
		type->_nameBindings.append("Dynamic_limit");
	}
	
}

daeMetaElement* registerDomElements(DAE& dae)
{
	daeMetaElement* meta = domCOLLADA::registerElement(dae);
	// Enable tracking of top level object by default
	meta->setIsTrackableForQueries(true);
	return meta;	
}

daeInt DLLSPEC colladaTypeCount() {
	return 969;
}
