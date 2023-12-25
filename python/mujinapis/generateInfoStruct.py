import copy

_ = lambda x: x

poseSchema = {
    "title": _("Cartesian pose"),
    "description": _(
        "A 7-element vector representing a 6D cartesian pose. Consists of a wxyz quaternion and an xyz position. The quaternion defines the orientation."
    ),
    "type": "array",
    "items": [
        {"title": _("w (quaternion)"), "type": "number", "default": 1.0},
        {"title": _("x (quaternion)"), "type": "number", "default": 0.0},
        {"title": _("y (quaternion)"), "type": "number", "default": 0.0},
        {"title": _("z (quaternion)"), "type": "number", "default": 0.0},
        {"title": _("x (position)"), "type": "number", "default": 0.0, "semanticType": "Length"},
        {"title": _("y (position)"), "type": "number", "default": 0.0, "semanticType": "Length"},
        {"title": _("z (position)"), "type": "number", "default": 0.0, "semanticType": "Length"},
    ],
    "minItems": 7,
    "maxItems": 7,
    "additionalItems": False,
    "semanticType": "Pose",
}

def MakePoseSchema(title, description):
    customPoseSchema = copy.deepcopy(poseSchema)
    customPoseSchema["title"] = title
    customPoseSchema["description"] = description
    return customPoseSchema

geometryInfosColorTriple = {
    'type': 'array',
    'minItems': 3,
    'maxItems': 3,
    "items": [
        {"type": "number"},
        {"type": "number"},
        {"type": "number"},
    ]
}

geometryInfoSchema = {  # TODO(felixvd): Link to kinbody.GeometryInfo
    "type": "object",
    "typeName": "Geometry",
    "properties": {
        "id": {"type": "string"},
        "name": {"type": "string"},
        "type": {
            "type": "string",
            "enum": [
                "mesh",
                "box",
                "container",
                "cage",
                "sphere",
                "cylinder",
                "axial",
                "trimesh",
                "calibrationboard",
                "conicalfrustum",
                ""]
        },
        "diffuseColor": geometryInfosColorTriple,
        "outerExtents": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": [
                {"type": "number"},
                {"type": "number"},
                {"type": "number"},
            ]
        },
        "innerExtents": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": [
                {"type": "number"},
                {"type": "number"},
                {"type": "number"},
            ]
        },
        "halfExtents": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": [
                {"type": "number"},
                {"type": "number"},
                {"type": "number"},
            ]
        },
        "transparency": {
            "type": "number"
        },
        "negativeCropContainerMargins": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": [
                {"type": "number"},
                {"type": "number"},
                {"type": "number"},
            ]
        },
        "negativeCropContainerEmptyMargins": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": [
                {"type": "number"},
                {"type": "number"},
                {"type": "number"},
            ]
        },
        "positiveCropContainerMargins": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": [
                {"type": "number"},
                {"type": "number"},
                {"type": "number"},
            ]
        },
        "positiveCropContainerEmptyMargins": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": [
                {"type": "number"},
                {"type": "number"},
                {"type": "number"},
            ]
        },
        "transform": MakePoseSchema(title=_("transform"), description=_("transform")),
    }
}

geometryInfosSchema = {
    "type": "array",
    "description": _("list of serialized geometry infos of the base link when the first geometry is cage (for now)"),
    "items": geometryInfoSchema,
}

def _JsonSchemaTypeToCppType(schema):
    print("raliao", schema)
    jsonType = schema.get('type', '')
    if isinstance(jsonType, list):
        assert len(jsonType) == 2 and jsonType[1] == 'null', 'Only support nullable types'
        copied = schema.copy()
        copied['type'] = jsonType[0]
        return _JsonSchemaTypeToCppType(copied)
    if jsonType == 'object':
        if 'typeName' in schema:
            # TODO(heman.gandhi): remove suffix
            return schema['typeName'] + 'Gen'
        return 'rapidjson::Value'
    if jsonType == 'number':
        return 'double'
    if jsonType == 'integer':
        #TODO(heman.gandhi): determine where to stash extra data to override numeric types
        return 'int32_t'
    if jsonType == 'string':
        return 'std::string'
    if jsonType == 'boolean':
        return 'bool'
    if jsonType == 'array':
        if schema.get('prefixItems'):
            return 'std::tuple<' + ', '.join(_JsonSchemaTypeToCppType(item) for item in schema['prefixItems']) + '>'
        elif schema.get('items'):
            return 'std::vector<' + ', '.join(_JsonSchemaTypeToCppType(item) for item in schema['items']) + '>'
        else:
            # TODO(heman.gandhi): raise a ValueError here?
            return 'void *'
    if jsonType == 'named_bitmask':
        return 'uint32_t'
    return jsonType

def _CppDefaultValueString(defaultValue):
    if defaultValue is None:
        return 'nullptr'
    if isinstance(defaultValue, list) and defaultValue == []:
        return '{}'
    if isinstance(defaultValue, dict) and defaultValue == {}:
        return '{}'
    if isinstance(defaultValue, bool):
        return 'true' if defaultValue else 'false'
    return str(defaultValue)

class _CppParamInfo:
    cppType = '' #type: str 
    needsExtraParamForNull = False # type: bool
    cppParamName = '' # type: str
    fieldNamePrefix = '_' # type: str
    defaultValue = None # type: Any
    hasDefault = False # type: bool
    needsJsonAlloc = False # type: bool
    def __init__(self, schema, paramName='', fieldNamePrefix='_'):
        self.cppParamName = paramName
        self.cppType = _JsonSchemaTypeToCppType(schema)
        if not self.cppType:
            # TODO(heman.gandhi): raise a ValueError here?
            return
        if isinstance(schema.get('type', ''), list) and len(schema['type']) > 1 and schema['type'][1] == 'null':
            raise ValueError("While a list of types is valid JSON schema, generation does not support it.")
        self.hasDefault = 'default' in schema
        if self.hasDefault:
            self.defaultValue = schema['default']
        self.fieldNamePrefix = fieldNamePrefix
    def RenderFields(self, prefixOverride=None):
        prefix = prefixOverride if prefixOverride is not None else self.fieldNamePrefix
        paramString = self.cppType + ' ' + prefix + self.cppParamName
        default = '' if not self.hasDefault else ' = ' + _CppDefaultValueString(self.defaultValue)
        if not self.needsExtraParamForNull:
            return [paramString + default]
        extraParam = 'bool ' + prefix + self.cppParamName + 'Null'
        if self.defaultValue is None:
            return [paramString, extraParam + ' = true']
        return [paramString + default, extraParam + ' = false']
    def RenderArgs(self, renderDefault, constAllowed=True):
        constIfy = lambda x: 'const ' + x if x.split(' ')[0][-1] in ['*', '&'] else x
        removeDefault = lambda x: x if renderDefault else x.split('=')[0].rstrip()
        return [removeDefault(constIfy(field) if constAllowed else field) for field in self.RenderFields(prefixOverride='')]
    def RenderInitializerList(self):
        initializers = [self.fieldNamePrefix + self.cppParamName + '(' + self.cppParamName + ')']
        if self.needsExtraParamForNull:
            initializers += [self.RenderNullParam(True) + '(' + self.RenderNullParam(False) + ')']
        return initializers
    def RenderName(self, needsPrefix):
        if needsPrefix:
            return self.fieldNamePrefix + self.cppParamName
        else:
            return self.cppParamName
    def RenderNullParam(self, needsPrefix):
        if not self.needsExtraParamForNull:
            return ''
        return self.RenderName + 'Null'
    
def OutputDiffFunction(schema):
    diffFunction = '    rapidjson::Value Diff(const rapidjson::Value& other)\n    {\n'
    diffFunction += '        return rapidjson::Value();\n'
    diffFunction += '    }\n'
    return diffFunction

def OutputOneClass(schema):
    structString = f"class OPENRAVE_API {schema['typeName']}Gen : public InfoBase\n{{"
    for fieldName, fieldSchema in schema.get('properties', dict()).items():
        param = _CppParamInfo(fieldSchema, fieldName)
        structString += '\n    ' + param.RenderFields()[0] + ';\n\n'
    structString += OutputDiffFunction(schema)
    return structString + "\n};"


def OutputFile(schemas):
    classDelimiter = '\n\n'
    return f"""\
#include <string>
#include <vector>

#include <openrave/config.h>
#include <openrave/openraveexception.h>
#include <openrave/openrave.h>
#include <openrave/units.h>
#include <openrave/interface.h>

#include "rapidjson/rapidjson.h"

namespace OpenRAVE {{

{classDelimiter.join(map(OutputOneClass, schemas))}

}}
"""

if __name__ == "__main__":
    print(OutputFile([geometryInfoSchema]))
