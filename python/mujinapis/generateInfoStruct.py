from geometryInfo import geometryInfoSchema
import re

def _JsonSchemaTypeToCppType(schema):
    if 'typeName' in schema:
        return schema['typeName']
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
            return 'std::vector<' + _JsonSchemaTypeToCppType(schema['items']) + '>'
        else:
            # TODO(heman.gandhi): raise a ValueError here?
            return 'void *'
    if jsonType == 'named_bitmask':
        return 'uint32_t'
    return jsonType

class _CppParamInfo:
    cppType = '' #type: str 
    needsExtraParamForNull = False # type: bool
    cppParamName = '' # type: str
    fieldNamePrefix = '_' # type: str
    defaultValue = None # type: Any
    hasDefault = False # type: bool
    shouldScaleInJson = False # type: bool
    def __init__(self, schema, paramName='', fieldNamePrefix='_', isRequired=False):
        self.cppParamName = paramName
        self.cppType = _JsonSchemaTypeToCppType(schema)
        if not self.cppType:
            # TODO(heman.gandhi): raise a ValueError here?
            raise ValueError(f"Got bad type {self.cppType} for schema {schema}")
        if isinstance(schema.get('type', ''), list) and len(schema['type']) > 1 and schema['type'][1] == 'null':
            raise ValueError("While a list of types is valid JSON schema, generation does not support it.")
        self.hasDefault = 'default' in schema
        if self.hasDefault:
            self.defaultValue = schema['default']
        self.fieldNamePrefix = fieldNamePrefix
        self.isRequired = isRequired
        self.shouldScaleInJson = schema.get('scaleInSerialization', False)
        self.isEnum = ('enum' in schema and 'typeName' in schema)
    def RenderFields(self, prefixOverride=None):
        prefix = prefixOverride if prefixOverride is not None else self.fieldNamePrefix
        paramString = self.cppType + ' ' + prefix + self.cppParamName
        default = '' if not self.hasDefault else ' = ' + self.RenderDefaultValue()
        return paramString + default
    def RenderName(self, needsPrefix):
        if needsPrefix:
            return self.fieldNamePrefix + self.cppParamName
        else:
            return self.cppParamName
    def RenderDefaultValue(self):
        if isinstance(self.defaultValue, bool):
            return 'true' if self.defaultValue else 'false'
        if self.cppType == "std::string":
            return '"' + (self.defaultValue if self.hasDefault else '') + '"'
        if self.hasDefault:
            return str(self.defaultValue)
        return self.cppType + "{}"

    def RenderSerialization(self, cppJsonVariable="rSerializedOutput", cppJsonAllocVariable="allocator"):
        serCode = ""
        indent = 8
        closeBraceStack = []
        serName = self.RenderName(True)
        if self.isEnum:
            serCode = f"""{' '*indent}{{
{' ' * (indent + 4)}std::string {serName}String = Get{self.cppType}String({serName});
"""
            serName += "String"
            closeBraceStack = [' '*indent + "}"]
            indent += 4
        if not self.isRequired:
            serCode += f"{' '*indent}if ({self.RenderName(True)} != {self.RenderDefaultValue()})"
            serCode += '\n' + ' '*indent + "{" + '\n'
            closeBraceStack += [' '*indent + "}"]
            indent += 4
        if self.shouldScaleInJson:
            if self.cppType == 'Transform':
                serCode += ' '*indent + '{\n'
                closeBraceStack += [' '*indent + "}"]
                indent += 4
                serCode += f"{' ' * indent}Transform scaled{serName} = {serName};\n"
                serName = "scaled" + serName
                serCode += f"{' ' * indent}{serName}.trans *= fUnitScale;\n"
            else:
                serName = f"{serName}*fUnitScale"
        serCode += f"{' '*indent}orjson::SetJsonValueByKey({cppJsonVariable}, \"{self.cppParamName}\", {serName}, {cppJsonAllocVariable});"
        return serCode + '\n' + '\n'.join(reversed(closeBraceStack))

    def RenderDeserialization(self):
        code = ""
        indent = 8
        closeBraceStack = []
        if not self.isRequired:
            code += f'{" "*indent}if (value.HasMember("{self.cppParamName}"))'
            code += '\n' + ' '*indent + "{" + '\n'
            closeBraceStack.append("\n" + ' '*indent + "}")
            indent += 4
        deserName = self.RenderName(True)
        if self.isEnum:
            code += f"{{\nstd::string {deserName}String;"
            deserName += "String"
            closeBraceStack.append("\n" + ' '*indent + "}")
            indent += 4
        code += f'{" "*indent}orjson::LoadJsonValueByKey(value, "{self.cppParamName}", {deserName});\n'
        if self.isEnum:
            code += f"{' '*indent}{self.RenderName(True)} = Get{self.cppType}FromString({deserName}.c_str());\n"
        if self.shouldScaleInJson:
            scaleField = self.RenderName(True)
            if self.cppType == 'Transform':
                scaleField = scaleField + '.trans'
            code += f"{' '*indent}{scaleField} *= fUnitScale;\n"
        return code  + '\n'.join(reversed(closeBraceStack))

    def RenderDiffing(self):
        code = ""
        indent = 8
        suffix = ""
        code += f'{" "*indent}if ({self.RenderName(True)} != other.{self.RenderName(True)})'
        code += '\n' + ' '*indent + "{" + '\n'
        suffix = "\n" + ' '*indent + "}"
        indent += 4
        code += f'{" "*indent}diffResult.{self.RenderName(True)} = {self.RenderName(True)};'
        return code + suffix

    def RenderReset(self):
        code = ""
        indent = 8
        code += f'{" "*indent}{self.RenderName(True)} = {self.RenderDefaultValue()};'
        return code

def OutputInClassSerialization(schema):
    fieldInfos = [
        _CppParamInfo(fieldSchema, fieldName, isRequired=(fieldName in schema.get('required', [])))\
            for fieldName, fieldSchema in schema.get('properties', dict()).items()
    ]
    newLine = '\n'  # Format string interpolation forbids backslashes inside substitutions.
    return f"""
    void DeserializeJSON(const rapidjson::Value &value, const dReal fUnitScale, int options)
    {{
{newLine.join(info.RenderDeserialization() for info in fieldInfos)}
    }}

    void SerializeJSON(rapidjson::Value& rSerializedOutput, rapidjson::Document::AllocatorType& allocator, const dReal fUnitScale, int options) const
    {{
{newLine.join(info.RenderSerialization() for info in fieldInfos)}
    }}
"""

def OutputDiffing(schema):
    fieldInfos = [
        _CppParamInfo(fieldSchema, fieldName)\
            for fieldName, fieldSchema in schema.get('properties', dict()).items()
    ]
    newLine = '\n'  # Format string interpolation forbids backslashes inside substitutions.
    return f"""
    {schema["typeName"]} Diff(const {schema["typeName"]}& other)
    {{
        {schema["typeName"]} diffResult;
{newLine.join(info.RenderDiffing() for info in fieldInfos)}
        return diffResult;
    }}
"""

def OutputReset(schema):
    fieldInfos = [
        _CppParamInfo(fieldSchema, fieldName, isRequired=(fieldName in schema.get('required', [])))\
            for fieldName, fieldSchema in schema.get('properties', dict()).items()
    ]
    newLine = '\n'  # Format string interpolation forbids backslashes inside substitutions.
    return f"""
    void Reset()
    {{
{newLine.join(info.RenderReset() for info in fieldInfos)}
    }}
"""

def OutputEnumDefinition(schema):
    enumDefinition = f"enum {schema['typeName']} : uint8_t\n{{\n    "
    enumInitials = ''.join(re.findall(r'([A-Z])+', schema['typeName']))
    enumDefinition += '\n    '.join(f"{enumInitials}_{value.title()} = {index}," for index, value in enumerate(schema['enum']))
    enumDefinition += "};"

    enumInstanceName = schema['typeName'][0].lower() + schema['typeName'][1:]
    enumInstancesList = f'static const char * {enumInstanceName}Strings[{len(schema["enum"])}] = {{"' + '", "'.join(schema['enum']) + '"};'
    enumToStringFunction = f"""OPENRAVE_API const char * Get{schema['typeName']}String({schema['typeName']} {enumInstanceName})
{{
    {enumInstancesList}
    BOOST_ASSERT({enumInstanceName} < {len(schema['enum'])});
    return {enumInstanceName}Strings[{enumInstanceName}];
}}
    """

    enumInstanceName += "String"
    stringToEnumFunction = f'''OPENRAVE_API {schema["typeName"]} Get{schema["typeName"]}FromString(const char* {enumInstanceName})
{{
    // TODO(heman.gandhi): consider generating a std::unordered_map instead?
    {enumInstancesList}
    const char ** foundValue = std::find_if(
        {enumInstanceName}s,
        {enumInstanceName}s + {len(schema['enum'])},
        [{enumInstanceName}](const char * enumStr) {{
            return strcmp(enumStr, {enumInstanceName}) == 0;
        }});
    if (foundValue - {enumInstanceName}s >= {len(schema['enum'])})
    {{
        return static_cast<{schema['typeName']}>(0);
    }}
    return static_cast<{schema['typeName']}>(foundValue - {enumInstanceName}s);
}}
'''

    return '\n\n'.join([enumDefinition, enumToStringFunction, stringToEnumFunction])



class CppFileGenerator:
    _enums = []  # type: List[str] # A list of enums to output before class definitions.
    def __init__(self):
        self._enums = []
    def OutputOneClass(self, schema):
        structString = f"class OPENRAVE_API {schema['typeName']} : public InfoBase\n{{"
        for fieldName, fieldSchema in schema.get('properties', dict()).items():
            param = _CppParamInfo(fieldSchema, fieldName)
            if param.isEnum:
                self._enums.append(OutputEnumDefinition(fieldSchema))
            if 'description' in fieldSchema:
                structString += '\n   /// ' + fieldSchema['description'] + '\n'
            structString += '\n    ' + param.RenderFields() + ';\n'
        structString += OutputInClassSerialization(schema)
        structString += OutputDiffing(schema)
        structString += OutputReset(schema)
        return structString + "\n};"

    def OutputFile(self, schemas):
        classDelimiter = '\n\n'
        classes = [self.OutputOneClass(schema) for schema in schemas]
        return f"""\
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include <openrave/openrave.h>

#include "rapidjson/rapidjson.h"

namespace OpenRAVE {{

namespace generated {{

{classDelimiter.join(self._enums)}

{classDelimiter.join(classes)}

}}

}}
"""

if __name__ == "__main__":
    print(CppFileGenerator().OutputFile([geometryInfoSchema]))
