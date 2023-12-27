from geometryInfo import geometryInfoSchema
from linkInfo import linkInfoSchema
import re

class CppCodeWriter:
    indent = 0
    code = ''

    def __init__(self, indent):
        self.indent = indent

    def WriteLine(self, line):
        self.code += f"{' ' * self.indent}{line}\n"
    
    def StartBlock(self):
        self.WriteLine("{")
        self.indent += 4
    
    def EndBlock(self):
        self.indent -= 4
        self.WriteLine("}")

    def GetCode(self):
        return self.code

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
        typeSuffix = 'Ptr' if schema.get('sharedPointerType', False) else ''
        if schema.get('prefixItems'):
            return 'std::tuple<' + ', '.join(f'{_JsonSchemaTypeToCppType(item)}{typeSuffix}' for item in schema['prefixItems']) + '>'
        elif schema.get('items'):
            return 'std::vector<' + f'{_JsonSchemaTypeToCppType(schema["items"])}{typeSuffix}' + '>'
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
    diffById = False # type: bool
    itemTypeName = '' # type: str
    sharedPointerType = False # type: bool
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
        self.sharedPointerType = schema.get('sharedPointerType', False)
        self.diffById = schema.get('diffById', False)
        if self.diffById:
            self.itemTypeName = _JsonSchemaTypeToCppType(schema['items'])
    def RenderFields(self, prefixOverride=None):
        prefix = prefixOverride if prefixOverride is not None else self.fieldNamePrefix
        # paramString = f"{self.cppType} {prefix}{self.cppParamName}"
        paramString = f"boost::optional<{self.cppType}> {prefix}{self.cppParamName} = {self.RenderDefaultValue()}"
        return paramString
    def RenderName(self, needsPrefix):
        if needsPrefix:
            return self.fieldNamePrefix + self.cppParamName
        else:
            return self.cppParamName
    def RenderDefaultValue(self):
        if isinstance(self.defaultValue, bool):
            return 'true' if self.defaultValue else 'false'
        if self.cppType == "std::string":
            return 'std::string("' + (self.defaultValue if self.hasDefault else '') + '")'
        if self.hasDefault:
            return str(self.defaultValue)
        return self.cppType + "{}"

    def RenderSerialization(self, writer, cppJsonVariable="rSerializedOutput", cppJsonAllocVariable="allocator"):
        serName = self.RenderName(True)

        if self.isEnum:
            writer.WriteLine(f'std::string {serName}String = Get{self.cppType}String({serName});')
            serName += "String"
        
        if not self.isRequired:
            writer.WriteLine(f'if (*{self.RenderName(True)} != {self.RenderDefaultValue()})')
            writer.StartBlock()
            
        if self.shouldScaleInJson:
            if self.cppType == 'Transform':
                writer.WriteLine(f"Transform scaled{serName} = *{serName};")
                serName = "scaled" + serName
                writer.WriteLine(f"{serName}.trans *= fUnitScale;")
            else:
                writer.WriteLine(f"{self.cppType} scaled{serName} = *{serName};")
                serName = "scaled" + serName
                writer.WriteLine(f"{serName} *= fUnitScale;")

        if self.diffById:
            writer.WriteLine(f'for(int i=0; i<{serName}->size(); i++)')
            writer.StartBlock()
            if self.sharedPointerType:
                writer.WriteLine(f'rapidjson::Value value;')
                writer.WriteLine(f'(*{serName})[i]->SerializeJSON(value, {cppJsonAllocVariable}, fUnitScale, options);\n')
                writer.WriteLine(f'{cppJsonVariable}.AddMember(rapidjson::Document::StringRefType("{self.cppParamName}"), value, {cppJsonAllocVariable});')
            else:
                writer.WriteLine(f'rapidjson::Value value;')
                writer.WriteLine(f'(*{serName})[i].SerializeJSON(value, {cppJsonAllocVariable}, fUnitScale, options);')
                writer.WriteLine(f'{cppJsonVariable}.AddMember(rapidjson::Document::StringRefType("{self.cppParamName}"), value, {cppJsonAllocVariable});')
            writer.EndBlock()
        else:
            if self.shouldScaleInJson:
                writer.WriteLine(f"orjson::SetJsonValueByKey({cppJsonVariable}, \"{self.cppParamName}\", {serName}, {cppJsonAllocVariable});")
            else:
                writer.WriteLine(f"orjson::SetJsonValueByKey({cppJsonVariable}, \"{self.cppParamName}\", *{serName}, {cppJsonAllocVariable});")
        
        if not self.isRequired:
            writer.EndBlock()

    def RenderDeserialization(self, writer):
        if not self.isRequired:
            writer.WriteLine(f'if (value.HasMember("{self.cppParamName}"))')
            writer.StartBlock()

        deserName = self.RenderName(True)

        if self.isEnum:
            writer.WriteLine(f"std::string {deserName}String;")
            deserName += "String"

        if self.diffById:
            writer.WriteLine(f'{deserName}->clear();')
            writer.WriteLine(f'for (rapidjson::Value::ConstValueIterator it = value["{self.cppParamName}"].Begin(); it != value["{self.cppParamName}"].End(); ++it)')
            writer.StartBlock()
            if self.sharedPointerType:
                writer.WriteLine(f'{deserName}->push_back(boost::make_shared<{self.itemTypeName}>());')
                writer.WriteLine(f'(*{deserName})[{deserName}->size()-1]->DeserializeJSON(*it, fUnitScale, options);')
            else:
                writer.WriteLine(f'{deserName}->push_back({self.itemTypeName}());')
                writer.WriteLine(f'(*{deserName})[{deserName}->size()-1].DeserializeJSON(*it, fUnitScale, options);')
            writer.EndBlock()
        else:
            writer.WriteLine(f'orjson::LoadOptionalJsonValueByKey(value, "{self.cppParamName}", {deserName});')
        
        if self.isEnum:
            writer.WriteLine(f"{self.RenderName(True)} = Get{self.cppType}FromString({deserName}.c_str());")

        if self.shouldScaleInJson:
            scaleField = self.RenderName(True)
            if self.cppType == 'Transform':
                scaleField = scaleField + '->trans'
                writer.WriteLine(f"{scaleField} *= fUnitScale;")
            else:
                writer.WriteLine(f"*{scaleField} *= fUnitScale;")

        if not self.isRequired:
            writer.EndBlock()

    def RenderDiffing(self, writer):
        writer.WriteLine(f'if ({self.RenderName(True)} != other.{self.RenderName(True)})')
        writer.StartBlock()
        writer.WriteLine(f'diffResult.{self.RenderName(True)} = {self.RenderName(True)};')
        writer.EndBlock()

    def RenderReset(self, writer):
        writer.WriteLine(f'{self.RenderName(True)} = {self.RenderDefaultValue()};')

def OutputInClassSerialization(schema):
    fieldInfos = [
        _CppParamInfo(fieldSchema, fieldName, isRequired=(fieldName in schema.get('required', [])))\
            for fieldName, fieldSchema in schema.get('properties', dict()).items()
    ]

    writer = CppCodeWriter(indent=4)
    writer.WriteLine(f"void DeserializeJSON(const rapidjson::Value &value, const dReal fUnitScale, int options)")
    writer.StartBlock()
    for info in fieldInfos:
        info.RenderDeserialization(writer)
    writer.EndBlock()

    writer.WriteLine(f"void SerializeJSON(rapidjson::Value& rSerializedOutput, rapidjson::Document::AllocatorType& allocator, const dReal fUnitScale, int options) const")
    writer.StartBlock()
    for info in fieldInfos:
        info.RenderSerialization(writer)
    writer.EndBlock()

    return writer.GetCode()

def OutputDiffing(schema):
    fieldInfos = [
        _CppParamInfo(fieldSchema, fieldName)\
            for fieldName, fieldSchema in schema.get('properties', dict()).items()
    ]

    writer = CppCodeWriter(indent=4)
    writer.WriteLine(f"{schema['typeName']} Diff(const {schema['typeName']}& other) const")
    writer.StartBlock()
    writer.WriteLine(f"{schema['typeName']} diffResult;")
    for info in fieldInfos:
        info.RenderDiffing(writer)
    writer.WriteLine(f"return diffResult;")
    writer.EndBlock()

    return writer.GetCode()

def OutputReset(schema):
    fieldInfos = [
        _CppParamInfo(fieldSchema, fieldName, isRequired=(fieldName in schema.get('required', [])))\
            for fieldName, fieldSchema in schema.get('properties', dict()).items()
    ]

    writer = CppCodeWriter(indent=4)
    writer.WriteLine("void Reset()")
    writer.StartBlock()
    for info in fieldInfos:
        info.RenderReset(writer)
    writer.EndBlock()

    return writer.GetCode()

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
        structString += "\npublic:\n"
        structString += OutputInClassSerialization(schema)
        structString += OutputDiffing(schema)
        structString += OutputReset(schema)
        structString += "\n};\n"
        structString += f"typedef boost::shared_ptr<{schema['typeName']}> {schema['typeName']}Ptr;\n"
        return structString
    
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
#include <boost/optional.hpp>

namespace OpenRAVE {{

namespace generated {{

{classDelimiter.join(self._enums)}

{classDelimiter.join(classes)}

}}

}}
"""

if __name__ == "__main__":
    print(CppFileGenerator().OutputFile([geometryInfoSchema, linkInfoSchema]))
