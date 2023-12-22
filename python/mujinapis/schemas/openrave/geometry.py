# -*- coding: utf-8 -*-
# Copyright (C) 2016 MUJIN Inc

from textwrap import dedent

from mujincommon.schema.basicSchemas import MakePoseSchema
from . import _


geometryInfosColorTriple = {
    'type': 'array',
    'minItems': 3,
    'maxItems': 3,
    'items': {
        'type': 'number'
    }
}

geometryInfoSchema = {  # TODO(felixvd): Link to kinbody.GeometryInfo
    "type": "object",
    "typeName": "Geometry",
    "properties":{
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
            "items": {
                "type": "number"
            }
        },
        "innerExtents": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {
                "type": "number"
            }
        },
        "halfExtents": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {
                "type": "number"
            }
        },
        "transparency": {
            "type": "number"
        },
        "negativeCropContainerMargins": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {
                "type": "number"
            }
        },
        "negativeCropContainerEmptyMargins": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {
                "type": "number"
            }
        },
        "positiveCropContainerMargins": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {
                "type": "number"
            }
        },
        "positiveCropContainerEmptyMargins": {
            "type": "array",
            "minItems": 3,
            "maxItems": 3,
            "items": {
                "type": "number"
            }
        },
        "transform": MakePoseSchema(title=_("transform"), description=_("transform")),
    }
}

geometryInfosSchema = {
    "type": "array",
    "description": _("list of serialized geometry infos of the base link when the first geometry is cage (for now)"),
    "items": geometryInfoSchema,
}
