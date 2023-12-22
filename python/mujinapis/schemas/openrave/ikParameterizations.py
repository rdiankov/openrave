from mujincommon.schema.basicSchemas import MakePointSchema, MakePoseSchema

from . import _, RaveCustomDataMapOf

ikParameterizationSchema = {
    "type": "object",
    "typeName": "IkParameterization",
    # TODO(felixvd): Link to openrave ikparameterization
    "properties":{
        "id": {
            "type": "string",
            "description": _("The unique ID used for tracking the resource.")
        },
        "name": {
            "type": "string",
            "description": _("The user-specified name used for targetting and displaying. Unique within a body.")
        },
        "type": {
            "type": "string",
            "description": _("The type of the IKParam."),
            "enum": [
                "None",
                "Transform6D",
                "Rotation3D",
                "Translation3D",
                "Direction3D",
                "Ray4D",
                "Lookat3D",
                "TranslationDirection5D",
                "TranslationXY2D",
                "TranslationXYOrientation3D",
                "TranslationLocalGlobal6D",
                "TranslationXAxisAngle4D",
                "TranslationYAxisAngle4D",
                "TranslationZAxisAngle4D",
                "TranslationXAxisAngleZNorm4D",
                "TranslationYAxisAngleXNorm4D",
                "TranslationZAxisAngleYNorm4D",
            ]
        },
        "transform": MakePoseSchema(title=_("The transform."), description=_("The transform of the coordinate system relative to the global coordinates.")),
        "quaternion": {
            "type": "array",
            "items": [
                {"title": "w", "description": "w", "type": "number"},
                {"title": "x", "description": "x", "type": "number"},
                {"title": "y", "description": "y", "type": "number"},
                {"title": "z", "description": "z", "type": "number"},
            ]
        },
        "translate": MakePointSchema(description=_("A translation in the global coordinate system."), dims=3),
        "localTranslate": MakePointSchema(description=_("A translation the local coordinate system (transformed by this transform)."), dims=3),
        "direction": MakePointSchema(description=_("A direction vector (e.g. for approach or depart motions)."), dims=3),
        "angle": {"type": "number"},
        "customData": RaveCustomDataMapOf("number")
    },
}

ikparamsSchema = {
    "type": "array",
    "description": _("list of ik params defined in container kinbody"),
    "items": ikParameterizationSchema,
}

ikParamNamesSchema = {
    "type": "array",
    "items": {
        "type": "string",
        "semanticType": "IkparamName"
    }
}