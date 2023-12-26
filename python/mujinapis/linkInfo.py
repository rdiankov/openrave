# from mujincommon.schema.basicSchemas import MakePoseSchema, MakePointSchema
# from . import _, RaveCustomDataMapOf
# from .readableInterface import readableInterfacesSchema

from geometryInfo import geometryInfoSchema
_ = lambda x: x

linkInfoSchema = {  # TODO(felixvd): Link from openrave kinbody.LinkInfo
    "type": "object",
    "typeName": "LinkInfo",
    "properties":{
        "id": {"type": "string"},
        "name": {"type": "string"},
        # "transform": MakePoseSchema(
        #     title=_("Transformation"),
        #     description=_('The link transform is relative to the body\'s coordinate system. Same convention as in OpenRAVE. For child links, the transform depends on the joint values and the parent link. A link that has only children and no parents is the body\'s "base link".')),
        "mass": {
            "type": "number",
            "description": _("The mass of the link, in kg.")
        },
        # "inertiaMoments": MakePointSchema(description=_("The moments of inertia around each rotation axis."), dims=3),
        # "massTransform": MakePoseSchema(
        #     title=_("Mass transformation"),
        #     description=_("The frame for inertia and center of mass of the link, in the link's coordinate system.")),
        # "floatParameters": RaveCustomDataMapOf("number"),
        # "intParameters": RaveCustomDataMapOf("integer"),
        # "stringParameters": RaveCustomDataMapOf("string"),
        # "forcedAdjacentLinks": {
        #     "type": "array",
        #     "items": {"type": "string"},
        #     "description": _('Links which will be considered "adjacent". Collisions between adjacent links are ignored.')
        # },
        # "geometries": geometryInfosSchema,
        # "readableInterfaces": readableInterfacesSchema,
        # "isStatic": {"type": "boolean"},
        # "isEnabled": {"type": "boolean"},
        # "isSelfCollisionIgnored": {"type": "boolean"},
        "geometries": {
            "type": "array",
            "items": geometryInfoSchema,
            "diffById": True,
            "sharedPointerType": True,
        },
    },
}