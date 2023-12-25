from . import _

from mujincommon.schema.basicSchemas import MakePoseSchema

grabbedInfoSchema = {  # TODO(felixvd): Link from openrave kinbody.GrabbedInfo
    "type": "object",
    "typeName": "GrabbedInfo",
    "properties":{
        "id": {"type": "string"},
        "grabbedName": {"type": "string"},
        "robotLinkName": {"type": "string"},
        "transform": MakePoseSchema(title=_("transform"), description=_("transform")),
        "ignoreRobotLinkNames": {
            "type": "array",
            "items": {
                "type": "string",
            },
        },
        "grabbedUserData": {"type": "object"},
    },
}
