from ... import _

def RaveCustomDataMapOf(valueType):
    return {
        "description": _("Custom key-value pairs that could not be fit in the current model."),
        "type": "array",
        "items": {
            "type": "object",
            "properties": {
                "id": {"type": "string"},
                "values": {"type": "array", "items": {"type": valueType}},
            },
        },
    }
