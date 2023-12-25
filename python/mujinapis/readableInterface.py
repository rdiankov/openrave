from . import _
from .readableInterfaceComponents.bodyParameters import bodyParametersSchema
from .readableInterfaceComponents.robotMotionParameters import robotMotionParametersSchema
from .readableInterfaceComponents.modelProcessorState import modelProcessorStateSchema
from .readableInterfaceComponents.feedbackHistory import feedbackHistorySchema

from mujincommon.schema import appearanceParametersSchema, modelProcessorPropertiesSchema

readableInterfacesSchema = {
    "type": "object",
    "typeName": "ReadableInterfaces",
    "properties": {
        "bodyparameters": bodyParametersSchema,
        "robotmotionparameters": robotMotionParametersSchema,
        # TODO(heman.gandhi): move that schema here? (2023/12 -- would be blocked by orchestrator.)
        "appearanceParameters": appearanceParametersSchema.appearanceParametersSchema,
        "modelProcessorState": modelProcessorStateSchema,
        # TODO(heman.gandhi): move that schema here? (2023/12 -- would be blocked by orchestrator.)
        "modelProcessorProperties": modelProcessorPropertiesSchema.modelProcessorPropertiesSchema,
        "feedbackHistory": feedbackHistorySchema,
    }
}