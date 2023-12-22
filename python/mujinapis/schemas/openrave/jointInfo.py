from textwrap import dedent
from . import _, RaveCustomDataMapOf

from .readableInterface import readableInterfacesSchema

jointInfoSchema = {  # TODO(felixvd): Link from openrave kinbody.JointInfo
    "type": "object",
    "typeName": "JointInfo",
    "properties": {
        "id": {
            "type": "string",
            "description": _("Unique ID of the joint.")
        },
        "name": {
            "type": "string",
            "description": _("Unique name of the joint.")
        },
        "type": {
            "type": "string",
            # NOTE: "specialbit" is omitted since setting it will lead to exceptions in JointInfo::GetDOF.
            "enum": ["revolute", "prismatic", "rr", "rp", "pr", "pp", "universal", "hinge2", "spherical", "trajectory"],
        },
        "anchors": {
            "type": "array",
            "items": {"type": "number"},
            "minItems": 3,
            "maxItems": 4,
            "description": _("The anchor of the rotation axes defined in _linkname0's coordinate system. This is only used to construct the internal left/right matrices.")
        },
        "parentLinkName": {
            "type": "string",
            "description": _("The parent link. All axes and anchors are defined in the link called this name's coordinate system.")
        },
        "childLinkName": {"type": "string"},
        "axes": {
            "type": "array",
            "items": {
                # See openrave's geometry's RaveVector.
                "minItems": 3,
                "maxItems": 4,
                "type": "array",
                "prefixItems": [
                    # (OpenRAVE::RaveVector::operator[])
                    {'title': 'x', 'type': 'number'},
                    {'title': 'y', 'type': 'number'},
                    {'title': 'z', 'type': 'number'},
                    {'title': 'w', 'type': 'number'},
                ]
            },
            "minItems": 3,
            "maxItems": 3,
            "description": _("Axes in _linkname0's or environment coordinate system used to define joint movement.")
        },
        "currentValues": {
            "type": "array",
            "items": {"type": "number"},
            "description": _("Joint values at initialization.")
        },
        "resolutions": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("Interpolation resolution.")
        },
        "maxVel": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The soft maximum velocity (rad/s) to move the joint when planning.")
        },
        "hardMaxVel": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The hard maximum velocity (rad/s) which the robot cannot exceed. Used for verification checking. Default hard limit is the 0 vector, meaning the user should not use the hard limit value."),
            "default": [0, 0, 0]
        },
        "maxAccel": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The soft maximum acceleration (rad/s^2) of the joint.")
        },
        "hardMaxAccel": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The hard maximum acceleration (rad/s^2) which the robot cannot exceed. Used for verification checking. Default hard limit is the 0 vector, meaning the user should not use the hard limit value."),
            "default": [0, 0, 0]
        },
        "maxJerk": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The soft maximum jerk (rad/s^3) of the joint.")
        },
        "hardMaxJerk": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The hard maximum jerk (rad/s^3) which the robot cannot exceed. Used for verification checking. Default hard limit is the 0 vector, meaning the user should not use the hard limit value."),
            "default": [0, 0, 0]
        },
        "maxTorque": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("Maximum torque (N.m, kg m^2/s^2) that should be applied to the joint. Usually this is computed from the motor nominal torque and gear ratio. Ignore if values are 0. Set max torque to 0 to notify the system that dynamics parameters might not be valid."),
        },
        "maxInertia": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("Maximum inertia (kg m^2) that the joint can exhibit. Usually this is set for safety reasons. Ignore if values are 0."),
        },
        "weights": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The weights of the joint for computing distance metrics."),
        },
        "offsets": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "title": _("Internal offset parameter that determines the branch the angle centers on."),
            "description": _(dedent("""\
                Wrap offsets are needed for rotation joints since the range is limited to 2*pi.
                This allows the wrap offset to be set so the joint can function in [-pi+offset,pi+offset]."""))
        },
        "lowerLimit": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The lower limit of the joint."),
        },
        "upperLimit": {
            "type": "array",
            "items": {"type": "number"},
            "maxItems": 3,
            "description": _("The upper limit of the joint."),
        },
        "mimics": {
            "description": _("The mimic properties of each of the joint axes. It is theoretically possible for a multi-dof joint to have one axes mimiced and the others free."),
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "equations": {
                        "type": "array",
                        "minItems": 3,
                        "maxItems": 3,
                        "prefixItems": [
                            {
                                "title": _("Position equation"),
                                "type": "string"
                            }, {
                                "title": _("Velocity equation"),
                                "type": "string"
                            }, {
                                "title": _("Acceleration equation"),
                                "type": "string"
                            },
                        ]
                    }
                }
            },
            "maxItems": 3,
        },
        "floatParameters": RaveCustomDataMapOf("number"),
        "intParameters": RaveCustomDataMapOf("integer"),
        "stringParameters": RaveCustomDataMapOf("string"),
        "electricMotorActuator": {
            "type": "object",
            "properties": {
                "modelType": {
                    "type": "string",
                    "description": _("The type of actuator it is. Usually just the motor model name, but can include other info like the gear box."),
                },
                "assignedPowerRating": {
                    "type": "number",
                    "description": _("The nominal power the electric motor can safely produce. Units are **Mass * Distance^2 / Time^3**."),
                },
                "maxSpeed": {
                    "type": "number",
                    "description": _("The maximum speed of the motor **Time^(-1)**.")
                },
                "noLoadSpeed": {
                    "type": "number",
                    "description": _("Specifies the speed of the motor powered by the nominal voltage when the motor provides zero torque. Units are **Time^(-1)**.")
                },
                "stallTorque": {
                    "type": "number",
                    "description": _("The maximum torque achievable by the motor at the nominal voltage. This torque is achieved at zero velocity (stall). Units are **Mass * Distance / Time^2**.")
                },
                "maxInstantaneousTorque": {
                    "type": "number",
                    "description": _("The maximum instantenous torque achievable by the motor when voltage <= nominal voltage. Motor going between nominal_torque and max_instantaneous_torque can overheat, so should not be driven at it for a long time. Units are **Mass * Distance / Time^2**.")
                },
                "nominalSpeedTorquePoints": {
                    "type": "array",
                    "description": _("The speed and torque achievable when the motor is powered by the nominal voltage. Given the speed, the max torque can be computed. If not specified, the speed-torque curve will just be a line connecting the no load speed and the stall torque directly (ideal). Should be ordered from increasing speed."),
                    "items": {
                        "type": "array",
                        "prefixItems": [{
                            "title": _("Speed"),
                            "type": "number"
                        }, {
                            "title": _("Torque"),
                            "type": "number"
                        }]
                    }
                },
                "maxSpeedTorquePoints": {
                    "type": "array",
                    "description": _("The speed and torque achievable when the motor is powered by the max voltage/current. Given the speed, the max torque can be computed. If not specified, the speed-torque curve will just be a line connecting the no load speed and the max_instantaneous_torque directly (ideal). Should be ordered from increasing speed."),
                    "items": {
                        "type": "array",
                        "prefixItems": [{
                            "title": _("Speed"),
                            "type": "number"
                        }, {
                            "title": _("Torque"),
                            "type": "number"
                        }]
                    }
                },
                "nominalTorque": {
                    "type": "number",
                    "description": _("The maximum torque the motor can provide continuously without overheating. Units are **Mass * Distance / Time^2**.")
                },
                "rotorInertia": {
                    "type": "number",
                    "description": _("The inertia of the rotating element about the axis of rotation. Units are **Mass * Distance^2**.")
                },
                "torqueConstant": {
                    "type": "number",
                    "description": _("Specifies the proportion relating current to torque. Units are **Mass * Distance / (Time * Charge)**.")
                },
                "nominalVoltage": {
                    "type": "number",
                    "description": _("The nominal voltage the electric motor can safely produce. Units are **Mass * Distance^2 * Charge / (Time^2)**.")
                },
                "speedConstant": {
                    "type": "number",
                    "description": _("The constant of proportionality relating speed to voltage. Units are **Time / (Mass * Distance^2 * Charge)**.")
                },
                "startingCurrent": {
                    "type": "number",
                    "description": _("Specifies the current through the motor at zero velocity, equal to the nominal voltage divided by the terminal resistance. Also called the stall current.  Units are **Charge/Time**.")
                },
                "terminalResistance": {
                    "type": "number",
                    "description": _("The resistance of the motor windings. Units are **Mass * Distance^2 / (Time * Charge^2)**.")
                },
                "gearRatio": {
                    "type": "number",
                    "description": _("Specifies the ratio between the input speed of the transmission (the speed of the motor shaft) and the output speed of the transmission.")
                },
                "coloumbFriction": {
                    "type": "number",
                    "description": _("Static coloumb friction on each joint after the gear box. Units are **Mass * Distance / Time^2**.")
                },
                "viscousFriction": {
                    "type": "number",
                    "description": _("Viscous friction on each joint after the gear box. Units are **Mass * Distance / Time^2**.")
                },
            },
        },
        "isCircular": {
            "title": _("True (>0) if the corresponding joint axis has identification at its upper and lower limits."),
            "description": _(dedent("""\
                An identification of the lower and upper limits means that once the joint reaches its upper limits, it is also
                at its lower limit. The most common identification on revolute joints at -pi and pi. 'circularity' means the
                joint does not stop at limits.""")),
            "type": "array",
            "items": {
                "type": "integer"
            },
            "maxItems": 3,
        },
        "isActive": {
            "type": "boolean",
            "description": _("If true, should belong to the DOF of the body, unless it is a mimic joint."),
        },
        "controlMode": {
            "type": "string",
            "enum": ["None", "RobotController", "IO", "ExternalDevice"]
        },
        "jointControlInfoRobotController": {
            "type": "object",
            "properties": {
                "controllerType": {
                    "type": "string",
                    "description": _("The type of the controller used to control this joint.")
                },
                "robotControllerAxisIndex": {
                    "type": "array",
                    "items": {
                        "type": "integer"
                    },
                    # NOTE: this is a boost::array<int16_t, 3>, hence I put maxItems: 3.
                    "maxItems": 3,
                    "description": _("Indicates which DOF in the robot controller controls which joint axis (up to the DOF of the joint). -1 if not specified or not valid.")
                },
                "robotControllerAxisMult": {
                    "type": "array",
                    "items": {
                        "type": "number"
                    },
                    "minItems": 3,
                    "maxItems": 3,
                    "description": _("Indicates the multiplier to convert the joint values from the environment units to the robot controller units, per joint axis. (valueInRobotControllerUnits - robotControllerAxisOffset) / robotControllerAxisMult = valueInEnvUnits")
                },
                "robotControllerAxisOffset": {
                    "type": "array",
                    "items": {
                        "type": "number"
                    },
                    "minItems": 3,
                    "maxItems": 3,
                    "description": _("Indicates the offset, in the robot controller units, that should be applied to the joint values, per joint axis.")
                },
                "robotControllerAxisProductCode": {
                    "type": "array",
                    "items": {
                        "type": "string"
                    },
                    "minItems": 3,
                    "maxItems": 3,
                    "description": _("Indicates the product codes of the servo devices per joint axis, in order for the robot controller to validate communication with the servo devices. It is different from ElectricMotorActuatorInfo::model_type, which is the name of the motor type attached to the servo devices.")
                },
            },
        },
        "jointControlInfoIO": {
            "type": "object",
            "properties": {
                "deviceType": {"type": "string"},
                "moveIONames": {
                    "type": "array",
                    "items": {
                        "type": "array",
                        "items": {"type": "string"}
                    },
                    "minItems": 3,
                    "maxItems": 3,
                    "description": _("IO names for controlling positions of this joint.")
                },
                "upperLimitIONames": {
                    "type": "array",
                    "items": {
                        "type": "array",
                        "items": {"type": "string"}
                    },
                    "minItems": 3,
                    "maxItems": 3,
                    "description": _("IO names for detecting if the joint is at its upper limit.")
                },
                "upperLimitSensorIsOn": {
                    "type": "array",
                    "items": {
                        "type": "array",
                        "items": {
                            "type": "integer",
                            "default": 1
                        }
                    },
                    "minItems": 3,
                    "maxItems": 3,
                    "description": _("If true, the corresponding upper limit sensor reads 1 when the joint is at its upper limit. Otherwise, the upper limit sensor reads 0 when the joint is at its upper limit. The default value is 1."),
                },
                "lowerLimitIONames": {
                    "type": "array",
                    "items": {
                        "type": "array",
                        "items": {"type": "string"}
                    },
                    "minItems": 3,
                    "maxItems": 3,
                    "description": _("IO names for detecting if the joint is at its lower limit.")
                },
                "lowerLimitSensorIsOn": {
                    "type": "array",
                    "items": {
                        "type": "array",
                        "items": {
                            "type": "integer",
                            "default": 1
                        }
                    },
                    "minItems": 3,
                    "maxItems": 3,
                    "description": _("If true, the corresponding lower limit sensor reads 1 when the joint is at its lower limit. Otherwise, the lower limit sensor reads 0 when the joint is at its lower limit. The default value is 1."),
                },
            },
        },
        "jointControlInfoExternalDevice": {
            "type": "object",
            "properties": {
                "externalDeviceType": {"type": "string"},
            },
        },
        "readableInterfaces": readableInterfacesSchema,
    },
}