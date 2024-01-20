from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory, \
    add_blockly_block, add_blockly_category
from . import sandbox_functions
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}
        
    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
            "type": "robot_movej",
            "message0": "robot movej with speed %1 wait %2 to degrees %3",
            "args0": [
                {
                "type": "field_number",
                "name": "SPEED",
                "value": 100,
                "min": 0,
                "max": 100,
                "precision": 1
                },
                {
                    "type": "field_checkbox",
                    "name": "WAIT",
                    "checked": True
                },
                {
                "type": "input_value",
                "name": "JOINT_VECTOR"
                }
            ],
            "previousStatement": None,
            "nextStatement": None,
            "colour": 0,
            "tooltip": "Move joints to absolute position",
            "helpUrl": ""
            },
        sandbox_function = (sandbox_functions.robot_movej,"JOINT_VECTOR","SPEED","WAIT")
    )

    
    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                "type": "robot_movel",
                "message0": "robot movel with speed %1 in frame %2 wait %3 to pose %4",
                "args0": [
                    {
                    "type": "field_number",
                    "name": "SPEED",
                    "value": 100,
                    "min": 0,
                    "max": 100,
                    "precision": 1
                    },
                    {
                    "type": "field_dropdown",
                    "name": "FRAME",
                    "options": [
                        [
                        "world",
                        "WORLD"
                        ],
                        [
                        "robot",
                        "ROBOT"
                        ]
                    ]
                    },
                    {
                        "type": "field_checkbox",
                        "name": "WAIT",
                        "checked": True
                    },
                    {
                    "type": "input_value",
                    "name": "ROBOT_POSE",
                    "align": "RIGHT"
                    }
                ],
                "previousStatement": None,
                "nextStatement": None,
                "colour": 340,
                "tooltip": "Move robot along a line to absolute position",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.robot_movel,"ROBOT_POSE","SPEED","FRAME","WAIT")
    )

    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                    "type": "robot_move_joint_trajectory",
                    "message0": "robot execute with speed %1 wait %2 joint trajectory %3",
                    "args0": [
                        {
                        "type": "field_number",
                        "name": "SPEED",
                        "value": 100,
                        "min": 0,
                        "max": 100,
                        "precision": 1
                        },
                        {
                        "type": "field_checkbox",
                        "name": "WAIT",
                        "checked": True
                        },
                        {
                        "type": "input_value",
                        "name": "TRAJECTORY"
                        }
                    ],
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 0,
                    "tooltip": "Execute a previously planned joint trajectory",
                    "helpUrl": ""
                    },
        sandbox_function = (sandbox_functions.robot_move_joint_trajectory,"TRAJECTORY","SPEED","WAIT")
    )

    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                "type": "robot_get_end_pose",
                "message0": "get robot end pose in frame %1",
                "args0": [
                    {
                    "type": "field_dropdown",
                    "name": "FRAME",
                    "options": [
                        [
                        "world",
                        "WORLD"
                        ],
                        [
                        "robot",
                        "ROBOT"
                        ]
                    ]
                    }
                ],
                "output": None,
                "colour": 340,
                "tooltip": "Get robot end pose in specified frame",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.robot_get_end_pose,"FRAME")
    )

    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                "type": "robot_get_joint_position",
                "message0": "get robot joint position",
                "output": None,
                "colour": 340,
                "tooltip": "Get robot joint position in deg or m",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.robot_get_joint_position,)
    )
    
    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                "type": "robot_tool_gripper",
                "message0": "Gripper %1",
                "args0": [
                    {
                    "type": "field_dropdown",
                    "name": "GRIPPER_STATUS",
                    "options": [
                        [
                        "OPEN",
                        "1"
                        ],
                        [
                        "CLOSED",
                        "0"
                        ]
                    ]
                    }
                ],
                "previousStatement": None,
                "nextStatement": None,
                "colour": 60,
                "tooltip": "Open or close the gripper",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.robot_tool_gripper,"GRIPPER_STATUS")        
    )

    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                "type": "robot_planar_grab",
                "message0": "robot planar grab object using using reference %1 with speed %2 wait %3 %4 Z offset before grab %5 Z offset grab %6 object pose %7",
                "args0": [
                    {
                    "type": "field_input",
                    "name": "REFERENCE_POSE",
                    "text": "reference_pose_global"
                    },
                    {
                    "type": "field_number",
                    "name": "SPEED",
                    "value": 100,
                    "min": 0,
                    "max": 100,
                    "precision": 1
                    },
                    {
                    "type": "field_checkbox",
                    "name": "WAIT",
                    "checked": True
                    },
                    {
                    "type": "input_dummy",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "Z_BEFORE",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "Z_GRAB",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "OBJECT_POSE",
                    "align": "RIGHT"
                    }
                ],
                "previousStatement": None,
                "nextStatement": None,
                "colour": 340,
                "tooltip": "Robot planar grab object",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.robot_planar_grab,"OBJECT_POSE","REFERENCE_POSE","Z_BEFORE","Z_GRAB","SPEED","WAIT")
    )

    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                "type": "robot_planar_place",
                "message0": "robot planar place object using using reference %1 with speed %2 wait %3 %4 Z offset before place %5 Z offset place %6 target pose %7",
                "args0": [
                    {
                    "type": "field_input",
                    "name": "REFERENCE_POSE",
                    "text": "reference_pose_global"
                    },
                    {
                    "type": "field_number",
                    "name": "SPEED",
                    "value": 100,
                    "min": 0,
                    "max": 100,
                    "precision": 1
                    },
                    {
                    "type": "field_checkbox",
                    "name": "WAIT",
                    "checked": True
                    },
                    {
                    "type": "input_dummy",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "Z_BEFORE",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "Z_PLACE",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "TARGET_POSE",
                    "align": "RIGHT"
                    }
                ],
                "previousStatement": None,
                "nextStatement": None,
                "colour": 340,
                "tooltip": "Place an object at a planar location",
                "helpUrl": ""
                },
        sandbox_function = (sandbox_functions.robot_planar_place,"TARGET_POSE","REFERENCE_POSE","Z_BEFORE","Z_PLACE","SPEED","WAIT")
    )

    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                    "type": "robot_set_active_robot",
                    "message0": "set active robot %1",
                    "args0": [
                        {
                        "type": "field_input",
                        "name": "ROBOT_NAME",
                        "text": "robot"
                        }
                    ],
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 340,
                    "tooltip": "Set active robot by device name",
                    "helpUrl": ""
                    },
        sandbox_function = (sandbox_functions.robot_set_active_robot,"ROBOT_NAME")  
    )

    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
                    "type": "robot_set_origin_calibration",
                    "message0": "set robot origin calibration by name %1",
                    "args0": [
                        {
                        "type": "field_input",
                        "name": "ROBOT_CALIBRATION_NAME",
                        "text": "robot_origin_calibration"
                        }
                    ],
                    "previousStatement": None,
                    "nextStatement": None,
                    "colour": 340,
                    "tooltip": "Set previously saved robot origin calibration",
                    "helpUrl": ""
                    },
        sandbox_function = (sandbox_functions.robot_set_origin_calibration,"ROBOT_CALIBRATION_NAME")       
    )

    add_blockly_block(blocks,
        category = "Robotics",
        blockly_json = {
            "type": "robot_set_active_tool",
            "message0": "set active tool %1",
            "args0": [
                {
                "type": "field_input",
                "name": "TOOL_NAME",
                "text": "tool"
                }
            ],
            "previousStatement": None,
            "nextStatement": None,
            "colour": 60,
            "tooltip": "Set active tool by device name",
            "helpUrl": ""
            },
            sandbox_function = (sandbox_functions.robot_set_active_tool,"TOOL_NAME")
    ) 

    return blocks

def _get_categories() -> Dict[str,PyriBlocklyCategory]:
    categories = {}
    add_blockly_category(categories,"Robotics",0)

    return categories


class PyriRoboticsBlocklyPluginFactory(PyriBlocklyPluginFactory):
    def get_plugin_name(self):
        return "pyri-robotics"

    def get_category_names(self) -> List[str]:
        return ["Robotics"]

    def get_categories(self) -> List[PyriBlocklyCategory]:
        return _get_categories()

    def get_block_names(self) -> List[str]:
        return list(_get_blocks().keys())

    def get_block(self,name) -> PyriBlocklyBlock:
        return _get_blocks()[name]

    def get_all_blocks(self) -> Dict[str,PyriBlocklyBlock]:
        return _get_blocks()

def get_blockly_factory():
    return PyriRoboticsBlocklyPluginFactory()