from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}

    blocks["robot_jog_joint_absolute"] = PyriBlocklyBlock(
        name = "robot_jog_joint_absolute",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
                "type": "robot_jog_joint_absolute",
                "message0": "Jog Joint %1 to degree %2 %3 with %4 %% speed",
                "args0": [
                    {
                    "type": "field_dropdown",
                    "name": "JOINT_SELECTED",
                    "options": [
                        [
                        "1",
                        "1"
                        ],
                        [
                        "2",
                        "2"
                        ],
                        [
                        "3",
                        "3"
                        ],
                        [
                        "4",
                        "4"
                        ],
                        [
                        "5",
                        "5"
                        ],
                        [
                        "6",
                        "6"
                        ],
                        [
                        "7",
                        "7"
                        ]
                    ]
                    },
                    {
                    "type": "input_dummy"
                    },
                    {
                    "type": "input_value",
                    "name": "DEGREE",
                    "check": "Number"
                    },
                    {
                    "type": "field_number",
                    "name": "SPEED",
                    "value": 100,
                    "min": 0,
                    "max": 100,
                    "precision": 1
                    }
                ],
                "inputsInline": true,
                "previousStatement": null,
                "nextStatement": null,
                "colour": 0,
                "tooltip": "",
                "helpUrl": ""
                }""",
        python_generator = """Blockly.Python['robot_jog_joint_absolute'] = function(block) {
                            var dropdown_joint_selected = block.getFieldValue('JOINT_SELECTED');
                            var value_degree = Blockly.Python.valueToCode(block, 'DEGREE', Blockly.Python.ORDER_ATOMIC);
                            var number_speed = block.getFieldValue('SPEED');
                            
                            var code = 'robot_jog_joint(' + dropdown_joint_selected + ', ' + value_degree + ', ' + number_speed + ')\\n';
                            return code;
                            };"""
    )

    
    blocks["robot_jog_joints_absolute"] = PyriBlocklyBlock(
        name = "robot_jog_joints_absolute",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
            "type": "robot_jog_joints_absolute",
            "message0": "Jog Joints with speed %1 to degrees %2",
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
                "type": "input_value",
                "name": "JOINT_VECTOR"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 0,
            "tooltip": "",
            "helpUrl": ""
            }""",
        python_generator = """Blockly.Python['robot_jog_joints_absolute'] = function(block) {
                            
                            var number_speed = block.getFieldValue('SPEED');
                            var value_joint_vector = Blockly.Python.valueToCode(block, 'JOINT_VECTOR', Blockly.Python.ORDER_ATOMIC);
                            var code = 'robot_jog_joints(' + value_joint_vector + ', ' + number_speed + ')\\n';
                            return code;
                            };"""
    )

    
    blocks["robot_jog_pose_absolute"] = PyriBlocklyBlock(
        name = "robot_jog_pose_absolute",
        category = "Robotics",
        doc = "Jog robot end effector to pose",
        json = """{
                "type": "robot_jog_pose_absolute",
                "message0": "Jog Cartesian with speed %1 in frame %2 to pose %3",
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
                    "type": "input_value",
                    "name": "ROBOT_POSE",
                    "align": "RIGHT"
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": 340,
                "tooltip": "Jog the robot to an end effector pose",
                "helpUrl": ""
                }""",
        python_generator = """Blockly.Python['robot_jog_pose_absolute'] = function(block) {
                            var number_speed = block.getFieldValue('SPEED');
                            var value_robot_pose = Blockly.Python.valueToCode(block, 'ROBOT_POSE', Blockly.Python.ORDER_ATOMIC);
                            var dropdown_frame = block.getFieldValue('FRAME');
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_jog_pose(' + value_robot_pose + ', ' + number_speed + ',\"' + dropdown_frame + '\")\\n';
                            return code;
                            };
                            """
    )

    blocks["robot_get_end_pose"] = PyriBlocklyBlock(
        name = "robot_get_end_pose",
        category = "Robotics",
        doc = "Get robot end pose",
        json = """
                {
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
                "output": null,
                "colour": 340,
                "tooltip": "Get robot end pose in specified frame",
                "helpUrl": ""
                }
               """,
        python_generator = """
                            Blockly.Python['robot_get_end_pose'] = function(block) {
                            var dropdown_frame = block.getFieldValue('FRAME');
                            // TODO: Assemble Python into code variable.     
                            var code = 'robot_get_end_pose(\"' + dropdown_frame + '\")';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return [code, Blockly.Python.ORDER_NONE];
                            };
                           """
    )

    blocks["robot_get_joint_position"] = PyriBlocklyBlock(
        name = "robot_get_joint_position",
        category = "Robotics",
        doc = "Get robot joint position in deg or m",
        json = """
                {
                "type": "robot_get_joint_position",
                "message0": "get robot joint position",
                "output": null,
                "colour": 340,
                "tooltip": "Get robot joint position in deg or m",
                "helpUrl": ""
                }
               """,
        python_generator = """
                            Blockly.Python['robot_get_joint_position'] = function(block) {
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_get_joint_position()';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return [code, Blockly.Python.ORDER_NONE];
                            };
                           """
    )
    
    blocks["robot_tool_gripper"] = PyriBlocklyBlock(
        name = "robot_tool_gripper",
        category = "Robotics",
        doc = "Robot Gripper Tool",
        json = """{
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
                "previousStatement": null,
                "nextStatement": null,
                "colour": 60,
                "tooltip": "Change the Gripper Mode",
                "helpUrl": ""
                }""",
        python_generator = """Blockly.Python['robot_tool_gripper'] = function(block) {
                            var dropdown_gripper_status = block.getFieldValue('GRIPPER_STATUS');
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_tool_gripper(' + dropdown_gripper_status + ')\\n';
                            return code;
                            };
                            """
        
    )

    blocks["robot_planar_grab"] = PyriBlocklyBlock(
        name = "robot_planar_grab",
        category = "Robotics",
        doc = "Robot planar grab object",
        json = """
                {
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
                    "checked": true
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
                "previousStatement": null,
                "nextStatement": null,
                "colour": 340,
                "tooltip": "Get inverse of pose",
                "helpUrl": ""
                }
               """,

        python_generator = """
                            Blockly.Python['robot_planar_grab'] = function(block) {
                            var text_reference_pose = block.getFieldValue('REFERENCE_POSE');
                            var number_speed = block.getFieldValue('SPEED');
                            var checkbox_wait = block.getFieldValue('WAIT') == 'TRUE' ? 'True':'False';
                            var value_z_before = Blockly.Python.valueToCode(block, 'Z_BEFORE', Blockly.Python.ORDER_ATOMIC);
                            var value_z_grab = Blockly.Python.valueToCode(block, 'Z_GRAB', Blockly.Python.ORDER_ATOMIC);
                            var value_object_pose = Blockly.Python.valueToCode(block, 'OBJECT_POSE', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble JavaScript into code variable.
                            var code = 'robot_planar_grab(' + value_object_pose+ ',\"' + text_reference_pose + '\",' + value_z_before + ',' + value_z_grab + ',' + number_speed + ',' + checkbox_wait + ')\\n';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return code;
                            };
                           """
    )

    return blocks

def _get_categories() -> Dict[str,PyriBlocklyCategory]:
    categories = {}
    categories["Robotics"] = PyriBlocklyCategory(
        name = "Robotics",
        json = '{"kind": "category", "name": "Robotics", "colour": 230 }'
    )

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