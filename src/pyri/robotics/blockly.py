from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}
        
    blocks["robot_movej"] = PyriBlocklyBlock(
        name = "robot_movej",
        category = "Robotics",
        doc = "Move joints to absolute position",
        json = """{
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
                    "checked": true
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
        python_generator = """Blockly.Python['robot_movej'] = function(block) {
                            var checkbox_wait = block.getFieldValue('WAIT') == 'TRUE' ? 'True':'False';
                            var number_speed = block.getFieldValue('SPEED');
                            var value_joint_vector = Blockly.Python.valueToCode(block, 'JOINT_VECTOR', Blockly.Python.ORDER_ATOMIC);
                            var code = 'robot_movej(' + value_joint_vector + ', ' + number_speed + ',' + checkbox_wait + ')\\n';
                            return code;
                            };"""
    )

    
    blocks["robot_movel"] = PyriBlocklyBlock(
        name = "robot_movel",
        category = "Robotics",
        doc = "Move robot along a line to absolute position",
        json = """{
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
                        "checked": true
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
                "tooltip": "",
                "helpUrl": ""
                }""",
        python_generator = """Blockly.Python['robot_movel'] = function(block) {
                            var checkbox_wait = block.getFieldValue('WAIT') == 'TRUE' ? 'True':'False';
                            var number_speed = block.getFieldValue('SPEED');
                            var value_robot_pose = Blockly.Python.valueToCode(block, 'ROBOT_POSE', Blockly.Python.ORDER_ATOMIC);
                            var dropdown_frame = block.getFieldValue('FRAME');
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_movel(' + value_robot_pose + ', ' + number_speed + ',\"' + dropdown_frame + '\",' + checkbox_wait + ')\\n';
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

    blocks["robot_planar_place"] = PyriBlocklyBlock(
        name = "robot_planar_place",
        category = "Robotics",
        doc = "Robot planar place object",
        json = """
                {
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
                    "name": "Z_PLACE",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "TARGET_POSE",
                    "align": "RIGHT"
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": 340,
                "tooltip": "Place an object at a planar location",
                "helpUrl": ""
                }
               """,

        python_generator = """
                            Blockly.Python['robot_planar_place'] = function(block) {
                            var text_reference_pose = block.getFieldValue('REFERENCE_POSE');
                            var number_speed = block.getFieldValue('SPEED');
                            var checkbox_wait = block.getFieldValue('WAIT') == 'TRUE' ? 'True':'False';
                            var value_z_before = Blockly.Python.valueToCode(block, 'Z_BEFORE', Blockly.Python.ORDER_ATOMIC);
                            var value_z_place = Blockly.Python.valueToCode(block, 'Z_PLACE', Blockly.Python.ORDER_ATOMIC);
                            var value_target_pose = Blockly.Python.valueToCode(block, 'TARGET_POSE', Blockly.Python.ORDER_ATOMIC);
                            // TODO: Assemble JavaScript into code variable.
                            var code = 'robot_planar_place(' + value_target_pose+ ',\"' + text_reference_pose + '\",' + value_z_before + ',' + value_z_place + ',' + number_speed + ',' + checkbox_wait + ')\\n';
                            // TODO: Change ORDER_NONE to the correct strength.
                            return code;
                            };
                           """
    )

    blocks["robot_set_active_robot"] = PyriBlocklyBlock(
        name = "robot_set_active_robot",
        category = "Robotics",
        doc = "Set active robot by device name",
        json = """{
                    "type": "robot_set_active_robot",
                    "message0": "set active robot %1",
                    "args0": [
                        {
                        "type": "field_input",
                        "name": "ROBOT_NAME",
                        "text": "robot"
                        }
                    ],
                    "previousStatement": null,
                    "nextStatement": null,
                    "colour": 340,
                    "tooltip": "Set active robot",
                    "helpUrl": ""
                    }""",
        python_generator = """
                            Blockly.Python['robot_set_active_robot'] = function(block) {
                            var text_robot_name = block.getFieldValue('ROBOT_NAME');
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_set_active_robot(\"' + text_robot_name + '\")\\n';
                            return code;
                            };
                           """        
    )

    blocks["robot_set_origin_calibration"] = PyriBlocklyBlock(
        name = "robot_set_origin_calibration",
        category = "Robotics",
        doc = "Set previously saved robot origin calibration",
        json = """{
                    "type": "robot_set_origin_calibration",
                    "message0": "set robot origin calibration by name %1",
                    "args0": [
                        {
                        "type": "field_input",
                        "name": "ROBOT_CALIBRATION_NAME",
                        "text": "robot_origin_calibration"
                        }
                    ],
                    "previousStatement": null,
                    "nextStatement": null,
                    "colour": 340,
                    "tooltip": "Set previously saved robot origin calibration",
                    "helpUrl": ""
                    }""",
        python_generator = """
                            Blockly.Python['robot_set_origin_calibration'] = function(block) {
                            var text_robot_calibration_name = block.getFieldValue('ROBOT_CALIBRATION_NAME');
                            // TODO: Assemble Python into code variable.
                            var code = 'robot_set_origin_calibration(\"' + text_robot_calibration_name + '\")\\n';
                            return code;
                            };                            
                           """        
    )

    blocks["robot_set_active_tool"] = PyriBlocklyBlock(
        name = "robot_set_active_tool",
        category = "Robotics",
        doc = "Set active tool by device name",
        json = """
            {
            "type": "robot_set_active_tool",
            "message0": "set active tool %1",
            "args0": [
                {
                "type": "field_input",
                "name": "TOOL_NAME",
                "text": "tool"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 60,
            "tooltip": "Set active tool",
            "helpUrl": ""
            }
            """,
            python_generator = """
                                Blockly.Python['robot_set_active_tool'] = function(block) {
                                var text_tool_name = block.getFieldValue('TOOL_NAME');
                                // TODO: Assemble Python into code variable.
                                var code = 'robot_set_active_tool(\"' + text_tool_name + '\")\\n';
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