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

    blocks["robot_jog_joint_relative"] = PyriBlocklyBlock(
        name = "robot_jog_joint_relative",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
                "type": "robot_jog_joint_relative",
                "message0": "Jog Joint %1 relatively by degree %2 %3 with %4 %% speed",
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
        python_generator = """Blockly.Python['robot_jog_joint_relative'] = function(block) {
                            var dropdown_joint_selected = block.getFieldValue('JOINT_SELECTED');
                            var value_degree = Blockly.Python.valueToCode(block, 'DEGREE', Blockly.Python.ORDER_ATOMIC);
                            var number_speed = block.getFieldValue('SPEED');
                            
                            var code = 'robot_jog_joint_relative(' + dropdown_joint_selected + ', ' + value_degree + ', ' + number_speed + ')\\n';
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

    blocks["robot_jog_joints_relative"] = PyriBlocklyBlock(
        name = "robot_jog_joints_relative",
        category = "Robotics",
        doc = "Jog joints relatively",
        json = """{
            "type": "robot_jog_joints_relative",
            "message0": "Jog Joints relatively with speed %1 degrees %2",
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
                "name": "JOINT_DIFF_VECTOR"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 0,
            "tooltip": "",
            "helpUrl": ""
            }""",
        python_generator = """Blockly.Python['robot_jog_joints_relative'] = function(block) {
                            
                            var number_speed = block.getFieldValue('SPEED');
                            var value_joint_vector = Blockly.Python.valueToCode(block, 'JOINT_DIFF_VECTOR', Blockly.Python.ORDER_ATOMIC);
                            var code = 'robot_jog_joints_relative(' + value_joint_vector + ', ' + number_speed + ')\\n';
                            return code;
                            };"""
    )

    blocks["robot_jog_cartesian_absolute"] = PyriBlocklyBlock(
        name = "robot_jog_cartesian_absolute",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
                "type": "robot_jog_cartesian_absolute",
                "lastDummyAlign0": "RIGHT",
                "message0": "Jog Cartesian to: %1 Position %2 Orientation %3 with  %4 %% speed",
                "args0": [
                    {
                    "type": "input_dummy",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "POSITION",
                    "check": "position",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "ORIENTATION",
                    "check": "orientation",
                    "align": "RIGHT"
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
                "colour": 340,
                "tooltip": "",
                "helpUrl": ""
                }""",
        python_generator = """Blockly.Python['robot_jog_cartesian_absolute'] = function(block) {
                            var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
                            var value_orientation = Blockly.Python.valueToCode(block, 'ORIENTATION', Blockly.Python.ORDER_ATOMIC);
                            var number_speed = block.getFieldValue('SPEED');
                            
                            var code = 'robot_jog_cartesian('+ value_position + ',' + value_orientation +','+ number_speed +')\\n';
                            return code;
                            };
                            """
    )

    blocks["robot_jog_cartesian_relative"] = PyriBlocklyBlock(
        name = "robot_jog_cartesian_relative",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
                "type": "robot_jog_cartesian_relative",
                "lastDummyAlign0": "RIGHT",
                "message0": "Jog Cartesian Relatively: %1 Position %2 Orientation %3 with  %4 %% speed",
                "args0": [
                    {
                    "type": "input_dummy",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "POSITION",
                    "check": "position",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "ORIENTATION",
                    "check": "orientation",
                    "align": "RIGHT"
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
                "colour": 340,
                "tooltip": "",
                "helpUrl": ""
                }""",
        python_generator = """Blockly.Python['robot_jog_cartesian_relative'] = function(block) {
                            var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
                            var value_orientation = Blockly.Python.valueToCode(block, 'ORIENTATION', Blockly.Python.ORDER_ATOMIC);
                            var number_speed = block.getFieldValue('SPEED');
                            
                            var code = 'robot_jog_cartesian_relative('+ value_position + ',' + value_orientation +','+ number_speed +')\\n';
                            return code;
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