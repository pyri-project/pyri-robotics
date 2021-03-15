from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}

    blocks["robot_jog_joint_not_relative"] = PyriBlocklyBlock(
        name = "robot_jog_joint_not_relative",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
                "type": "robot_jog_joint_not_relative",
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
        python_generator = """Blockly.Python['robot_jog_joint_not_relative'] = function(block) {
                            var dropdown_joint_selected = block.getFieldValue('JOINT_SELECTED');
                            var value_degree = Blockly.Python.valueToCode(block, 'DEGREE', Blockly.Python.ORDER_ATOMIC);
                            var number_speed = block.getFieldValue('SPEED');
                            
                            var code = 'jog_joint(' + dropdown_joint_selected + ', ' + value_degree + ', ' + number_speed + ')\\n';
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
                            
                            var code = 'jog_joint_relative(' + dropdown_joint_selected + ', ' + value_degree + ', ' + number_speed + ')\\n';
                            return code;
                            };"""
    )

    blocks["robot_jog_joints_not_relative"] = PyriBlocklyBlock(
        name = "robot_jog_joints_not_relative",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
                "type": "robot_jog_joints_not_relative",
                "lastDummyAlign0": "RIGHT",
                "message0": "Jog Joints to %1 1 %2 2 %3 3 %4 4 %5 5 %6 6 %7 7 %8 degrees with  %9 %% speed",
                "args0": [
                    {
                    "type": "input_dummy",
                    "align": "CENTRE"
                    },
                    {
                    "type": "input_value",
                    "name": "DEGREE_1",
                    "check": "Number",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "DEGREE_2",
                    "check": "Number",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "DEGREE_3",
                    "check": "Number",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "DEGREE_4",
                    "check": "Number",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "DEGREE_5",
                    "check": "Number",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "DEGREE_6",
                    "check": "Number",
                    "align": "RIGHT"
                    },
                    {
                    "type": "input_value",
                    "name": "DEGREE_7",
                    "check": "Number",
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
                "colour": 0,
                "tooltip": "",
                "helpUrl": ""
                }""",
        python_generator = """Blockly.Python['robot_jog_joints_not_relative'] = function(block) {
                            var deg_1 = Blockly.Python.valueToCode(block, 'DEGREE_1', Blockly.Python.ORDER_ATOMIC);
                            var deg_2 = Blockly.Python.valueToCode(block, 'DEGREE_2', Blockly.Python.ORDER_ATOMIC);
                            var deg_3 = Blockly.Python.valueToCode(block, 'DEGREE_3', Blockly.Python.ORDER_ATOMIC);
                            var deg_4 = Blockly.Python.valueToCode(block, 'DEGREE_4', Blockly.Python.ORDER_ATOMIC);
                            var deg_5 = Blockly.Python.valueToCode(block, 'DEGREE_5', Blockly.Python.ORDER_ATOMIC);
                            var deg_6 = Blockly.Python.valueToCode(block, 'DEGREE_6', Blockly.Python.ORDER_ATOMIC);
                            var deg_7 = Blockly.Python.valueToCode(block, 'DEGREE_7', Blockly.Python.ORDER_ATOMIC);
                            var number_speed = block.getFieldValue('SPEED');
                        
                            var code = 'jog_joints([' +deg_1+', ' +deg_2+', '+deg_3+', '+deg_4+', '+deg_5+', '+deg_6+', '+deg_7+'], ' + number_speed + ')\\n';
                            return code;
                            };"""
    )

    blocks["robot_jog_joints_relative"] = PyriBlocklyBlock(
        name = "robot_jog_joints_relative",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
            "type": "robot_jog_joints_relative",
            "lastDummyAlign0": "RIGHT",
            "message0": "Jog Joints relatively %1 1 %2 2 %3 3 %4 4 %5 5 %6 6 %7 7 %8 degrees with  %9 %% speed",
            "args0": [
                {
                "type": "input_dummy",
                "align": "CENTRE"
                },
                {
                "type": "input_value",
                "name": "DEGREE_1",
                "check": "Number",
                "align": "RIGHT"
                },
                {
                "type": "input_value",
                "name": "DEGREE_2",
                "check": "Number",
                "align": "RIGHT"
                },
                {
                "type": "input_value",
                "name": "DEGREE_3",
                "check": "Number",
                "align": "RIGHT"
                },
                {
                "type": "input_value",
                "name": "DEGREE_4",
                "check": "Number",
                "align": "RIGHT"
                },
                {
                "type": "input_value",
                "name": "DEGREE_5",
                "check": "Number",
                "align": "RIGHT"
                },
                {
                "type": "input_value",
                "name": "DEGREE_6",
                "check": "Number",
                "align": "RIGHT"
                },
                {
                "type": "input_value",
                "name": "DEGREE_7",
                "check": "Number",
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
            "previousStatement": null,
            "nextStatement": null,
            "colour": 0,
            "tooltip": "",
            "helpUrl": ""
            }""",
        python_generator = """Blockly.Python['robot_jog_joints_relative'] = function(block) {
                            var deg_1 = Blockly.Python.valueToCode(block, 'DEGREE_1', Blockly.Python.ORDER_ATOMIC);
                            var deg_2 = Blockly.Python.valueToCode(block, 'DEGREE_2', Blockly.Python.ORDER_ATOMIC);
                            var deg_3 = Blockly.Python.valueToCode(block, 'DEGREE_3', Blockly.Python.ORDER_ATOMIC);
                            var deg_4 = Blockly.Python.valueToCode(block, 'DEGREE_4', Blockly.Python.ORDER_ATOMIC);
                            var deg_5 = Blockly.Python.valueToCode(block, 'DEGREE_5', Blockly.Python.ORDER_ATOMIC);
                            var deg_6 = Blockly.Python.valueToCode(block, 'DEGREE_6', Blockly.Python.ORDER_ATOMIC);
                            var deg_7 = Blockly.Python.valueToCode(block, 'DEGREE_7', Blockly.Python.ORDER_ATOMIC);
                            var number_speed = block.getFieldValue('SPEED');
                            
                            var code = 'jog_joints_relative([' +deg_1+', ' +deg_2+', '+deg_3+', '+deg_4+', '+deg_5+', '+deg_6+', '+deg_7+'], ' + number_speed + ')\\n';
                            return code;
                            };"""
    )

    blocks["robot_jog_cartesian_not_relative"] = PyriBlocklyBlock(
        name = "robot_jog_cartesian_not_relative",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
                "type": "robot_jog_cartesian_not_relative",
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
        python_generator = """Blockly.Python['robot_jog_cartesian_not_relative'] = function(block) {
                            var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
                            var value_orientation = Blockly.Python.valueToCode(block, 'ORIENTATION', Blockly.Python.ORDER_ATOMIC);
                            var number_speed = block.getFieldValue('SPEED');
                            
                            var code = 'self.jog_cartesian('+ value_position + ',' + value_orientation +','+ number_speed +')\\n';
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
                            
                            var code = 'self.jog_cartesian_relative('+ value_position + ',' + value_orientation +','+ number_speed +')\\n';
                            return code;
                            };
                            """
    )

    blocks["robot_wait"] = PyriBlocklyBlock(
        name = "robot_wait",
        category = "Robotics",
        doc = "Jog joints",
        json = """{
                "type": "robot_wait",
                "message0": "Wait %1 seconds",
                "args0": [
                    {
                    "type": "field_number",
                    "name": "WAIT_TIME",
                    "value": 1,
                    "min": 0
                    }
                ],
                "previousStatement": null,
                "nextStatement": null,
                "colour": 120,
                "tooltip": "",
                "helpUrl": ""
                }""",
        python_generator = """Blockly.Python['robot_wait'] = function(block) {
                            var number_wait_time = block.getFieldValue('WAIT_TIME');
                            
                            var code = 'time_sleep(' + number_wait_time+ ')\\n';
                            return code;
                            };
                            """
        
    )
    blocks["tool_gripper"] = PyriBlocklyBlock(
        name = "tool_gripper",
        category = "Robotics",
        doc = "Robot Gripper Tool",
        json = """{
                "type": "tool_gripper",
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
        python_generator = """Blockly.Python['tool_gripper'] = function(block) {
                            var dropdown_gripper_status = block.getFieldValue('GRIPPER_STATUS');
                            // TODO: Assemble Python into code variable.
                            var code = 'self.tool_gripper(' + dropdown_gripper_status + ')\\n';
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