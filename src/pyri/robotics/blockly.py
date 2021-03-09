from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}

    blocks["text_print2"] = PyriBlocklyBlock(
        name = "text_print2",
        category = "Example",
        doc = "This is a test print() function",
        json = '{ "type": "text_print2", "message0": "print2 %1", "args0": [ { "type": "input_value", "name": "TEXT" } ],"previousStatement": null, "nextStatement": null,"colour": 230,"tooltip": "","helpUrl": ""}',
        python_generator = """Blockly.Python['text_print2'] = function(block) {
                                // Print statement.
                                var msg = Blockly.Python.valueToCode(block, 'TEXT',
                                Blockly.Python.ORDER_NONE) || '\\'\\'';
                                return 'text_print2(' + msg + ')\\n';
                              };"""
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
        return ["text_print2"]

    def get_block(self,name) -> PyriBlocklyBlock:
        return _get_blocks()[name]

    def get_all_blocks(self) -> Dict[str,PyriBlocklyBlock]:
        return _get_blocks()

def get_blockly_factory():
    return PyriRoboticsBlocklyPluginFactory()