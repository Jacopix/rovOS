# utils/param_utils.py

import rclpy
from rclpy.node import Node

def load_node_parameters(node: Node, schema: dict) -> dict:
    loaded = {}
    for param_name, param_info in schema.items():
        default_value = param_info.get("default")
        expected_type = param_info.get("type")
        validator = param_info.get("validate")

        node.declare_parameter(param_name, default_value)
        value = node.get_parameter(param_name).value

        # Tipo
        if expected_type:
            try:
                value = expected_type(value)
            except Exception:
                node.get_logger().error(
                    f"[param_utils] Parameter '{param_name}' has wrong type. Expected {expected_type.__name__}"
                )
                rclpy.shutdown()
                return {}

        # Validazione personalizzata
        if validator:
            try:
                validator(value)
            except Exception as e:
                node.get_logger().error(
                    f"[param_utils] Parameter '{param_name}' is invalid: {e}"
                )
                rclpy.shutdown()
                return {}

        loaded[param_name] = value

    return loaded
