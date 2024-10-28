"""
This is the implementation of the OGN node defined in OgnMinValueNode.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy
import math


class OgnMinValueNode:
    """
         Get min value from an numeric array
    """
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""
        try:
            db.outputs.minimum = min(db.inputs.array)
        except:
            db.outputs.minimum = math.inf
        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True
