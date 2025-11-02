import pybullet as p
import numpy as np

class Node:
    def __init__(self, name, joint_index, parent=None, joint_to_child=None):
        self.name = name
        self.joint_index = joint_index      # index in physics engine (-1 if fixed root)
        self.parent = parent                # parent Node or None
        self.children = []                  # list of child Nodes
        self.joint_to_child = joint_to_child# call(q)->SE3 local transform (function)
        self.T_world = np.eye(4)            # cached world transform
        self.dirty = True                   # flag for recompute

    def add_child(self, child):
        self.children.append(child)
        child.parent = self

def mark_dirty_from_changed_joints(root, changed_joint_indices):
    # propagate dirty flags downward from any node whose joint changed
    stack = [root]
    while stack:
        node = stack.pop()
        if node.joint_index in changed_joint_indices:
            # mark this node's subtree dirty
            qstack = [node]
            while qstack:
                n = qstack.pop()
                n.dirty = True
                qstack.extend(n.children)
        else:
            stack.extend(node.children)

def recompute_fk(root):
    # iterative DFS; compute T_world only for dirty nodes
    stack = [root]
    while stack:
        node = stack.pop()
        if node.parent is None:
            if node.dirty:
                # root pose usually comes from odometry; keep current T_world
                node.dirty = False
        else:
            if node.dirty:
                # read joint position from simulator (or control interface)
                q = p.getJointState(bodyUniqueId, node.joint_index)[0]
                X = node.joint_to_child(q)         # local transform from parent
                node.T_world = node.parent.T_world @ X
                node.dirty = False
        # push children (order ensures parent computed before child)
        stack.extend(node.children)

# Example usage (setup omitted): build nodes, wiring, and joint_to_child using real kinematic params.
# In control loop:
# 1) read changed joints since last loop (sensor/encoder diff)
# 2) mark_dirty_from_changed_joints(tree_root, changed_indices)
# 3) recompute_fk(tree_root)