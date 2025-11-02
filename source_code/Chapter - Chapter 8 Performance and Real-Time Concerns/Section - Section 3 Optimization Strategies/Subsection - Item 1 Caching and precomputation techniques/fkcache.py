import threading
import pinocchio as pin
import numpy as np

class FKCache:
    def __init__(self, model):
        self.model = model
        self.data = model.createData()
        self.lock = threading.Lock()
        # store latest joint configuration
        self.q_last = np.zeros(model.nq)
        # per-link cache and dirty flags
        self.placements = [pin.SE3.Identity() for _ in range(model.njoints)]
        self.dirty = [True]*model.njoints
        # adjacency: children list for quick invalidation
        self.children = [[] for _ in range(model.njoints)]
        for i in range(1, model.njoints):
            parent = model.parents[i]
            self.children[parent].append(i)

    def mark_dirty_descendants(self, joint):
        stack = [joint]
        while stack:
            j = stack.pop()
            if not self.dirty[j]:
                self.dirty[j] = True
                stack.extend(self.children[j])

    def update(self, q):
        with self.lock:
            # mark changed joints
            changed = np.where(q != self.q_last)[0]
            for j in changed:
                self.mark_dirty_descendants(j)
            # perform minimal FK: traverse joints in topological order
            pin.forwardKinematics(self.model, self.data, q)  # fills data.oMi per joint
            for i in range(self.model.njoints):
                if self.dirty[i]:
                    self.placements[i] = self.data.oMi[i]  # cache placement
                    self.dirty[i] = False
            self.q_last = q.copy()

    def get_frame(self, frame_id):
        with self.lock:
            return self.placements[frame_id]  # return cached SE3