import numpy as np

class PoseEstimation:
    def __init__(self):
        # TODO
        self.d = {
            "1" : np.array([6,5,2])
        }

    def realPos(self, id, measure):
        return self.d[str(id)] + measure



p = PoseEstimation()
print p.realPos(1,([0,0,2]))