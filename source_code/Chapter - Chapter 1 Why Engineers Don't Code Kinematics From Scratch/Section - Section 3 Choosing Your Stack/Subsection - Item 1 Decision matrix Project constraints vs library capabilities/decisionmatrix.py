import numpy as np
import pandas as pd

# rows = constraints, cols = libraries (values normalized 0..1)
libs = ["MoveIt","Pinocchio","Drake","PyBullet","MuJoCo"]
constraints = ["time_to_market","realtime","accuracy","team_skill","license","sim_parity"]

X = np.array([
    [0.8, 0.5, 0.4, 0.9, 0.85],   # time_to_market
    [0.2, 0.9, 0.85, 0.2, 0.4],   # realtime
    [0.7, 0.95,0.98,0.6, 0.9],    # accuracy
    [0.8, 0.6, 0.5, 0.9, 0.7],    # team_skill
    [0.9, 0.8, 0.6, 0.9, 0.4],    # license
    [0.7, 0.6, 0.7, 0.95,0.98]    # sim_parity
])
X = X.T  # now rows: libs, cols: constraints

# weights reflecting industrial manipulator priorities
w = np.array([0.15,0.30,0.20,0.15,0.10,0.10])

scores = X.dot(w)            # linear score S_j = w^T x_j
df = pd.DataFrame(X, index=libs, columns=constraints)
df["score"] = scores
df = df.sort_values("score", ascending=False)
print(df.round(3))          # top-ranked libraries