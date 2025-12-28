import numpy as np
import matplotlib.pyplot as plt

path = "/home/janmikolajczyk/uav-coverage-sim/data/coverage/coverage_time_s.npy"
cov = np.load(path)

nonzero = np.argwhere(cov > 0.0)
if nonzero.size:
    margin = 2
    r0, c0 = nonzero.min(axis=0)
    r1, c1 = nonzero.max(axis=0)
    r0 = max(r0 - margin, 0)
    c0 = max(c0 - margin, 0)
    r1 = min(r1 + margin, cov.shape[0] - 1)
    c1 = min(c1 + margin, cov.shape[1] - 1)
    cov = cov[r0 : r1 + 1, c0 : c1 + 1]

plt.figure()
plt.imshow(cov, origin="lower")
plt.colorbar(label="coverage time [s]")
plt.title("Coverage heatmap")
plt.xlabel("grid x")
plt.ylabel("grid y")
plt.tight_layout()
plt.show()
