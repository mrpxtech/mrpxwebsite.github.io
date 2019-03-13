import numpy as np
frontiers_scaled = np.array([[1,2],[2,3],[3,4]])
frontier_dist_to_robot_thresh =0.25
distances= np.array([.05,.6,1,5])
mask = (distances>frontier_dist_to_robot_thresh)
frontiers_scaled = frontiers_scaled[mask] 
print(frontiers_scaled)