import numpy as np

occ_map = np.array([[1,1,1,1,1,1,1],
	                [1,1,1,0,1,0,1],
	                [1,1,1,0,0,0,1],
	                [1,1,1,0,0,1,1],
	                [1,1,1,0,0,0,1],
	                [1,1,1,0,1,0,1],
	                [1,1,1,0,0,0,1],
	                [1,1,1,1,0,0,1],
	                [1,1,1,0,0,0,1],
	                [1,0,0,0,1,0,1],
	                [1,1,1,1,1,1,1]]).astype(np.float32)

# convert to coordinates
occ_xcoords = np.zeros(occ_map.shape)
occ_ycoords = np.zeros(occ_map.shape)
for j in range(occ_xcoords.shape[0]):
	for i in range(occ_xcoords.shape[1]):
		occ_xcoords[j,i] = 0.204 * (np.float32(i) + 0.5)
		occ_ycoords[j,i] = 0.204 * (np.float32(j) + 0.5)

print(occ_xcoords)
print(occ_ycoords)
