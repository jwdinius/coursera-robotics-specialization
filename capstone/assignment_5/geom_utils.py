import numpy as np

def tag_pos(markers, marker_id):
    for i in range(len(markers)):
        marker_i = np.copy(markers[i])
        if int(float(marker_i[3])) == marker_id:
            return marker_i[:3]
    return None


def robot_pos(w_pos, r_pos):
    H_W = np.array([[np.cos(w_pos[2]), -np.sin(w_pos[2]), w_pos[0]],
                    [np.sin(w_pos[2]),  np.cos(w_pos[2]), w_pos[1]],
                    [0., 0., 1.]])
    H_R = np.array([[np.cos(r_pos[2]), -np.sin(r_pos[2]), r_pos[0]],
                    [np.sin(r_pos[2]),  np.cos(r_pos[2]), r_pos[1]],
                    [0., 0., 1.]])
    w_r = np.dot(H_W, np.linalg.inv(H_R))
    return np.array([[w_r[0,2]],[w_r[1,2]],[np.arctan2(w_r[1,0],w_r[0,0])]])


