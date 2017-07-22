#!/usr/bin/python

import numpy as np
from math import floor,pi
import yaml
def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    # get number of rows and columns of occupancy map
    N,M = occupancy_map.shape

    # convert to row-major indexing (like Matlab/Fortran)
    oned_occ_map = occupancy_map.flatten('F')
    # convert metric to index
    start_i = start[1] / y_spacing - 0.5
    start_j = start[0] / x_spacing - 0.5
    goal_i =  goal[1]  / y_spacing - 0.5
    goal_j =  goal[0] / x_spacing - 0.5

    # use heading to identify which cell to start (end) at, in case of
    # ambiguity
    start_hdg = start[-1]
    goal_hdg = goal[-1]
    
    # identify starting grid point, there may be some ambiguity if the
    # starting point is at the corner of two (or four) cells
    if start_i - floor(start_i) > .5:
      start_i = floor(start_i) + 1.
    elif start_i - floor(start_i) < .5:
      start_i = floor(start_i)
    else:
      if start_hdg >= 0.:
        start_i = floor(start_i) + 1.
      else:
        start_i = floor(start_i)
    if start_j - floor(start_j) > .5:
      start_j = floor(start_j) + 1.
    elif start_j - floor(start_j) < .5:
      start_j = floor(start_j)
    else:
      if start_hdg > -pi/2. and start_hdg < pi/2:
        start_j = floor(start_j) + 1.
      else:
        start_j = floor(start_j)

    if goal_i - floor(goal_i) > .5:
      goal_i = floor(goal_i) + 1.
    elif goal_i - floor(goal_i) < .5:
      goal_i = floor(goal_i)
    else:
      if goal_hdg >= 0.:
        goal_i = floor(goal_i) + 1.
      else:
        goal_i = floor(goal_i)

    if goal_j - floor(goal_j) > .5:
      goal_j = floor(goal_j) + 1.
    elif goal_j - floor(goal_j) < .5:
      goal_j = floor(goal_j)
    else:
      if goal_hdg > -pi/2. and goal_hdg < pi/2:
        goal_j = floor(goal_j) + 1.
      else:
        goal_j = floor(goal_j)

    # convert 2d to 1d indexing
    oned_start = oned_ind(start_i,start_j,N)
    oned_goal = oned_ind(goal_i,goal_j,N)
    
    # setup distance and parent arrays
    dist_from_start = np.inf*np.ones((N,M),dtype=np.float)
    parent = np.zeros((N,M),dtype=np.int)
    oned_parent = parent.flatten('F')
    dist_from_start[int(start_i),int(start_j)] = 0.
    oned_dist = dist_from_start.flatten('F')
    
    considered = []

    #actual heart of dijkstra algorithm
    while True:
      min_dist = np.amin(oned_dist)
      current = np.argmin(oned_dist)
      # break out of loop once goal is found or if no path can be founds
      if current == oned_goal or np.isinf(min_dist):
        break
      # add current point to list of those considered
      considered.append(current)
      oned_dist[current] = np.inf
      
      #get 2d index of current considered point
      i,j = twod_ind(current,N)
      
      # set up neighbors and distance traveled arrays
      nbrs = []
      alt = []
      if j-1>=0:
        nbrs.append(oned_ind(i,j-1,N))
        alt.append(min_dist + x_spacing)
      if j+1<M:
        nbrs.append(oned_ind(i,j+1,N))
        alt.append(min_dist + x_spacing)
      if i-1>=0:
        nbrs.append(oned_ind(i-1,j,N))
        alt.append(min_dist + y_spacing)
      if i+1<N:
        nbrs.append(oned_ind(i+1,j,N))
        alt.append(min_dist + y_spacing)
      
      # loop over neighbors to find shortest path
      for ii in range(len(nbrs)):
        if oned_occ_map[nbrs[ii]] != 1:
          if alt[ii] < oned_dist[nbrs[ii]] and not (np.any(np.in1d(considered,nbrs[ii]))):
            oned_dist[nbrs[ii]] = alt[ii]
            oned_parent[nbrs[ii]] = current
            considered.append(current)
    
    # construct 1d route
    oned_route = []
    if not np.isinf(oned_dist[oned_goal]):
      oned_route.append(oned_goal)
      while oned_parent[oned_route[0]] != 0:
        oned_route.insert(0,oned_parent[oned_route[0]])
    else:
      return []

    #convert to 2d indices
    route_metric = np.empty((0,2))
    for ii in range(len(oned_route)):
      i,j = twod_ind(oned_route[ii],N)
      x = (j+.5)*x_spacing
      y = (i+.5)*y_spacing
      route_metric = np.append(route_metric, [[x,y]],0)
    
    # add start and goal in metric
    route_metric = np.insert(route_metric, 0, np.array(np.transpose(start[:2])) , 0)
    route_metric = np.append(route_metric, np.array(np.transpose(goal[:2])) , 0)

    #delete duplicates
    if distance(route_metric[0],route_metric[1]) < 1.e-3:
      route_metric = np.delete(route_metric,0,0)
    if distance(route_metric[-1],route_metric[-2]) < 1.e-3:
      rows,cols = route_metric.shape
      route_metric = np.delete(route_metric,rows-1,0)

    #delete cells that start or goal point lies on the edge of
    dx = abs(route_metric[0][0] - route_metric[2][0])
    dy = abs(route_metric[0][1] - route_metric[2][1])
    if dx <= x_spacing and dy <= y_spacing:
      route_metric = np.delete(route_metric,1,0)
    dx = route_metric[-1][0] - route_metric[-3][0]
    dy = route_metric[-1][1] - route_metric[-3][1]
    if dx <= x_spacing and dy <= y_spacing:
      rows,cols = route_metric.shape
      route_metric = np.delete(route_metric,rows-2,0)
    
    return route_metric
    
def oned_ind(i,j,nrows):
  return int(i) + nrows*int(j)

def twod_ind(ind,nrows):
  i = np.remainder(ind,nrows)
  j = int(floor(ind / nrows))
  return i,j

def distance(x,y):
  s =  np.sqrt((x[0]-y[0])**2 + (x[1]-y[1])**2)
  return s

def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")

    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



def main():
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    main()

