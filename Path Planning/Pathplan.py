from Astar import AStarPlanner
import matplotlib.pyplot as plt
import pickle
import numpy as np

show_animation = True


def main():
    print(__file__ + " start!!")

    # start and goal position
    f = open('OutputFiles/Coordinates.pkl','rb')
    coord = pickle.load(f)
    f.close()
   
    sx = coord['green'][0][0]  # [m]
    sy = coord['green'][0][1]  # [m] 
    goals = np.array(coord['red'])
    gx = goals[:,0]  # [m]
    gx = gx.tolist()
    gy = goals[:,1]  # [m]
    gy = gy.tolist()
    grid_size = 1.5  # [m]
    robot_radius = 1  # [m]

    # set obstacle positions
    f = open('OutputFiles/Outline.pkl','rb')
    obs = pickle.load(f)
    f.close()

    ox, oy = [], []

    for o in obs:
        olist = np.array(o)
        olist = olist.reshape(len(olist),2)
        for x in olist[:,0]:
            ox.append(x)
        for y in olist[:,1]:
            oy.append(y)
    

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rxs, rys, ds = a_star.planning(sx, sy, gx, gy, True)
    
if __name__ == '__main__':
    main()

        


