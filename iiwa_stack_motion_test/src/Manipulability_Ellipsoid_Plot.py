
import numpy as np
import modern_robotics as mr

# for plotting the ellipsoid
import plotly as py
import plotly.graph_objs as go
import plotly.figure_factory as ff
from skimage import measure

from numpy import pi

i=0
#thetalist = np.array([[0,1,0,1,0.0,0,0]]).T   # Home position (initial thetalist)


def plot_1D(D,U):
    
    ln_x = [D[0]*U[0][0]/2, 0, -D[0]*U[0][0]/2]
    ln_y = [D[0]*U[1][0]/2, 0, -D[0]*U[1][0]/2]
    ln_z = [D[0]*U[2][0]/2, 0, -D[0]*U[2][0]/2]
            
    trace = go.Scatter3d(x=ln_x, y=ln_y, z=ln_z, mode='lines',marker=dict(size=10),line=dict(width=10))
    py.offline.plot([trace])
    return 


def plot_2D(D,U):
    
    phi = np.linspace(0, 2*np.pi, num=200, endpoint=False)
    p_x = D[0]*np.cos(phi)*U[0][0] + D[1]*np.sin(phi)*U[0][1]
    p_y = D[0]*np.cos(phi)*U[1][0] + D[1]*np.sin(phi)*U[1][1]
    p_z = D[0]*np.cos(phi)*U[2][0] + D[1]*np.sin(phi)*U[2][1]
    
    trace = go.Scatter3d(x=p_x, y=p_y, z=p_z,line=dict(width=10))
    py.offline.plot([trace])
    
    return


def plot_3D(D,U):
    
    X,Y,Z = np.mgrid[-2:2:100j, -2:2:100j, -2:2:100j]
    surf_eq = ((U[0][0]*X + U[1][0]*Y + U[2][0]*Z)**2)/(D[0]**2) + ((U[0][1]*X + U[1][1]*Y + U[2][1]*Z)**2)/(D[1]**2) + ((U[0][2]*X + U[1][2]*Y + U[2][2]*Z)**2)/(D[2]**2) -1 
    #surf_eq = X**2 + Y**2 + Z**2 - 1

    #Triangulate the Given Surface
    vertices, simplices, a, b = measure.marching_cubes_lewiner(surf_eq, 0)
    x,y,z = zip(*vertices)
    
    #Color According to a colormap and Plot
    colormap=['rgb(255,105,180)','rgb(255,255,51)','rgb(0,191,255)']
    fig = ff.create_trisurf(x=x,
                        y=y, 
                        z=z, 
                        plot_edges=False,
                        colormap=colormap,
                        simplices=simplices,
                        title="Manipulability Ellipsoid")
    py.offline.plot(fig)
    
    return



def manipulability(thetalist):

    thetalist = thetalist.T
    ep = 0.0001

    # Joint screw-axes in body frame  (when robot is at home pos.)
    B1 = np.array([[0, 0, 1, 0,     0, 0]]).T
    B2 = np.array([[0, 1, 0, 0.901, 0, 0]]).T
    B3 = np.array([[0, 0, 1, 0,     0, 0]]).T
    B4 = np.array([[0,-1, 0,-0.481, 0, 0]]).T
    B5 = np.array([[0, 0, 1, 0,     0, 0]]).T
    B6 = np.array([[0, 1, 0, 0.081, 0, 0]]).T
    B7 = np.array([[0, 0, 1, 0,     0, 0]]).T
    
    Blist = np.column_stack((B1,B2,B3,B4,B5,B6,B7))
    #Blist = np.array([[0, 0,     0,  0,     0, 0,     0],
    #                  [0, 1,     0, -1,     0, 1,     0],
    #                  [1, 0,     1,  0,     1, 0,     1],
    #                  [0, 0.901, 0, -0.481, 0, 0.081, 0],
    #                  [0, 0,     0,  0,     0, 0,     0],
    #                  [0, 0,     0,  0,     0, 0,     0]])
    
    # Joint screw-axes in space frame  (when robot is at home pos.)
    S1 = np.array([[0, 0, 1, 0,     0, 0]]).T
    S2 = np.array([[0, 1, 0, -0.36, 0, 0]]).T
    S3 = np.array([[0, 0, 1, 0,     0, 0]]).T
    S4 = np.array([[0,-1, 0, 0.78,  0, 0]]).T
    S5 = np.array([[0, 0, 1, 0,     0, 0]]).T
    S6 = np.array([[0, 1, 0,-1.18,  0, 0]]).T
    S7 = np.array([[0, 0, 1, 0,     0, 0]]).T
    
    Slist = np.column_stack((S1,S2,S3,S4,S5,S6,S7))
    
    #Slist = np.array([[0,  0,    0, 0,    0,  0,    0],
    #                  [0,  1,    0,-1,    0,  1,    0],
    #                  [1,  0,    1, 0,    1,  0,    1],
    #                  [0, -0.36, 0, 0.78, 0, -1.18, 0],
    #                  [0,  0,    0, 0,    0,  0,    0],
    #                  [0,  0,    0, 0,    0,  0,    0]])
    
    M = np.array([[1, 0, 0, 0   ],         # End-effector pos. in home config.
                  [0, 1, 0, 0   ],
                  [0, 0, 1, 1.26],
                  [0, 0, 0, 1   ]])
    
    Jb = mr.JacobianBody(Blist, thetalist)
    Jb_w = Jb[[0,1,2],:]
    Jb_v = Jb[[3,4,5],:]

    Js = mr.JacobianSpace(Slist, thetalist)
    Js_w = Js[[0,1,2],:]
    Js_v = Js[[3,4,5],:]

    mu_b_v = np.sqrt(np.linalg.det(np.matmul(Jb_v, Jb_v.T)))
    mu_s_v = np.sqrt(np.linalg.det(np.matmul(Js_v, Js_v.T)))
    print('Manipulability capacity =', mu_s_v)

    U, D, Vt = np.linalg.svd(Js_v, full_matrices=False)  
    
    if D[0]*D[1]*D[2] == 0:
        if D[0] < ep:
            print('Warning: large axis == 0!')
        else:
            if D[1] < ep: 
               # means robot can move only in one direction
               # so plot a line
               print('Plot a line')
               plot_1D(D,U)
               print("D0 = ", D[0])

                
            else:
                # means D[2]= 0, means 2D ellipsoid
                print('Plot a 2D ellipse')
                plot_2D(D,U)
                print("D0 = ", D[0])
                print("D1 = ", D[1])

            
    else: # means 3D ellipsoid
        me_axes_length = D
        plot_3D(D,U)
        print("D0 = ", D[0])
        print("D1 = ", D[1])
        print("D2 = ", D[2])
    
    return


    

#thetalist = np.array([[0,-pi/4,0,-pi/2,0,pi/3,0]])
thetalist = np.array([[1.6692, -0.7624, 0.0534, -1.5705, 0, 1.0470, 1.4280]])
manipulability(thetalist)
    

    





    
    
