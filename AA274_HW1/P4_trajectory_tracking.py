import numpy as np
from numpy import linalg
from scipy.integrate import cumtrapz
from P3_pose_stabilization import ctrl_pose

def ctrl_traj(x, y, th,
              ctrl_prev,
              x_d, y_d,
              xd_d, yd_d,
              xdd_d, ydd_d,
              x_g, y_g, th_g):
    '''
    This function computes the closed-loop control law.
    Inputs:
        (x,y,th): current state
        ctrl_prev: previous control input (V,om)
        (x_d, y_d): desired position
        (xd_d, yd_d): desired velocity
        (xdd_d, ydd_d): desired acceleration
        (x_g,y_g,th_g): desired final state
    Outputs:
        (V, om): a numpy array np.array([V, om]) containing the desired control inputs
    '''

    # Timestep
    dt = 0.005

    #Previous steps
    Vprev=ctrl_prev[0]
    omprev=ctrl_prev[1]

    ########## Code starts here ##########
    #compute xd and y
    xd=Vprev*np.cos(th)
    yd=Vprev*np.sin(th)
    #gains
    k_px=1.0
    k_dx=0.8
    k_py=0.4
    k_dy=0.8

    #virtual control law for trajectory tracking
    u1=xdd_d+k_px*(x_d-x)+k_dx*(xd_d-xd)
    u2=ydd_d+k_py*(y_d-y)+k_dy*(yd_d-yd)

    #define time
    tf=15 #can tune accuracy, but increase computational time
    N = int(tf/dt)
    t = dt*np.array(range(N+1)) 
    

    #Vdot
    Vd=np.cos(th)*u1+np.sin(th)*u2
    #switch case when robot gets 0.25 m to target
    epsilon=0.1
    if np.abs(x-x_g) > epsilon and np.abs(y-y_g) > epsilon:
        #Numerical integration
        V= Vprev+Vd*dt
        #if Velocity approaches 0
        if V <= 0.00:
            V=np.sqrt(xd**2+yd**2)
    #omega 
        om=-(np.sin(th)*u1)/V+(u2*np.cos(th))/V
    else:
        [V, om]=ctrl_pose(x, y, th, x_g, y_g, th_g)

    #bounding Controls 
    V=np.sign(V)*min(0.5,np.abs(V))
    om=np.sign(om)*min(1.0,np.abs(om))

    ########## Code ends here ##########

    return np.array([V, om])