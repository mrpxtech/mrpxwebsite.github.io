import numpy as np
from utils import wrapToPi

def ctrl_pose(x, y, th, xg, yg, thg):
    '''
    This function implements the pose stabilization controller.
    Inputs:
        x, y, th: the current pose of the robot
        xg, yg, thg: the desired pose of the robot
    Outputs:
        ctrl: a numpy array np.array([V, om]) containing the desired control inputs
    HINT: you need to use the wrapToPi function
    HINT: don't forget to saturate your control inputs
    '''

    ########## Code starts here ##########
    #gain values
    k1=0.5
    k2=0.5
    k3=1.0

    #Parameters Definitions
    #wrap to pi returns a value in range of -pi to pi
    alpha=wrapToPi(np.arctan2((yg-y),(xg-x))-th)
    rho=np.sqrt((xg-x)**2+(yg-y)**2)
    delta=wrapToPi(alpha+th-thg)
    
    #Control Laws
    V=k1*rho*np.cos(alpha)
    om=k2*alpha+k1*(np.sinc(alpha)*np.cos(alpha))*(alpha+k3*delta)

    V=np.sign(V)*min(0.5,np.abs(V))
    om=np.sign(om)*min(1.0,np.abs(om))

    ctrl=np.array([V, om])
    ########## Code ends here ##########

    return np.array(ctrl)