import numpy as np
from numpy import sin, cos
import scipy.linalg    # you may find scipy.linalg.block_diag useful
from ExtractLines import ExtractLines, normalize_line_parameters, angle_difference
from maze_sim_parameters import LineExtractionParams, NoiseParams, MapParams

class EKF(object):

    def __init__(self, x0, P0, Q):
        self.x = x0    # Gaussian belief mean
        self.P = P0    # Gaussian belief covariance
        self.Q = Q     # Gaussian control noise covariance (corresponding to dt = 1 second)

    # Updates belief state given a discrete control step (Gaussianity preserved by linearizing dynamics)
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def transition_update(self, u, dt):
        g, Gx, Gu = self.transition_model(u, dt)

        #### TODO ####
        # update self.x, self.P
        ##############
        Pt=self.P
        Q=self.Q
        self.x=g
        self.P=np.dot(Gx,Pt.dot(Gx.T))+dt*np.dot(Gu,Q.dot(Gu.T))

    # Propagates exact (nonlinear) state dynamics; also returns associated Jacobians for EKF linearization
    # INPUT:  (u, dt)
    #       u - zero-order hold control input
    #      dt - length of discrete time step
    # OUTPUT: (g, Gx, Gu)
    #      g  - result of belief mean self.x propagated according to the system dynamics with control u for dt seconds
    #      Gx - Jacobian of g with respect to the belief mean self.x
    #      Gu - Jacobian of g with respect to the control u
    def transition_model(self, u, dt):
        raise NotImplementedError("transition_model must be overriden by a subclass of EKF")

    # Updates belief state according to a given measurement (with associated uncertainty)
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: none (internal belief state (self.x, self.P) should be updated)
    def measurement_update(self, rawZ, rawR):
        z, R, H = self.measurement_model(rawZ, rawR)
        if z is None:    # don't update if measurement is invalid (e.g., no line matches for line-based EKF localization)
            return

        #### TODO ####
        # update self.x, self.P
        ##############
        P=self.P
        Sig=np.dot(H.dot(P),H.T)+R
        K=np.dot(P.dot(H.T),np.linalg.inv(Sig))
        self.x=self.x+K.dot(z)
        self.P=self.P-np.dot(K.dot(Sig),K.T)

    # Converts raw measurement into the relevant Gaussian form (e.g., a dimensionality reduction);
    # also returns associated Jacobian for EKF linearization
    # INPUT:  (rawZ, rawR)
    #    rawZ - raw measurement mean
    #    rawR - raw measurement uncertainty
    # OUTPUT: (z, R, H)
    #       z - measurement mean (for simple measurement models this may = rawZ)
    #       R - measurement covariance (for simple measurement models this may = rawR)
    #       H - Jacobian of z with respect to the belief mean self.x
    def measurement_model(self, rawZ, rawR):
        raise NotImplementedError("measurement_model must be overriden by a subclass of EKF")


class Localization_EKF(EKF):

    def __init__(self, x0, P0, Q, map_lines, tf_base_to_camera, g):
        self.map_lines = map_lines                    # 2xJ matrix containing (alpha, r) for each of J map lines
        self.tf_base_to_camera = tf_base_to_camera    # (x, y, theta) transform from the robot base to the camera frame
        self.g = g                                    # validation gate
        super(self.__class__, self).__init__(x0, P0, Q)

    # Unicycle dynamics (Turtlebot 2)
    def transition_model(self, u, dt):
        v, om = u
        x, y, th = self.x

        if abs(om)>0.0001:
            g=np.array([x+(v/om)*(np.sin(th+om*dt)-np.sin(th)),
            y+(v/om)*(-np.cos(th+om*dt)+cos(th)),
            th+om*dt])

            Gx=np.array([[1.0,0.0,(v/om)*(np.cos(th+om*dt)-np.cos(th))],
                        [0.0,1.0,(v/om)*(np.sin(th+om*dt)-np.sin(th))],
                        [0.0,0.0,1.0]])

            Gux_om=(v/om)*(np.sin(th)*-dt*np.sin(om*dt))+(-v/om**2)*(np.sin(th)*np.cos(om*dt)) + \
                 (v/om)*(np.cos(th)*dt*np.cos(om*dt)) +(-v/om**2)*np.cos(th)*np.sin(om*dt)+(v/om**2)*np.sin(th)

            Guy_om=(-v/om)*(np.cos(th)*-dt*np.sin(om*dt))+(v/om**2)*(np.cos(th)*np.cos(om*dt)) + \
                (v/om)*(np.sin(th)*dt*np.cos(om*dt))+(-v/om**2)*np.sin(th)*np.sin(om*dt)-(v/om**2)*np.cos(th)

            Gu=np.array([[1/om*(np.sin(th+om*dt)-np.sin(th)),Gux_om],
                        [1/om*(-np.cos(th+om*dt)+np.cos(th)),Guy_om],
                        [0,dt]])
            # print('large')

        #For very small values of omega
        else:
            g=np.array([x+v*np.cos(th)*dt,y+v*np.sin(th)*dt,th+om*dt])

            Gx=np.array([[1.0,0.0,-v*dt*np.sin(th)],
                        [0.0,1.0,v*dt*np.cos(th)],
                        [0.0,0.0,1.0]])

            Gu=np.array([[dt*np.cos(th),-v/2*(dt**2)*np.sin(th)],
                        [dt*np.sin(th),v/2*(dt**2)*np.cos(th)],
                        [0,dt]])
            # print('small')
            
        return g, Gx, Gu

    # Given a single map line m in the world frame, outputs the line parameters in the scanner frame so it can
    # be associated with the lines extracted from the scanner measurements
    # INPUT:  m = (alpha, r)
    #       m - line parameters in the world frame
    # OUTPUT: (h, Hx)
    #       h - line parameters in the scanner (camera) frame
    #      Hx - Jacobian of h with respect to the the belief mean self.x
    def map_line_to_predicted_measurement(self, m):
        alpha, r = m

        #### TODO ####
        # compute h, Hx
        ##############
        x, y, th=self.x
        xc,yc,thc=self.tf_base_to_camera
        Rz=np.array([[np.cos(th),-np.sin(th),x],
                     [np.sin(th),np.cos(th),y],
                     [0.0,0.0,1.0]])
        xw,yw,n=Rz.dot(np.array([xc,yc,1.0]))

        h=np.array([alpha-thc-th,r-xw*np.cos(alpha)-yw*np.sin(alpha)])

        Hx=np.array([[0.0,0.0,-1.0],
                    [-np.cos(alpha),-np.sin(alpha),np.sin(th)*xc*np.cos(alpha)+np.cos(th)*np.cos(alpha)*yc-np.cos(th)*xc*np.sin(alpha)+np.sin(th)*yc*sin(alpha)]])
       
        flipped, h = normalize_line_parameters(h)
        if flipped:
            Hx[1,:] = -Hx[1,:]

        return h, Hx

    # Given lines extracted from the scanner data, tries to associate to each one the closest map entry
    # measured by Mahalanobis distance
    # INPUT:  (rawZ, rawR)
    #    rawZ - 2xI matrix containing (alpha, r) for each of I lines extracted from the scanner data (in scanner frame)
    #    rawR - list of I 2x2 covariance matrices corresponding to each (alpha, r) column of rawZ
    # OUTPUT: (v_list, R_list, H_list)
    #  v_list - list of at most I innovation vectors (predicted map measurement - scanner measurement)
    #  R_list - list of len(v_list) covariance matrices of the innovation vectors (from scanner uncertainty)
    #  H_list - list of len(v_list) Jacobians of the innovation vectors with respect to the belief mean self.x
    def associate_measurements(self, rawZ, rawR):

        #### TODO ####
        # compute v_list, R_list, H_list
        ##############
        v_list=[]
        R_list=[]
        H_list=[]

        g=self.g
        val=g**2
        P=self.P
        m=self.map_lines

        for i in range(rawZ.shape[1]):
            validation=g**2
            for j in range(self.map_lines.shape[1]):
                h,H=self.map_line_to_predicted_measurement(m[:,j])
                v=rawZ[:, i]-h
                S=np.dot(H,np.dot(P,H.T))+rawR[i]
                d=np.dot(v.T,np.dot(np.linalg.inv(S),v))
                if abs(d)<validation:
                    validation=d
                    vg=v
                    Rg=rawR[i]
                    Hg=H
            if abs(validation) < val:
                v_list.append(vg)
                R_list.append(Rg)
                H_list.append(Hg)

        return v_list, R_list, H_list

    # Assemble one joint measurement, covariance, and Jacobian from the individual values corresponding to each
    # matched line feature
    def measurement_model(self, rawZ, rawR):
        v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
        if not v_list:
            print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
            return None, None, None

        #### TODO ####
        # compute z, R, H
        ##############
        z=np.concatenate(v_list)
        H=np.concatenate(H_list)

        # Refer to: https://stackoverflow.com/questions/28095803/using-linalg-block-diag-for-variable-number-of-blocks
        R=scipy.linalg.block_diag(*R_list)

        return z, R, H


class SLAM_EKF(EKF):

    def __init__(self, x0, P0, Q, tf_base_to_camera, g):
        self.tf_base_to_camera = tf_base_to_camera    # (x, y, theta) transform from the robot base to the camera frame
        self.g = g                                    # validation gate
        super(self.__class__, self).__init__(x0, P0, Q)

    # Combined Turtlebot + map dynamics
    # Adapt this method from Localization_EKF.transition_model.
    def transition_model(self, u, dt):
        v, om = u
        x, y, th = self.x[:3]

        #### TODO ####
        # compute g, Gx, Gu (some shape hints below)
        # g = np.copy(self.x)
        # Gx = np.eye(self.x.size)
        # Gu = np.zeros((self.x.size, 2))
        ##############
        g = np.copy(self.x)
        Gx = np.eye(self.x.size)
        Gu = np.zeros((self.x.size, 2))

        if abs(om)>0.0001:
            g[0:3]=np.array([x+(v/om)*(np.sin(th+om*dt)-np.sin(th)),
            y+(v/om)*(-np.cos(th+om*dt)+cos(th)),
            th+om*dt])

            Gx[0:3,0:3]=np.array([[1.0,0.0,(v/om)*(np.cos(th+om*dt)-np.cos(th))],
                        [0.0,1.0,(v/om)*(np.sin(th+om*dt)-np.sin(th))],
                        [0.0,0.0,1.0]])

            Gux_om=(v/om)*(np.sin(th)*-dt*np.sin(om*dt))+(-v/om**2)*(np.sin(th)*np.cos(om*dt)) + \
                 (v/om)*(np.cos(th)*dt*np.cos(om*dt)) +(-v/om**2)*np.cos(th)*np.sin(om*dt)+(v/om**2)*np.sin(th)

            Guy_om=(-v/om)*(np.cos(th)*-dt*np.sin(om*dt))+(v/om**2)*(np.cos(th)*np.cos(om*dt)) + \
                (v/om)*(np.sin(th)*dt*np.cos(om*dt))+(-v/om**2)*np.sin(th)*np.sin(om*dt)-(v/om**2)*np.cos(th)

            Gu[0:3,0:2]=np.array([[1/om*(np.sin(th+om*dt)-np.sin(th)),Gux_om],
                        [1/om*(-np.cos(th+om*dt)+np.cos(th)),Guy_om],
                        [0,dt]])
            # print('large')

        #For very small values of omega
        else:
            g[0:3]=np.array([x+v*np.cos(th)*dt,y+v*np.sin(th)*dt,th+om*dt])

            Gx[0:3,0:3]=np.array([[1.0,0.0,-v*dt*np.sin(th)],
                        [0.0,1.0,v*dt*np.cos(th)],
                        [0.0,0.0,1.0]])

            Gu[0:3,0:2]=np.array([[dt*np.cos(th),-v/2*(dt**2)*np.sin(th)],
                        [dt*np.sin(th),v/2*(dt**2)*np.cos(th)],
                        [0,dt]])
            # print('small')

        return g, Gx, Gu

    # Combined Turtlebot + map measurement model
    # Adapt this method from Localization_EKF.measurement_model.
    #
    # The ingredients for this model should look very similar to those for Localization_EKF.
    # In particular, essentially the only thing that needs to change is the computation
    # of Hx in map_line_to_predicted_measurement and how that method is called in
    # associate_measurements (i.e., instead of getting world-frame line parameters from
    # self.map_lines, you must extract them from the state self.x)
    def measurement_model(self, rawZ, rawR):
        v_list, R_list, H_list = self.associate_measurements(rawZ, rawR)
        if not v_list:
            print "Scanner sees", rawZ.shape[1], "line(s) but can't associate them with any map entries"
            return None, None, None

        #### TODO ####
        # compute z, R, H (should be identical to Localization_EKF.measurement_model above)
        ##############
        z=np.concatenate(v_list)
        H=np.concatenate(H_list)

        # Refer to: https://stackoverflow.com/questions/28095803/using-linalg-block-diag-for-variable-number-of-blocks
        R=scipy.linalg.block_diag(*R_list)

        return z, R, H

    # Adapt this method from Localization_EKF.map_line_to_predicted_measurement.
    #
    # Note that instead of the actual parameters m = (alpha, r) we pass in the map line index j
    # so that we know which components of the Jacobian to fill in.
    def map_line_to_predicted_measurement(self, j):
        alpha, r = self.x[(3+2*j):(3+2*j+2)]    # j is zero-indexed! (yeah yeah I know this doesn't match the pset writeup)

        #### TODO ####
        # compute h, Hx (you may find the skeleton for computing Hx below useful)
        x, y, th=self.x[:3]
        xc,yc,thc=self.tf_base_to_camera
        Rz=np.array([[np.cos(th),-np.sin(th),x],
                     [np.sin(th),np.cos(th),y],
                     [0.0,0.0,1.0]])
        xw,yw,n=Rz.dot(np.array([xc,yc,1.0]))

        h=np.array([alpha-thc-th,r-xw*np.cos(alpha)-yw*np.sin(alpha)])

        Hx = np.zeros((2,self.x.size))
        Hx[:,:3] = np.array([[0.0,0.0,-1.0],
                    [-np.cos(alpha),-np.sin(alpha),np.sin(th)*xc*np.cos(alpha)+np.cos(th)*np.cos(alpha)*yc-np.cos(th)*xc*np.sin(alpha)+np.sin(th)*yc*sin(alpha)]])
       
        # First two map lines are assumed fixed so we don't want to propagate any measurement correction to them
        if j > 1:
            Hx[0, 3+2*j] = 1.0
            Hx[1, 3+2*j] = xw*np.sin(alpha)-yw*np.cos(alpha)
            Hx[0, 3+2*j+1] = 0.0
            Hx[1, 3+2*j+1] = 1.0

        ##############

        flipped, h = normalize_line_parameters(h)
        if flipped:
            Hx[1,:] = -Hx[1,:]

        return h, Hx

    # Adapt this method from Localization_EKF.associate_measurements.
    def associate_measurements(self, rawZ, rawR):

        #### TODO ####
        # compute v_list, R_list, H_list
        ##############

        v_list=[]
        R_list=[]
        H_list=[]

        counter=(len(self.x)-3)/2

        g=self.g
        val=g**2
        P=self.P

        for i in range(rawZ.shape[1]):
            validation=g**2
            for j in range(counter):
                h,H=self.map_line_to_predicted_measurement(j)
                v=rawZ[:, i]-h
                S=np.dot(H,np.dot(P,H.T))+rawR[i]
                d=np.dot(v.T,np.dot(np.linalg.inv(S),v))
                if abs(d)<validation:
                    validation=d
                    vg=v
                    Rg=rawR[i]
                    Hg=H
            if abs(validation) < val:
                v_list.append(vg)
                R_list.append(Rg)
                H_list.append(Hg)


        return v_list, R_list, H_list
