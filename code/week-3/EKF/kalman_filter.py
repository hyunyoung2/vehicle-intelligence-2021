import numpy as np
from math import sqrt
from math import atan2
from tools import Jacobian

class KalmanFilter:
    def __init__(self, x_in, P_in, F_in, H_in, R_in, Q_in):
        self.x = x_in
        self.P = P_in
        self.F = F_in
        self.H = H_in
        self.R = R_in
        self.Q = Q_in

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # Calculate new estimates
        self.x = self.x + np.dot(K, z - np.dot(self.H, self.x))
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

    

    def update_ekf(self, z):
        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix H_j
        # 2. Calculate S = H_j * P' * H_j^T + R
        # 3. Calculate Kalman gain K = P' * H_j^T * S^-1
        # 4. Estimate y = z - h(x')
        # 5. Normalize phi so that it is between -PI and +PI
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * H_j) * P        

        ## the function to normalize phi into between -PI and +PI
        def norm_phi(angle):
            
            if angle < 0:
                angle *= -1
                angle %= np.pi 
                angle *= -1
            else:
                angle = angle % np.pi
 
            return angle

        # h(x') function 
        def radar_h(x):

            sqrt_x = sqrt(x[0]*x[0]+x[1]*x[1])
            atan_val = atan2(x[1], x[0])
            row = (x[0]*x[2]+x[1]*x[3])/sqrt_x

            h_x = np.array([sqrt_x, atan_val, row])
 
            return h_x

        ## 1. compute Jacobian Matrix H_j      
        H_j = Jacobian(self.x)
                      
        ## 2. Calculate S = H_j*P'*H_j^T
        S = np.dot(np.dot(H_j, self.P), H_j.T) + self.R

        ## 3. Calcualte Kalman gain K = P' * H_j * S^-1
        K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S))

        ## 4. Estimate y = z - h(x')
        y = z - radar_h(self.x)

        ## 5. Normalize phi so that it is between -PI and +PI
        _y_1 = norm_phi(y[1]) 
        y[1] = _y_1
        
        ## 6. Calculate new estimates
        self.x = self.x + np.dot(K, y)

        I = np.eye(4)
        
        self.P = np.dot(I - np.dot(K, H_j), self.P)


if __name__ == "__main__":

    ## the function to normalize phi into between -PI and +PI
    def norm_phi(angle):
            
        angle = angle % (2 * np.pi)  

        if angle > np.pi: 
            angle -= 2 * np.pi
            
        return angle

    print(norm_phi(-1.0))

  
