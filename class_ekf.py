
def state_vector_to_scalars(state_vector):
    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0])

def cartesian_to_polar(state_vector):
    px,py,vx,vy = state_vector_to_scalars(state_vector)
    ro= sqrt(px**2 + py**2)
    phi     = atan2(py,px)
    ro_dot  = (px*vx + py*vy)/ro
    return np.matrix([ro, phi, ro_dot]).T     
    
class EKF:
    def __init__(self):
        self.x = None         
        self.I = np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        self.p = np.matrix ([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        self.H_imu = None
        # self.H_imu = np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        
        self.R_imu1 = np.matrix([[0.017,0,0],[0,0.017,0],[0,0,0.0002]]) #
        self.R_imu2 = np.matrix([[0.00024,0,0],[0,0.00024,0],[0,0,0.00001]]) #

    def init_state_vector(self, x,y, theta):
        self.x = np.matrix([[x,y,theta]]).T
    def current_estimate(self):
        return (self.x, self.p)
    def recompute_F_and_Q(self,dt,xx,yy,www, prr):
        V = (xx**2 + yy**2)**0.5
        if www != 0:
            f13 = (V/www)*(-cos(prr)+cos(prr + w*dt))
            f23 = (V/www)*(-sin(prr)+sin(prr + w*dt))
        else:
            f13 = (V)*(cos(prr)*dt)
            f23 = (V)*(sin(prr)*dt)
        self.F = np.matrix([[1.0, 0.0, f13],[0.0,1.0,f23],[0.0,0.0,1.0]])
        sigma2_a = 5
        sigma2_alpha = 5
        dt2 = dt**2
        dt3 = dt**3
        dt4 = dt**4

        e11 = dt4 * np.cos(prr)**2 * sigma2_a / 4
        e12 = dt4 * np.cos(prr) * np.sin(prr) * sigma2_a / 4
        e21 = dt4 * np.cos(prr) * np.sin(prr) * sigma2_a / 4
        e22 = dt4 * np.sin(prr)**2 * sigma2_a/ 4
        e33 = dt4 * sigma2_alpha/ 4
        self.Q = np.matrix([[e11,e12,0.0],[e21,e22,0.0],[0.0,0.0,e33]])        

    def recompute_HR(self,dt,aax,aay,vvx,vvy,xx,yy,a,b,tt):

        px,py,t = state_vector_to_scalars(self.x)
        # pxpy_squared = px**2+py**2
        # pxpy_squared_sqrt = sqrt(pxpy_squared)
        # pxpy_cubed = (pxpy_squared*pxpy_squared_sqrt)
        e11 = Vx*aax*dt/(xx-a)**2
        e22 = Vy*aay*dt/(yy-b)**2
        e33 = -(tt)/dt
        # if pxpy_squared < 1e-4:
        #     self.H_radar = np.matlib.zeros((3,4))
        #     return
        # e11 = px/pxpy_squared_sqrt
        # e12 = py/pxpy_squared_sqrt
        # e21 = -py/pxpy_squared
        # e22 = px/pxpy_squared
        # e31 = py*(vx*py - vy*px)/pxpy_cubed
        # e32 = px*(px*vy - py*vx)/pxpy_cubed
        self.H_imu = np.matrix([[e11, 0.0, 0.0],
                            [ 0.0,e22, 0.0],
                            [0.0,0.0, 1]])
    
    def predict(self):
        self.x = self.F * self.x
        self.p = (self.F * self.p * self.F.T) + self.Q       

