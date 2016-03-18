from datetime import datetime
import rospy

class PT1:
    """
        This class implements a simple PT1 element. Tustin discretization method. Gp(s) = Kp/(1+Tp*s)

        y(k) = -(T-2*Tp)/(T+2*Tp)*y(k-1) + (T*Kp)/(T+2*Tp)*(u(k) + u(k-1))
    """

    def __init__(self):
        """
            Initializes PT1 constants (proportional - Kp, time constant - Tp, discretization time - T) and control values to zero.
        """

        # initialize 
        self.kp = 0
        self.Tp = 0
        self.T = 0

        # initialize algorithm values
        self.y_k = 0
        self.y_k1 = 0
        self.u_k = 0
        self.u_k1 = 0



    def reset(self):
        ''' Resets PT1 algorithm by setting all algotihm parts to zero'''
        self.y_k = 0
        self.y_k1 = 0
        self.u_k = 0
        self.u_k1 = 0

    def set_kp(self, invar):
        """ Set gain. """
        self.kp = invar

    def get_kp(self):
        """Returns gain"""
        return self.kp

    def set_Tp(self, invar):
        """ Set time constant. """
        self.Tp = invar

    def get_Tp(self):
        """Returns time constant."""
        return self.Tp

    def set_T(self, invar):
        """ Set discretisation time. """
        self.T = invar

    def get_T(self):
        """Returns discretisation time."""
        return self.T

    
    def compute(self, uk):
        '''
        Performs a PT1 computation and returns a output value
        '''

        self.u_k = uk
        self.y_k = -((self.T - 2*self.Tp)/(self.T + 2*self.Tp))*self.y_k1 + ((self.T*self.kp)/(self.T + 2*self.Tp))*(self.u_k+self.u_k1)

        #save variables
        self.y_k1 = self.y_k
        self.u_k1 = self.u_k

        return self.y_k

