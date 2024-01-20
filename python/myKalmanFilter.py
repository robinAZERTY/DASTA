#version 0.4 : extended kalman filter
# used for non-linear problem
import math
import numpy as np

class myEKF:
    
    def Jacobian(self, vector_function, x, y_dim=None, x_dim=None):
        #vector_function(x) = [f1(x), f2(x), ...] as np.array
        #Jacobian(x) = [[df1/dx1, df1/dx2, ...],
        if(y_dim is None):
            y_dim = len(vector_function(x))
        if(x_dim is None):
            x_dim = len(x)

        j= np.zeros((y_dim, x_dim))
        
        #use central difference to calculate the Jacobian
        delta = 1e-6
        for i in range(len(x)):
            x1 = x.copy()
            x1[i] += delta
            x2 = x.copy()
            x2[i] -= delta
            j[:,i] = ((vector_function(x1)-vector_function(x2))/(2*delta)).reshape(y_dim)
        return j

        
    def __init__(self, f ,h , x_dim=1, z_dim=1, u_dim =1, Fx = None, Fu=None, H = None):        
        self.x_dim = x_dim
        self.z_dim = z_dim
        
        #input
        self.Un = np.zeros((u_dim,1)) #control vector
        self.Zn = np.zeros((z_dim,1)) #measurement vector
        self.R = np.eye(z_dim) #measurement covariance matrix
        
        #output
        self.Xn = np.zeros((x_dim,1)) #state vector
        self.Pn = np.zeros((x_dim,x_dim)) #state variance matrix
        self.Xn_known = False #state knowledge  (true if all X is known, false if unknown)
        
        #constant
        self.B = np.eye(x_dim) #control matrix 
        self.h = h #measurement function
        self.H = H #Jacobian of h
        self.Q = np.zeros((x_dim,x_dim)) #transition covariance matrix (or process noise covariance matrix)
        self.I = np.eye(x_dim) #identity matrix
        self.infinit = 10**7 #infinite value

        #intermediate variables
        self.f = f #transition function Xn = f(Xn-1, Un)
        self.Fx = Fx #Jacobian of f by Xn
        self.Fu = Fu #Jacobian of f by Un
        self.K = np.zeros((x_dim,z_dim)) #kalman gain
        self.S = np.zeros((x_dim,x_dim)) #innovation covariance
        self.y = np.zeros((z_dim,1)) #innovation residual
        self.SI = np.zeros((z_dim,z_dim)) #innovation covariance

        self.init = False
        
        #if F and H are not provided, use Jacobian to calculate them
        if Fx is None:
            self.Fx = lambda x,u: self.Jacobian(lambda x: self.f(x,u), x)#Jacobian for transition function (by Xn)
        if Fu is None:
            self.Fu = lambda u,x: self.Jacobian(lambda u: self.f(x,u), u)#Jacobian for transition function (by Un)
        if H is None:
            self.H = lambda x: self.Jacobian(self.h, x)#Jacobian for measurement function
            
    def predict(self):
        #state transition
        Stx=self.Fx(self.Xn,self.Un)
        Stu=self.Fu(self.Un,self.Xn)
        self.Xn = self.f(self.Xn, self.Un)
        self.Pn = Stx.dot(self.Pn).dot(Stx.T) # propager l'incertitude de l'état
        self.Pn += Stu.dot(self.Q).dot(Stu.T) # propager l'incertitude de la commande
        
    def update_Xn_known(self):
        for i in range(self.Pn.shape[0]):
            if self.Pn[i][i] >= self.infinit/2:
                self.Xn_known = False
                return
        else:
            self.Xn_known = True
            
    def initialise_Xn_Using_Zn(self,Hk):
        
        self.Xn = (np.linalg.pinv(Hk.T@Hk))@Hk.T@self.Zn                           
        # Propager l'incertitude de la mesure sur l'incertitude de l'état
        #on commence par calculer le bruit de mesure (seulement les diagonales de R)
        std_noise = np.sqrt(self.R.diagonal())
        #on calcule ensuite le bruit de l'état
        std_state =  (np.linalg.pinv(Hk.T@Hk))@Hk.T@std_noise
        #on initialise la matrice de covariance de l'état
        self.Pn = np.diag(std_state**2)
             
        for i in range(self.x_dim):       
            if np.sum(Hk.T[i]) == 0:
                    for j in range(self.Pn.shape[0]):
                        if i == j:
                            self.Pn[j][i] = self.infinit
                            self.Pn[i][j] = self.infinit
                        else:
                            
                            self.Pn[j][i] = 0
                            self.Pn[i][j] = 0                   
        self.update_Xn_known()
        
    def update(self):
        # if(not self.init):
        #     if (np.any(np.isnan(self.Zn))):
        #         return
        #     Hk=self.H(self.Xn)
        #     self.initialise_Xn_Using_Zn(Hk)
        #     self.init = True
        #     return
        
        if(np.any(np.isnan(self.Zn))):
            return
        
        Hk=self.H(self.Xn)
        
        #calculate innovation residual
        self.y = self.Zn - self.h(self.Xn)

        #calculate innovation covariance
        self.S = Hk.dot(self.Pn).dot(Hk.T) + self.R
        
        #calculate kalman gain
        self.K = self.Pn.dot(Hk.T).dot(np.linalg.pinv(self.S))
        
        #update state vector
        self.Xn = self.Xn + self.K.dot(self.y)
        #update state variance matrix
        self.Pn = (self.I - self.K.dot(Hk)).dot(self.Pn)
        
        self.update_Xn_known()
        
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import numpy as np

    #example 1: robot localization (x,y,theta) with control (v,omega) and measurement (distance from beacon)
    #state transition function
    
    dt = 0.1
    beacon=np.array([[0.3],[0.6]])
    measurementNoise=0.3
    velocity_noise = 0.1
    gyro_noise = 0.05
    def f(Xn, Un):
        x = Xn[0][0]
        y = Xn[1][0]
        theta = Xn[2][0]
        v = Un[0][0]
        omega = Un[1][0]
        
        x += v*math.cos(theta)*dt
        y += v*math.sin(theta)*dt
        theta += omega*dt
        
        return np.array([[x],[y],[theta]])
    
    def h(Xn):
        #fonction du capteur
        dx = Xn[0]-beacon[0]
        dy = Xn[1]-beacon[1]
        d= np.sqrt(dx**2+dy**2)
        return np.array([d])
    
    #generate signal
    T=20
    t=np.arange(0,T,dt)
    #generate trajectory (construct from polar coordinates)
    r = np.linspace(1,0.6,len(t))**2
    theta = np.linspace(0,4*math.pi,len(t))
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    
    direction = np.zeros((2,len(t)))
    direction[0,1:] = (x[1:]-x[:-1])/dt
    direction[1,1:] = (y[1:]-y[:-1])/dt
    direction[0,0] = direction[0,1]
    direction[1,0] = direction[1,1]
    theta = np.arctan2(direction[1,:],direction[0,:])
    #comute the derivative of theta
    omega=np.zeros(len(t))
    omega[1:] = (theta[1:]-theta[:-1])/dt
    omega[0] = omega[1]
    
    velocity = np.sqrt(direction[0,:]**2+direction[1,:]**2)

    
    
    X = np.array([x,y,theta])
    
    #noisy samples
    Zn=h(X)+np.random.normal(0,measurementNoise,(3,len(t)))
    velocity_measured = velocity + np.random.normal(0,velocity_noise,len(t))
    omega_measured = omega + np.random.normal(0,gyro_noise,len(t))
    Un = np.array([velocity_measured,omega_measured])
    
    #biais dans la mesure de la vitesse angulaire
    # Un[1,:] += 0.1
    
    #instanciation of the kalman filter
    ekf = myEKF(f,h,x_dim=3,z_dim=1,u_dim=2)
    
    #initialisation
    ekf.Xn = X[:,0].reshape(3,1).copy()
    ekf.Pn = np.diag([0.01,0.01,0.01])#cov of the initial state
    ekf.Q = np.diag([velocity_noise**2,gyro_noise**2])# covariance matrix of the process noise (who much the control can change between two time steps)
    ekf.R = np.diag([measurementNoise**2])
    
    #save the result
    result = np.zeros((len(t)+1,3))
    Pn_res = np.zeros((len(t)+1,3,3))
    integration=np.zeros((len(t)+1,3))
    integration[0,:]=X[:,0].copy()
    
    result[0,:] = ekf.Xn.reshape(3)
    Pn_res[0,:,:] = ekf.Pn

    for i in range(len(t)):
        ekf.Un = Un[:,i].reshape(2,1)
        ekf.Zn = Zn[0,i]
        ekf.predict()
        ekf.update()
        print(ekf.Pn)

        result[i+1,:] = ekf.Xn.reshape(3)
        Pn_res[i+1,:,:] = ekf.Pn
        
        v=Un[0,i]
        omega=Un[1,i]

        integration[i+1,:] = integration[i,:] + np.array([v*math.cos(integration[i,2])*dt,v*math.sin(integration[i,2])*dt,omega*dt]).reshape(3)

    import matplotlib.animation as animation
    #annimation
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(211, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
    ax.grid()
    ax.set_aspect('equal')
    time_template = 'time = %.1fs'
    time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
    ax2 = fig.add_subplot(212)
    ax2.set_xlabel('t')
    ax2.set_ylabel('w')


    
    def init():
        pass

    from matplotlib.patches import Ellipse
    
    def plot_cov_ellipse(cov, pos, nstd=2, ax=None, **kwargs):
        """
        Plots an `nstd` sigma error ellipse based on the specified covariance
        matrix (`cov`). Additional keyword arguments are passed on to the 
        ellipse patch artist.

        Parameters
        ----------
            cov : The 2x2 covariance matrix to base the ellipse on
            pos : The location of the center of the ellipse. Expects a 2-element
                sequence of [x0, y0].
            nstd : The radius of the ellipse in numbers of standard deviations.
                Defaults to 2 standard deviations.
            ax : The axis that the ellipse will be plotted on. Defaults to the 
                current axis.
            Additional keyword arguments are pass on to the ellipse patch.

        Returns
        -------
            A matplotlib ellipse artist
        """
        def eigsorted(cov):
            vals, vecs = np.linalg.eigh(cov)
            order = vals.argsort()[::-1]
            return vals[order], vecs[:,order]

        if ax is None:
            ax = plt.gca()

        vals, vecs = eigsorted(cov)
        theta = np.degrees(np.arctan2(*vecs[:,0][::-1]))

        # Width and height are "full" widths, not radius
        width, height = 2 * nstd * np.sqrt(vals)
        ellip = Ellipse(xy=pos, width=width, height=height, angle=theta, **kwargs)

        ax.add_artist(ellip)
        return ellip
    
    def animate(i):
        ax.clear()
        #plot the real position history
        ax.plot(X[0,:i],X[1,:i],label='real')
        #plot the distance to the beacon
        ax.plot(beacon[0],beacon[1],'o',label='beacon')
        #arrow for direction
        # ax.arrow(X[0,i],X[1,i],0.1*math.cos(X[2,i]),0.1*math.sin(X[2,i]),head_width=0.05, head_length=0.1, fc='b', ec='b',label='real direction')
        ax.plot(result[:i,0],result[:i,1],label='filtered')
        # ax.arrow(result[i,0],result[i,1],0.1*math.cos(result[i,2]),0.1*math.sin(result[i,2]),head_width=0.05, head_length=0.1, fc='r', ec='r',label='filtered direction')
        time_text.set_text('')
        time_text.set_text(time_template%(i*dt))
        #plot the covariance matrix as an ellipse to show the uncertainty (with 3 sigma and angle)
        
        plot_cov_ellipse(Pn_res[i,:2,:2], result[i,:2], nstd=1, ax=ax, alpha=0.5, color='red')
        
        #compare with the integration of the control
        ax.plot(integration[:i,0],integration[:i,1],label='integration')
        # ax.arrow(integration[i,0],integration[i,1],0.1*math.cos(integration[i,2]),0.1*math.sin(integration[i,2]),head_width=0.05, head_length=0.1, fc='g', ec='g',label='integration')

        ax.legend()
        ax.axis('equal')
        
        ax2.clear()
        ax2.plot(t[:i],result[:i,2],label='theta')
        ax2.plot(t[:i],X[2,:i],label='theta real')
        #with uncertainty around theta
        ax2.fill_between(t[:i], result[:i,2]-np.sqrt(Pn_res[:i,2,2]), result[:i,2]+np.sqrt(Pn_res[:i,2,2]), alpha=0.2, label='theta variance')
        ax2.plot(t[:i],integration[:i,2],label='theta integration')
        ax2.legend()
    
    ani = animation.FuncAnimation(fig, animate, np.arange(1, len(t)),
                                interval=25, blit=False, init_func=init, repeat=False)
    plt.show()
    
    #save the animation
    ani.save('ekf_localization.gif', fps=20)

    # plt.plot(X[0,:],X[1,:],label='real')
    # plt.plot(result[:,0],result[:,1],label='filtered')

    # plt.legend()
    # plt.axis('equal')
    # plt.show()

