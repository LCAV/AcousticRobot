
import numpy as np
import matplotlib.pyplot as plt

# We have 4 markers and their interdistance.
# We want to reconstruct their relative coordinates

class MarkerSet:

    def __init__(self, m=4, diameter=0.04, dim=2, X=None):

        self.m = m
        self.dim = dim
        self.diameter = diameter
        if X is None:
            self.X = np.zeros((self.dim, self.m))

    def fromEDM(self, D, dim=2, method='mds'):
        ''' Compute the position of markers from their Euclidean Distance Matrix
        D: Euclidean Distance Matrix (matrix containing squared distances between points
        dim: dimension of underlying space (default: 2)
        method: 'mds' for multidimensional scaling (default)
                'tri' for trilateration
        '''

        if method == 'tri':
            self.trilateration(D)
        else:
            self.classical_mds(D)

    def classical_mds(self, D):

        if D.shape != (self.m, self.m):
            raise ValueError('The distance matrix must be square of size ' + str(self.m))


        # Apply MDS algorithm for denoising
        n = D.shape[0]
        J = np.eye(n) - np.ones((n,n))/float(n)
        G = -0.5*np.dot(J, np.dot(D, J))

        s, U = np.linalg.eig(G)

        # we need to sort the eigenvalues in decreasing order
        s = np.real(s)
        o = np.argsort(s)
        s = s[o[::-1]]
        U = U[:,o[::-1]]

        S = np.diag(s)[0:self.dim,:]
        self.X = np.dot(np.sqrt(S),U.T)

    def trilateration_single_point(self, c, Dx, Dy):
        '''
        Given x at origin (0,0) and y at (0,c) the distances from a point
        at unknown location Dx, Dy to x, y, respectively, finds the position of the point.
        '''

        z = (c**2 - (Dy**2 - Dx**2)) / (2*c)
        t = np.sqrt(Dx**2 - z**2)

        return np.array([t,z])

    def trilateration(self, D, dim=2):
        '''
        Find the location of points based on their distance matrix using trilateration
        D: EDM corresponding to point set
        dim: dimension of underlying space (default: 2)
        '''

        dist = np.sqrt(D)

        # Simpler algorithm (no denoising)
        self.X = np.zeros((self.dim, self.m))

        self.X[:,1] = np.array([0, dist[0,1]])
        for i in xrange(2,m):
            self.X[:,i] = self.trilateration_single_point(self.X[1,1], 
                    dist[0,i], dist[1,i])

    def EDM(self):
        G = np.dot(self.X.T, self.X)
        return np.outer(np.ones(self.m), np.diag(G)) \
            - 2*G + np.outer(np.diag(G), np.ones(self.m))

    def normalize(self):

        # set first point to origin
        X0 = self.X[:,0,np.newaxis]
        Y = self.X - X0

        # set second point to lie on x-axis
        theta = np.arctan2(Y[1,1],Y[0,1])
        c = np.cos(theta)
        s = np.sin(theta)
        H = np.array([[c, s],[-s, c]])
        Y = np.dot(H,Y)

        # set third point to lie above x-axis
        if Y[1,2] < 0:
            Y[1,:] *= -1

        # center the point set
        Y += X0
        Y -= Y.mean(axis=1, keepdims=True)

        self.X = Y

    def plot(self, axes=None, marker='x', labels=False):

        if axes is None:
            axes = plt.subplot(111)

        axes.plot(self.X[0,:], self.X[1,:], marker)
        axes.axis(aspect='equal')

        if labels:
            eps = np.linalg.norm(self.X[:,0] - self.X[:,1])/100
            for i in xrange(self.m):
                axes.text(self.X[0,i]+eps, self.X[1,i]+eps, str(i+1))


if __name__ == '__main__':

    # number of markers
    m = 4
    dim = 2

    D = np.zeros((m,m))

    marker_diameter = 0.040 # 4 cm

    D[0,1] = D[1,0] = 4.126 + marker_diameter/2
    D[0,2] = D[2,0] = 6.878 + marker_diameter/2
    D[0,3] = D[3,0] = 4.508 + marker_diameter/2
    D[1,2] = D[2,1] = 4.401 + marker_diameter/2
    D[1,3] = D[3,1] = 7.113 + marker_diameter/2
    D[3,2] = D[2,3] = 7.002 + marker_diameter/2

    M1 = MarkerSet(m=m, dim=dim, diameter=marker_diameter)
    M1.fromEDM(D**2)
    M1.normalize()

    M2 = MarkerSet(m=m, dim=dim, diameter=marker_diameter)
    M2.fromEDM(D**2, method='tri')
    M2.normalize()

    M2.plot(marker='ko', labels=True)
    M1.plot(marker='rx')
    plt.show()

