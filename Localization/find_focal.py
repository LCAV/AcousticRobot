
import numpy as np

def find_focal(X, H):
    """
    Find the location of the focal point of the camera (assuming no distortion)
    X: a 2xm nd-array where each column is the location of a marker point in the image plane
    H: the transformation matrix mapping points in the image plane to their location in the real plane

    Return:
    The least-square estimate of the intersection of all perspective lines
    """

    if X.shape[0] != 2:
        raise ValueError('Only for 3D vectors')

    m = X.shape[1]
    Q = np.concatenate((X, np.ones((1,m))), axis=0)

    U = np.dot(np.eye(3) - H, Q)
    U /= np.linalg.norm(U, axis=0)

    A,b = build_system(U,Q)

    # solve the overdetermined system in the least-square sense
    c, res, rank, s = np.linalg.lstsq(A,b)

    print res

    return c


def build_system(U,Q):
    """
    Build the system that corresponds to finding the point closest to all lines defined by U[:,i]*t + Q[:,i]
    """

    if U.shape[0] != 3 or Q.shape[0] != 3:
        raise ValueError('Only for 3D vectors')

    m = U.shape[1]
    u2cm = lambda x: np.array([[0,x[2],-x[1]],[-x[2],0,x[0]],[x[1],-x[0],0]])

    A = np.zeros((3*m, 3))
    for i in xrange(m):
        A[3*i:3*(i+1),:] = u2cm(U[:,i])

    b = np.cross(Q,U, axisa=0, axisb=0, axisc=0).flatten(order='F')

    return A,b


if __name__ == '__main__':

    m = 4

    # points in xy plane
    X = np.random.exponential(size=(3,m))
    X[2,:] = 1.
    X = X.astype('float32')

    # create a basis for the plane that contains x-axis and goes through (0,1,1)
    scaling = 0.2
    v1 = np.r_[1.,0.,0.]
    v2 = np.r_[0,1,1]/np.sqrt(2)
    P = np.array([v1, v2]).T.astype('float32')

    Y = scaling*np.dot(P.T, X) + np.random.randn(2,m)*0.
    Y = Y.astype('float32')

    import cv2
    H = cv2.getPerspectiveTransform(X[:2,:].T, Y.T)
    #H /= np.linalg.norm(H, axis=1)
    print P - H[:,:2]
    print P
    print H
    print Y
    #print np.dot(H, np.concatenate((Y, np.ones((1,4))), axis=0))
    Y_p = np.dot(H, X)
    print Y_p[:,:]
    #print Y_p[:2,:]/Y_p[2,:]

    c = find_focal(X[:2,:], H)
    print 'c=',c

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    Y3 = scaling*np.dot(P, Y)

    ax.scatter(X[0,:], X[1,:], zs=X[2,:], c='b', depthshade=False)
    ax.scatter(Y_p[0,:], Y_p[1,:], zs=Y_p[2,:], c='k', depthshade=False)
    ax.scatter([c[0]], [c[1]], zs=[c[2]], c='r', depthshade=False)

    plt.show()

