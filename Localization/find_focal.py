
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
        A[3*i:3*(i+1),:] = u2cm(u[:,i])

    b = np.cross(Q,U, axisa=0, axisb=0, axisc=0).flatten(order='F')

    return A,b

