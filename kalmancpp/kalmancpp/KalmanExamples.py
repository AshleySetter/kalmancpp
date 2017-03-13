import numpy as np
import sympy

def TransformSystemsDynamicsMatrix(F):
    s = sympy.symbols('s', real = True, positive=True)
    t = sympy.symbols('t', real = True, positive=True)
    MatrixSize = F.shape[0]
    temp = s*sympy.eye(MatrixSize)-F #sI-F
    temp_inv = temp.inv()
    invLaplace = sympy.inverse_laplace_transform(temp_inv, s, t)
    return invLaplace

def SineKalmanFilter(Freq, SampleFreq):
    """
    Calcualtes the Matricies needed for a sinusoidal kalman filter with
    no bias.

    Parameters
    ----------
    Freq : float
        Frequency for the Kalman filter to filter about.
    SampleFreq : float
        Sample frequency of the data to be filtered

    Returns
    -------
    A : ndarray
        The A matrix for the sinusoidal Kalman filter
    H : ndarray
        The H vector for the sinusoidal Kalman filter
    """

    # Calculate parameters for Kalman filter
    w = sympy.symbols('w', real = True, positive=True)
    t = sympy.symbols('t', real = True, positive=True)
    F = sympy.Matrix([
        [0, 1],
        [-w**2, 0]
    ])

    A_sympy = TransformSystemsDynamicsMatrix(F)
    A_fn = sympy.lambdify([w,t], A_sympy)

    W = 2*np.pi*Freq
    Ts = 1/SampleFreq

    A = A_fn(W, Ts)

    A = np.array(A)

    H = np.array([1.0, 0.0])

    return A, H
