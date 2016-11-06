from numpy.linalg import svd
from numpy import sqrt, ones


def error_ellipse(covariances):
    l_eigenvectors, eigenvalues, r_eigenvectors= svd(covariances)
    sizes = sqrt(eigenvalues)
    return sizes, l_eigenvectors
