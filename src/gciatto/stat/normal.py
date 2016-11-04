from numpy.linalg import eig
from numpy import sqrt


def error_ellipse(covariances):
    eigenvalues, eigenvectors = eig(covariances)
    sizes = sqrt(eigenvalues)
    return sizes, eigenvectors
