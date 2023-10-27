import matlab.engine
import numpy as np
import matplotlib.pyplot as plt

# Start MATLAB engine
eng = matlab.engine.start_matlab()

# Problem size
n = 256
ratio = .3
p = n
q = n  # p x q is the size of image
m = round(ratio * n**2)

# Sensing matrix
temp3 = eng.rand(m, p*q)
# Convert Python float to MATLAB double before subtraction
A = eng.minus(temp3, matlab.double([[0.5]] * m * p * q))

# Original image
I = eng.phantom(n)
nrmI = eng.norm(I, 'fro')
plt.figure('TVAL3')
plt.subplot(121)
plt.imshow(eng.matlab.double(I), cmap='gray')
plt.title('Original phantom', fontsize=18)
temp1 = I
temp2 = eng.reshape(I, (n*n, 1))  # Reshaping as per MATLAB code

# Observation
f = eng.mtimes(A, temp2)
favg = eng.mean(eng.abs(f))

# Add noise
f = f + .00 * favg * eng.randn(m, 1)

# Run TVAL3
opts = {'mu': 2**8, 'beta': 2**5, 'tol': 1e-3, 'maxit': 300, 'TVnorm': 1, 'nonneg': True}
[U, out] = eng.TVAL3(A, f, p, q, opts, nargout=2)

# Display the result
plt.subplot(122)
plt.imshow(eng.matlab.double(U), cmap='gray')
plt.title('Recovered by TVAL3', fontsize=18)
plt.xlabel(f' {ratio*100:.0f}% measurements \n Rel-Err: {eng.norm(U-I,"fro")/nrmI*100:.2f}%, CPU: {eng.cputime():.2f}s ', fontsize=16)
plt.show()

