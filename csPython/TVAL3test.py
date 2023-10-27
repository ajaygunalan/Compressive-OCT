import numpy as np
import matplotlib.pyplot as plt
from skimage.data import shepp_logan_phantom
import matlab.engine

# Problem size
n = 256
ratio = 0.3
p, q = n, n  # p x q is the size of image
m = round(ratio * n ** 2)

# Generate Shepp-Logan phantom image
I = shepp_logan_phantom()
I = np.resize(I, (n, n))  # Resize to the desired dimensions
nrmI = np.linalg.norm(I, 'fro')

plt.figure()
plt.subplot(1, 2, 1)
plt.imshow(I, cmap='gray')
plt.title('Original phantom')
plt.draw()

# Sensing matrix
temp3 = np.random.rand(m, p * q)
A = temp3 - 0.5

# Observation
temp1 = I
temp2 = I.ravel()
f = np.matmul(A, temp2)
favg = np.mean(np.abs(f))

# Add noise
f += 0.00 * favg * np.random.randn(m)

# Run TVAL3 (MATLAB specific part)
eng = matlab.engine.start_matlab()
eng.run('add_all_paths.m', nargout=0)
opts = {'mu': 2 ** 8, 'beta': 2 ** 5, 'tol': 1e-3, 'maxit': 300, 'TVnorm': 1, 'nonneg': True}
[U, out, t] = eng.TVAL3(A.tolist(), f.tolist(), p, q, opts, nargout=3)
U = np.array(U)  # Convert back to numpy array

# Plotting the recovered image
plt.subplot(1, 2, 2)
plt.imshow(U, cmap='gray')
plt.title('Recovered by TVAL3')
plt.xlabel(f'{ratio * 100:.2f}% measurements\nRel-Err: {np.linalg.norm(U - I, "fro") / nrmI * 100:.2f}%, CPU: {t:.2f}s')
plt.show()


