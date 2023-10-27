import numpy as np
import matplotlib.pyplot as plt
from skimage.data import shepp_logan_phantom

# Original image
I = shepp_logan_phantom()  # Creating a phantom image similar to MATLAB's phantom
I = np.interp(I, (I.min(), I.max()), (0, 1))  # Normalize the image to [0, 1]

# Obtain the dimensions of the image
p, q = I.shape  # p x q is the size of the image

# Create sensing matrix A with ones at alternate rows and columns, and zeros elsewhere
A = np.zeros((p, q), dtype=int)
A[::2, ::2] = 1

# Displaying and saving the original image
plt.imshow(I, cmap='gray')
plt.show()
plt.imsave('x.png', I, cmap='gray')

# Displaying and saving the sensing matrix A
plt.imshow(A, cmap='gray')
plt.show()
plt.imsave('A_2dMask.png', A, cmap='gray')

# Element-wise multiplication to get y
y = I * A

# Displaying and saving the sampled image
plt.imshow(y, cmap='gray')
plt.show()
plt.imsave('y.png', y, cmap='gray')



# Flatten the image I into a 1D vector x
x = I.flatten()

# Obtain the linear indices of the sampled pixels from the flattened version of A
sampler_linear_idx = np.nonzero(A.flatten())[0]

# Use these indices to extract the corresponding elements from x to form y_1d
y_1d = x[sampler_linear_idx]

# Save y_1d to a text file
np.savetxt('y_1d.txt', y_1d)