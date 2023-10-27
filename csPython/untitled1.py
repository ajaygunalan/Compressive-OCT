import cv2
import numpy as np

def preprocess(image, rows, cols):
    image = cv2.resize(image, (cols, rows))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = image.astype(np.float64) / 255.0  # Assuming the original image was 8-bit (0-255)
    return image

def generate_sensing_matrix(rows, cols, method='random', compression_ratio=0.3):
    if method == 'alternate':
        A = np.zeros((rows, cols), dtype=bool)
        A[::2, ::2] = True
    elif method == 'random':
        measurement_len = round(rows * cols * compression_ratio)
        A = np.zeros(rows * cols, dtype=bool)
        A[:measurement_len] = True
        np.random.shuffle(A)
        A = A.reshape((rows, cols))
    else:
        raise ValueError('Invalid method. Use "alternate" or "random".')
    return A

# Read the original image
filename = 'nature.jpg'
I = cv2.imread(filename)
rows, cols, _ = I.shape
I = preprocess(I, rows, cols)

# Generate sensing matrix
A = generate_sensing_matrix(rows, cols, method='alternate')
# A = generate_sensing_matrix(rows, cols, 'random', 0.3)

# Save matrices and images
np.savetxt('Python_A.txt', A, fmt='%d')
cv2.imwrite('x.png', (I * 255).astype(np.uint8))
cv2.imwrite('A_2dMask.png', (A * 255).astype(np.uint8))

# Element-wise multiplication to get y
y = A * I

# Save y image
cv2.imwrite('y.png', (y * 255).astype(np.uint8))

# Flatten the image I into a 1D vector x
x = I.flatten(order='F')

# Obtain the linear indices of the sampled pixels from the flattened version of A
sampler_linear_idx = np.flatnonzero(A)

# Use these indices to extract the corresponding elements from x to form y_1d
y_1d = x[sampler_linear_idx]

# Save y_1d to a text file
np.savetxt('Python_y.txt', y_1d)
