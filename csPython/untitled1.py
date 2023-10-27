import cv2
import numpy as np

def preprocess(image, image_rows, image_cols):
    image = cv2.resize(image, (image_cols, image_rows))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = image.astype(np.float64) / np.max(image)
    return image

# Read the original image
filename = 'nature.jpg'  # Replace with the actual path to your image if it's not in the same directory
I = cv2.imread(filename)
rows, cols, _ = I.shape

I = preprocess(I, rows, cols)

# Obtain the dimensions of the image
p, q = I.shape  # p x q is the size of the image

# Create sensing matrix A with ones at alternate rows and columns, and zeros elsewhere
A = np.zeros((p, q), dtype=bool)
A[::2, ::2] = True

# Displaying and saving the original resized image
cv2.imshow('I', I)
cv2.imwrite('x.png', I * 255)

# Displaying and saving the sensing matrix A
cv2.imshow('A', A.astype(np.uint8) * 255)
cv2.imwrite('A_2dMask.png', A.astype(np.uint8) * 255)

# Element-wise multiplication to get y
y = A.astype(np.uint8) * I

# Displaying and saving the sampled image
cv2.imshow('y', y)
cv2.imwrite('y.png', y * 255)

# Flatten the image I into a 1D vector x
x = I.flatten(order='F')

# Obtain the linear indices of the sampled pixels from the flattened version of A
sampler_linear_idx = np.flatnonzero(A)

# Use these indices to extract the corresponding elements from x to form y_1d
y_1d = x[sampler_linear_idx]

# Save y_1d to a text file
np.savetxt('PYTHONy_1d.txt', y_1d)
