import numpy as np
import cv2
import matplotlib.pyplot as plt

def preprocess(image, rows, cols):
    image = cv2.resize(image, (cols, rows))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image = image.astype(np.float64) / 255.0  # Assuming the original image was 8-bit (0-255)
    return image

# Read the original image
filename = 'nature.jpg'
I = cv2.imread(filename)
rows, cols, _ = I.shape
I = preprocess(I, rows, cols)
# Flatten the image I into a 1D vector x
x = I.flatten(order='F')

# Load the data from the text file, specifying the delimiter as ','
A = np.loadtxt('Matlab_A.txt', delimiter=',')

# Convert the loaded data to boolean type
A = A.astype(bool)

# Find the indices where the value is True on the transposed matrix
true_indices = np.argwhere(A.T)

# Swap rows and columns back to their original order
true_indices = true_indices[:, ::-1]

num_rows = A.shape[0]  # Get the number of rows in Matlab_A
python_linear_idx = true_indices[:, 0] + (true_indices[:, 1] * num_rows)
matlab_linear_idx = python_linear_idx + 1

# Use these indices to extract the corresponding elements from x to form y_1d
y_1d_M = x[python_linear_idx]

# Save y_1d to a text file
np.savetxt('Python_y_M.txt', y_1d_M)
