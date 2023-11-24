import cv2
import numpy as np
import matlab.engine
import matplotlib.pyplot as plt 

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
        measurement_len = round(rows * cols * compression_ratio)  # 30% True values
        a = np.full(rows * cols, False)
        a[:measurement_len] = True
        np.random.shuffle(a)
        A = a.reshape((rows, cols))
    else:
        raise ValueError('Invalid method. Use "alternate" or "random".')
    return A


def getYforMatlab(I, A):
    # Flatten the image I into a 1D vector x
    x = I.flatten(order='F')

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
    y_1d = x[python_linear_idx]
    a = y_1d
    
    return y_1d

# Read the original image
filename = 'nature.jpg'
I = cv2.imread(filename)
rows, cols, _ = I.shape
I = preprocess(I, rows, cols)

# Get A and Y
# A = generate_sensing_matrix(rows, cols, method='alternate')
A = generate_sensing_matrix(rows, cols, 'random', 0.3)
# Element-wise multiplication to get y
y_2D = A * I

# Save y image
cv2.imwrite('y.png', (y_2D * 255).astype(np.uint8))


y = getYforMatlab(I, A)

# # Start MATLAB engine
# eng = matlab.engine.start_matlab()
# eng.run('add_all_paths.m', nargout=0)
# [reconstructed_img, time_taken] = eng.csAj(A, y)
# eng.quit()
# print(f"Time taken: {time_taken:.4f} seconds")



# # Display the reconstructed image along with the time taken
# plt.figure()
# plt.imshow(reconstructed_img, cmap='gray')  # Assumes the image is grayscale
# plt.title(f'Reconstructed Image\nTime taken: {time_taken:.4f} seconds')
# plt.axis('off')  # Hide the axis

# # Save the reconstructed image and the display figure
# cv2.imwrite('reconstructed_image.png', (reconstructed_img * 255).astype(np.uint8))
# plt.savefig('reconstructed_image_figure.png')

# # Show the figure
# plt.show()





# Save matrices and images
np.savetxt('Python_A.txt', A, fmt='%d')
np.savetxt('Python_y.txt', y)


cv2.imwrite('x.png', (I * 255).astype(np.uint8))
cv2.imwrite('A_2dMask.png', (A * 255).astype(np.uint8))




