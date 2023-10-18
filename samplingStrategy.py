from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math


def uniform_sampling(num_points):
    global img, img_gray, center_x, center_y, CameraScalingX, CameraScalingY, boundary_points_mm
    
    n = int(math.sqrt(num_points))
    x_coords, y_coords = np.linspace(boundary_points_mm[0][0], boundary_points_mm[1][0], n), np.linspace(boundary_points_mm[0][1], boundary_points_mm[2][1], n)
    xx_uni, yy_uni = np.meshgrid(x_coords, y_coords)
    
    global points_px
    points_px = np.vstack([xx_uni.flatten(), yy_uni.flatten()]).T * [CameraScalingX, -CameraScalingY] + [center_x, center_y]
    plot_points('Image with Uniformly Spaced Points Within Square')


def intelligent_sampling(num_points, min_radius):
    global img, img_gray, center_x, center_y, CameraScalingX, CameraScalingY, boundary_points_mm
    
    # Compute boundary points in pixel coordinates
    boundary_points_px = boundary_points_mm * [CameraScalingX, -CameraScalingY] + [center_x, center_y]
    min_x, max_x = int(min(boundary_points_px[:, 0])), int(max(boundary_points_px[:, 0]))
    min_y, max_y = int(min(boundary_points_px[:, 1])), int(max(boundary_points_px[:, 1]))

    # Limit the gradient computation to within the boundary
    img_roi = img_gray[min_y:max_y, min_x:max_x]
    sobelx = cv2.Sobel(img_roi, cv2.CV_64F, 1, 0, ksize=5)
    sobely = cv2.Sobel(img_roi, cv2.CV_64F, 0, 1, ksize=5)
    grad_mag = np.sqrt(sobelx ** 2 + sobely ** 2)
    
    # Normalize and visualize the gradient
    grad_mag = grad_mag / np.sum(grad_mag)
    plt.imshow(grad_mag, cmap='gray')
    plt.title("Normalized Gradient Magnitude")
    plt.show()

    # Generate points based on the gradient, ensuring minimum radius
    valid_points = []
    x_range, y_range = np.arange(min_x, max_x), np.arange(min_y, max_y)
    xx, yy = np.meshgrid(x_range, y_range)
    coord_pairs = np.vstack([xx.flatten(), yy.flatten()]).T
    
    while len(valid_points) < num_points:
        sampled_indices = np.random.choice(coord_pairs.shape[0], num_points, p=grad_mag.flatten())
        new_points = coord_pairs[sampled_indices]
        for point in new_points:
            if valid_points:  # Check if valid_points is not empty
                if all(np.linalg.norm(np.array(valid_points) - point, axis=1) >= min_radius):
                    valid_points.append(point)
            else:
                valid_points.append(point)  # Directly append if valid_points is empty
            if len(valid_points) >= num_points:
                break

    global points_px
    points_px = np.array(valid_points)
    plot_points('Image with Intelligently Spaced Points Within Defined Boundary')



def plot_points(title):
    global img, points_px, boundary_points_mm
    boundary_points_px = boundary_points_mm * [CameraScalingX, -CameraScalingY] + [center_x, center_y]
    
    fig, ax = plt.subplots(1, figsize=(12, 9))
    ax.imshow(img)
    plt.scatter(points_px[:, 0], points_px[:, 1], c='yellow', s=50)
    plt.scatter(boundary_points_px[:, 0], boundary_points_px[:, 1], c='black', s=200)
    
    for i, point in enumerate(boundary_points_mm):
        plt.annotate(f"{point[0]},{point[1]}", (boundary_points_px[i, 0] + 10, boundary_points_px[i, 1] + 10), color='white')
    
    plt.title(title)
    plt.show()
    
    
# Global variables
center_x, center_y = 324, 242
CameraScalingX, CameraScalingY = 32.3351, 32.3402
boundary_points_mm = np.array([[5, 5], [-5, 5], [-5, -5], [5, -5], [0, 0], [5, 0], [0, 5], [0, -5], [-5, 0]])

# Read and grayscale the image
img = Image.open('octRGB.jpg')
img_gray = np.array(img.convert('L'))

if __name__ == "__main__":
    uniform_sampling(300)
    intelligent_sampling(300, 13)
