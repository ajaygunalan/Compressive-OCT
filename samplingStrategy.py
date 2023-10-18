import os
import tempfile
import matplotlib.pyplot as plt
import matplotlib

# Set the path to ffmpeg
matplotlib.rcParams['animation.ffmpeg_path'] = 'C:/ffmpeg/bin/ffmpeg.exe'

from PIL import Image
import numpy as np
import cv2
import math
import matplotlib.animation as animation
from scipy.spatial import distance_matrix

def basic_sorting(points, _):
    # Sorting based on y-coordinate and then x-coordinate
    return sorted(points, key=lambda x: (x[1], x[0]))



def sort_points_in_cells(points, cell_size):
    # Determine the boundaries of the points
    min_x, min_y = points.min(axis=0)
    max_x, max_y = points.max(axis=0)
    
    # Determine the number of cells in each dimension
    num_cells_x = int((max_x - min_x) / cell_size)
    num_cells_y = int((max_y - min_y) / cell_size)
    
    # Initialize an empty list to hold the sorted points
    sorted_points = []
    
    # Loop over the cells in raster order
    for i in range(num_cells_y):
        for j in range(num_cells_x):
            # Determine the boundaries of the current cell
            cell_min_x = min_x + j * cell_size
            cell_max_x = min_x + (j + 1) * cell_size
            cell_min_y = min_y + i * cell_size
            cell_max_y = min_y + (i + 1) * cell_size
            
            # Get the points within the current cell
            cell_points = points[(points[:, 0] >= cell_min_x) & 
                                 (points[:, 0] < cell_max_x) & 
                                 (points[:, 1] >= cell_min_y) & 
                                 (points[:, 1] < cell_max_y)]
            
            # If this is the first cell, or there are no points in the cell, continue
            if len(sorted_points) == 0 or len(cell_points) == 0:
                sorted_points.extend(cell_points)
                continue
            
            # Otherwise, sort the points in the cell based on distance to the
            # ending point of the previous cell
            prev_point = sorted_points[-1]
            distances = np.linalg.norm(cell_points - prev_point, axis=1)
            cell_points = cell_points[np.argsort(distances)]
            sorted_points.extend(cell_points)
    
    return np.array(sorted_points)



def nearest_neighbor_tsp(points, _=None):
    # Compute a distance matrix
    dist_matrix = distance_matrix(points, points)
    # Set diagonal to a high value since we don't want to return to the same point
    np.fill_diagonal(dist_matrix, np.inf)
    # Start from the first point
    current_point = 0
    # List to hold the order of visited points
    visit_order = [current_point]
    # Set of unvisited points
    unvisited = set(range(1, len(points)))
    while unvisited:
        # Find the nearest unvisited point
        nearest_point = min(unvisited, key=lambda x: dist_matrix[current_point, x])
        # Mark the nearest point as visited
        visit_order.append(nearest_point)
        unvisited.remove(nearest_point)
        # Move to the nearest point
        current_point = nearest_point
    # Convert the visit order to the order of points
    sorted_points = points[visit_order]
    return sorted_points



def raster_scan(points, video_title):
    # Adjust the sorting to have the order begin at the top and go from left to right
    sorted_points = sorted(points, key=lambda x: (x[1], x[0]))  # Adjusted the key function
    sorted_points = np.array(sorted_points)  # Convert to numpy array for easier indexing

    # Creating the animation
    def animate(i):
        ax.clear()
        ax.imshow(img)
        ax.scatter(sorted_points[:i+1, 0], sorted_points[:i+1, 1], c='yellow', s=50)  # +1 to have the indexing start at 1
        plt.title(f'{video_title} : scan {i+1}')  # +1 to have the indexing start at 1

    fig, ax = plt.subplots(1, figsize=(12, 9))
    ani = animation.FuncAnimation(fig, animate, frames=len(sorted_points), repeat=False)

    # Saving the animation
    ani.save(f'{video_title}.mp4', writer='ffmpeg', fps=5)  # Use video_title as the file name

    return sorted_points  # Return the sorted points if needed


def intelli_scan(points, video_title, sorting_function, sorting_arg=None, max_jump_threshold=2):
    # Apply the sorting function to the points
    sorted_points = sorting_function(points, sorting_arg)  # Pass the additional argument, if any
    sorted_points = np.array(sorted_points)  # Convert to numpy array for easier indexing

    # Creating the animation
    def animate(i):
        ax.clear()
        ax.imshow(img)
        ax.scatter(sorted_points[:i + 1, 0], sorted_points[:i + 1, 1], c='yellow', s=50)  # +1 to have the indexing start at 1
        plt.title(f'{video_title} : scan {i + 1} TO {len(sorted_points)}')  # +1 to have the indexing start at 1
        
        # Draw a line from the previous point to the current point
        if i > 0:
            prev_point = sorted_points[i - 1]
            curr_point = sorted_points[i]
            ax.plot([prev_point[0], curr_point[0]], [prev_point[1], curr_point[1]], color='red', linewidth=2)

    fig, ax = plt.subplots(1, figsize=(12, 9))
    ani = animation.FuncAnimation(fig, animate, frames=len(sorted_points), repeat=False)
    
    # Saving the animation
    ani.save(f'{video_title}.mp4', writer='ffmpeg', fps=5)  # Use video_title as the file name

    return sorted_points  # Return the sorted points if neede




def uniform_sampling(num_points):
    global img, img_gray, center_x, center_y, CameraScalingX, CameraScalingY, boundary_points_mm
    
    n = int(math.sqrt(num_points))
    x_coords, y_coords = np.linspace(boundary_points_mm[0][0], boundary_points_mm[1][0], n), np.linspace(boundary_points_mm[0][1], boundary_points_mm[2][1], n)
    xx_uni, yy_uni = np.meshgrid(x_coords, y_coords)
    
    global points_px
    points_px = np.vstack([xx_uni.flatten(), yy_uni.flatten()]).T * [CameraScalingX, -CameraScalingY] + [center_x, center_y]
    plot_points('Image with Uniformly Spaced Points Within Square')
    
    return points_px


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
    
    return points_px



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
    uniform_points = uniform_sampling(300)
    intelligent_points = intelligent_sampling(300, 10)
    
    
    basic_sorted_points = intelli_scan(intelligent_points, 'Basic Sorting', basic_sorting)
    cell_sorted_points = intelli_scan(intelligent_points, 'Cell Sorting', sort_points_in_cells, sorting_arg=10)
    tsp_sorted_points = intelli_scan(intelligent_points, 'Travelling Salesman Sorting', nearest_neighbor_tsp)


    
