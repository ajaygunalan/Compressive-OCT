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
import scipy



class SamplerClass:
    def __init__(self):
        self.octvideo_coords = set()
        self.octvideoscannerdict = {}
        self.octscannersurfacedict = {}
        self.surfacemap_to_value = {}
        self.center_x = None
        self.center_y = None
        self.CameraScalingX = None
        self.CameraScalingY = None
        self.boundary_points_mm = None
        self.surgical_img = None
        self.surgical_img_gray = None

    def set_camera_parameters(self, center_x, center_y, CameraScalingX, CameraScalingY):
        self.center_x = center_x
        self.center_y = center_y
        self.CameraScalingX = CameraScalingX
        self.CameraScalingY = CameraScalingY

    def set_boundary_points(self, boundary_points_mm):
        self.boundary_points_mm = boundary_points_mm

    def set_surgical_image(self, image_path):
        self.surgical_img = Image.open(image_path)
        self.surgical_img_gray = np.array(self.surgical_img.convert('L'))

    def add_octvideo_coords(self, coords):
        for coord in coords:
            self.octvideo_coords.add(tuple(coord))

    def octvideo_to_octscanner(self):
        points_px = np.array(list(self.octvideo_coords))
        points_mm = np.empty_like(points_px)
        points_mm[:, 0] = (points_px[:, 0] - self.center_x) / self.CameraScalingX
        points_mm[:, 1] = (points_px[:, 1] - self.center_y) / -self.CameraScalingY
        
        # Store the mapping
        for px_coord, mm_coord in zip(points_px, points_mm):
            self.octvideoscannerdict[tuple(px_coord)] = tuple(mm_coord)
        
        return points_mm

    def octscanner_to_surfacemap(self, surfacemap_cols, surfacemap_rows):
        real_world_coords = np.array(list(self.octvideoscannerdict.values()))
        center_x, center_y = surfacemap_cols / 2, surfacemap_rows / 2
        max_coords = np.max(self.boundary_points_mm, axis=0)
        min_coords = np.min(self.boundary_points_mm, axis=0)
        real_world_range_x = max_coords[0] - min_coords[0]
        real_world_range_y = max_coords[1] - min_coords[1]
        scaling_x = surfacemap_cols / real_world_range_x
        scaling_y = surfacemap_rows / real_world_range_y

        for i, real_world_coord in enumerate(real_world_coords):
            pixel_coord = np.empty_like(real_world_coord)
            pixel_coord[0] = real_world_coord[0] * scaling_x + center_x
            pixel_coord[1] = -real_world_coord[1] * scaling_y + center_y  # Flipping the y-coordinate
            
            # Store the mapping
            self.octscannersurfacedict[tuple(real_world_coord)] = tuple(pixel_coord)

    def add_mapping(self, octscanner_coord, surfacemap_coord, value=None):
        self.octscannersurfacedict[octscanner_coord] = surfacemap_coord
        if value is not None:
            self.surfacemap_to_value[surfacemap_coord] = value

    def get_octscanner_coord(self, octvideo_coord):
        return self.octvideoscannerdict.get(octvideo_coord)

    def get_surfacemap_coord(self, octscanner_coord):
        return self.octscannersurfacedict.get(octscanner_coord)

    def get_surface_value(self, surfacemap_coord):
        return self.surfacemap_to_value.get(surfacemap_coord)

 
    def animate_scan(self, video_title='Animation'):  # Default title is 'Animation' if none is provided
        sorted_points = np.array(list(self.octvideo_coords))  # Assuming the points are already sorted
    
        # Creating the animation
        def animate(i):
            ax.clear()
            ax.imshow(self.surgical_img)
            ax.scatter(sorted_points[:i+1, 0], sorted_points[:i+1, 1], c='yellow', s=50)  # +1 to have the indexing start at 1
            plt.title(f'{video_title} : scan {i+1}')  # +1 to have the indexing start at 1
            
            # Draw a line from the previous point to the current point
            if i > 0:
                prev_point = sorted_points[i - 1]
                curr_point = sorted_points[i]
                ax.plot([prev_point[0], curr_point[0]], [prev_point[1], curr_point[1]], color='red', linewidth=2)
    
        fig, ax = plt.subplots(1, figsize=(12, 9))
        ani = animation.FuncAnimation(fig, animate, frames=len(sorted_points), repeat=False)
        
        # Saving the animation
        ani.save(f'{video_title}.mp4', writer='ffmpeg', fps=5)  # Use video_title as the file name

            
    def plot_points(self, title):
        # Now using self.surgical_img and self.boundary_points_mm
        points_px = np.array(list(self.octvideo_coords))
        fig, ax = plt.subplots(1, figsize=(12, 9))
        ax.imshow(self.surgical_img)
        plt.scatter(points_px[:, 0], points_px[:, 1], c='yellow', s=50)

        # Reverse the mapping dictionary for easier lookup
        reverse_mapping = {value: key for key, value in self.octvideoscannerdict.items()}

        # Use the mapping to get pixel coordinates for the boundary points
        for real_world_coord in self.boundary_points_mm:  # Updated line
            px_coord = reverse_mapping.get(tuple(real_world_coord))
            if px_coord:  # Check if mapping exists
                plt.scatter(px_coord[0], px_coord[1], c='black', s=50)  # Adjusted color to black for boundary points
                plt.annotate(f"RW: {real_world_coord[0]},{real_world_coord[1]}", 
                             (px_coord[0] + 10, px_coord[1] + 10), color='white')

                # Get the surfacemap coordinate from the mapping
                surfacemap_coord = self.octscannersurfacedict.get(tuple(real_world_coord))
                if surfacemap_coord:  # Check if mapping exists
                    plt.annotate(f"SM: {surfacemap_coord[0]},{surfacemap_coord[1]}", 
                                 (px_coord[0] + 10, px_coord[1] + 30), color='red')  # New annotation for surfacemap points

        plt.title(title)
        plt.show()
        
    def uniform_sampling(self, num_points):
        n = int(math.sqrt(num_points))
        x_coords, y_coords = np.linspace(self.boundary_points_mm[0][0], self.boundary_points_mm[1][0], n), np.linspace(self.boundary_points_mm[0][1], self.boundary_points_mm[2][1], n)
        xx_uni, yy_uni = np.meshgrid(x_coords, y_coords)
        points_px = np.vstack([xx_uni.flatten(), yy_uni.flatten()]).T * [self.CameraScalingX, -self.CameraScalingY] + [self.center_x, self.center_y]
        self.add_octvideo_coords(coords=points_px) 
        

    def intelligent_sampling(self, num_points, min_radius):
        # Compute boundary points in pixel coordinates
        boundary_points_px = self.boundary_points_mm * [self.CameraScalingX, -self.CameraScalingY] + [self.center_x, self.center_y]
        min_x, max_x = int(min(boundary_points_px[:, 0])), int(max(boundary_points_px[:, 0]))
        min_y, max_y = int(min(boundary_points_px[:, 1])), int(max(boundary_points_px[:, 1]))

        # Limit the gradient computation to within the boundary
        img_roi = self.surgical_img_gray[min_y:max_y, min_x:max_x]
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
        
        # Store the valid points into the class's octvideo_coords set attribute
        self.add_octvideo_coords(coords=np.array(valid_points))
        
        # Optionally plot the points if needed
        self.plot_points('Intelligently Sampling by Leveraging the Gradient of the Surgical Image')
        
    
    def raster_scan(self):
        # Adjust the sorting to have the order begin at the top and go from left to right
        sorted_points = sorted(self.octvideo_coords, key=lambda x: (x[1], x[0]))  # Adjusted the key function to sort self.octvideo_coords
        self.octvideo_coords = [tuple(coord) for coord in sorted_points]  # Update self.octvideo_coords to the sorted order as a list
        # here the datastruct of self.octvideo_coords sud maintain order
        
        
    def intelli_scan(self):
        points = np.array(list(self.octvideo_coords))
        # Determine the boundaries of the points
        min_x, min_y = points.min(axis=0)
        max_x, max_y = points.max(axis=0)
    
        # Determine the height of each segment
        segment_height = (max_y - min_y) / 10
    
        # Initialize an empty list to hold the sorted points
        sorted_points = []
    
        # Loop over the segments
        for i in range(10):
            # Determine the boundaries of the current segment
            seg_min_y = min_y + i * segment_height
            seg_max_y = min_y + (i + 1) * segment_height
    
            # Get the points within the current segment
            segment_points = points[(points[:, 1] >= seg_min_y) & (points[:, 1] < seg_max_y)]
    
            # If there are no points in the segment, continue
            if len(segment_points) == 0:
                continue
    
            # Sort the points within the segment based on their x-coordinate to start from the left
            segment_points = segment_points[segment_points[:, 0].argsort()]
    
            # Start with the top-left point in the segment
            current_point = segment_points[0]
            segment_points = np.delete(segment_points, 0, axis=0)
            segment_sorted = [current_point]
    
            # While there are points remaining in the segment
            while len(segment_points) > 0:
                # Find the nearest point to the current point
                distances = np.linalg.norm(segment_points - current_point, axis=1)
                nearest_point_index = np.argmin(distances)
                nearest_point = segment_points[nearest_point_index]
    
                # Remove the nearest point from segment_points and set it as the current point
                segment_points = np.delete(segment_points, nearest_point_index, axis=0)
                segment_sorted.append(nearest_point)
                current_point = nearest_point
    
            # Extend the sorted_points list with the sorted points from the current segment
            sorted_points.extend(segment_sorted)
    
        # Update the order of self.octvideo_coords to the sorted order
        self.octvideo_coords = [tuple(coord) for coord in np.array(sorted_points)]


    def nearest_neighbor_scan(self):
        points = np.array(list(self.octvideo_coords))
        # Compute a distance matrix
        dist_matrix = scipy.spatial.distance_matrix(points, points)
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
        # Update the order of self.octvideo_coords to the sorted order
        self.octvideo_coords = [tuple(coord) for coord in sorted_points]

    
    
if __name__ == "__main__":
    
    samplerObj1 = SamplerClass()
    
    samplerObj1.set_camera_parameters(
        center_x=324, 
        center_y=242, 
        CameraScalingX=32.3351, 
        CameraScalingY=32.3402
    )
    
    samplerObj1.set_boundary_points(
        boundary_points_mm=np.array([[5, 5], [-5, 5], [-5, -5], [5, -5], [0, 0], [5, 0], [0, 5], [0, -5], [-5, 0]])
    )
    
    samplerObj1.set_surgical_image(image_path='octRGB.jpg')

    samplerObj1.intelligent_sampling(num_points=300, min_radius=8)
    samplerObj1.intelli_scan()
    

    samplerObj1.octvideo_to_octscanner()
    samplerObj1.octscanner_to_surfacemap(surfacemap_cols=14, surfacemap_rows=14)


    samplerObj1.plot_points(title='IntelliSense - Intelligently Sampling and Scan')
    samplerObj1.animate_scan(video_title='IntelliSense - Intelligently Sampling and Scan')


