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

    def uniform_sampling(self, num_points):
        n = int(math.sqrt(num_points))
        x_coords, y_coords = np.linspace(self.boundary_points_mm[0][0], self.boundary_points_mm[1][0], n), np.linspace(self.boundary_points_mm[0][1], self.boundary_points_mm[2][1], n)
        xx_uni, yy_uni = np.meshgrid(x_coords, y_coords)
        points_px = np.vstack([xx_uni.flatten(), yy_uni.flatten()]).T * [self.CameraScalingX, -self.CameraScalingY] + [self.center_x, self.center_y]
        self.add_octvideo_coords(coords=points_px)  

    def raster_scan(self):
        # Adjust the sorting to have the order begin at the top and go from left to right
        sorted_points = sorted(self.octvideo_coords, key=lambda x: (x[1], x[0]))  # Adjusted the key function to sort self.octvideo_coords
        self.octvideo_coords = [tuple(coord) for coord in sorted_points]  # Update self.octvideo_coords to the sorted order as a list
        # here the datastruct of self.octvideo_coords sud maintain order
    

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

    samplerObj1.uniform_sampling(num_points=300)
    samplerObj1.raster_scan()
    

    samplerObj1.octvideo_to_octscanner()
    samplerObj1.octscanner_to_surfacemap(surfacemap_cols=14, surfacemap_rows=14)

    samplerObj1.plot_points(title='Compressive 3-D Raster Scan (Checkerboard Pattern)')
    samplerObj1.animate_scan(video_title='Compressive 3-D Raster Scan')