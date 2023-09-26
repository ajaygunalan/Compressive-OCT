import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import RectangleSelector

def onselect(eclick, erelease):
    global xmin, ymin, xmax, ymax
    xmin, ymin = eclick.xdata, eclick.ydata
    xmax, ymax = erelease.xdata, erelease.ydata

def filterClosePoints(points, min_distance):
    n = len(points)
    filtered_points = np.array([points[0]])
    for i in range(1, n):
        distances = np.linalg.norm(filtered_points - points[i], axis=1)
        if np.all(distances > min_distance):
            filtered_points = np.vstack([filtered_points, points[i]])
    return filtered_points


def depthEstimationFrom2D(folderPath, filename):
    global xmin, ymin, xmax, ymax  # Declare them as global
    # 1. Detect the ablate surface -> Binary, Dilate, Erode, edge, Contour.
    
    # Full path to the image
    fullPath = f"{folderPath}/{filename}"
    name, ext = filename.rsplit('.', 1)
    middle_image_filename = f"{folderPath}/{name}middle.{ext}"
    final_image_filename = f"{folderPath}/{name}final.{ext}"
    txt_filename = f"{folderPath}/{name}middle.txt"
    
    # Read the image
    depthI = cv2.imread(f"{folderPath}/{filename}", cv2.IMREAD_GRAYSCALE)
    
    # Get image dimensions
    img_height, img_width = depthI.shape
    
    # Adaptive Thresholding
    BW = cv2.adaptiveThreshold(depthI, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    
    # Structuring Element
    se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    
    # Dilation + Erosion
    BW = cv2.dilate(BW, se)
    BW = cv2.erode(BW, se)
    
    # Canny Edge Detection
    lowerThreshold = 100  # Adjust as needed
    upperThreshold = 200  # Adjust as needed
    BW = cv2.Canny(BW, lowerThreshold, upperThreshold)
    
    # Find contours
    contours, _ = cv2.findContours(BW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize a threshold for boundary size
    sizeThreshold = 100  # Adjust as needed
    
    # Draw contours
    frame_contours = depthI.copy()
    surface = []
    for cnt in contours:
        if len(cnt) > sizeThreshold:
            surface.append(cnt)
            cv2.drawContours(frame_contours, [cnt], 0, 255, 1)
    
    # Plotting
    plt.figure()
    plt.imshow(frame_contours, cmap='gray')
    plt.title('Processed Image')
    plt.show()


    # 2. Approximate contour points with Line Segments
    surface_reduced = []
    ablate_surface = []
    
    # Loop over each surface
    for cnt in surface:
        # Reduce the polygon using Douglas-Peucker algorithm
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        surface_reduced.append(approx)
    
    # Initialize ablate_surface index
    ablate_surface_idx = 0
    frame_contours_with_reduced = depthI.copy()
    
    for approx in surface_reduced:
        dx = abs(approx[1][0][0] - approx[0][0][0])
        
        if dx > 2:
            ablate_surface.append(approx)
            continue
        
        for point in approx:
            x, y = point[0]
            frame_contours_with_reduced[y, x] = 255
    
    # Remove close points
    minDistance = 3.6  # Adjust this as needed
    ablate_surface = [filterClosePoints(cnt, minDistance) for cnt in ablate_surface]
    
    # Combine multiple ablate surfaces if they exist
    ablate_surface_combined = np.vstack(ablate_surface)
    
    # Visualize the image with reduced contours
    plt.figure()
    plt.imshow(frame_contours_with_reduced, cmap='gray')
    
    # Draw each ablate surface
    for idx, cnt in enumerate(ablate_surface):
        cnt = cnt.reshape(-1, 2)
        plt.plot(cnt[:, 0], cnt[:, 1], 'ro-', linewidth=1.5, markersize=5)
        
        # Annotate the surface with surface_idx value
        mid_point_idx = len(cnt) // 2
        mid_point = cnt[mid_point_idx]
        plt.text(mid_point[0], mid_point[1], str(idx), color='yellow', fontsize=14)
    
    plt.title('Ablated Surfaces (Red), Top Layer (Green), and Depth (Yellow) on Depth Image')
    plt.show()
    
    # Capture only the content within the axes as an image
    plt.savefig(middle_image_filename)

    # 3. Select the ablated contour and finalize the width and depth
    fig, ax = plt.subplots()
    plt.imshow(frame_contours_with_reduced, cmap='gray')
    rs = RectangleSelector(ax, onselect, drawtype='box')
    plt.show()
    
    # Normalize the coordinates and dimensions
    center_x = (xmin + xmax) / 2 / img_width
    center_y = (ymin + ymax) / 2 / img_height
    width = (xmax - xmin) / img_width
    height = (ymax - ymin) / img_height

    class_id = 1  # Replace with the actual class ID if needed
    with open(txt_filename, 'w') as file:
        file.write(f"{class_id} {center_x} {center_y} {width} {height}\n")
    
    # Filter the contour points within the bounding box
    idx = np.logical_and.reduce((ablate_surface_combined[:, 0] >= xmin, ablate_surface_combined[:, 0] <= xmax,
                                 ablate_surface_combined[:, 1] >= ymin, ablate_surface_combined[:, 1] <= ymax))
    filtered_points = ablate_surface_combined[idx]
    
    # Sort by x and y coordinates
    sorted_by_x = filtered_points[np.argsort(filtered_points[:, 0])]
    sorted_by_y = filtered_points[np.argsort(filtered_points[:, 1])]
    point1, point2 = sorted_by_x[0], sorted_by_x[-1]
    point3 = sorted_by_y[-1]
    
    # Calculate distance and equations of lines
    distance = np.linalg.norm(point2 - point1)
    m = (point2[1] - point1[1]) / (point2[0] - point1[0])
    c = point1[1] - m * point1[0]
    m_perpendicular = -1 / m
    c_perpendicular = point3[1] - m_perpendicular * point3[0]
    x_intersection = (c_perpendicular - c) / (m - m_perpendicular)
    y_intersection = m * x_intersection + c
    perpendicular_distances = np.sqrt((x_intersection - point3[0])**2 + (y_intersection - point3[1])**2) * 0.25 / 54
    
    # Plotting
    plt.figure()
    plt.imshow(frame_contours_with_reduced, cmap='gray')
    plt.plot([point1[0], point2[0]], [point1[1], point2[1]], 'g', linewidth=2)
    plt.plot([point3[0], x_intersection], [point3[1], y_intersection], 'y', linewidth=2)
    plt.plot([point1[0], point2[0], point3[0]], [point1[1], point2[1], point3[1]], 'ro', markersize=10)
    plt.savefig(final_image_filename)
    plt.close()
    
    return depth_points, perpendicular_distances

# Clear all variables (not usually necessary in Python)
folderPath = 'data/salmone/'
filename = 'oct1.jpg'

depth_points, perpendicular_distances = depthEstimationFrom2D(folderPath, filename)

# Construct the filename for the final image
name, ext = filename.rsplit('.', 1)
final_image_filename = f"{folderPath}/{name}final.{ext}"

# Plotting the final_image
final_image = cv2.imread(final_image_filename)
final_image = cv2.cvtColor(final_image, cv2.COLOR_BGR2RGB)  # Convert from BGR to RGB

plt.figure()
plt.imshow(final_image)
plt.title('Ablated Surfaces (Red), Top Layer (Green), and Depth (Yellow) on Depth Image')
plt.show()
