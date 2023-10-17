import cv2
import numpy as np
import matplotlib.pyplot as plt

# Declare and initialize global variables
roi_selected = False
x1, y1, x2, y2 = 0, 0, 0, 0

def plot_mapping(image1, image2, points1, points2, affine_matrix, line_thickness=3, num_random_points=100):
    fig, ax = plt.subplots(1, 2, figsize=(15, 7))
    
    # Plot triangle ABC in both images with increased line thickness
    ax[0].imshow(image1)
    ax[0].plot([points1[0][0], points1[1][0]], [points1[0][1], points1[1][1]], 'g-', linewidth=line_thickness)
    ax[0].plot([points1[1][0], points1[2][0]], [points1[1][1], points1[2][1]], 'g-', linewidth=line_thickness)
    ax[0].plot([points1[2][0], points1[0][0]], [points1[2][1], points1[0][1]], 'g-', linewidth=line_thickness)
    ax[0].scatter(points1[:, 0], points1[:, 1], c='r')
    ax[0].set_title('OCT Image with Triangle ABC')

    ax[1].imshow(image2)
    ax[1].plot([points2[0][0], points2[1][0]], [points2[0][1], points2[1][1]], 'g-', linewidth=line_thickness)
    ax[1].plot([points2[1][0], points2[2][0]], [points2[1][1], points2[2][1]], 'g-', linewidth=line_thickness)
    ax[1].plot([points2[2][0], points2[0][0]], [points2[2][1], points2[0][1]], 'g-', linewidth=line_thickness)
    ax[1].scatter(points2[:, 0], points2[:, 1], c='r')
    ax[1].set_title('Calm Image with Triangle ABC')
    
    # Plot random points and the rest of the elements as before
    height, width, _ = image1.shape
    random_points = np.random.randint([0, 0], [width, height], size=(num_random_points, 2))
    random_points_homo = np.hstack((random_points, np.ones((num_random_points, 1))))
    transformed_random_points = random_points_homo.dot(np.vstack((affine_matrix, [0, 0, 1])).T)[:, :2]
    
    for i, (x, y) in enumerate(random_points):
        ax[0].text(x, y, str(i+1), color='blue')
        
    for i, (x, y) in enumerate(transformed_random_points):
        ax[1].text(x, y, str(i+1), color='blue')
        
    plt.show()

def draw_roi(event, x, y, flags, param):
    global x1, y1, x2, y2, roi_selected
    if event == cv2.EVENT_LBUTTONDOWN:
        x1, y1 = x, y
        roi_selected = False
    elif event == cv2.EVENT_LBUTTONUP:
        x2, y2 = x, y
        roi_selected = True

def convert_to_original_coordinates(centers, radii, crop_start_x, crop_start_y, zoom_factor):
    original_centers = []
    original_radii = []
    for center, radius in zip(centers, radii):
        orig_x = int(center[0] / zoom_factor + crop_start_x)
        orig_y = int(center[1] / zoom_factor + crop_start_y)
        orig_radius = int(radius / zoom_factor)
        original_centers.append([orig_x, orig_y])
        original_radii.append(orig_radius)
    return original_centers, original_radii

def process_image(image_path):
    global roi_selected, x1, y1, x2, y2
    image = cv2.imread(image_path)
    cv2.namedWindow("Select ROI")
    cv2.setMouseCallback("Select ROI", draw_roi)
    data = {}
    
    while True:
        img_copy = image.copy()
        if roi_selected:
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.imshow("Select ROI", img_copy)
        key = cv2.waitKey(1)
        if key == ord('n'):
            break

    if roi_selected and x1 != x2 and y1 != y2:
        crop_start_x, crop_start_y = min(x1, x2), min(y1, y2)
        roi = image[crop_start_y:max(y1, y2), crop_start_x:max(x1, x2)]
        zoom_factor = 4
        zoomed_roi = cv2.resize(roi, None, fx=zoom_factor, fy=zoom_factor, interpolation=cv2.INTER_CUBIC)
        
        centers = [[zoomed_roi.shape[1]//4, zoomed_roi.shape[0]//4], [zoomed_roi.shape[1]//2, zoomed_roi.shape[0]//2], [3*zoomed_roi.shape[1]//4, 3*zoomed_roi.shape[0]//4]]
        radii = [7, 7, 7]
        
        for i in range(3):
            while True:
                zoomed_roi_copy = zoomed_roi.copy()
                for j in range(3):
                    cv2.circle(zoomed_roi_copy, tuple(centers[j]), radii[j], (0, 255, 0), 2)
                cv2.imshow("Adjust Circles", zoomed_roi_copy)
                
                key = cv2.waitKey(1)
                if key == ord('w'):
                    centers[i][1] -= 1
                elif key == ord('s'):
                    centers[i][1] += 1
                elif key == ord('a'):
                    centers[i][0] -= 1
                elif key == ord('d'):
                    centers[i][0] += 1
                elif key == ord('i'):
                    radii[i] += 1
                elif key == ord('j'):
                    radii[i] -= 1
                elif key == ord('o'):
                    break
        
        original_centers, original_radii = convert_to_original_coordinates(centers, radii, crop_start_x, crop_start_y, zoom_factor)
        
        data["original_centers"] = original_centers
        data["original_radii"] = original_radii
        
        cv2.destroyAllWindows()
    
    return data

data_calm = process_image("calmvideo.png")
data_oct = process_image("octvideo.jpg")

calm_image = cv2.imread("calmvideo.png")
oct_image = cv2.imread("octvideo.jpg")

# data_calm = {'original_centers': [[352, 285], [357, 320], [389, 314]], 'original_radii': [1, 1, 1]}
# data_oct = {'original_centers': [[372, 183], [362, 281], [449, 296]], 'original_radii': [10, 9, 10]}

# Extract points
points_calm = np.float32(data_calm['original_centers'])
points_oct = np.float32(data_oct['original_centers'])

# Compute the affine transformation matrix
affine_matrix = cv2.getAffineTransform(points_oct, points_calm)
plot_mapping(oct_image, calm_image, points_oct, points_calm, affine_matrix)







