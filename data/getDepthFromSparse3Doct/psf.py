import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def calculate_mean_filter_mask():
    num_folders = 5
    filter_masks = []

    for folder in range(1, num_folders + 1):
        true_data_path = f'{folder}/ScanNum_1_TrueData.csv'
        estimation_data_path = f'{folder}/ScanNum_2_Estimation.csv'

        # Load the datasets
        true_data = pd.read_csv(true_data_path, header=None)
        estimation_data = pd.read_csv(estimation_data_path, header=None)

        # Compute the FFT of both datasets
        fft_true_data = np.fft.fft2(true_data)
        fft_estimation_data = np.fft.fft2(estimation_data)

        # Create the filter mask and add to list
        filter_mask = fft_estimation_data / fft_true_data
        filter_masks.append(filter_mask)

    # Calculate the mean filter mask
    mean_filter_mask = np.mean(filter_masks, axis=0)
    return filter_masks, mean_filter_mask

def plot_filter_masks_histogram(filter_masks, mean_filter_mask):
    num_masks = len(filter_masks)

    # First Plot: Magnitudes
    fig1, axs1 = plt.subplots(2, 3, figsize=(15, 10))  # 2 rows, 3 columns for magnitudes
    for i in range(num_masks):
        row, col = divmod(i, 3)
        mask_values = np.abs(filter_masks[i]).flatten()  # Magnitude
        axs1[row, col].hist(mask_values, bins=30, color='blue', alpha=0.3)
        axs1[row, col].set_title(f'Filter Mask {i+1} Magnitude Histogram')
        axs1[row, col].set_xlabel('Magnitude')
        axs1[row, col].set_ylabel('Pixel Count')

    mean_mask_values = np.abs(mean_filter_mask).flatten()
    axs1[1, 2].hist(mean_mask_values, bins=30, color='red', alpha=0.3)
    axs1[1, 2].set_title(f'Mean Filter Mask Magnitude Histogram')
    axs1[1, 2].set_xlabel('Magnitude')
    axs1[1, 2].set_ylabel('Pixel Count')

    plt.tight_layout()
    plt.show()

    # Second Plot: Phases
    fig2, axs2 = plt.subplots(2, 3, figsize=(15, 10))  # 2 rows, 3 columns for phases
    for i in range(num_masks):
        row, col = divmod(i, 3)
        mask_phases = np.angle(filter_masks[i]).flatten()  # Phase
        axs2[row, col].hist(mask_phases, bins=30, color='green', alpha=0.7)
        axs2[row, col].set_title(f'Filter Mask {i+1} Phase Histogram')
        axs2[row, col].set_xlabel('Phase (radians)')
        axs2[row, col].set_ylabel('Pixel Count')

    mean_mask_phases = np.angle(mean_filter_mask).flatten()
    axs2[1, 2].hist(mean_mask_phases, bins=30, color='orange', alpha=0.7)
    axs2[1, 2].set_title(f'Mean Filter Mask Phase Histogram')
    axs2[1, 2].set_xlabel('Phase (radians)')
    axs2[1, 2].set_ylabel('Pixel Count')

    plt.tight_layout()
    plt.show()

def plot_filter_masks(filter_masks, mean_filter_mask):
    num_masks = len(filter_masks)
    pixel_indices = np.arange(filter_masks[0].size)  # Create an array of pixel indices

    # First Plot: Magnitudes
    fig1, axs1 = plt.subplots(2, 3, figsize=(15, 10))  # 2 rows, 3 columns for magnitudes
    for i in range(num_masks):
        row, col = divmod(i, 3)
        mask_values = np.abs(filter_masks[i]).flatten()  # Magnitude
        axs1[row, col].plot(pixel_indices, mask_values, color='blue', alpha=0.3)
        axs1[row, col].set_title(f'Filter Mask {i+1} Magnitude\nSize: {filter_masks[i].shape}')
        axs1[row, col].set_xlabel('Pixel Location')
        axs1[row, col].set_ylabel('Magnitude')

    mean_mask_values = np.abs(mean_filter_mask).flatten()
    axs1[1, 2].plot(pixel_indices, mean_mask_values, color='red', alpha=0.3)
    axs1[1, 2].set_title(f'Mean Filter Mask Magnitude\nSize: {mean_filter_mask.shape}')
    axs1[1, 2].set_xlabel('Pixel Location')
    axs1[1, 2].set_ylabel('Magnitude')

    plt.tight_layout()
    plt.show()

    # Second Plot: Phases
    fig2, axs2 = plt.subplots(2, 3, figsize=(15, 10))  # 2 rows, 3 columns for phases
    for i in range(num_masks):
        row, col = divmod(i, 3)
        mask_phases = np.angle(filter_masks[i]).flatten()  # Phase without unwrapping
        axs2[row, col].scatter(pixel_indices, mask_phases, color='green', alpha=0.7)
        axs2[row, col].set_title(f'Filter Mask {i+1} Phase\nSize: {filter_masks[i].shape}')
        axs2[row, col].set_xlabel('Pixel Location')
        axs2[row, col].set_ylabel('Phase (radians)')
        axs2[row, col].set_ylim([min(mask_phases), max(mask_phases)])  # Limit y-axis to min and max of mask_phases

    mean_mask_phases = np.angle(mean_filter_mask).flatten()
    axs2[1, 2].scatter(pixel_indices, mean_mask_phases, color='orange', alpha=0.7)
    axs2[1, 2].set_title(f'Mean Filter Mask Phase\nSize: {mean_filter_mask.shape}')
    axs2[1, 2].set_xlabel('Pixel Location')
    axs2[1, 2].set_ylabel('Phase (radians)')
    axs2[1, 2].set_ylim([min(mean_mask_phases), max(mean_mask_phases)])  # Limit y-axis to min and max of mean_mask_phases

    plt.tight_layout()
    plt.show()

def plot_filter_masks_realImag(filter_masks, mean_filter_mask):
    num_masks = len(filter_masks)

    # First Plot: Real Parts
    fig1, axs1 = plt.subplots(2, 3, figsize=(15, 10))  # 2 rows, 3 columns for real parts
    for i in range(num_masks):
        row, col = divmod(i, 3)
        real_part = np.real(filter_masks[i]).flatten()
        axs1[row, col].hist(real_part, bins=30, color='blue', alpha=0.3)
        axs1[row, col].set_title(f'Filter Mask {i+1} Real Part Histogram')
        axs1[row, col].set_xlabel('Real Value')
        axs1[row, col].set_ylabel('Pixel Count')

    mean_real_part = np.real(mean_filter_mask).flatten()
    axs1[1, 2].hist(mean_real_part, bins=30, color='red', alpha=0.3)
    axs1[1, 2].set_title(f'Mean Filter Mask Real Part Histogram')
    axs1[1, 2].set_xlabel('Real Value')
    axs1[1, 2].set_ylabel('Pixel Count')

    plt.tight_layout()
    plt.show()

    # Second Plot: Imaginary Parts
    fig2, axs2 = plt.subplots(2, 3, figsize=(15, 10))  # 2 rows, 3 columns for imaginary parts
    for i in range(num_masks):
        row, col = divmod(i, 3)
        imaginary_part = np.imag(filter_masks[i]).flatten()
        axs2[row, col].hist(imaginary_part, bins=30, color='green', alpha=0.7)
        axs2[row, col].set_title(f'Filter Mask {i+1} Imaginary Part Histogram')
        axs2[row, col].set_xlabel('Imaginary Value')
        axs2[row, col].set_ylabel('Pixel Count')

    mean_imaginary_part = np.imag(mean_filter_mask).flatten()
    axs2[1, 2].hist(mean_imaginary_part, bins=30, color='orange', alpha=0.7)
    axs2[1, 2].set_title(f'Mean Filter Mask Imaginary Part Histogram')
    axs2[1, 2].set_xlabel('Imaginary Value')
    axs2[1, 2].set_ylabel('Pixel Count')

    plt.tight_layout()
    plt.show()

# Calculate and plot the filter masks
filter_masks, mean_filter_mask = calculate_mean_filter_mask()
plot_filter_masks_realImag(filter_masks, mean_filter_mask)



# # Now, let's apply this mean filter mask to the FFT of the true data from folder '5'
# folder = '4'
# true_data_path = f'{folder}/ScanNum_1_TrueData.csv'
# estimation_data_path = f'{folder}/ScanNum_2_Estimation.csv'
#
# # Load the datasets
# true_data = pd.read_csv(true_data_path, header=None)
# estimation_data = pd.read_csv(estimation_data_path, header=None)
#
# # Compute the FFT of the true data
# fft_true_data = np.fft.fft2(true_data)
#
# # Apply the mean filter mask to the FFT of the true data
# filtered_fft_true_data = fft_true_data * mean_filter_mask
#
# # Convert back to spatial domain
# filtered_true_data = np.fft.ifft2(filtered_fft_true_data)
#
# # Plotting
# plt.figure(figsize=(12, 6))
#
# plt.subplot(1, 2, 1)
# plt.imshow(estimation_data, cmap='gray')
# plt.title('Estimation Data')
# plt.colorbar()
#
# plt.subplot(1, 2, 2)
# plt.imshow(np.abs(filtered_true_data), cmap='gray')  # Use the absolute value for visualization
# plt.title('Filtered True Data (Frequency Domain)')
# plt.colorbar()
#
# plt.tight_layout()
# plt.show()

# Note: The plotting will not be displayed here, but this will work in your local Python environment.



# import numpy as np
# import cv2
# import matplotlib.pyplot as plt
# from typing import Callable
#
# def load_csv_data(file_path: str) -> np.ndarray:
#     """Load data from a CSV file into a numpy array."""
#     return np.loadtxt(file_path, delimiter=',')
#
# def convert_to_uint8(image: np.ndarray) -> np.ndarray:
#     """Convert the image data to 8-bit unsigned integers."""
#     normalized_image = cv2.normalize(image, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
#     return np.uint8(normalized_image * 255)
#
# def apply_filter(image: np.ndarray, filter_func: Callable) -> np.ndarray:
#     """Apply a given filter function to an image."""
#     return filter_func(image)
#
# def custom_filter(image: np.ndarray) -> np.ndarray:
#     """Custom filter to replace values with two leading zeros after the decimal point,
#     followed by median and Gaussian filtering."""
#     filtered_image = image.copy()
#     rows, cols = image.shape
#
#     for i in range(1, rows - 1):
#         for j in range(1, cols - 1):
#             value = image[i, j]
#             if 0 < value < 0.1:
#                 neighbors = image[i-1:i+2, j-1:j+2]
#                 average = np.mean(neighbors)
#                 filtered_image[i, j] = average
#
#     # Apply median and Gaussian filters
#     filtered_image = gaussian_blur(filtered_image)
#     filtered_image = median_blur(filtered_image)
#
#     return filtered_image
#
#
# def gaussian_blur(image: np.ndarray) -> np.ndarray:
#     return cv2.GaussianBlur(image, (5, 5), 0)
#
# def median_blur(image: np.ndarray) -> np.ndarray:
#     return cv2.medianBlur(image, 5)
#
# def bilateral_filter(image: np.ndarray) -> np.ndarray:
#     return cv2.bilateralFilter(image, 9, 75, 75)
#
# def calculate_relative_error(truth: np.ndarray, estimation: np.ndarray) -> float:
#     """Calculate the relative error using the Frobenius norm."""
#     return np.linalg.norm(truth - estimation, 'fro') / np.linalg.norm(truth, 'fro')
#
# def plot_images_in_grid(images: dict, errors: dict, true_image: np.ndarray, estimation_image: np.ndarray, title: str):
#     """Plot a set of images along with the true and estimation images in a grid layout."""
#     plt.figure(figsize=(12, 18))
#     plt.subplot(3, 2, 1)
#     plt.imshow(true_image, cmap='gray')
#     plt.title("True Data")
#     plt.axis('off')
#
#     plt.subplot(3, 2, 2)
#     plt.imshow(estimation_image, cmap='gray')
#     plt.title("Estimation Data")
#     plt.axis('off')
#
#     for i, (filter_name, image) in enumerate(images.items(), 3):
#         plt.subplot(3, 2, i)
#         plt.imshow(image, cmap='gray')
#         plt.title(f"{filter_name}\nError: {errors[filter_name]:.2f}")
#         plt.axis('off')
#
#     plt.suptitle(title)
#     plt.tight_layout()
#     plt.show()
#
# # Load the True Data and Estimation data
# file_true_data = '5/ScanNum_1_TrueData.csv'  # Replace with the actual file path
# file_estimation = '5/ScanNum_2_Estimation.csv'  # Replace with the actual file path
#
# true_data = load_csv_data(file_true_data)
# estimation_data = load_csv_data(file_estimation)
#
# true_data_uint8 = convert_to_uint8(true_data)
# estimation_data_uint8 = convert_to_uint8(estimation_data)
#
# filtered_images_uint8 = {
#     "Custom Filter": apply_filter(true_data_uint8, custom_filter),
#     "Gaussian Blurring": apply_filter(true_data_uint8, gaussian_blur),
#     "Median Blurring": apply_filter(true_data_uint8, median_blur),
#     "Bilateral Filtering": apply_filter(true_data_uint8, bilateral_filter)
# }
#
# relative_errors = {filter_name: calculate_relative_error(filtered_image, estimation_data_uint8)
#                    for filter_name, filtered_image in filtered_images_uint8.items()}
#
# plot_images_in_grid(filtered_images_uint8, relative_errors, true_data_uint8, estimation_data_uint8, "Image Filters and Error Analysis")
