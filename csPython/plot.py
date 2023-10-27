import numpy as np
import matplotlib.pyplot as plt

# Read data from 1y_1d.txt
with open('MATLABy_1d.txt', 'r') as file1:
    data1 = np.array([float(line.strip()) for line in file1.readlines()])

# Read data from newly uploaded y_1d.txt
with open('PYTHONy_1d.txt', 'r') as file2_new:
    data2_new = np.array([float(line.strip()) for line in file2_new.readlines()])

# Plotting
plt.figure(figsize=(12, 6))

# Plot for data from 1y_1d.txt
plt.subplot(2, 1, 1)
plt.plot(data1, label='Data from MATLAB', color='blue', marker='o', markersize=2, linestyle='', alpha=0.5)
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Data from MATLAB')
plt.legend()

# Plot for data from newly uploaded y_1d.txt
plt.subplot(2, 1, 2)
plt.plot(data2_new, label='Data from Python', color='green', marker='x', markersize=2, linestyle='', alpha=0.5)
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Data from Python')
plt.legend()

plt.tight_layout()
plt.show()
