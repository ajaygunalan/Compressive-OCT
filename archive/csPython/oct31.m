clear all; close all; clc;
%%

% Read the CSV file into a table
data_table = readtable('..\data\3rdYeraReport\UniformRaster1.csv');

% Extract individual columns into variables
PosX = data_table.PosX;
PosY = data_table.PosY;
SurfaceValue = data_table.SurfaceValue;

% Count the number of data points (rows in table)
num_data_points = height(data_table);

% Display the count
disp(['Number of data points: ', num2str(num_data_points)]);

% Find and display limits for PosX
min_PosX = min(PosX);
max_PosX = max(PosX);
disp(['Min of PosX: ', num2str(min_PosX), ', Max of PosX: ', num2str(max_PosX)]);

% Find and display limits for PosY
min_PosY = min(PosY);
max_PosY = max(PosY);
disp(['Min of PosY: ', num2str(min_PosY), ', Max of PosY: ', num2str(max_PosY)]);

% Initialize a 34x34 matrix to hold SurfaceValue data
SurfaceValueMatrix = zeros(34, 34);

% Find and display limits for PosX and PosY
min_PosX = min(PosX); max_PosX = max(PosX);
min_PosY = min(PosY); max_PosY = max(PosY);

% Calculate the scaling factors based on observed min and max
scaleX = (34 - 1) / (max_PosX - min_PosX);
scaleY = (34 - 1) / (max_PosY - min_PosY);

% Set the offsets to center the coordinate system in the middle of the matrix
centerX = 17;
CenterY = 17;

% Populate the SurfaceValueMatrix
for i = 1:num_data_points
    % Convert PosX and PosY to Row and Col indices
    Col = round(PosX(i) * scaleX + centerX);
    Row = round(-PosY(i) * scaleY + CenterY); % Negate PosY because positive is upwards
    % Populate the matrix
    SurfaceValueMatrix(Row, Col) = SurfaceValue(i);
end

% Find the minimum and maximum values in the SurfaceValueMatrix
min_val = min(SurfaceValueMatrix(:));
max_val = max(SurfaceValueMatrix(:));

% Normalize the SurfaceValueMatrix
if max_val ~= min_val  % Check to avoid division by zero
    SurfaceValueMatrix = (SurfaceValueMatrix - min_val) / (max_val - min_val);
else
    disp('Max and Min values are the same, cannot normalize');
    SurfaceValueMatrix = SurfaceValueMatrix;  % Keep it as is
end
%% Ignore boundary values sicne data is corrupt here

% Find rows and columns where all values are zero
rowsWithAllZeros = ~any(SurfaceValueMatrix, 2);
colsWithAllZeros = ~any(SurfaceValueMatrix, 1);

% Remove rows and columns with all zero values
SurfaceValueMatrix(rowsWithAllZeros, :) = [];
SurfaceValueMatrix(:, colsWithAllZeros) = [];


rowsToRemoveStart = 3;
colsToRemoveStart = 3;
rowsToRemoveEnd = 3;
colsToRemoveEnd = 4;

SurfaceValueMatrix = SurfaceValueMatrix((1 + rowsToRemoveStart):(end - rowsToRemoveEnd), (1 + colsToRemoveStart):(end - colsToRemoveEnd));
Truth = SurfaceValueMatrix;
x = reshape(SurfaceValueMatrix, [], 1);
%% Sampling
% Create A
A_2dMask = true(size(SurfaceValueMatrix));
A_2dMask(2:2:end, :) = false;
A_2dMask(:, 2:2:end) = false;

% Linearise A
A_1dMask = reshape(A_2dMask, [], 1);
A_LinearIdx = find(A_1dMask == 1);
%% Get Measuremnts
y = x(A_LinearIdx);
%% Estimate depth

[Estimation, t] = csAj(A_2dMask, y);
results = RelErr(Truth, Estimation);


% Vectorize the matrices
Truth_vectorized = Truth(:);
Estimation_vectorized = Estimation(:);

% Compute the correlation coefficient
correlation_matrix = corrcoef(Truth_vectorized, Estimation_vectorized);

% The correlation coefficient is the (1, 2) and (2, 1) element of the correlation_matrix
correlation_coefficient = correlation_matrix(1, 2);

% Display the result
fprintf('The correlation coefficient between the two matrices is: %.2f\n', correlation_coefficient);
fprintf('The relative error between the two matrices is: %.2f\n', results);









%%

