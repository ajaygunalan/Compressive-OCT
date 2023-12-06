clear all; clc; close all;

% Example values for the size of the matrix, numOfInterval, and padSize
rows = 10; 
cols = 150;
intervalSize = 1;
padSize = 1;

% Create an n by m random matrix
M = rand(rows, cols);


newMatrix = [];
for startIdx = 1:intervalSize:size(M, 2)
    % Intervals 
    endIdx = startIdx + intervalSize - 1;
    selectedColumns = M(:, startIdx:endIdx);
    newMatrix = [newMatrix, selectedColumns];
    % Pad Zeros
    zeroColumns = zeros(rows, padSize);
    newMatrix = [newMatrix, zeroColumns];
end