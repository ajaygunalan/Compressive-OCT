function [data, info] = depthResponse
%Depth gives an empty data for oct_msgs/DepthResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'oct_msgs/DepthResponse';
[data.Depth, info.Depth] = ros.internal.ros.messages.std_msgs.float64;
info.Depth.MLdataType = 'struct';
info.MessageType = 'oct_msgs/DepthResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'depth';
info.MatPath{2} = 'depth.data';
