function [data, info] = num
%Num gives an empty data for simple_msgs/Num
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'simple_msgs/Num';
[data.Num_, info.Num_] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'simple_msgs/Num';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'num';
