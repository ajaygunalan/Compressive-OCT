genDir = fullfile(pwd,'rosCustomMessages');
packagePath = fullfile(genDir,'simple_msgs');
mkdir(packagePath);
mkdir(packagePath,'msg');
%%
messageDefinition = {'int64 num'};

fileID = fopen(fullfile(packagePath,'msg', ...
               'Num.msg'),'w');
fprintf(fileID,'%s\n',messageDefinition{:});
fclose(fileID);
%%
mkdir(packagePath,'srv')
serviceDefinition = {'int64 a'
                     'int64 b'
                     '---'
                     'int64 sum'};
 
fileID = fopen(fullfile(packagePath,'srv', ...
               'AddTwoInts.srv'),'w');
fprintf(fileID,'%s\n',serviceDefinition{:});
fclose(fileID);
%%