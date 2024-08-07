clc
clear all
close all
% 定义两个矩阵a和b，它们具有相同的行数，但不同的列数
a = [1 2 3; 4 5 6];
b = [7 8; 9 10];

% 创建一个空的结果矩阵，准备存放组合后的数据
result = zeros(size(a, 1), size(a, 2) + size(b, 2));

% 按照指定的顺序填充数据
for i = 1:size(a, 1)
    result(i, 1:size(a, 2)) = a(i, :);
    result(i, size(a, 2)+1:end) = b(i, :);
end

% 显示结果
disp(result);

matrix = [0 1 0 2 3; 4 5 6 7 8; 9 10 11 12 13];

% 将矩阵按每行元素排列后转换成一个行向量
rowVector = matrix(:).';

% 显示结果
disp(rowVector);


% 假设这是你的矩阵
% matrixToSave = [1 2 3; 4 5 6; 7 8 9];
matrixToSave = [1; 2; 3; 4; 5; 6; 7; 8; 9];


% 指定文件名
filename = 'output.dat';

% 打开文件进行写入，'w' 表示覆盖模式
fileID = fopen(filename, 'w');

if fileID == -1
    error('无法打开文件进行写入');
else
    % 将矩阵直接写入.dat文件
    fwrite(fileID, matrixToSave, 'float'); % 'double' 表示矩阵的数据类型
    
    % 关闭文件
    fclose(fileID);
end

% 打开.DAT文件
fileID = fopen('output.dat','rb');
% 读取数据
data = fread(fileID,Inf,'float');
% 关闭文件
fclose(fileID);

% 打开.DAT文件
fileID1 = fopen('mc_flip.dat','rb');
% 读取数据
data1 = fread(fileID1,Inf,'float');
% 关闭文件
fclose(fileID1);