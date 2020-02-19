% read image and convert it to a double
lena=imread('lena.jpg'); 
lena = double(lena);
% ---- Filters ---------------------- 
% filter to detect horizontal edges
gx = [1;0;-1]./sqrt(2);
% filter to detect vertical edges
gy = [1 0 -1]./sqrt(2);
% smoothing filter
F = [1 4 6 4 1; 4 16 24 16 4; 6 24 36 24 6; 4 16 24 16 4; 1 4 6 4 1];
F = F./norm(F);
% use 'conv2' and the filters given above to detect the edges in lena
final_lena = conv2(conv2(conv2(gx,gy),F),lena);

plotFilteredLenaQuiz(lena, gx, gy, F);
