%%
%get points
clear;
close all;
I = imread('villa_image_2.png');
I=im2double(I);
s=size(I)
figure(1);
imshow(I);
hold on;

%%
f = 2; % number of families of parallel lines
numSegmentsPerFamily = 2;
parallelLines =cell(f,1); % store parallel lines
fprintf(['Draw ', num2str(f) , ' families of parallel segments\n']);
col = 'rgbm';
for i = 1:f
    count = 1;
    parallelLines{i} = nan(numSegmentsPerFamily,3);
    while(count <=numSegmentsPerFamily)
        figure(gcf);
        title(['Draw ', num2str(numSegmentsPerFamily),' segments: step ',num2str(count) ]);
        segment1 = drawline('Color',col(i));
        parallelLines{i}(count, :) = segToLine(segment1.Position);
        count = count +1;
    end
    %fprintf('Press enter to continue\n');
    %pause
end
%%
V = nan(2,f);
for i =1:f
    A = parallelLines{i}(:,1:2);
    B = -parallelLines{i}(:,3);
    V(:,i) = A\B;
end
%%
imLinfty = fitline(V);
imLinfty = imLinfty./(imLinfty(3));

figure(2);
hold all;
for i = 1:f
    plot(V(1,i),V(2,i),'o','Color',col(i),'MarkerSize',20,'MarkerFaceColor',col(i));
end
hold all;
imshow(I);

%%
%build the rectification matrix
H = [eye(2),zeros(2,1); imLinfty(:)'];
% we can check that H^-T* imLinfty is the line at infinity in its canonical
% form:
fprintf('The vanishing line is mapped to:\n');
disp(inv(H)'*imLinfty);
%%
%rectify the image and show the result
tform = projective2d(H');
Iaff = imwarp(I,tform);

figure(3);
imshow(Iaff);
 %%
% %metric transform
% numConstraints = 2; % 2 is the minimum number
% hold all;
% fprintf('Draw pairs of orthogonal segments\n');
% count = 1;
% A = zeros(numConstraints,3);
% % select pairs of orthogonal segments
% while (count <=numConstraints)
%     figure(gcf);
%     title('Select pairs of orthogonal segments')
%     col = 'rgbcmykwrgbcmykw';
%     segment1 = drawline('Color',col(count));
%     segment2 = drawline('Color',col(count));
% 
%     l = segToLine(segment1.Position);
%     m = segToLine(segment2.Position);
% 
%     % each pair of orthogonal lines gives rise to a constraint on s
%     % [l(1)*m(1),l(1)*m(2)+l(2)*m(1), l(2)*m(2)]*s = 0
%     % store the constraints in a matrix A
%      A(count,:) = [l(1)*m(1),l(1)*m(2)+l(2)*m(1), l(2)*m(2)];
% 
%     count = count+1;
% end
% %%
% %S = [x(1) x(2); x(2) 1];
% [~,~,v] = svd(A);
% s = v(:,end); %[s11,s12,s22];
% S = [s(1),s(2); s(2),s(3)];
% 
% %%
% imDCCP = [S,zeros(2,1); zeros(1,3)]; % the image of the circular points
% [U,D,V] = svd(S);
% A = U*sqrt(D)*V';
% H = eye(3);
% H(1,1) = A(1,1);
% H(1,2) = A(1,2);
% H(2,1) = A(2,1);
% H(2,2) = A(2,2);
% 
% Hrect = inv(H);
% Cinfty = [eye(2),zeros(2,1);zeros(1,3)];
% 
% tform = projective2d(Hrect');
% Iaffmetr = imwarp(Iaff,tform);
% 
% figure(4);
% imshow(Iaffmetr);

%%
function [l] = segToLine(pts)
% convert the endpoints of a line segment to a line in homogeneous
% coordinates.
%
% pts are the endpoits of the segment: [x1 y1;
%                                       x2 y2]

% convert endpoints to cartesian coordinates
a = [pts(1,:)';1];
b = [pts(2,:)';1];
l = cross(a,b);
l = l./norm(l);
end
