clear;
close all;

%%
% Start
I = imread('villa_image_2.png');
I=im2double(I);
I = im2double(rgb2gray(I));
size=size(I);
figure(1),imshow(I),title('original image');
%%
%edges
edges = edge(I, 'canny');
figure(2), imshow([edges]),title('edges');
%%
%Harris measure
dx = [-1 0 1; -1 0 1; -1 0 1];   % Derivative masks
dy = dx';
Ix = conv2(I, dx, 'same');      % Image derivatives
Iy = conv2(I, dy, 'same');

SIGMA_gaussian=4;
g = fspecial('gaussian',max(1,fix(3*SIGMA_gaussian)+1), SIGMA_gaussian);

Ix2 = conv2(Ix.^2, g, 'same'); % Smoothed squared image derivatives
Iy2 = conv2(Iy.^2, g, 'same');
Ixy = conv2(Ix.*Iy, g, 'same');

% cim = det(M) - k trace(M)^2.
k = 0.04;
cim = (Ix2.*Iy2 - Ixy.^2) - k * (Ix2 + Iy2);
BORDER=20;
cim(1:BORDER,:)=0;
cim(end-BORDER:end,:)=0;
cim(:,end-BORDER:end)=0;
cim(:,1:BORDER)=0;
T=mean(cim(:));
CIM=cim;
CIM(find(cim<T))=0;
figure(3), imshow(CIM,[]),title('Harris measure');
figure(4), mesh(CIM),title('Harris measure diagram');
%%
%corners
support=true(50); %The population of the corners depends on this
maxima=ordfilt2(CIM,sum(support(:)),support);
[loc_x,loc_y]=find((cim==maxima).*(CIM>0));
indx = find((cim==maxima).*(CIM>0));
% draw a cross on the image in the local maxima
figure(5), imshow(I,[]), hold on,
plot(loc_y,loc_x,'g+', 'LineWidth', 4)
title('Local maxima of Harris measure')
%%
%prebuild function
corners = detectHarrisFeatures(I);
figure(6),imshow(I); hold on;
plot(corners.selectStrongest(300));


%%
%Hough transform
[H,theta,rho] = hough(edges);
figure(7)
imshow(imadjust(rescale(H)),[],...
       'XData',theta,...
       'YData',rho,...
       'InitialMagnification','fit');
xlabel('\theta (degrees)')
ylabel('\rho')
axis on
axis normal 
hold on
colormap(gca,hot)

P = houghpeaks(H,8,'threshold',ceil(0.3*max(H(:))));
x = theta(P(:,2));
y = rho(P(:,1));
plot(x,y,'s','color','black');
title('Hough transform') 
%%
%Find straight lines with Hough
lines = houghlines(edges,theta,rho,P,'FillGap',20,'MinLength',40);
figure(8), imshow(I),title('Straight Lines') ,hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','cyan');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'o','LineWidth',2,'Color','red');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','green');
end

