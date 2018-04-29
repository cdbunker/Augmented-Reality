% https://ags.cs.uni-kl.de/fileadmin/inf_ags/3dcv-ws11-12/3DCV_WS11-12_lec04.pdf
% http://www.cse.psu.edu/~rtc12/CSE486/lecture16.pdf
% http://www.epixea.com/research/multi-view-coding-thesisse9.html
addpath('output_images_calibration')
img = imread('CAM0_image_0000.jpg');
imshow(img);
hold on;
[imagePoints,~] = detectCheckerboardPoints(img);
plot(imagePoints(:,1),imagePoints(:,2),'ro');

numImages = 26;
files = cell(1, numImages);
for i = 1:numImages
    files{i} = fullfile(sprintf('CAM0_image_%04d.jpg', i-1));
end

[imagePoints,boardSize] = detectCheckerboardPoints(files);

squareSize = 38.1; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

imageSize = [size(img, 1), size(img, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize);
         
K = cameraParams.IntrinsicMatrix;

[im, newOrigin] = undistortImage(img, cameraParams, 'OutputView', 'full');
figure; 
imshow(im);
title('Undistorted Image')

rmpath('output_images_calibration')
addpath('output_images_sequence')


template = rgb2gray(imread('Fiducial.jpg'));
templateSmall = imresize(rgb2gray(imread('Fiducial.jpg')),0.5);
numImages = 299;

worldPoints2D = [0,0; 0,5; 7,5; 7,2.5; 7,0];
worldPoints = [worldPoints2D, zeros(size(worldPoints2D,1),1)];




[verts, faces, ~] = teapotGeometry;
verts(:,1) = verts(:,1) + 2;
verts(:,2) = verts(:,2) + 2;
verts(:,3) = -verts(:,3);
teapotSides = zeros(4,3,length(faces));

for i = 1:length(faces)
    for j=1:4
        teapotSides(j,:,i) = verts(faces(i,j),:);
    end
end

teapotMean = mean(teapotSides);

count=1;
tic
for n = 1:numImages

    n
    if (n == 135)
       continue 
    end
    if (n == 142)
       continue 
    end

    Iname = sprintf('CAM0_image_%04d.jpg', n-1);
    I = rgb2gray(imread(Iname));

    [im, newOrigin] = undistortImage(I, cameraParams, 'OutputView', 'full');
    
    [i2, j2] = findPointsNCCorr(template,im);

    imagePoints = [j2,i2];
    [R,t] = findP(imagePoints, worldPoints2D, K);
    reproject = worldToImage(cameraParams,R,t,worldPoints);
    
    mssd = mean(sum((reproject-imagePoints).^2));
    if (mssd > 200)
        'y';
        [i2, j2] = findPointsNCCorr(templateSmall,im);
    end
    
    imagePoints = [j2,i2];
    [R,t] = findP(imagePoints, worldPoints2D, K);
%     
%     imshow(im);
%     hold on
%     scatter(j2,i2,'ro');
%     a = [1:5]';
%     b = num2str(a);
%     c = cellstr(b);
%     text(j2+5, i2+5, c);
%     drawnow
    

    %drawBox(im, cameraParams,R,t)
    drawTeapot2(teapotSides, teapotMean, im, cameraParams,R,t)
    
    M(count) = getframe; 
    g{count} = frame2im(M(count));

    count = count+1;
end
toc

movie(M)