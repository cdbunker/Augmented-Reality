% https://ags.cs.uni-kl.de/fileadmin/inf_ags/3dcv-ws11-12/3DCV_WS11-12_lec04.pdf
% http://www.cse.psu.edu/~rtc12/CSE486/lecture16.pdf
% http://www.epixea.com/research/multi-view-coding-thesisse9.html

% Needs Computer Vision Toolbox and Mapping Toolbox

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
    
    imshow(im);
    hold on
    scatter(j2,i2,'ro');
    a = [1:5]';
    b = num2str(a);
    c = cellstr(b);
    text(j2+5, i2+5, c);
    drawnow
    

    drawBox(im, cameraParams,R,t)
    %drawTeapot2(teapotSides, teapotMean, im, cameraParams,R,t)
    
    M(count) = getframe; 
    g{count} = frame2im(M(count));

    count = count+1;
end
toc

%movie(M)

myVideo = VideoWriter('finalProduct2.avi');
uncompressedVideo = VideoWriter('myfile.avi', 'Uncompressed AVI');
myVideo.FrameRate = 15;  % Default 30
myVideo.Quality = 100;    % Default 75
open(myVideo);
writeVideo(myVideo, M);
close(myVideo);



filename = 'testAnimated2.gif'; % Specify the output file name
for idx = 1:count-1
    [A,map] = rgb2ind(g{idx},256);
    if idx == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',1);
    end
end



wp = worldPoints2D*50+100;
[H, inliers] = ransac(wp,imagePoints);

s = size(im);
boundaries = [1, s(2), s(2), 1; 1, 1, s(1), s(1),; 1, 1, 1, 1];
tl=H*boundaries(:,1);
tl=tl/tl(3);
tr=H*boundaries(:,2);
tr=tr/tr(3);
br=H*boundaries(:,3);
br=br/br(3);
bl=H*boundaries(:,4);
bl=bl/bl(3);
point_boundaries = [tl';tr';br';bl'];
new_boundaries = ceil([max(point_boundaries(:,1)), max(point_boundaries(:,2))]);

f = uint8(zeros(new_boundaries(2), new_boundaries(1), 1));

for x=1: new_boundaries(1)
    for y=1: new_boundaries(2)
        xy = [x, y, 1]';
        original_point = H\xy;
        original_point = round(original_point/original_point(3));
        
        if (original_point(1) < s(2) && original_point(2) < s(1))
            if (original_point(1) > 0 && original_point(2) > 0)
                f(y, x) = im(original_point(2), original_point(1));
            end
        end
    end
end



template2 = f;
templateBW = imbinarize(f,0.65);
templateBW = bwareafilt(templateBW,1);
stats = regionprops(templateBW,'BoundingBox');
bb = stats.BoundingBox;
template2 = template2(bb(2):bb(2)+bb(4),bb(1):bb(1)+bb(3));
imshow(template2);

points2 = detectBRISKFeatures(template2, 'MinQuality', 0.001,'MinContrast', 0.01);
[features2,vpts2] = extractFeatures(template2, points2);


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
    points1 = detectBRISKFeatures(im, 'MinQuality', 0.01,'MinContrast', 0.1);
    [features1,vpts1] = extractFeatures(im,points1);

    indexPairs = matchFeatures(features1,features2, 'MatchThreshold', 100);
    matchedPoints1 = vpts1(indexPairs(1:end, 1));
    matchedPoints2 = vpts2(indexPairs(1:end, 2));
    %showMatchedFeatures(im,template2,matchedPoints1,matchedPoints2,'montage');

    pts1 = matchedPoints1.Location;
    pts2 = matchedPoints2.Location;

    [H, inliers] = ransac(pts2,pts1);
    figure
    imshow(im)
    hold on
    plot(pts1(inliers,1), pts1(inliers,2))
    
    figure
    imshow(template2)
    hold on
    plot(pts2(inliers,1), pts2(inliers,2))
    
    showMatchedFeatures(im,template2,matchedPoints1(inliers),matchedPoints2(inliers),'montage');
% % %     
% % %     s = size(im);
% % %     boundaries = [1, s(2), s(2), 1; 1, 1, s(1), s(1),; 1, 1, 1, 1];
% % %     tl=H*boundaries(:,1);
% % %     tl=tl/tl(3);
% % %     tr=H*boundaries(:,2);
% % %     tr=tr/tr(3);
% % %     br=H*boundaries(:,3);
% % %     br=br/br(3);
% % %     bl=H*boundaries(:,4);
% % %     bl=bl/bl(3);
% % %     point_boundaries = [tl';tr';br';bl'];
% % %     new_boundaries = ceil([max(point_boundaries(:,1)), max(point_boundaries(:,2))]);
% % %     
% % %     f = uint8(zeros(new_boundaries(2), new_boundaries(1), 1));
% % %     
% % %     for x=1: new_boundaries(1)
% % %         x
% % %         for y=1: new_boundaries(2)
% % %             xy = [x, y, 1]';
% % %             original_point = H\xy;
% % %             original_point = round(original_point/original_point(3));
% % %             
% % %             if (original_point(1) < s(2) && original_point(2) < s(1))
% % %                 if (original_point(1) > 0 && original_point(2) > 0)
% % %                     f(y, x) = im(original_point(2), original_point(1));
% % %                 end
% % %             end
% % %         end
% % %     end
% % %     
% % %     imshow(f)

    [H, inliers] = ransac(pts1,pts2);
    [R,t] = findPfromH(K, H);
    drawBox2(im, cameraParams,R,t)
end