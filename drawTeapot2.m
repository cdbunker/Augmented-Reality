function [] = drawTeapot2(sides, teapotMean, im, cameraParams,R,t)
    imshow(im)
    hold on
    T = -R*t;

    D = sqrt(sum((teapotMean-T').^2));
    [~,order] = sort(D,'descend');

    for j=1:length(sides)
        i = order(j);
        side = sides(:,:,i);
        side = worldToImage(cameraParams,R,t,side);
        fill(side(:,1),side(:,2),'r');
    end
    drawnow
    hold off
end

