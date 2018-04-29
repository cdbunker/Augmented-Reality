function [] = drawBox(im, cameraParams,R,t)
boxSide1 = [2,2,0; 4,2,0; 4,4,0; 2,4,0];
boxSide2 = [2,2,0; 4,2,0; 4,2,-2; 2,2,-2];
boxSide3 = [4,2,0; 4,4,0; 4,4,-2; 4,2,-2];
boxSide4 = [2,4,0; 4,4,0; 4,4,-2; 2,4,-2];
boxSide5 = [2,2,0; 2,4,0; 2,4,-2; 2,2,-2];
boxSide6 = [2,2,-2; 4,2,-2; 4,4,-2; 2,4,-2];
imshow(im)
hold on

u1=mean(boxSide1);
u2=mean(boxSide2);
u3=mean(boxSide3);
u4=mean(boxSide4);
u5=mean(boxSide5);
u6=mean(boxSide6);

T = -R*t;

d1=sqrt(sum((u1-T').^2));
d2=sqrt(sum((u2-T').^2));
d3=sqrt(sum((u3-T').^2));
d4=sqrt(sum((u4-T').^2));
d5=sqrt(sum((u5-T').^2));
d6=sqrt(sum((u6-T').^2));

[~,order] = sort([d1,d2,d3,d4,d5,d6],'descend');

side1 = worldToImage(cameraParams,R,t,boxSide1);
side2 = worldToImage(cameraParams,R,t,boxSide2);
side3 = worldToImage(cameraParams,R,t,boxSide3);
side4 = worldToImage(cameraParams,R,t,boxSide4);
side5 = worldToImage(cameraParams,R,t,boxSide5);
side6 = worldToImage(cameraParams,R,t,boxSide6);

sides = zeros(4,2,6);
sides(:,:,1) = side1;
sides(:,:,2) = side2;
sides(:,:,3) = side3;
sides(:,:,4) = side4;
sides(:,:,5) = side5;
sides(:,:,6) = side6;

hold on
for j=1:6
    i = order(j);
    switch i
        case 1
            fill(sides(:,1,i),sides(:,2,i),'r');
        case 2
            fill(sides(:,1,i),sides(:,2,i),'g');
        case 3
            fill(sides(:,1,i),sides(:,2,i),'b');
        case 4
            fill(sides(:,1,i),sides(:,2,i),'c');
        case 5
            fill(sides(:,1,i),sides(:,2,i),'m');
        otherwise
            fill(sides(:,1,i),sides(:,2,i),'w');
    end
end
% 
% fill(side1(:,1),side1(:,2),'r');
% fill(side2(:,1),side2(:,2),'g');
% fill(side3(:,1),side3(:,2),'b');
% fill(side4(:,1),side4(:,2),'c');
% fill(side5(:,1),side5(:,2),'m');
% fill(side6(:,1),side6(:,2),'w');

drawnow
hold off
end
