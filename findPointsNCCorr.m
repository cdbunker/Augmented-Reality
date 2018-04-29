function [i2,j2] = findPointsNCCorr(template,im)
C = normxcorr2(gpuArray(template), gpuArray(im));
C = gather(C);
%Cmax = ordfilt2(C,121,ones(11,11));
Cmax = ordfilt2(C,9,ones(3,3));
Cdiff = Cmax-C;
ind = find(Cdiff == 0);
temp = zeros(size(C));
temp(ind)=1;
temp = temp.*C;
[~, ind2] = maxk(temp(:), 5);
[i,j] = ind2sub([size(C,1), size(C,2)],ind2);
i2 = i - size(template,1)/2;
j2 = j - size(template,2)/2;
[i2, j2] = findTopLeft(i2, j2);
end

