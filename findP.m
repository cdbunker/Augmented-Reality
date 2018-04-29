function [R,t] = findP(imagePoints, worldPoints, K)
    % Compute homography.
    H = findHomography(imagePoints, worldPoints);
    h1 = H(:, 1);
    h2 = H(:, 2);
    h3 = H(:, 3);
    lambda1 = 1 / norm(inv(K') * h1);
    lambda2 = 1 / norm(inv(K') * h2);
    lambda3 = (lambda1+lambda2)/2;
    r1 = lambda1 * inv(K') * h1;
    r2 = lambda2 * inv(K') * h2;
    r3 = cross(r1, r2);
    R = [r1'; r2'; r3'];
    [U, ~, V] = svd(R);
    R = U * V';
    t = lambda3 * inv(K') * h3;
end
