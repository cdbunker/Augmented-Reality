
function [x2, y2] = findTopLeft(x, y)
    cx = mean(x);
    cy = mean(y);

    a = atan2(y - cy, x - cx);

    [~, order] = sort(a);

    x2 = x(order);
    y2 = y(order);
    [~, U] = mink(x2, 2);
    [~, V] = mink(y2, 2);
    points = intersect(U,V);
    
    if ~isempty(points)
        x2 = circshift(x2, -points+1);
        y2 = circshift(y2, -points+1);
    end
end