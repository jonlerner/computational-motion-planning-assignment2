function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

% Sanity check
if (~isequal(size(P1), [3 2]) || ~isequal(size(P2), [3 2]))
    flag = false;
    return;
end

% For each edge, determine if it separates all other vertices from the two
% different triangles.
if (checkSeparated(P1(1,:), P1(2,:), P1(3,:), P2) || ...
    checkSeparated(P1(1,:), P1(3,:), P1(2,:), P2) || ...
    checkSeparated(P1(2,:), P1(3,:), P1(1,:), P2) || ...
    checkSeparated(P2(1,:), P2(2,:), P2(3,:), P1) || ...
    checkSeparated(P2(1,:), P2(3,:), P2(2,:), P1) || ...
    checkSeparated(P2(2,:), P2(3,:), P2(1,:), P1))
    flag = false;
    return;
end
flag = true;
% *******************************************************************
end

function separated = checkSeparated(v1, v2, T1, T2)
% v1 = 1st vertix of edge
% v2 = 1st vertix of edge
% T1 = vertices of 1st triangle excluding edge
% T2 = vertices of 2nd triangle excluding edge

% Determine edge equation
% y = mx + b
m = (v2(2) - v1(2)) / (v2(1) - v1(1));
b = v1(2) - m * v1(1);

% check if all vertices of T1 are on the same side of the edge
direction1 = zeros(size(T1,1), 1);
for i = 1:size(T1,1)
    point = T1(i,:);
    y = m * point(1) + b;  % y point on edge corresponding to point's x
    direction1(i) = point(2) >= y;
end

direction2 = zeros(size(T2,1), 1);
for i = 1:size(T2,1)
    point = T2(i,:);
    y = m * point(1) + b;  % y point on edge corresponding to point's x
    direction2(i) = point(2) >= y;
end

separated = all(direction1 == direction1(1)) && ...
            all(direction2 == direction2(1)) && ...
            direction1(1) ~= direction2(1);

end