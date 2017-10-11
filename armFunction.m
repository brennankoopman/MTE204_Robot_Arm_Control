function W = armFunction(q, P)
% rearranged to minimize W1,W2,W3 

len = [0 1 1];
W1 = (len(2)*cos(q(2)) + len(3)*cos(q(2)-q(3)))*cos(q(1)) - P(1); % x

W2 = (len(2)*cos(q(2)) + len(3)*cos(q(2)-q(3)))*sin(q(1)) - P(2); % y

W3 = (len(2)*sin(q(2)) + len(3)*sin(q(2)-q(3))) - P(3); % z

W = [W1;W2;W3];