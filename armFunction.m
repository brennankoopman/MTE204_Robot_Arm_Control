%this function calculates the postion of the end effector with the inputs of
%q, which is the joints angles of the arm
%P is the reference point of the arm in 3D space, typically (0,0,0)
function s = armFunction(q, P)

len = [0 1 1];
x = (len(2)*cos(q(2)) + len(3)*cos(q(2)-q(3)))*cos(q(1)) - P(1); % x

y = (len(2)*cos(q(2)) + len(3)*cos(q(2)-q(3)))*sin(q(1)) - P(2); % y

z = (len(2)*sin(q(2)) + len(3)*sin(q(2)-q(3))) - P(3); % z

s = [x;y;z];