
%this function calculates the postion of the midjoint with the inputs of
%q, which is the joints angles of the arm
%P is the reference point of the arm in 3D space, typically (0,0,0)

function [smid] = armFunction_midJoint (q, P)

y = sin(q(1)) - P(1);
x = cos(q(2))*cos(q(1)) - P(2);
z = sin(q(2)) - P(3);

smid = [x;y;z];

endfunction
