
function P = Jacobian(q)

q = [0 0 0];

len = [0 1 1];

%{
doesn't work
J = [-(len(2)*cos(q(2))+len(3)*cos(q(2)-q(3)))*sin(q(1))
(-len(2)*sin(q(2))-sin(q(2)-q(3)))*cos(q(1))
(len(2)*cos(q(2))-sin(q(2)-q(3)))*cos(q(1));

(len(2)*cos(q(2))+len(3)*cos(q(2)-q(3)))*cos(q(1))
(-len(2)*sin(q(2))-sin(q(2)-q(3)))*cos(q(1))
(len(2)*cos(q(2))-sin(q(2)-q(3)))*cos(q(1));


len(2)*sin(Jq(2))+len(3)*sin(q(2)-q(3)) 
len(2)*cos(q(2))+len(3)*cos(q(2)-q(3)) 
len(2)*sin(q(2))-len(3)*cos(q(2)-q(3))];
%}


dx1 = -(len(2)*cos(q(2))+len(3)*cos(q(2)-q(3)))*sin(q(1));
dx2 = (-len(2)*sin(q(2))-sin(q(2)-q(3)))*cos(q(1));
dx3 = (len(2)*cos(q(2))-sin(q(2)-q(3)))*cos(q(1));

dy1 = (len(2)*cos(q(2))+len(3)*cos(q(2)-q(3)))*cos(q(1));
dy2 = (-len(2)*sin(q(2))-sin(q(2)-q(3)))*cos(q(1));
dy3 = (len(2)*cos(q(2))-sin(q(2)-q(3)))*cos(q(1));


dz1 = len(2)*sin(q(2))+len(3)*sin(q(2)-q(3));
dz2 = len(2)*cos(q(2))+len(3)*cos(q(2)-q(3));
dz3 = len(2)*sin(q(2))-len(3)*cos(q(2)-q(3));

J = [dx1 dx2 dx3;
     dy1 dy2 dy3;
		 dz1 dz2 dz3];

