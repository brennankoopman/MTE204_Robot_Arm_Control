% Moves the end effector to the 

more off; % allows lots of output to the command window without having to press f

q = [0;0;90]; % in degrees

q = q*(pi/180); % now in radians
initialQ = q; % stores the initial value of where the arm is

s = armFunction(q, [0;0;0]); % function gets the end effector
t = [1;1;1]; % the goal of where the end effector must be

M = transpose(s);
e = t - s;
for i = 1:1:100 % runs the jacobian method 30 times

	e = t - s; % recalculates the direction that the arm is supposed to move towards
%{
	if(norm(e) > 0.25)
		e = 0.25*e/norm(e);
	end
%}
	norm(e); % gets the distance from the target (remove semicolon to print value)

	J = Jacobian(q); % gets the partial derivatives of end effector co-ords wrt the angles
	JT = transpose(J); % gets the transpose to slightly reduce calc time by using more memory

	jjte = J*JT*e; % again reduces calc time

	a = (dot(e,jjte))/(dot(jjte,jjte)); % gets the amount of movement towards the goal using the Jacobian

	deltaQ = a*JT*e; % calcualtes the amount to move the joints towards the goal

	q = q + deltaQ; % moves the joints towards the goal

	s = armFunction(q, [0;0;0]); % gets the new position after the movement

	M = [M;transpose(s)];
endfor

%csvwrite('stuff.csv', M);
% Plots the path of the end effector
plot3(M(:,1),M(:,2),M(:,3));

xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);

finalQ = q;

s = armFunction(q, [0;0;0])

MAXSPEED = 0.5; % rads/s

%{
sprintf('final Pos');

s = armFunction(q, [0;0;0])
%}