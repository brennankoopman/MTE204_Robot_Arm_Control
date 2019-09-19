% Moves the end effector to the target by running the jacobian tranpose method a given number of times
% and visualizes it to test congervence cases.

more off; % allows lots of output to the command window without having to press f

iter = 200;   %number if iteration we want to run the method
P = [0;0;0];

q = [0;90;0]; % in degrees

q = q*(pi/180); % now in radians
initialQ = q; % stores the initial value of where the arm is

s = armFunction(q, [0;0;0]) % function gets the end effector
t = [0;2;0]; % the goal of where the end effector must be

M = transpose(s);
e = t - s;
Q = [q];

for i = 1:1:iter % runs the jacobian method 30 times

	
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
  Q = [Q,q];

	s = armFunction(q, [0;0;0]); % gets the new position after the movement

	M = [M;transpose(s)];   %convert the column vectors to row vectors for nicer outputs to a csv
  
  e = t - s; % recalculates the direction that the arm is supposed to move towards
endfor

norm(e)
 



%set up a matrix representing the ideal path
idPath = [armFunction(Q(:,1), [0;0;0]) , t];


% Plots the path of the end effector
plot3(M(:,1),M(:,2),M(:,3), 'c','linewidth',3,'DisplayName',sprintf('path of arm') );
hold on;
plot3(idPath(1,:),idPath(2,:),idPath(3,:), 'g', 'linewidth',3,'DisplayName',sprintf('ideal linear path') );

%plots the arm after each iteration

for i = [1:1:8]  
Arm = [ [0;0;0] , armFunction_midJoint(Q(:,i), [0;0;0]) , armFunction(Q(:,i),[0;0;0])];
plot3(Arm(1,:),Arm(2,:),Arm(3,:), 'bk');
end
xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);

title (sprintf('Example of Divergence'), 'fontsize', 14);
xlabel('X', 'fontsize', 14, 'fontweight', 'bold');
ylabel('Y', 'fontsize', 14, 'fontweight', 'bold');
zlabel('Z', 'fontsize', 14, 'fontweight', 'bold');
lgd = legend('show', 'location', 'northwest'); 
hold off;




