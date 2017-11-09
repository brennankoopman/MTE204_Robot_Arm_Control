% Moves the end effector to the arm between each iteration of the jacobian tranpose method,
% results in fractured pathing, but it is interesting


more off; % allows lots of output to the command window without having to press f
P = [0;0;0];    %reference point for drawing each arm
q = [0;20;70]; % in degrees
n = 25;

q = q*(pi/180); % now in radians

Q = [q];
s = armFunction(q, [0;0;0]) % function gets the end effector
init = s;
t = [-1;-1;0.4]; % the goal of where the end effector must be

M = transpose(s);
e = t - s;


for i = 1:1:n % runs the jacobian method n times

	e = t - s; % recalculates the direction that the arm is supposed to move towards

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
endfor
  
  X = [init(1),t(1)];
  Y = [init(2),t(2)];
  Z = [init(3),t(3)];

  for a = 1:n
    Arm = [P ,armFunction_midJoint(Q(:,a), [0;0;0]) , armFunction(Q(:,a),[0;0;0])];
   
    plot3(X,Y,Z, 'r','linewidth',3,'DisplayName',sprintf('Ideal Linear Path') );
    hold on;
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    view(15,45);
    xlabel('X', 'fontsize', 14, 'fontweight', 'bold');
    ylabel('Y', 'fontsize', 14, 'fontweight', 'bold');
    zlabel('Z', 'fontsize', 14, 'fontweight', 'bold');
    plot3(M([1:a],1),M([1:a],2),M([1:a],3), 'c','linewidth',3,'DisplayName',sprintf('path of arm') );
    plot3(Arm(1,:),Arm(2,:),Arm(3,:), 'bk', 'linewidth',2);
    %lgd = legend('show', 'location', 'northwest');
    pause(0.2);
    hold off;
  end


