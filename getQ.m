% uses the Jacobian transpose method to get the joint angles q that bring the end effector within error distance of target

function[q] = getQ(target, initialQ, error)

q = initialQ;

s = armFunction(q, [0;0;0]); % function gets the end effector
t = target; % the goal of where the end effector must be

e = t - s;

i = 0;

while(i < 2000 && norm(e) > error) % runs the jacobian method 200 times
	i = i+1;
	
	% recalculates the direction that the arm is supposed to move towards
	e = t - s;
	
	% gets the partial derivatives of end effector co-ords wrt the angles
	J = Jacobian(q);
	% gets the transpose to slightly reduce calc time by using more memory
	JT = transpose(J);
	% again reduces calc time
	jjte = J*JT*e;
	% gets the amount of movement towards the goal using the Jacobian
	a = (dot(e,jjte))/(dot(jjte,jjte));
	% calcualtes the amount to move the joints towards the goal
	deltaQ = a*JT*e;
	
	% moves the joints towards the goal
	q = q + deltaQ;
	% gets the new position after the movement
	s = armFunction(q, [0;0;0]);
end

s
