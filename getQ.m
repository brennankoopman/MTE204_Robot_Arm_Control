function[q] = getQ(target, initialQ, error)

q = initialQ;

s = armFunction(q, [0;0;0]); % function gets the end effector
t = [1;1;1]; % the goal of where the end effector must be

e = t - s;

i = 0;

while(norm(e) > error | i < 30) % runs the jacobian method 30 times
i = i+1;

e = t - s; % recalculates the direction that the arm is supposed to move towards
J = Jacobian(q); % gets the partial derivatives of end effector co-ords wrt the angles
JT = transpose(J); % gets the transpose to slightly reduce calc time by using more memory
jjte = J*JT*e; % again reduces calc time
a = dot(e,jjte)/dot(jjte,jjte); % gets the amount of movement towards the goal using the Jacobian
deltaQ = a*JT*e; % calcualtes the amount to move the joints towards the goal

q = q + deltaQ; % moves the joints towards the goal
s = armFunction(q, [0;0;0]); % gets the new position after the movement
end

if(i ~= 30)
norm(e)
end