more off;
q = [180;0;90];
q = q*(pi/180);
t = [0;1;1];

Q = [q];    %list of all angles for each target point starting at the initial position

Points = armLinMove(t,q)


n = size(Points,2);

  %CALCULATES THE ANGLES FOR EACH TARGET POINT USING THE JACOBIAN METHOD
  for a = 1:n-1
     Q = [Q,getQ( Points(:,a+1), Q(:,a),0.001)]; % ERROR of 1 mm 
  end
  
 
 cPoints = [Points(:,1)] %list of points, with 100 points for each arc, ie target point, here we just set the first entry the initial position
  
  for a = 1:n-1                      %for each jump in angles
    
    dQ = Q(:,a+1) - Q(:,a) ;         %finds the change in angle needed for this step
    dq = dQ/10;                 %these are the incremental changes to q to show that the end effector does not move linearly
   
    
    for b = 1:10                    %find 100 points on the arc
      nu = armFunction( ( Q(:,a) + b*dq) , [0;0;0] );
      cPoints = [cPoints, nu];
    end
   
  end
  %cPoints

plot3(cPoints(1,:),cPoints(2,:),cPoints(3,:),'r'); %plotting the endpoint between each point

hold on;  
plot3(Points(1,:),Points(2,:),Points(3,:),'g');   %plotting the endpoint @ each target point

xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
xlabel ("x");
ylabel ("y");
zlabel ("Z");

hold off;

