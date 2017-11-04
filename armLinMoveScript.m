more off;
q = [0;-160;-45];
q = q*(pi/180);
t = [-1.6;0.8;-0.3];

Q = [q];    %list of all angles for each target point starting at the initial position

n=3;
Points = armLinMove(t,q,n)


n = n+1;

  %CALCULATES THE ANGLES FOR EACH TARGET POINT USING THE JACOBIAN METHOD
  for a = 1:n-1
     Q = [Q,getQ( Points(:,a+1), Q(:,a),0.001)]; % ERROR of 1 mm 
  end
  
 
 cPoints = [Points(:,1)] %list of points, with 100 points for each arc, ie target point, here we just set the first entry the initial position
  
  for a = 1:n-1                      %for each jump in angles
    
    dQ = Q(:,a+1) - Q(:,a) ;         %finds the change in angle needed for this step
    
    
    if( abs(dQ(1)) > pi ) 
      dQ(1) = -sign(dQ(1))*2*pi + dQ(1);
    end
    
    if( abs(dQ(2)) > pi ) 
      dQ(2) = -sign(dQ(2))*2*pi + dQ(2);
    end
    
    if( abs(dQ(3)) > pi ) 
      dQ(3) = -sign(dQ(2))*2*pi + dQ(3);
    end
    
    
    dq = dQ/30;                 %these are the incremental changes to q to show that the end effector does not move linearly
   
    
    for b = 1:30                    %find 100 points on the arc
      nu = armFunction( ( Q(:,a) + b*dq) , [0;0;0] );
      cPoints = [cPoints, nu];
    end
   
  end
  %cPoints

plot3(cPoints(1,:),cPoints(2,:),cPoints(3,:),'r','DisplayName',sprintf('Linear Rotation of Joints'),'linewidth',3); %plotting the endpoint between each point

hold on;  
plot3(Points(1,:),Points(2,:),Points(3,:),'g','DisplayName',sprintf('Ideal Path'),'linewidth',3);   %plotting the endpoint @ each target point

xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
title (sprintf('Example of Linearization of Joint Angles Between Targets'), 'fontsize', 14);
xlabel('X', 'fontsize', 14, 'fontweight', 'bold');
ylabel('Y', 'fontsize', 14, 'fontweight', 'bold');
zlabel('Z', 'fontsize', 14, 'fontweight', 'bold');
lgd = legend('show', 'location', 'northwest'); 

hold off;

