<<<<<<< HEAD:armLinMoveScript.m
more off;
q = [10;0;20];
q = q*(pi/180);
t = [.1;.2;1.8];

Q = [q];    %list of all angles for each target point starting at the initial position
n=5;
Points = sectionPath(t,q,n);


n = size(Points,2);

  %CALCULATES THE ANGLES FOR EACH TARGET POINT USING THE JACOBIAN METHOD
  for a = 1:n-1
     Q = [Q,getQ( Points(:,a+1), Q(:,a),0.001)]; % ERROR of 1 mm 
  end
  
 
 cPoints = [Points(:,1)]; %list of points, with 100 points for each arc, ie target point, here we just set the first entry the initial position
 
  for a = 1:n-1                       %for each jump in angles
    
    dQ = Q(:,a+1) - Q(:,a) ;          %finds the change in angle needed for this step
    
    %it turns out that the rotations dont linearlize nicely when a path crosses
    %the refrence axis, since the change in angle from 350 -> 10 should be 20, 
    % NOT -340, thus a single dq value cannot exceed 180 deg, as the smaller 
    % rotation value is preffered. note that angles are in RADIANS
    
    if( abs(dQ(1))>pi) 
     dQ(1) = dQ(1)-2*pi*sign(dQ(1));  %takes the opposite of the sign of your dQ * 360, and adds your dQ to find your opposite rotation
    end
    
    if( abs(dQ(2))>pi) 
     dQ(2) = dQ(2)-2*pi*sign(dQ(2)); 
    end 
    
    if( abs(dQ(3))>pi) 
     dQ(3) = dQ(3)-2*pi*sign(dQ(3));  %takes the opposite of the sign of your dQ * 360, and adds your dQ to find your opposite rotation
    end
     
    dq = dQ/40;                     %these are the incremental changes to q to show that the end effector does not move linearly
   
    
    for b = 1:40                   %find 100 points on the arc
      nu = armFunction( ( Q(:,a) + b*dq) , [0;0;0] );
      cPoints = [cPoints, nu];
    end
   
  end

  %cPoints

plot3(cPoints(1,:),cPoints(2,:),cPoints(3,:),'r','DisplayName',sprintf('Linear Rotation of Joints'),'linewidth',3); %plotting the endpoint between each point

hold on;
scatter3(Points(1,:),Points(2,:),Points(3,:),'g','DisplayName',sprintf('Ideal Path'),'linewidth',3);   %plotting the endpoint @ each target point

%plotting each iteration of the arm
for i = [1:1:n]  
Arm = [ [0;0;0] , armFunction_midJoint(Q(:,i), [0;0;0]) , armFunction(Q(:,i),[0;0;0])];
plot3(Arm(1,:),Arm(2,:),Arm(3,:), 'bk', 'linewidth', 2); 
end 



title (sprintf('Example of Linearization of Joint Angles Between Targets'), 'fontsize', 14);
xlabel('X', 'fontsize', 14, 'fontweight', 'bold');
ylabel('Y', 'fontsize', 14, 'fontweight', 'bold');
zlabel('Z', 'fontsize', 14, 'fontweight', 'bold');
lgd = legend('show', 'location', 'northwest'); 



xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);

hold off;

=======
more off;
q = [0;-160;-45];   %initial position in joint angles
q = q*(pi/180);
t = [1.5;0.8;-0.3];  %target point

Q = [q];                      %list of all angles for each target point starting at the initial position
n=15;                         %number of subintervals
Points = sectionPath(t,q,n);  %section the path into n subsection


n = size(Points,2);

  %CALCULATES THE ANGLES FOR EACH TARGET POINT USING THE JACOBIAN METHOD
  for a = 1:n-1
     Q = [Q,getQ( Points(:,a+1), Q(:,a),0.001)]; % ERROR of 1 mm 
  end
  
 
 cPoints = [Points(:,1)]; %list of points, with 100 points for each arc, ie target point, here we just set the first entry the initial position
 
  for a = 1:n-1                       %for each jump in angles
    
    dQ = Q(:,a+1) - Q(:,a) ;          %finds the change in angle needed for this step
    
    %it turns out that the rotations dont linearlize nicely when a path crosses
    %the refrence axis, since the change in angle from 350 -> 10 should be 20, 
    % NOT -340, thus a single dq value cannot exceed 180 deg, as the smaller 
    % rotation value is preffered. note that angles are in RADIANS
    
    if( abs(dQ(1))>pi) 
     dQ(1) = dQ(1)-2*pi*sign(dQ(1));  %takes the opposite of the sign of your dQ * 360, and adds your dQ to find your opposite rotation
    end
    
    if( abs(dQ(2))>pi) 
     dQ(2) = dQ(2)-2*pi*sign(dQ(2)); 
    end 
    
    if( abs(dQ(3))>pi) 
     dQ(3) = dQ(3)-2*pi*sign(dQ(3));  %takes the opposite of the sign of your dQ * 360, and adds your dQ to find your opposite rotation
    end
     
    dq = dQ/40;                     %these are the incremental changes to q to show that the end effector does not move linearly
   
    
    for b = 1:40                   %find 40 points on the arc
      nu = armFunction( ( Q(:,a) + b*dq) , [0;0;0] );
      cPoints = [cPoints, nu];
    end
   
  end

  %cPoints
  
  %I am trying to make a mock animation using continuously updating graph outputs
  %I want to use the pause funciton to delay each output_max_field_width
  %scale the pause rate based on distance of travel.
  %I want to continuously print the the ideal path, and each arm for each of the points on the arc
  

plot3(cPoints(1,:),cPoints(2,:),cPoints(3,:),'r','DisplayName',sprintf('Linear Rotation of Joints'),'linewidth',3); %plotting the endpoint between each point

hold on;  
plot3(Points(1,:),Points(2,:),Points(3,:),'g','DisplayName',sprintf('Ideal Path'),'linewidth',3);   %plotting the endpoint @ each target point
for c = 1:n
    Arm = [P ,armFunction_midJoint(Q(:,c),[0;0;0]), armFunction(Q(:,c),[0;0;0])];
    plot3(Arm(1,:),Arm(2,:),Arm(3,:), 'bk', 'linewidth',2,'marker','o'); 
end

xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
title (sprintf('Example of Linearization of Joint Angles Between Targets'), 'fontsize', 14);
xlabel('X', 'fontsize', 14, 'fontweight', 'bold');
ylabel('Y', 'fontsize', 14, 'fontweight', 'bold');
zlabel('Z', 'fontsize', 14, 'fontweight', 'bold');
lgd = legend('show', 'location', 'northwest'); 

hold off;

>>>>>>> 8f2a07d4bc6d42208b8866335a903bd9df93fac3:Pathing.m
