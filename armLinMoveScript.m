more off;
q = [75;0;40];
q = q*(pi/180);
t = [0.2;-1;1.5];

Q = [q];    %list of all angles for each target point starting at the initial position

Points = armLinMove(t,q);


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

hold on;
plot3(cPoints(1,:),cPoints(2,:),cPoints(3,:),'r'); %plotting the true path between each point
 
plot3(Points(1,:),Points(2,:),Points(3,:),'g');   %plotting the desired path @ with each target point

xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);
xlabel ("x");
ylabel ("y");
zlabel ("Z");

hold off;

