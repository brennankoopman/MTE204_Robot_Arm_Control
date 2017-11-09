
%{
q - the initial position in angles of deg
t - the target position in cartesian coords, col vector
Jerr - the end case precision on the Jacobian transpose method
n = number of steps you break the linear movement into
Emax = the max positonal error in the end effector
CompTime = the worst case computation time of the algorithm
arcIter = number of points you take on the arc
%}

%User inputs based on client's required path, and restrictions
q = [0;-160;-45];
t = [1.5;0.8;-0.3];
Jerr = 0.001; %a jacobian absiolute error of 1 mm
Emax = 0.05;   %absolute worst positional error of 1 cm
exTime = 6; %worst case computation time in seconds
arcIter = 10;
q = q*(pi/180);     %convert q to radians


%Based on the report n has a max value, being the greater value between
%extime = 0.0111n
%n2 =< ceil(extime / 0.0111)

%norm(e) /n = Emax
%n1 => ceil(norm(e)/Emax)
s = armFunction(q,[0;0;0]);
e = t - s;      %this is the length of your linear path in the form of a vector

if(norm(t)>1.9 || norm(s) > 1.9) %if the arm is any of the divergent areas of its reach return and dont produce an output
  return
end

n1 = ceil(norm(e)/Emax);
n2 = ceil(exTime / 0.0111);
n=0;
if(n2 >= n1)
  n = n1
else
  return; %cannot calculate with the given specifications!!!
end

Q = [q,zeros(3,n)];    %list of all angles for each target point starting at the initial position
Points = sectionPath(t,q,n); %points is a 3 x n+1 matrix, it will track the ideal path of the arm 

  %CALCULATES THE ANGLES FOR EACH STEP POINT USING THE JACOBIAN TRANSPOSE METHOD
  for a = 1:n
     Q(:,a+1) = getQ( Points(:,a+1), Q(:,a),Jerr); % ERROR of Jerr
  end
  
 %dQ = [zeros(3,n+1)];       %list of the difference between each set of Target angles
 
 truePath_end = [Points(:,1),zeros(3,arcIter*n)]; %will hold the points for the end effector on the true path
 truePath_mid = [armFunction_midJoint(q,[0;0;0]),zeros(3,arcIter*n)]; %will hold the points for the midjoint on the true path
 
  for a = 1:n                       %for each jump in angles
    
    %dQ(:,a+1) = Q(:,a+1) - Q(:,a);          %finds the change in angle needed for this step
    dQ = Q(:,a+1) - Q(:,a);
    
    %it turns out that the rotations dont linearlize nicely when a path crosses
    %the refrence axis, since the change in angle from 350 -> 10 should be 20, 
    % NOT -340, thus a single dq value cannot exceed 180 deg, as the smaller 
    % rotation value is preffered. note that angles are in RADIANS
    
    if( abs(dQ(1))>pi) 
     dQ(1)= dQ(1)-2*pi*sign(dQ(1));  %takes the opposite of the sign of your dQ * 360, and adds your dQ to find your opposite rotation
    end
    
    if( abs(dQ(2))>pi) 
     dQ(2) = dQ(2)-2*pi*sign(dQ(2)); 
    end 
    
    if( abs(dQ(3))>pi) 
     dQ(3) = dQ(3)-2*pi*sign(dQ(3));  
    end
    
    dq = dQ/arcIter;                     %these are the incremental changes to q to show that the end effector does not move linearly
   
    %I LEFT OFF HERE
    for b = 1:arcIter                   %find arcIter number of points on the arc between the subsection of the ideal path
      nu = armFunction( ( Q(:,a) + b*dq) , [0;0;0] );
      cPoints = [cPoints, nu];
    end
    
  end
 
  
 
 

