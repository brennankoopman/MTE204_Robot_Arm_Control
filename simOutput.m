
%{
q - the initial position in angles of deg
t - the target position in cartesian coords, col vector
Jerr - the end case error on the Jacobian transpose method
n = number of steps you break the linear movement into
Emax = the max positonal error in the end effector
CompTime = the worst case computation time of the algorithm

%}
function QdQP = simOutput(q, t, Jerr, Emax, exTime) 

%I need something here to validate the position of the arm, because it cannot fully stretch out

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
Points = sectionPath(t,q,n); %points is a 3 x n+1 matrix

  %CALCULATES THE ANGLES FOR EACH STEP POINT USING THE JACOBIAN TRANSPOSE METHOD
  for a = 1:n
     Q(:,a+1) = getQ( Points(:,a+1), Q(:,a),Jerr); % ERROR of Jerr
  end
  
 dQ = [zeros(3,n+1)];       %list of the difference between each set of Target angles
 
 
  for a = 1:n                       %for each jump in angles
    
    dQ(:,a+1) = Q(:,a+1) - Q(:,a);          %finds the change in angle needed for this step
    
    %it turns out that the rotations dont linearlize nicely when a path crosses
    %the refrence axis, since the change in angle from 350 -> 10 should be 20, 
    % NOT -340, thus a single dq value cannot exceed 180 deg, as the smaller 
    % rotation value is preffered. note that angles are in RADIANS
    
    if( abs(dQ(1,a))>pi) 
     dQ(1,a)= dQ(1,a)-2*pi*sign(dQ(1,a));  %takes the opposite of the sign of your dQ * 360, and adds your dQ to find your opposite rotation
    end
    
    if( abs(dQ(2,a))>pi) 
     dQ(2,a) = dQ(2,a)-2*pi*sign(dQ(2,a)); 
    end 
    
    if( abs(dQ(3,a))>pi) 
     dQ(3,a) = dQ(3,a)-2*pi*sign(dQ(3,a));  
    end
    
  end
 
 QdQP = [transpose(Q),transpose(dQ),transpose(Points)]; %process the matrices for output;
 
 

