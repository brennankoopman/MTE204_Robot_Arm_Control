
%{
q - the initial position in angles of deg
t - the target position in cartesian coords, col vector
Jerr - the end case error on the Jacobian transpose method
n = number of steps you break the linear movement into

%}
function time = simOutput(q, t, Jerr, n) 
tic                    %this records the innitial time stamp for the time trials
q = q*(pi/180);
Q = [q,zeros(3,n)];    %list of all angles for each target point starting at the initial position
Points = sectionPath(t,q,n);

  %CALCULATES THE ANGLES FOR EACH STEP POINT USING THE JACOBIAN TRANSPOSE METHOD
  for a = 1:n
     Q(:,a+1) = getQ( Points(:,a+1), Q(:,a),Jerr); % ERROR of 1 mm 
  end
  
 dQ = [zeros(3,n)];
 cPoints = [Points(:,1)]; %list of points, with 100 points for each arc, ie target point, here we just set the first entry the initial position
 
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
 
 %csvwrite('dQ.csv', transpose(dQ));
 %csvwrite('Q.csv', transpose(Q));
 
 time = toc;    %return time stamp for the time the method took
 

