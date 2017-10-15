
%{
this function will take any linear path between s and t, and break it up into an
efficient number of smaller steps in order to make the path as straight as
possible
%}

%t is the target position, q is the initial position given in is angles
%sNew is a list of new positions to be outputted to the CSV file
function sNew = armLinMove(t, qi)
  s = armFunction( qi, [0;0;0]);
  e = t - s;
  
  %assuming our scale is in meters, then lets start with movement every 2.5 cm
  %n is the number of subcuts we have per movment
  n = round( norm(e) / 0.6 );
  %n=1;
  eN = e / n;
  
  sNew = [s];
  for a = 1:n
   sNew = [sNew, s + a*eN];   
  end
    
end



%WIP
%{
we are clamping the maximum movement for a single step for 2 reasons
  
  1) if the movement is too large, convergence may not occur
  
  2) we want to be able to use the jacobian as a control mechanism as well so if we want the end effector to move in
      approxamitely straight lines, then we must break a straight path into segments of very small arcs, because moving the arm joints linearly
      from point to point will always cause an ark.
      
  3) if we want to have arcs, then it becomes much harder on how to decide how many subtargets we need to break the path into
  
  4) I think we could take the input form of the path as a parametric function, and scale the number of steps based on the curvature.
  
  

%}