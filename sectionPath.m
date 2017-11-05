%{
  Version = Test
  this version will be used for the analysis of the time efficiency of the algorithm
  
  this function will take any linear path between s and t, and break it up into an
  efficient number of smaller steps in order to make the path as straight as
  possible
%}

%t is the target position, q is the initial position given in its angles
%n = number of steps
%sNew is a list of new positions to be outputted
function sNew = sectionPath(t, qi, n)
  s = armFunction( qi, [0;0;0]);
  e = t - s;
  

  %n is the number of substeps we have per movement
 
  eN = e / n;
  
  sNew = [s,zeros(3, n)];         %set the first value to the initial position
    
  for a = 1:n
    sNew(:,a+1) = sNew(:,a) + eN; %the size of sNew is 3 x n+1
  end
    
end



