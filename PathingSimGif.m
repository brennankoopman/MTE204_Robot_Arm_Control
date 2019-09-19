more off;
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
%q = [0;-120;45];
%t = [1.5;1.2;-0.3];
%Jerr = 0.001;     %a jacobian absiolute error of 1 mm
%Emax = 1.2;      %absolute worst positional error of 1 cm
%exTime = 20;       %worst case computation time in seconds
%arcIter = 10;
%q = q*(pi/180);     %convert q to radians
%P = [0;0;0];

%try this input for an invisible christmas tree with red ribbon and an extreme diverging case
%q3 just gets very very large in this case
%------------------------------------------------------------
q = [0;90;0];
t = [0;2;0];
Jerr = 0.001;     %a jacobian absiolute error of 1 mm
Emax = 6;      %absolute worst positional error of 1 cm
exTime = 4000;       %worst case computation time in seconds
arcIter = 140;
q = q*(pi/180);     %convert q to radians
%-------------------------------------------

%Based on the report n has a max value, being the greater value between
%extime = 0.0111n
%n2 =< ceil(extime / 0.0111)

%norm(e) /n = Emax
%n1 => ceil(norm(e)/Emax)
s = armFunction(q,[0;0;0]);
e = t - s;      %this is the length of your linear path in the form of a vector

%if(norm(t)>1.9 || norm(s) > 1.9) %if the arm is in any of the divergent areas of its reach return and dont produce an output
%  return
%end

n1 = ceil(norm(e)/Emax);
n2 = ceil(exTime / 0.0111);
n=0;
if(n2 >= n1)
  n = n1
else
  n1
  n2      %some sort of error output to console or message to the real time system
  return; %cannot calculate with the given specifications!!!
end

time = 1/(4*arcIter^2*n^2);    %scale the printing time to make the sim look nice

Q = [q,zeros(3,n)];    %list of all angles for each target point starting at the initial position
Points = sectionPath(t,q,n); %points is a 3 x n+1 matrix, it will track the ideal path of the arm 

  %CALCULATES THE ANGLES FOR EACH STEP POINT USING THE JACOBIAN TRANSPOSE METHOD
  for a = 1:n
     Q(:,a+1) = getQ( Points(:,a+1), Q(:,a),Jerr); % ERROR of Jerr
  end
  
 
 truePath_end = [Points(:,1),zeros(3,arcIter*n)]; %will hold the points for the end effector on the true path
 truePath_mid = [armFunction_midJoint(q,[0;0;0]),zeros(3,arcIter*n)]; %will hold the points for the midjoint on the true path
 
  for a = 1:n                       %for each jump in angles
    
    %dQ(:,a+1) = Q(:,a+1) - Q(:,a);          %finds the change in angle needed for this step
    dQ = Q(:,a+1) - Q(:,a);
    
    %it turns out that the rotations dont linearlize nicely when a joint angle crosses
    %the refrence axis, since the change in angle from 350 -> 10 should be 20 degrees, 
    % NOT -340, thus a single dq value cannot exceed 180 deg, as the smaller 
    % rotation value is preffered. note that angles are in RADIANS in the sim
    
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
   
   
    for b = 1:arcIter                   %find arcIter number of points on the arc between the subsection of the ideal path
      truePath_end(:, (a-1)*arcIter+b+1) = armFunction( ( Q(:,a) + b*dq) , [0;0;0] );
      truePath_mid(:, (a-1)*arcIter+b+1) = armFunction_midJoint( ( Q(:,a) + b*dq) , [0;0;0] );
    end
    
  end
  
  %--------------------------------------------------------------------
  %PLOTTING MARGARET
  %--------------------------------------------------------------------
  
  for c = 1:n*arcIter+1  %the +1 us to include the print of the initial position
    %N is the subsection we are surrently printing
    
    N = ceil(c/arcIter);
    axis tight manual; % this ensures that getframe() returns a consistent size
    
    %print the ideal path up to each current target point with points and lines
    plot3(Points(1,[1:N]),Points(2,[1:N]),Points(3,[1:N]), 'g','linewidth',1,'DisplayName',sprintf('Ideal Path with Target Points'),'marker','o','markersize',4, 'MarkerEdgeColor',[0 0.7 0]);
    
    %set up the calibration of each plot
    hold on;
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    view(-100,15+c);
    xlabel('X', 'fontsize', 14, 'fontweight', 'bold');
    ylabel('Y', 'fontsize', 14, 'fontweight', 'bold');
    zlabel('Z', 'fontsize', 14, 'fontweight', 'bold');
    
    %plot the current arm
    Arm = [P ,truePath_mid(:,c), truePath_end(:,c)];
    plot3(Arm(1,:),Arm(2,:),Arm(3,:), 'bk', 'linewidth',2,'marker','o');
    
    %plot the curved path of the end effecor up to the current point
    plot3(truePath_end(1,[1:c]),truePath_end(2,[1:c]),truePath_end(3,[1:c]),'r', 'linewidth',1,'DisplayName',sprintf('True Path')) 
    %maybe have an output legend????
    lgd = legend('show', 'location', 'northwest');
    if c ==1
       print christmas.pdf -append
    end
    print christmas.pdf -append
    %pause(time); %time between each frame, scaled by the number of iterations on the arc as well as n
    hold off; %clear the current frame
   
     
  end
 
  

  
  
% x = (-5:.1:5);
%for p = 1:5
%  plot (x, x.^p)
%  print animation.pdf -append
%endfor
%im = imread ("animation.pdf", "Index", "all");
%imwrite (im, "animation.gif", "DelayTime", .5)


%this is code for outputting a gif that i am referencing
%https://www.mathworks.com/matlabcentral/answers/94495-how-can-i-create-animated-gif-images-in-matlab  
  
%  h = figure;
%axis tight manual % this ensures that getframe() returns a consistent size
%filename = 'testAnimated.gif';
%for n = 1:0.5:5
%    % Draw plot for y = x.^n
%    x = 0:0.01:1;
%    y = x.^n;
%    plot(x,y) 
%    drawnow 
%      % Capture the plot as an image 
%      frame = getframe(h); 
%      im = frame2im(frame); 
%      [imind,cm] = rgb2ind(im,256); 
%      % Write to the GIF File 
%      if n == 1 
%          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%      else 
%          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%      end 
%  end
 
 

