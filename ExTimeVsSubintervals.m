more off; % allows lots of output to the command window without having to press f
q = [10;0;160];
t = [0.3;-1.3;1.5];
Jerr = 0.001; %testing a Jacobian endcase error

exTime = zeros(1,51);
NumberOfTrials = 3;

for h = 1:NumberOfTrials 
  
   count =0;
  for n = [1,10:10:200] %the numer of subintervals broken
    count = count+1;
    exTime(1,count) = exTime(1,count) + simOutput(q, t, Jerr, n); 
    
  end

end
exTime = exTime/NumberOfTrials;


n = [1,10:10:200];
scatter( n, exTime(1,:), 'b');
title('Computation Time of Path Variables Vs. Number of Subintervals (Average of 3 Trials');
xlabel('Number of Subintervals'); % x-axis label
ylabel('Computation Time (s)'); % y-axis label
hold on;
P = polyfit(n, exTime, 1);
yfit = P(1)*n+P(2);
p(1)
P(2)
plot(n,yfit, 'r-.');
hold off;