q = [0;-160;-45];
t = [1.5;0.8;-0.3];
Jerr = 0.001; %a jacobian absiolute error of 1 mm
Emax = 0.05;   %absolute worst positional error of 1 cm
exTime = 6; %worst case computation time in seconds

output = simOutput(q, t, Jerr, Emax, exTime);
csvwrite('Output.csv', output);