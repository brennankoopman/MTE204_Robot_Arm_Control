function [x,fval] = armFsolver()

P = [1 0.5 1];
q = [180 0 0];
f = @(q) armFunction(q,P); % function of dummy variable y
[out,fval] = fsolve(f,q)

%source: https://www.mathworks.com/matlabcentral/answers/18991-passing-arguments-into-fsolve-without-using-globals

%[x, fval] = fsolve(@armFunction, [-90 0 0]);