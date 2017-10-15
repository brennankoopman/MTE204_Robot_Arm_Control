% doesn't do anything yet this was just a test


x = [1 2 3];
%{
xlim(3);
ylim(3);
zlim(3);
%}
%plot3(10,10,10);


t = 0:pi/50:10*pi;
st = sin(t);
ct = cos(t);

plot3(st,ct,t)

for i = [1:1:1000000000]
	
end

plot(st,ct,0)
%{
for i = [1:1:10]
  x(1).+1;
  x
  plot3(x);
end
%}


% Possible resources
% 
% 
% https://www.youtube.com/watch?v=Kus5nHW7Twc
% https://github.com/JimKerns/Video-samples/blob/master/DrawSpringMassDamper2.m