clearvars;
degToRad = 2 * pi / 360;
a = 3.16e04 * degToRad;
b = -2.047e04 * degToRad;
c = 5304 * degToRad;
d = -677.8 * degToRad;
e = 37.42 * degToRad;
  
% kp = -1.197216857283312e03;
% kd = -0.078399989905992e03;
% ks = 0.752916589319886e03;


ks = 106.13;
kd = -24.23;
kp = -358.7 / 2;

endTime = 0.2;

times = 0 : 0.001 : endTime;
count = size(times, 2);
for i = 1 : count
    t = times(i);
    x(i) = a * t^4 + b * t^3 + c * t^2 + d * t + e;
    xdot(i) = (4 * a * t^3 + 3 * b * t^2 + 2 * c * t + d);
    xddot(i) = (12 * a * t^2 + 6 * b * t + 2 * c);
    xtrdot(i) = (24 * a * t + 6 * b);
end
plot(times, x, times, xdot / 100, times, xddot / 1000);
% plot(x, xddot);
A = zeros([count, 3]);
rhs = zeros([count, 1]);

for i = 1 : count
        A(i, 1) = 1;
        A(i, 2) = xdot(i);
        A(i, 3) = x(i);
        rhs(i) = xddot(i);
end

params = A \ rhs
ks = params(1);
% kj = params(2);
kd = params(2);
kp = params(3);
% plot(x, xddot);
timeStep = 0.0001;
newTimes = 0 : timeStep : endTime;
count = size(newTimes, 2);

recoveredXDot(1) = xdot(1);
recoveredX(1) = x(1);

for i = 2 : count
%     xTrDot = 24 * a * ((i - 1) * timeStep) + 6 * b;
    recoveredAcc = kp * recoveredX(i - 1) + kd * recoveredXDot(i - 1) + ks;
    recoveredXDot(i) = recoveredXDot(i - 1) + timeStep * recoveredAcc;
    recoveredX(i) = recoveredX(i - 1) + timeStep * recoveredXDot(i);
end
hold on;
plot(times, x, 'b');
plot(newTimes, recoveredX, 'r');


