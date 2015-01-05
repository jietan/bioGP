clear all;

degToRad = 2 * pi / 360;
a = -1.25e06 * degToRad;
b = 4.646e04 * degToRad;
c = 1.026e04 * degToRad;
d = -681.3 * degToRad;
e = 9.368 * degToRad;
% kp = -4.009218337159577e03;
% kd = -0.078399989905992e03;
% ks = 0.752916589319886e03;

ks = 0.006234832611976e03;
kd = -0.083291497874863e03;
kp = -4.912754171591265e03;

endTime = 0.065;

times = 0 : 0.001 : endTime;
count = size(times, 2);
for i = 1 : count
    t = times(i);
    x(i) = a * t^4 + b * t^3 + c * t^2 + d * t + e;
    xdot(i) = 4 * a * t^3 + 3 * b * t^2 + 2 * c * t + d;
    xddot(i) = 12 * a * t^2 + 6 * b * t + 2 * c;
    
end
plot(times, x, times, xdot / 100, times, xddot / 1000);
% plot(x, xddot);
timeStep = 0.0001;
newTimes = 0 : timeStep : endTime;
count = size(newTimes, 2);
recoveredXDot(1) = xdot(1);
recoveredX(1) = x(1);

for i = 2 : count
%     sgn = 1;
    sgn = -recoveredXDot(i - 1) / abs(recoveredXDot(i - 1));
    recoveredAcc = kp * recoveredX(i - 1) + kd * recoveredXDot(i - 1) + sgn * ks;
    recoveredXDot(i) = recoveredXDot(i - 1) + timeStep * recoveredAcc;
    recoveredX(i) = recoveredX(i - 1) + timeStep * recoveredXDot(i);
end
hold on;
plot(times, x, 'b');
plot(newTimes, recoveredX, 'r');
% plot(times, x, 'b');
% plot(newTimes, recoveredX, newTimes, recoveredXDot / 100);




