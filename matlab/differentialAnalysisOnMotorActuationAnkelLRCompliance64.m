clear all;
% % compliance 64
a = -2.749e04;
b = 7992;
c = -675.8;
d = 18.3;
kp = -358.7;
kd = -24.23;
ks = 6080;
endTime = 0.06;

% % compliance 32
% a = -9.516e04;
% b = 1.539e04;
% c = -745.1;
% d = 9.539;
% kp = -2563;
% kd = -66.03;
% ks = 4111;
% endTime = 0.06;

times = 0 : 0.001 : endTime;
count = size(times, 2);
for i = 1 : count
    t = times(i);
    x(i) = a * t^3 + b * t^2 + c * t + d;
    xdot(i) = 3 * a * t^2 + 2 * b * t + c;
    xddot(i) = 6 * a * t + 2 * b;
    
end
plot(times, x, times, xdot / 100, times, xddot / 1000);
% plot(x, xddot);
timeStep = 0.0001;
newTimes = 0 : timeStep : endTime;
count = size(newTimes, 2);
recoveredXDot(1) = xdot(1);
recoveredX(1) = x(1);
for i = 2 : count
    recoveredAcc = kp * recoveredX(i - 1) + kd * recoveredXDot(i - 1) + ks;
    recoveredXDot(i) = recoveredXDot(i - 1) + timeStep * recoveredAcc;
    recoveredX(i) = recoveredX(i - 1) + timeStep * recoveredXDot(i);
end
hold on;
plot(times, x, 'b');
plot(newTimes, recoveredX, 'r');


