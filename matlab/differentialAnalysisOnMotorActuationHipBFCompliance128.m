clear all;
a = -1.473e05;
b = 4.794e04;
c = -3728;
d = -281.9;
e = 36.71;
kp = -440.7;
kd = -26.05;
ks = 1056;
endTime = 0.13;

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
    recoveredAcc = kp * recoveredX(i - 1) + kd * recoveredXDot(i - 1) + ks;
    recoveredXDot(i) = recoveredXDot(i - 1) + timeStep * recoveredAcc;
    recoveredX(i) = recoveredX(i - 1) + timeStep * recoveredXDot(i);
end
hold on;
plot(times, x, 'b');
plot(newTimes, recoveredX, 'r');


