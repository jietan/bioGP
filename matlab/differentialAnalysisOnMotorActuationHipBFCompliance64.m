clearvars;

degToRad = 2 * pi / 360;
a = -2808 * degToRad;
b = -1.399e04 * degToRad;
c = 5783 * degToRad;
d = -669.8 * degToRad;
e = 19.13 * degToRad;

inertiaRatio = 0.3994;
ks = 0.006234832611976e03 * inertiaRatio;
kd = -0.083291497874863e03 * inertiaRatio;
kp = -1.228188542897816e03 * inertiaRatio;

endTime = 0.08;

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


