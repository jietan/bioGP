a = 3.16e04;
b = -2.047e04;
c = 5304;
d = -677.8;
e = 37.42;
kp = -703.3;
kd = -54.1;
ks = 111.8;
endTime = 0.2;

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


