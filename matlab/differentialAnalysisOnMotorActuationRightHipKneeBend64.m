clear all;

degToRad = 2 * pi / 360;
% a = 1.852e06 * degToRad;
% b = -2.127e05 * degToRad;
% c = 2170 * degToRad;
% d = 599.4 * degToRad;
% e = -18.29 * degToRad;


a = 2.718e04 * degToRad;
b = 1.308e04 * degToRad;
c = -6500 * degToRad;
d = 705.7 * degToRad;
e = -18.52 * degToRad;

inertiaRatio = 0.400580495539649;




% ks = -281.8;
% kd = -94.45;
% kp = 2523;

% ks = 2.231;
% kd = -38.83;
% kp = 967.5;

endTime = 0.16;

times = 0 : 0.001 : endTime;
count = size(times, 2);
for i = 1 : count
    t = times(i);
    x(i) = a * t^4 + b * t^3 + c * t^2 + d * t + e;
    xdot(i) = -(4 * a * t^3 + 3 * b * t^2 + 2 * c * t + d);
    xddot(i) = -(12 * a * t^2 + 6 * b * t + 2 * c);
    
end
plot(times, x, times, xdot / 100, times, xddot / 1000);
% plot(x, xddot);
A = zeros([count, 3]);
b = zeros([count, 1]);
for i = 1 : count
        A(i, 1) = abs(xdot(i)) / xdot(i);
        A(i, 2) = xdot(i);
        A(i, 3) = x(i);
        b(i) = xddot(i);
end

params = A \ b
ks = params(1);
kd = params(2);
kp = params(3);


ks = -0.036103383760814e02;
kd = -0.405350054329964e02;
kp = 5.801067928411886e02 * 2;

timeStep = 0.0001;
newTimes = 0 : timeStep : endTime + 0.1;
count = size(newTimes, 2);
recoveredXDot(1) = xdot(1);
recoveredX(1) = x(1);
for i = 2 : count
    sign = abs(recoveredXDot(i - 1)) / recoveredXDot(i - 1);
    recoveredAcc = kp * recoveredX(i - 1) + kd * recoveredXDot(i - 1) + sign * ks;
    recoveredXDot(i) = recoveredXDot(i - 1) + timeStep * recoveredAcc;
    recoveredX(i) = recoveredX(i - 1) - timeStep * recoveredXDot(i);
end
hold on;
plot(times, x, 'b');
plot(newTimes, recoveredX, 'r');


