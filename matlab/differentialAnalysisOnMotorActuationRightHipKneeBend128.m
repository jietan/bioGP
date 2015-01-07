clear all;

degToRad = 2 * pi / 360;
a = 3.254e04 * degToRad;
b = -1480 * degToRad;
c = -3482 * degToRad;
d = 690.4 * degToRad;
e = -36.84 * degToRad;

inertiaRatio = 0.400580495539649;
% ks = -201.6;
% kd = -77.57;
% kp = -1965;

% ks = -71.9;
% kd = -51.7;
% kp = -1372.1;

ks = -0.036103383760814e02;
kd = -0.405350054329964e02;
kp = 5.801067928411886e02;

% ks = 0.008242871639897e+03 * inertiaRatio;
% kd = -0.084492985125687e+03 * inertiaRatio;
% kp = -1.276354765284573e+03* inertiaRatio * 2;


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
kp = 5.801067928411886e02;

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


