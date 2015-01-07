% filename = '../data/measurement/pControlAnkelLR7.txt';
clearvars;
filename = '../../data/measurement/RightHipKneeBendSpeed0Compliance128.txt';

data = load(filename);
numSamples = size(data, 1);
t = data(:, 1) / 1000.0;
goalPos = data(:, 2) * 0.29 / 360 * 2 * pi;
currentPos = data(:, 3) * 0.29 / 360 * 2 * pi;

plot(t, goalPos, t, currentPos);
xlim([3, 8]);
xlabel('t');
ylabel('rad');
legend('goalPos', 'currentPos');
title('goal vs current position, RightHipKneeBended128');
figure;
err = goalPos(2:numSamples) - currentPos(1:numSamples-1);
tUsed = t(2:numSamples-1);
errUsed = err(2:numSamples - 1);
% % compliance 128
tStart(1, 1) = 4.445;   tStart(1, 2) = 4.454;   tStart(1, 3) = 4.613;
tEnd(1, 1) = 4.454;     tEnd(1, 2) = 4.613;     tEnd(1, 3) = 4.773;
tStart(2, 1) = 5.883;   tStart(2, 2) = 5.892;   tStart(2, 3) = 6.049;
tEnd(2, 1) = 5.892;     tEnd(2, 2) = 6.049;     tEnd(2, 3) = 6.209;
numDataSegments = 2;


t3 = [];
err3 = [];
for i = 1 : numDataSegments
    phase3 = and(tUsed >= tStart(i, 3), tUsed < tEnd(i, 3));
    t3 = [t3; tUsed(phase3) - tStart(i, 3)];
    err3 = [err3; errUsed(phase3)];
end
scatter(t3, err3, '+', 'black');

degToRad = 2 * pi / 360;

a = 3.254e04 * degToRad;
b = -1480 * degToRad;
c = -3482 * degToRad;
d = 690.4 * degToRad;
e = -36.84 * degToRad;

ks = -0.036103383760814e02;
kd = -0.405350054329964e02;
kp = 5.801067928411886e02;

endTime = 0.16;

times = 0 : 0.001 : endTime;
count = size(times, 2);
for i = 1 : count
    t = times(i);
    x(i) = a * t^4 + b * t^3 + c * t^2 + d * t + e;
    xdot(i) = -(4 * a * t^3 + 3 * b * t^2 + 2 * c * t + d);
    xddot(i) = -(12 * a * t^2 + 6 * b * t + 2 * c);
    
end

timeStep = 0.0001;
newTimes = 0 : timeStep : endTime;
count = size(newTimes, 2);
recoveredXDot(1) = xdot(1);
recoveredX(1) = x(1);
for i = 2 : count
    sgn = recoveredXDot(i - 1) / abs(recoveredXDot(i - 1));
    recoveredAcc = kp * recoveredX(i - 1) + kd * recoveredXDot(i - 1) + sgn * ks;
    recoveredXDot(i) = recoveredXDot(i - 1) + timeStep * recoveredAcc;
    recoveredX(i) = recoveredX(i - 1) - timeStep * recoveredXDot(i);
end
hold on;
plot(times, x, 'g');
plot(newTimes, recoveredX, 'r');
xlabel('t');
ylabel('delta q');
legend('measuredQ', 'polyFit', 'odeFit');
title('regression on motor parameters, RightHipKneeBended128');






