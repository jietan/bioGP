% filename = '../data/measurement/pControlAnkelLR7.txt';
filename = '../data/measurement/HipSpeed0Compliance64.txt';
data = load(filename);
numSamples = size(data, 1);
t = data(:, 1) / 1000.0;
goalPos = data(:, 2) * 0.29;
currentPos = data(:, 3) * 0.29;
figure;
plot(t, goalPos, t, currentPos);
xlim([0, 10]);
acc = computeAcc(currentPos, t);
speed = computeSpeed(currentPos, t);
speed = speed(1:numSamples-2);
err = goalPos(2:numSamples) - currentPos(1:numSamples-1);
dt = t(2:numSamples) - t(1:numSamples-1);
errorVel = err ./ dt;
errorVel = errorVel(1:numSamples - 2);
figure;
tUsed = t(2:numSamples-1);
errUsed = err(2:numSamples - 1);
plot(tUsed, errUsed, tUsed, speed, tUsed, acc / 100);
% plot(t(2:numSamples-1), errorVel, t(2:numSamples-1), speed);
xlim([3, 8]);

% % ankel LR
% % compliance 64
% tStart(1, 1) = 4.356;   tStart(1, 2) = 4.374;   tStart(1, 3) = 4.537;
% tEnd(1, 1) = 4.374;     tEnd(1, 2) = 4.537;     tEnd(1, 3) = 4.599;
% tStart(2, 1) = 5.666;   tStart(2, 2) = 5.682;   tStart(2, 3) = 5.849;
% tEnd(2, 1) = 5.682;     tEnd(2, 2) = 5.849;     tEnd(2, 3) = 5.909;
% tStart(3, 1) = 6.975;   tStart(3, 2) = 6.988;   tStart(3, 3) = 7.157;
% tEnd(3, 1) = 6.988;     tEnd(3, 2) = 7.157;     tEnd(3, 3) = 7.216;
% numDataSegments = 3;

% % compliance 128
% tStart(1, 1) = 4.45;   tStart(1, 2) = 4.461;   tStart(1, 3) = 4.599;
% tEnd(1, 1) = 4.461;     tEnd(1, 2) = 4.599;     tEnd(1, 3) = 4.799;
% tStart(2, 1) = 6.021;   tStart(2, 2) = 6.031;   tStart(2, 3) = 6.17;
% tEnd(2, 1) = 6.031;     tEnd(2, 2) = 6.17;     tEnd(2, 3) = 6.361;
% numDataSegments = 2;

% % compliance 32
% tStart(1, 1) = 4.273;   tStart(1, 2) = 4.28;   tStart(1, 3) = 4.471;
% tEnd(1, 1) = 4.28;     tEnd(1, 2) = 4.471;     tEnd(1, 3) = 4.529;
% tStart(2, 1) = 5.582;   tStart(2, 2) = 5.589;   tStart(2, 3) = 5.779;
% tEnd(2, 1) = 5.589;     tEnd(2, 2) = 5.779;     tEnd(2, 3) = 5.387;
% tStart(3, 1) = 6.891;   tStart(3, 2) = 6.899;   tStart(3, 3) = 7.088;
% tEnd(3, 1) = 6.899;     tEnd(3, 2) = 7.088;     tEnd(3, 3) = 7.145;
% numDataSegments = 3;

% % Hip
% % compliance 64
tStart(1, 1) = 4.122;   tStart(1, 2) = 4.134;   tStart(1, 3) = 4.203;
tEnd(1, 1) = 4.134;     tEnd(1, 2) = 4.203;     tEnd(1, 3) = 4.36;
tStart(2, 1) = 5.691;   tStart(2, 2) = 5.706;   tStart(2, 3) = 5.77;
tEnd(2, 1) = 5.706;     tEnd(2, 2) = 5.77;     tEnd(2, 3) = 5.92;
numDataSegments = 2;

% % Hip
% % compliance 128
% tStart(1, 1) = 4.023;   tStart(1, 2) = 4.036;   tStart(1, 3) = 4.057;
% tEnd(1, 1) = 4.036;     tEnd(1, 2) = 4.057;     tEnd(1, 3) = 4.188;
% tStart(2, 1) = 5.331;   tStart(2, 2) = 5.345;   tStart(2, 3) = 5.365;
% tEnd(2, 1) = 5.345;     tEnd(2, 2) = 5.365;     tEnd(2, 3) = 5.494;
% tStart(3, 1) = 6.638;   tStart(3, 2) = 6.651;   tStart(3, 3) = 6.671;
% tEnd(3, 1) = 6.651;     tEnd(3, 2) = 6.671;     tEnd(3, 3) = 6.803;
% numDataSegments = 3;

t1 = []; t2 = []; t3 = [];
sp1 = []; sp2 = []; sp3 = [];
err1 = []; err2 = []; err3 = [];
for i = 1 : numDataSegments
    phase1 = and(tUsed >= tStart(i, 1), tUsed < tEnd(i, 1));
    t1 = [t1; tUsed(phase1) - tStart(i, 1)];
    sp1 = [sp1; speed(phase1)];
    err1 = [err1; errUsed(phase1)];
    
    phase2 = and(tUsed >= tStart(i, 2), tUsed < tEnd(i, 2));
    t2 = [t2; tUsed(phase2) - tStart(i, 2)];
    sp2 = [sp2; speed(phase2)];
    err2 = [err2; errUsed(phase2)];
    
    phase3 = and(tUsed >= tStart(i, 3), tUsed < tEnd(i, 3));
    t3 = [t3; tUsed(phase3) - tStart(i, 3)];
    sp3 = [sp3; speed(phase3)];
    err3 = [err3; errUsed(phase3)];
end

