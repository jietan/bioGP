% filename = '../data/measurement/pControlAnkelLR7.txt';
filename = '../data/measurement/RightHipKneeBendSpeed0Compliance128.txt';
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


% % Left Hip Knee Bended
% % compliance 64
% tStart(1, 1) = 4.419;   tStart(1, 2) = 4.428;   tStart(1, 3) = 4.617;
% tEnd(1, 1) = 4.428;     tEnd(1, 2) = 4.617;     tEnd(1, 3) = 4.776;
% tStart(2, 1) = 5.987;   tStart(2, 2) = 5.997;   tStart(2, 3) = 6.186;
% tEnd(2, 1) = 5.997;     tEnd(2, 2) = 6.186;     tEnd(2, 3) = 6.341;
% numDataSegments = 2;

% % compliance 128
% tStart(1, 1) = 4.405;   tStart(1, 2) = 4.413;   tStart(1, 3) = 4.573;
% tEnd(1, 1) = 4.413;     tEnd(1, 2) = 4.573;     tEnd(1, 3) = 4.757;
% tStart(2, 1) = 5.975;   tStart(2, 2) = 5.982;   tStart(2, 3) = 6.142;
% tEnd(2, 1) = 5.982;     tEnd(2, 2) = 6.142;     tEnd(2, 3) = 6.31;
% numDataSegments = 2;

% % Right Hip Knee Bended
% % compliance 64
% tStart(1, 1) = 4.407;   tStart(1, 2) = 4.415;   tStart(1, 3) = 4.608;
% tEnd(1, 1) = 4.415;     tEnd(1, 2) = 4.608;     tEnd(1, 3) = 4.768;
% tStart(2, 1) = 6.106;   tStart(2, 2) = 6.115;   tStart(2, 3) = 6.306;
% tEnd(2, 1) = 6.115;     tEnd(2, 2) = 6.306;     tEnd(2, 3) = 6.466;
% numDataSegments = 2;

% % compliance 128
tStart(1, 1) = 4.445;   tStart(1, 2) = 4.454;   tStart(1, 3) = 4.613;
tEnd(1, 1) = 4.454;     tEnd(1, 2) = 4.613;     tEnd(1, 3) = 4.773;
tStart(2, 1) = 5.883;   tStart(2, 2) = 5.892;   tStart(2, 3) = 6.049;
tEnd(2, 1) = 5.892;     tEnd(2, 2) = 6.049;     tEnd(2, 3) = 6.209;
numDataSegments = 2;


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

