function acc = computeAcc(p, t)
numSamples = size(p, 1);
filteredP = (p(1:numSamples-2) + p(2:numSamples-1) + p(3:numSamples)) / 3;
p(2:numSamples-1) = filteredP;
dt = t(2:numSamples) - t(1:numSamples-1);
v = (p(2:numSamples) - p(1:numSamples-1)) ./ dt;
dtAvg = (dt(1:numSamples-2) + dt(2:numSamples-1)) / 2.0;
acc = (v(2:numSamples-1) - v(1:numSamples-2)) ./ dtAvg;

