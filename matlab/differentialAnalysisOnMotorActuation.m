a = -2.749e04;
b = 7992;
c = -675.8;
d = 18.3;

times = 0 : 0.001 : 0.06;
count = size(times, 2);
for i = 1 : count
    t = times(i);
    x(i) = a * t^3 + b * t^2 + c * t + d;
    xdot(i) = 3 * a * t^2 + 2 * b * t + c;
    xddot(i) = 6 * a * t + 2 * b;
    
end
% plot(times, x, times, xdot / 100, times, xddot / 1000);
plot(x, xddot);