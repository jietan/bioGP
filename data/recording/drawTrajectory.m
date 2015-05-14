directory = 'C:\Users\JieTan\Documents\MyProjects\bioloid\bioloidgp-master\data\recording\lean-to-stand\';
files = dir(strcat(directory, '*.measure'));
for file = files'
    data = dlmread(strcat(directory, file.name),' ',1,0);
    hold all;
    t = data(:,1);
    rotData = data(:,2:4);
    angle = sqrt(sum(abs(rotData).^2,2));
    plot(t, angle);
end


