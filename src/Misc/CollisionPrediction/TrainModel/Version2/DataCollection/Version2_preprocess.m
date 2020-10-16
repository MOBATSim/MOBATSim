%% Create tables of data from vehicles
% Data = cell2table(cell(0,13), 'VariableNames', {'maxSpeed_A', 'Speed_A','DistancetoCrosswaypoint_A',  'frontSensorRange_A', 'AEBdistance_A', 'minDeceleration_A','maxSpeed_B','Speed_B','DistancetoCrosswaypoint_B', 'frontSensorRange_B', 'AEBdistance_B', 'minDeceleration_B','collision'});


load('idx.mat')
load('Data0.mat')
load('Data.mat')
d = [];
n = 1;
h = (1:(idx - 1));

if idx > 2
    for i = 1:(idx - 2)  % idx -1 is the rows of the table
        for k = (i + 1):(idx - 1)
            if Data0.A(i) == Data0.A(k) && Data0.B(i) == Data0.B(k) % only keep one row for each crossway point
                d(n) = k;
                n = n + 1;
            end
        end
    end
end
d1 = h(~ismember(h, d));
Data0 = Data0(d1,:);

Data0 = removevars(Data0,{'A','B'});
Data = [Data;Data0];


save('Data.mat','Data');

