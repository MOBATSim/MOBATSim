%% Clear the previous data
load('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat');
T_noColi(1:end,:)=[];
save('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat','T_noColi');

load('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat');
T_Coli(1:end,:)=[];
save('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat','T_Coli');

load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4.mat');
speedV3V4 = [];
save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4.mat','speedV3V4');

load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4M1.mat');
speedV3V4M1 = [];
save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4M1.mat','speedV3V4M1');

load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4M2.mat');
speedV3V4M2 = [];
save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4M2.mat','speedV3V4M2');

load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4L1.mat');
speedV3V4L1 = [];
save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4L1.mat','speedV3V4L1');

load('.\src\Misc\Safety Evaluation Report\test_noColiM1.mat');
T_noColiM1(1:end,:)=[];
save('.\src\Misc\Safety Evaluation Report\test_noColiM1','T_noColiM1');

load('.\src\Misc\Safety Evaluation Report\test_ColiM1.mat');
T_ColiM1(1:end,:)=[];
save('.\src\Misc\Safety Evaluation Report\test_ColiM1','T_ColiM1');

load('.\src\Misc\Safety Evaluation Report\test_noColiM2.mat');
T_noColiM2(1:end,:)=[];
save('.\src\Misc\Safety Evaluation Report\test_noColiM2.mat','T_noColiM2');

load('.\src\Misc\Safety Evaluation Report\test_ColiM2.mat');
T_ColiM2(1:end,:)=[];
save('.\src\Misc\Safety Evaluation Report\test_ColiM2','T_ColiM2');

load('.\src\Misc\Safety Evaluation Report\test_noColiL1.mat');
T_noColiL1(1:end,:)=[];
save('.\src\Misc\Safety Evaluation Report\test_noColiL1','T_noColiL1');

load('.\src\Misc\Safety Evaluation Report\test_ColiL1.mat');
T_ColiL1(1:end,:)=[];
save('.\src\Misc\Safety Evaluation Report\test_ColiL1','T_ColiL1');

% Create table
        %T_noColi = cell2table(cell(0,7), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'min_distance', 'min_TTC','min_TTCtime'});
% Create table
        % T_Coli = cell2table(cell(0,6), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'rela_impactSpeed','collisionTime'});
%% Normal situation without malfunctions
nr_Simulations = 9;
for k = 1:nr_Simulations
    prepare_simulator("Analysing",0,"FI_id",3,"FI_value",0.4*k) % for every loop the speed of vehicle 3 is changed by "k"
    evalin('base', 'run_Sim');
    
    load('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat');
    load('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat');
    
    idx_nocoli = height(T_noColi);
    idx_coli = height(T_Coli);
       
    % Log Data + Add Data to Table
    if Vehicles(1,3).dynamics.position(1) < Vehicles(1,4).dynamics.position(1)
        front_vehicleID = Vehicles(4).id;
        rear_vehicleID = Vehicles(3).id;
        maxspeed_rearVehicle = Vehicles(3).dynamics.maxSpeed;
    end
    has_collision = Vehicles(3).status.collided;
    %                 AEBdistance_rearVehicle = Vehicles(3).sensors.AEBdistance;
    %                 sensorRange_rearVehicle = Vehicles(3).sensors.frontSensorRange;
    %                 minDec_rearVehicle = Vehicles(3).dynamics.minDeceleration;
    % judge if there are is collision
    if has_collision ==0
        min_distance = min(allTestData(8,3,:));
        filtTTC = allTestData(7,3,:);  %filter out the negative TTC
        filtTTC(find(filtTTC<0)) = 1000;
        [min_TTC,min_TTCidx]= min(filtTTC); %[minimum TTC, index]
        min_TTCtime = min_TTCidx*Sim_Ts;
        
        T_noColi(idx_nocoli+1,:) = {front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, min_distance, min_TTC, min_TTCtime};
        save('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat','T_noColi');
        
    elseif has_collision ==1
        %load speed of v3 and v4
        
        load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4.mat');
        relaSpeed_calcu = [allTestData(1,3,:), allTestData(1,4,:)]; %[speed_v3,speed_v4]
        speedV3V4 = [speedV3V4; relaSpeed_calcu]; % speed of v3 and v4 for plotting
        save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4.mat','speedV3V4');
        % Find relative impact speed and colision time
        i=1;
        slopeRange = 100; % the range of the slop for the speed change in collasion
        %slope = abs((speedV3V4(end,1,i+1)-speedV3V4(end,1,i))/Sim_Ts);
        while(abs((speedV3V4(end,1,i+1)-speedV3V4(end,1,i))/Sim_Ts)<slopeRange) % the slope of the speed is big enough
            i=i+1;
        end
        speedColi_V3 = speedV3V4(end,1,i); % the speed of v3 at collision time
        speedColi_V4 = speedV3V4(end,2,i); % the speed of v4 at collision time
        collisionTime = i*Sim_Ts;
        rela_impactSpeed = speedColi_V3 - speedColi_V4;
    
        T_Coli(idx_coli+1,:) = {front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, rela_impactSpeed, collisionTime};
        save('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat','T_Coli');
    end
end


%% Malfunction 1 -- time delay of driving model

nr_Simulations = 9;
nr_delays = 3;
% Create the matrix of the loops, if we use two layers loop, k will be cleared in prepare_simulations()
loops_M1 = []; %[nr_delays, nr_Simulations]
for k = 1:nr_Simulations
    for j =1: nr_delays
        loops_M1 = [loops_M1; j, k];
    end
end
save('.\src\Misc\Safety Evaluation Report\MalFunLoops1.mat','loops_M1');

for i = 1:size(loops_M1,1)
    load('.\src\Misc\Safety Evaluation Report\MalFunLoops1.mat'); % load the value of delay number and simulation number
    prepare_simulator("Analysing",0,"FI_id",3,"FI_value",0.4*loops_M1(i,2),"FI_delay",0.5*loops_M1(i,1))
    evalin('base', 'run_Sim');
    
    load('.\src\Misc\Safety Evaluation Report\test_noColiM1.mat');
    load('.\src\Misc\Safety Evaluation Report\test_ColiM1.mat');
    
    idx_nocoliM1 = height(T_noColiM1);
    idx_coliM1 = height(T_ColiM1);
    
    if Vehicles(1,3).dynamics.position(1) < Vehicles(1,4).dynamics.position(1)
        delay_time = delayTimeV3; % delay time of the driving mode which is set in longitudinal control of vehicle 3
        front_vehicleID = Vehicles(3).id;
        rear_vehicleID = Vehicles(4).id;
        maxspeed_rearVehicle = Vehicles(3).dynamics.maxSpeed;
    end
    has_collision = Vehicles(3).status.collided;
    if has_collision == 0
        min_distance = min(allTestData(8,3,:));
        filtTTC = allTestData(7,3,:);  %filter out the negative TTC
        filtTTC(find(filtTTC<0)) = 1000;
        [min_TTC,min_TTCidx]= min(filtTTC); %[minimum TTC, index]
        min_TTCtime = min_TTCidx*Sim_Ts;
        T_noColiM1(idx_nocoliM1+1,:) = {delay_time, front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, min_distance, min_TTC, min_TTCtime};
        save('.\src\Misc\Safety Evaluation Report\test_noColiM1.mat','T_noColiM1');
    elseif has_collision ==1
        %load speed of v3 and v4
        load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4M1.mat');
        relaSpeed_calcu = [allTestData(1,3,:), allTestData(1,4,:)]; %[speed_v4,speed_v3]
        speedV3V4M1 = [speedV3V4M1; relaSpeed_calcu]; % speed of v3 and v4 for plotting
        save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4M1.mat','speedV3V4M1');
        % Find relative impact speed and colision time
        i=1;
        slopeRange = 100; % the range of the slop for the speed change in collasion
        %slope = abs((speedV3V4(end,1,i+1)-speedV3V4(end,1,i))/Sim_Ts);
        while(abs((speedV3V4M1(end,1,i+1)-speedV3V4M1(end,1,i))/Sim_Ts)<slopeRange) % the slope of the speed is big enough
            i=i+1;
        end
        speedColi_V3 = speedV3V4M1(end,1,i); % the speed of v3 at collision time
        speedColi_V4 = speedV3V4M1(end,2,i); % the speed of v4 at collision time
        collisionTime = i*Sim_Ts;
        rela_impactSpeed = speedColi_V3 - speedColi_V4;
        T_ColiM1(idx_coliM1+1,:) = {delay_time, front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, rela_impactSpeed, collisionTime};
        save('.\src\Misc\Safety Evaluation Report\test_ColiM1.mat','T_ColiM1');
    end
    
end

%% Malfunction 2 -- failure rate of V3 sensor

nr_Simulations = 9;
nr_failure = 0.9;
% Create the matrix of the loops, if we use two layers loop, k in firs layer will be cleared in prepare_simulations()
loops_M2 = []; %[nr_failure, nr_Simulations]
for k = 1:nr_Simulations
    for j =0.3:0.2: nr_failure
        loops_M2 = [loops_M2; j, k];
    end
end
save('.\src\Misc\Safety Evaluation Report\MalFunLoops2.mat','loops_M2');

for i = 1:size(loops_M2,1)
    load('.\src\Misc\Safety Evaluation Report\MalFunLoops2.mat'); % load the value of failure number and simulation number
    prepare_simulator("Analysing",0,"FI_id",3,"FI_value",0.4*loops_M2(i,2),"FI_failure",loops_M2(i,1)) % for every loop the speed of vehicle 3 and failure rate are changed
    evalin('base', 'run_Sim');
    
    % Create table
    %T_noColi = cell2table(cell(0,10), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'min_distance', 'min_TTC','min_TTCtime', 'AEBdistance_rearVehicle', 'sensorRange_rearVehicle', 'minDec_rearVehicle'});
    load('.\src\Misc\Safety Evaluation Report\test_noColiM2.mat');
    load('.\src\Misc\Safety Evaluation Report\test_ColiM2.mat');
    
    % Log Data + Add Data to Table
    idx_nocoliM2 = height(T_noColiM2);
    idx_coliM2 = height(T_ColiM2);
    
    if Vehicles(1,3).dynamics.position(1) < Vehicles(1,4).dynamics.position(1)
        failure_rate = V3_FailureRate;
        front_vehicleID = Vehicles(3).id;
        rear_vehicleID = Vehicles(4).id;
        maxspeed_rearVehicle = Vehicles(3).dynamics.maxSpeed;
    end
    has_collision = Vehicles(3).status.collided;
    if has_collision == 0
        min_distance = min(allTestData(8,3,:));
        filtTTC = allTestData(7,3,:);  %filter out the negative TTC
        filtTTC(find(filtTTC<0)) = 1000;
        [min_TTC,min_TTCidx]= min(filtTTC); %[minimum TTC, index]
        min_TTCtime = min_TTCidx*Sim_Ts;
        T_noColiM2(idx_nocoliM2+1,:) = {failure_rate, front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, min_distance, min_TTC, min_TTCtime};
        save('.\src\Misc\Safety Evaluation Report\test_noColiM2.mat','T_noColiM2');
    elseif has_collision ==1
        %load speed of v3 and v4
        load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4M2.mat');
        relaSpeed_calcu = [allTestData(1,3,:), allTestData(1,4,:)]; %[speed_v4,speed_v3]
        speedV3V4M2 = [speedV3V4M2; relaSpeed_calcu]; % speed of v3 and v4 for plotting
        save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4M2.mat','speedV3V4M2');
        
        % Find relative impact speed and colision time
        i=1;
        slopeRange = 100; % the range of the slop for the speed change in collasion
        %slope = abs((speedV3V4(end,1,i+1)-speedV3V4(end,1,i))/Sim_Ts);
        while(abs((speedV3V4M2(end,1,i+1)-speedV3V4M2(end,1,i))/Sim_Ts)<slopeRange) % the slope of the speed is big enough
            i=i+1;
        end
        speedColi_V3 = speedV3V4M2(end,1,i); % the speed of v3 at collision time
        speedColi_V4 = speedV3V4M2(end,2,i); % the speed of v4 at collision time
        collisionTime = i*Sim_Ts;
        rela_impactSpeed = speedColi_V3 - speedColi_V4;
        T_ColiM2(idx_coliM2+1,:) = {failure_rate front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, rela_impactSpeed, collisionTime};
        save('.\src\Misc\Safety Evaluation Report\test_ColiM2.mat','T_ColiM2');
    end
    
end

%% Limitation 1 -- time delay with 2s in driving mode
nr_Simulations = 9;
for k = 1:nr_Simulations
    
    prepare_simulator("Analysing",0,"FI_id",3,"FI_value",0.4*k,"FI_delay",2)
    evalin('base', 'run_Sim');
    
    load('.\src\Misc\Safety Evaluation Report\test_noColiL1.mat');
    load('.\src\Misc\Safety Evaluation Report\test_ColiL1.mat');
    
    idx_nocoliL1 = height(T_noColiL1);
    idx_coliL1 = height(T_ColiL1);
    
    if Vehicles(1,3).dynamics.position(1) < Vehicles(1,4).dynamics.position(1)
        delay_time = delayTimeV3; % delay time of the driving mode which is set in longitudinal control of vehicle 3
        front_vehicleID = Vehicles(3).id;
        rear_vehicleID = Vehicles(4).id;
        maxspeed_rearVehicle = Vehicles(3).dynamics.maxSpeed;
    end
    has_collision = Vehicles(3).status.collided;
    if has_collision == 0
        min_distance = min(allTestData(8,3,:));
        filtTTC = allTestData(7,3,:);  %filter out the negative TTC
        filtTTC(find(filtTTC<0)) = 1000;
        [min_TTC,min_TTCidx]= min(filtTTC); %[minimum TTC, index]
        min_TTCtime = min_TTCidx*Sim_Ts;
        T_noColiL1(idx_nocoliL1+1,:) = {delay_time, front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, min_distance, min_TTC, min_TTCtime};
        save('.\src\Misc\Safety Evaluation Report\test_noColiL1.mat','T_noColiL1');
    elseif has_collision ==1
        %load speed of v3 and v4
        load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4L1.mat');
        relaSpeed_calcu = [allTestData(1,3,:), allTestData(1,4,:)]; %[speed_v4,speed_v3]
        speedV3V4L1 = [speedV3V4L1; relaSpeed_calcu]; % speed of v3 and v4 for plotting
        save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4L1.mat','speedV3V4L1');
        
        % Find relative impact speed and colision time
        i=1;
        slopeRange = 100; % the range of the slop for the speed change in collasion
        %slope = abs((speedV3V4(end,1,i+1)-speedV3V4(end,1,i))/Sim_Ts);
        while(abs((speedV3V4L1(end,1,i+1)-speedV3V4L1(end,1,i))/Sim_Ts)<slopeRange) % the slope of the speed is big enough
            i=i+1;
        end
        speedColi_V3 = speedV3V4L1(end,1,i); % the speed of v3 at collision time
        speedColi_V4 = speedV3V4L1(end,2,i); % the speed of v4 at collision time
        collisionTime = i*Sim_Ts;
        rela_impactSpeed = speedColi_V3 - speedColi_V4;
        T_ColiL1(idx_coliL1+1,:) = {delay_time, front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, rela_impactSpeed, collisionTime};
        save('.\src\Misc\Safety Evaluation Report\test_ColiL1.mat','T_ColiL1');
    end
    
end