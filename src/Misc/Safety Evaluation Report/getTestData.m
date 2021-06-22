%tic;

%% Create table
% switch testGoal
%     case 'noCollision'
%         T_noColi = cell2table(cell(0,9), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'min_distance', 'min_TTC' 'AEBdistance_rearVehicle', 'sensorRange_rearVehicle', 'minDec_rearVehicle'});
% %         load('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat');
%     case 'Collision'
%         T_Coli = cell2table(cell(0,8), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'rela_impactSpeed', 'AEBdistance_rearVehicle', 'sensorRange_rearVehicle', 'minDec_rearVehicle'});
% end

%% Situation without collision
% delete the useless data from the table for next test
% load('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat');
% T_noColi(1:9,:)=[]; % delet all the data in the table for the next test
% save('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat','T_noColi');

% % Start several loops of simulation
% nr_Simulations = 4;   
% for k = 1:nr_Simulations
%     
%       
%             prepare_simulator("Analysing",0,"FI_id",3,"FI_value",-k) % for every loop the speed of vehicle 3 is changed by "k"
%             evalin('base', 'run_Sim');
%             
%             % Create table
%             %T_noColi = cell2table(cell(0,9), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'min_distance', 'min_TTC' 'AEBdistance_rearVehicle', 'sensorRange_rearVehicle', 'minDec_rearVehicle'});
%             load('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat');
%            
%             % Log Data + Add Data to Table
%             idx = height(T_noColi);
%             
%             front_vehicleID = Vehicles(3).id;
%             rear_vehicleID = Vehicles(4).id;
%             maxspeed_rearVehicle = Vehicles(3).dynamics.maxSpeed;
%             has_collision = Vehicles(3).status.collided;
%             min_distance = 1;
%             min_TTC=1;
%             AEBdistance_rearVehicle = Vehicles(3).sensors.AEBdistance;
%             sensorRange_rearVehicle = Vehicles(3).sensors.frontSensorRange;
%             minDec_rearVehicle = Vehicles(3).dynamics.minDeceleration;
%             
%             T_noColi(idx+1,:) = {front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, min_distance,min_TTC, AEBdistance_rearVehicle, sensorRange_rearVehicle, minDec_rearVehicle};
%             save('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat','T_noColi');
% end

%% Situation with collision
% clear the data in table and array before every loop of test
% load('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat');
% T_Coli(1:4,:)=[];
% save('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat','T_Coli');

% load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4.mat');
% speedV3V4 = [];
% save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4.mat','speedV3V4');

nr_Simulations = 4;   
for k = 1:nr_Simulations
    prepare_simulator("Analysing",0,"FI_id",3,"FI_value",0.45*k)
    evalin('base', 'run_Sim');
    
    % Create table
    % T_Coli = cell2table(cell(0,8), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'rela_impactSpeed', 'AEBdistance_rearVehicle', 'sensorRange_rearVehicle', 'minDec_rearVehicle'});
    load('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat');
    %load speed of v3 and v4
    load('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4.mat');
    
    % Log Data + Add Data to Table
    relaSpeed_calcu = [allTestData(1,3,:); allTestData(1,4,:)];
    speedV3V4 = [speedV3V4; relaSpeed_calcu]; % speed of v3 and v4 for plotting
    save('.\src\Misc\Safety Evaluation Report\SpeedPlotV3V4.mat','speedV3V4');
    
    %remove 0 in the speed array for finding relative impact speed
    relaSpeed_calcu(find(relaSpeed_calcu==0)) = [];
    
    idx = height(T_Coli);
    
    front_vehicleID = Vehicles(3).id;
    rear_vehicleID = Vehicles(4).id;
    maxspeed_rearVehicle = Vehicles(3).dynamics.maxSpeed;
    has_collision = Vehicles(3).status.collided;
    if has_collision ==1
        rela_impactSpeed = relaSpeed_calcu(1,end-1)-relaSpeed_calcu(1,end); %relative impact speed calculation
    else
        rela_impactSpeed = 1000; % if no collision
    end
    AEBdistance_rearVehicle = Vehicles(3).sensors.AEBdistance;
    sensorRange_rearVehicle = Vehicles(3).sensors.frontSensorRange;
    minDec_rearVehicle = Vehicles(3).dynamics.minDeceleration;
    
    T_Coli(idx+1,:) = {front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, rela_impactSpeed, AEBdistance_rearVehicle, sensorRange_rearVehicle, minDec_rearVehicle};
    save('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat','T_Coli');
end
        


%% TODO: function to calculate min distance and minTTC for no collision condition

    %toc;
    
% function maxSpeeds = changeMaxSpeedofAVehicle(maxSpeeds,vehicleid,FiVal)
% % Change the corresponding vehicle's maxSpeed
%   maxSpeeds(vehicleid) = maxSpeeds(vehicleid) + FiVal;
% end

