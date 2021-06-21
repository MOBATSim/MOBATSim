%tic;

%testGoal = 'noCollision';
%test goal = 'Collision';

%% Create table
% switch testGoal
%     case 'noCollision'
%         T_noColi = cell2table(cell(0,9), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'min_distance', 'min_TTC' 'AEBdistance_rearVehicle', 'sensorRange_rearVehicle', 'minDec_rearVehicle'});
% %         load('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat');
%     case 'Collision'
%         T_Coli = cell2table(cell(0,8), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'rela_impactSpeed', 'AEBdistance_rearVehicle', 'sensorRange_rearVehicle', 'minDec_rearVehicle'});
% end
%% delete the useless data from the table for next test
% load('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat');
% T_noColi(1:9,:)=[]; % delet all the data in the table for the next test
% save('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat','T_noColi');
%% Start several loops of simulation
nr_Simulations = 4;   
for k = 1:nr_Simulations
    %switch testGoal
       % case 'noCollision'
            prepare_simulator("Analysing",0,"FI_id",3,"FI_value",-k) % for every loop the speed of vehicle 3 is changed by "k"
            evalin('base', 'run_Sim');
            
            % Create table
            %T_noColi = cell2table(cell(0,9), 'VariableNames', {'front_vehicleID', 'rear_vehicleID', 'maxspeed_rearVehicle','has_collision', 'min_distance', 'min_TTC' 'AEBdistance_rearVehicle', 'sensorRange_rearVehicle', 'minDec_rearVehicle'});
            load('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat');
           
            % Log Data + Add Data to Table
            idx = height(T_noColi);
            
            front_vehicleID = Vehicles(3).id;
            rear_vehicleID = Vehicles(4).id;
            maxspeed_rearVehicle = Vehicles(3).dynamics.maxSpeed;
            has_collision = Vehicles(3).status.collided;
            min_distance = 1;
            min_TTC=1;
            AEBdistance_rearVehicle = Vehicles(3).sensors.AEBdistance;
            sensorRange_rearVehicle = Vehicles(3).sensors.frontSensorRange;
            minDec_rearVehicle = Vehicles(3).dynamics.minDeceleration;
            
            T_noColi(idx+1,:) = {front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, min_distance,min_TTC, AEBdistance_rearVehicle, sensorRange_rearVehicle, minDec_rearVehicle};
            save('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat','T_noColi');
            
        %case 'Collsion'
         %   prepare_simulator("Analysing",0,"FI_id",3,"FI_value",1.4*k)
          %  evalin('base', 'run_Sim');
            
            % Log Data + Add Data to Table
              %idx = height(T_noColi);
            %for i = 1:nr_Simulations
%                 front_vehicleID = Vehicles(3).id; 
%                 rear_vehicleID = Vehicles(4).id;
%                 maxspeed_rearVehicle = Vehicles(3).dynamics.maxSpeed;
%                 has_collision = Vehicles(3).status.collided;
%                 rela_impactSpeed = 1;
%                 AEBdistance_rearVehicle = Vehicles(3).sensors.AEBdistance;
%                 sensorRange_rearVehicle = Vehicles(3).sensors.frontSensorRange;
%                 minDec_rearVehicle = Vehicles(3).dynamics.minDeceleration;
%                 
%                 T_Coli(k,:) = {front_vehicleID, rear_vehicleID, maxspeed_rearVehicle, has_collision, rela_impactSpeed, AEBdistance_rearVehicle, sensorRange_rearVehicle, minDec_rearVehicle};
   % end
        
end

%% TODO: function to calculate min distance and minTTC for no collision condition
%% TODO: another part with collision situation

% switch testGoal
%     case 'noCollision'
%         save('.\src\Misc\Safety Evaluation Report\test_noColiNormal.mat','T_noColi');
%     case 'Collision'  
%         save('.\src\Misc\Safety Evaluation Report\test_ColiNormal.mat','T_Coli');
% end
    %toc;
    
% function maxSpeeds = changeMaxSpeedofAVehicle(maxSpeeds,vehicleid,FiVal)
% % Change the corresponding vehicle's maxSpeed
%   maxSpeeds(vehicleid) = maxSpeeds(vehicleid) + FiVal;
% end

