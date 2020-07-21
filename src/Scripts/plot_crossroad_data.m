additiveTimeInCrossroad = 0;

figure

for vehicle = Vehicles
    indices = find(vehicle.dataLog.speedInCrossroad == 0);
    for index = indices
        vehicle.dataLog.speedInCrossroad(index) = vehicle.dataLog.speedInCrossroad(index+1)  ;
    end
     
    plot(vehicle.dataLog.speedInCrossroad)
    hold on
    
    additiveTimeInCrossroad = additiveTimeInCrossroad + length(vehicle.dataLog.speedInCrossroad);
    
end
xlabel('simulation steps')
ylabel('speed in m/s')

energyData = [];
passingOrder = [];
additiveEnergyLoss=0;
for vehicle = Vehicles
    
   indices = find(vehicle.dataLog.speedInCrossroad2 == 0);
    for index = indices
        vehicle.dataLog.speedInCrossroad2(index) = vehicle.dataLog.speedInCrossroad2(index+1);
    end
    
    if ~isempty(vehicle.dataLog.speedInCrossroad2)
     
    speedMaximum = max(vehicle.dataLog.speedInCrossroad2(1:length(vehicle.dataLog.speedInCrossroad2)-length(vehicle.dataLog.speedInCrossroad)));
    speedMinimum = min(vehicle.dataLog.speedInCrossroad2);
    energyLoss = 0.001 * 0.5* vehicle.physics.mass * (speedMaximum-speedMinimum)^2; % energy in kJ
    energyData = [energyData energyLoss];
    additiveEnergyLoss = additiveEnergyLoss + energyLoss;
    end
    
       
    
end
figure
bar(energyData);
xlabel('vehicle')
ylabel('energy loss in kJ')