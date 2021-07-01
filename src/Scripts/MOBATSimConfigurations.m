classdef MOBATSimConfigurations
    %MOBATSimConfigurations Contains the configuration of the current
    %simulation
    %   Detailed explanation goes here
    properties
        modelName
        simStopTime
        simTs
        mapType
        mapName
        scenarioName
    end
    
    methods
        function obj = MOBATSimConfigurations(modelName, simStopTime, simTs, mapType, mapName, scenarioName)
            obj.modelName = modelName;
            obj.simStopTime = simStopTime;
            obj.simTs = simTs;
            obj.mapType = mapType;
            obj.mapName = mapName;
            obj.scenarioName = scenarioName;
        end
        
    end
end

