classdef MOBATSimConfigurations
    %MOBATSimConfigurations Contains the configuration of the current
    %simulation
    %   Detailed explanation goes here
    properties
        modelName
        simStopTime
        simTs
        mapName
        scenarioName
    end
    
    methods
        function obj = MOBATSimConfigurations(modelName, simStopTime, simTs, mapName, scenarioName)
            obj.modelName = modelName;
            obj.simStopTime = simStopTime;
            obj.simTs = simTs;
            obj.mapName = mapName;
            obj.scenarioName = scenarioName;
        end
        
    end
end

