classdef MOBATSimConfigurations
    %MOBATSimConfigurations Contains the configuration of the current
    %simulation
    %   Detailed explanation goes here
    properties
        modelName
        mapName
        simStopTime
        simTs        
        scenarioName
        startingPoints
        destinationPoints
        maxSpeeds
        analysing           % analysing mode activated
        simpleMap           % Displays the map in a simpler way
    end
    
    methods
        function obj = MOBATSimConfigurations(modelName, mapName, simStopTime, simTs,  scenarioName, startingPoints, destinationPoints, maxSpeeds, analysing, simpleMap)
            % Save the current configuration
            obj.modelName = modelName;
            obj.mapName = mapName;
            obj.simStopTime = simStopTime;
            obj.simTs = simTs;            
            obj.scenarioName = scenarioName;
            obj.startingPoints = startingPoints;
            obj.destinationPoints = destinationPoints;
            obj.maxSpeeds = maxSpeeds;
            obj.analysing = analysing;
            obj.simpleMap = simpleMap;
        end
        
        function differences = compareConfigurations(obj, modelName, mapName, simStopTime, simTs,  scenarioName, startingPoints, destinationPoints, maxSpeeds, analysing, simpleMap)
            % compare the input values with this configuration
            
            differences = [ obj.modelName           ~= modelName; ...
                            obj.mapName             ~= mapName; ...
                            obj.simStopTime         ~= simStopTime; ...
                            obj.simTs               ~= simTs; ...
                            obj.scenarioName        ~= scenarioName; ...
                            any(obj.startingPoints      ~= startingPoints); ...
                            any(obj.destinationPoints   ~= destinationPoints); ...
                            any(obj.maxSpeeds           ~= maxSpeeds); ...
                            obj.analysing           ~= analysing; ...
                            obj.simpleMap           ~= simpleMap];
         
        end
        
    end
end

