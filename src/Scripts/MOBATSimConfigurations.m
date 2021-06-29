classdef MOBATSimConfigurations
    %MOBATSimConfigurations Summary of this class goes here
    %   Detailed explanation goes here
    properties
        modelName
        MapType
        Sim_Ts
    end
    
    methods
        function obj = MOBATSimConfigurations(modelName,Sim_Ts,MapType)
            obj.modelName = modelName;
            obj.Sim_Ts = Sim_Ts;
            obj.MapType = MapType;
            
        end
        
    end
end

