classdef MOBATSimConfigurations
    %MOBATSimConfigurations Summary of this class goes here
    %   Detailed explanation goes here
%     enumeration
%         GridMap, DigraphMap
%     end
    properties
        modelName
        simSpeed
        MapType
        Sim_Ts
    end
    
    methods
        function obj = MOBATSimConfigurations(modelName,simSpeed,Sim_Ts,MapType)
            obj.modelName = modelName;
            obj.simSpeed = simSpeed;
            obj.Sim_Ts = Sim_Ts;
            obj.MapType = MapType;
            
        end
      
      function tf = isGridMap(obj)
         tf = strcmp(obj.MapType,'GridMap');
      end
        
    end
end

