classdef GridLocation
    %Every grid location needs an object to store data
    
    properties
        coordinates = [];       %[x,y]
        successors = [];        %[every GridLocation we can reach from this GL]
        predecessors = [];      %[every GL from which we can reach this GL]
        transMapSucc;           %key = start node of edge, value = successor key "x,y"
        transMapPred;           %key = end node of edge, value = predecessor key "x,y"
        gValue = 0;             % G value for search
        fValue = 0;             % F value for search
        nodeNR;                 % 0 = no node, NR of a node in graph
        edgeNR;                 % 0 = no edge, NR of edge it belongs
        distance=0;             %distance to get over the GL
        parent;                 %GL that came before this one on our path
        edgeStart;              %node nr of the start of the current edge
        totalDistance;          %distance travelled so far
        speedLimit = 0;             %maximum speed on this GL
        deviation = 0;          %standard deviation
        blocked;                %true = blocked
        
        speedVector;            %the average speed of every vehicle on this GL
        timeVector;             %the estimated time a vehicle will arrive on this GL
        probabilityVector;      %standard deviation of each vehicle
    end
    
    methods
        %% constructor
        function obj = GridLocation(xy,nrOfCars,node,edge)
            %Construct an instance of this class
            obj.coordinates = xy;
            obj.nodeNR = node;
            obj.edgeNR = edge;
            obj.transMapSucc = containers.Map();
            obj.transMapPred = containers.Map();
            obj.parent = 0;
            obj.totalDistance = 0;
            obj.blocked = false;
            
            obj.speedVector = zeros(nrOfCars,1);
            obj.timeVector = zeros(nrOfCars,1);
            obj.probabilityVector = zeros(nrOfCars,1);
        end
        
        %% alter properties
        function obj = addSucc(obj,succ)
            if isempty(obj.successors) || ~ismember(succ,obj.successors)
                obj.successors = [obj.successors,succ];
            end
        end
        function obj = addPred(obj,pred)
            if isempty(obj.predecessors) || ~ismember(pred,obj.predecessors)
                obj.predecessors = [obj.predecessors,pred];
            end
        end
        function obj = addTransSucc(obj,succ,startNodeNR)
            if obj.nodeNR ~= 0
                %if we are a node
                obj = obj.addSucc(succ);
            else
                %if we are a part of an edge
                if obj.transMapSucc.isKey(num2str(startNodeNR))
                    oldSucc = obj.transMapSucc(num2str(startNodeNR));
                    if ~ismember(succ,oldSucc)
                        obj.transMapSucc(num2str(startNodeNR))=[oldSucc,succ];
                    end
                else
                    obj.transMapSucc(num2str(startNodeNR))=succ;
                end
            end
        end
        function obj = addTransPred(obj,pred,endNodeNR)
            if obj.nodeNR ~= 0
                %if we are a node
                obj = obj.addPred(pred);
            else
                %if we are a part of an edge
                if obj.transMapPred.isKey(num2str(endNodeNR))
                    oldPred = obj.transMapPred(num2str(endNodeNR));
                    if ~ismember(pred,oldPred)
                        obj.transMapPred(num2str(endNodeNR))=[oldPred,pred];
                    end
                else
                    obj.transMapPred(num2str(endNodeNR))=pred;
                end
            end
        end
        function obj = assignDistance(obj,newDistance)
            if obj.distance == 0
                obj.distance = newDistance;
            else
                obj.distance = (obj.distance + newDistance)/2;
            end
        end
        
    end
end

