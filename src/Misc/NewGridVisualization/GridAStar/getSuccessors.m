function [successors,lastNodeNR] = getSuccessors(curGL,startNodeNR)
            %returns the successors of the grid location curGL
            if curGL.nodeNR ~= 0
                    %node
                    successors = curGL.successors;
                    lastNodeNR = curGL.nodeNR;
                else
                    %part of edge
                    successors = curGL.transMapSucc(num2str(startNodeNR));                    
                    lastNodeNR = curGL.edgeStart;
            end
        end