classdef Pedestrian < handle

    properties
        position
        positionLocal
        destinationLocal
        positionGlobal
        destinationGlobal
        direction
        map                 %for boundaries
        mapnow              %for occupation
        buffer
        Dmax
        s_ij
        conversion
        col
    end
    
    methods
        function obj = Pedestrian(pos,dest,ori,col,map)
            
            obj.position=pos(1:2)
            obj.positionLocal = [(pos(1)-65)*2 (pos(2)+160)*2];
            obj.destinationLocal=[(dest(1)-65)*2 (dest(2)+160)*2];
            obj.direction=ori;
            obj.map=map;
            obj.col=col
            
            occM=occupancyMatrix(map);
%             obj.conversion=size(occM,1)/2+0.5;
            obj.conversion=size(occM,1)+1
%             obj.positionGlobal=[size(occM,1)/2+0.5-obj.positionLocal(2),obj.positionLocal(1)];
            obj.positionGlobal=[size(occM,1)+1-obj.positionLocal(2),obj.positionLocal(1)];
%             obj.destinationGlobal=[size(occM,1)/2+0.5-obj.destinationLocal(2),obj.destinationLocal(1)];
            obj.destinationGlobal=[size(occM,1)+1-obj.destinationLocal(2),obj.destinationLocal(1)];
            for i=1:size(occM,1)
                for j=1:size(occM,2)
                    d(i,j)=sqrt((obj.destinationGlobal(1)-i)^2+(obj.destinationGlobal(2)-j)^2);
                end
            end
%             k=1:40;
%             for i=1:size(occM,1)
%                 for j=1:size(occM,2)
% %                     for k=1:6
%                         d(i,j)=min(sqrt((obj.destinationGlobal(1)-i)^2+(k-j).^2));
% %                     end
%                 end
%             end
            
            d;
            obj.Dmax=max(d,[],'all');
            obj.s_ij=d/obj.Dmax;
            sff=occupancyMap(obj.s_ij)
%             f1=figure
%             show(sff)
        end
        
        function desnewpos = findnewpos(obj,mapnow,info2)
            for i=1:size(info2,1)
                info(i,:)=[obj.conversion-2*info2(i,2),2*info2(i,1)];
            end
            
            obj.mapnow=mapnow;
%             if any(obj.positionLocal ~= obj.destinationLocal)
            if obj.positionLocal(2) ~= obj.destinationLocal(2)
                
                obj.positionGlobal=[obj.conversion-obj.positionLocal(2),obj.positionLocal(1)];

                left=[obj.positionGlobal(1),obj.positionGlobal(2)-obj.direction];            
                front=[obj.positionGlobal(1)-obj.direction,obj.positionGlobal(2)];
                right=[obj.positionGlobal(1),obj.positionGlobal(2)+obj.direction];
                neighborsGlobal=[left;front;right]
                S(1)=obj.s_ij(left(1),left(2));
                S(2)=obj.s_ij(front(1),front(2));
                S(3)=obj.s_ij(right(1),right(2));
%                 S(1)=sqrt((obj.destinationGlobal(1)-left(1))^2+(obj.destinationGlobal(2)-left(2))^2);
%                 S(2)=sqrt((obj.destinationGlobal(1)-front(1))^2+(obj.destinationGlobal(2)-front(2))^2);
%                 S(3)=sqrt((obj.destinationGlobal(1)-right(1))^2+(obj.destinationGlobal(2)-right(2))^2);
%                 neighborsLocal=[neighborsGlobal(:,2)/2,(obj.conversion-neighborsGlobal(:,1))/2]
                neighborsLocal=[neighborsGlobal(:,2),(-neighborsGlobal(:,1)+obj.conversion)]
                eps=getOccupancy(obj.mapnow,[neighborsLocal])';
                eps=~(eps>0.1);
                SS=S/obj.Dmax;
                
                %repulsive force
                leftfield=[left;[left(1)-2*obj.direction left(2)];[left(1)-obj.direction left(2)]];
                frontfield=[front;[front(1)-obj.direction front(2)];[front(1)-2*obj.direction front(2)]];
                rightfield=[right;[right(1)-2*obj.direction right(2)];[right(1)-obj.direction right(2)]]; 
                
                A1=0.42;
                A2=1.2;
                B=1.65;
                
                f11=0;
                f12=0;             
                for i=1:size(leftfield,1)
                    for j=1:size(info,1)
                        if leftfield(i,1)==info(j,1) & leftfield(i,2)==info(j,2)
                    
                            bb=sqrt((obj.positionGlobal(1)-info(j,1))^2+(obj.positionGlobal(2)-info(j,2))^2)
                             theta1=abs(obj.positionGlobal(2)-info(j,2))/bb
                            
                            if sign(info2(j,3))==sign(obj.direction)                                
                                f=A1*exp(-bb/B);
                                f11=f11+f*theta1;
                            else
                                f=A2*exp(-bb/B);
                                f12=f12+f*theta1                                
                            end
                        end
                    end
                end
                
                f21=0;
                f22=0;             
                for i=1:size(frontfield,1)
                    for j=1:size(info,1)
                        if frontfield(i,1)==info(j,1) & frontfield(i,2)==info(j,2)
                    
                            bb=sqrt((obj.positionGlobal(1)-info(j,1))^2+(obj.positionGlobal(2)-info(j,2))^2)
                            theta1=1
                            
                            if sign(info2(j,3))==sign(obj.direction)                                
                                f=A1*exp(-bb/B);
                                f21=f21+f*theta1;
                            else
                                f=A2*exp(-bb/B);
                                f22=f22+f*theta1                                
                            end
                        end
                    end
                end
                
                f31=0;
                f32=0;             
                for i=1:size(rightfield,1)
                    for j=1:size(info,1)
                        if rightfield(i,1)==info(j,1) & rightfield(i,2)==info(j,2)
                    
                            bb=sqrt((obj.positionGlobal(1)-info(j,1))^2+(obj.positionGlobal(2)-info(j,2))^2)
                            theta1=abs(obj.positionGlobal(2)-info(j,2))/bb
                            
                            if sign(info2(j,3))==sign(obj.direction)                                
                                f=A1*exp(-bb/B);
                                f31=f31+f*theta1;
                            else
                                f=A2*exp(-bb/B);
                                f32=f32+f*theta1                                
                            end
                        end
                    end
                end
                                
                f1=f11+f12;
                f2=f21+f22;
                f3=f31+f32;
                fso=[f1 f2 f3];
                                
                prs=exp(-8.*SS).*eps
                pi=prs.*exp(-5*fso)
                if any(pi)==0
                    desnewpos=obj.positionLocal;
                else
                    [~,idx]=max(pi);
                    obj.buffer=neighborsLocal(idx,:);
                    desnewpos=obj.buffer;
                end
            else
                desnewpos=obj.positionLocal;
            end
        end
        
        function newpos=updatepos(obj,decision)
            if decision
                obj.positionLocal=obj.buffer;
                obj.position=[obj.buffer(1)/2+65 obj.buffer(2)/2-160];
                
            end
            newpos=[obj.position obj.col];
            
        end
    end
end
