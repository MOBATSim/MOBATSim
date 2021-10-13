function p = plotTS(S, Act, Tr, I, AP, L)
%PLOTTS Plot a transition system as digraph
%   Detailed explanation goes here


% Digraph
% table with all states
NodeTable = table(S', L(:,2),'VariableNames',{'Name' 'Label'});
% table with all edges (transitions) and labled with actions
EdgeTable = table([Tr(:,1) Tr(:,2)], Tr(:,3), 'VariableNames',{'EndNodes' 'Code'});
% build digraph with it
graph = digraph(EdgeTable,NodeTable);
figure(2)
p = plot(graph,'EdgeLabel', graph.Edges.Code, 'NodeLabel', graph.Nodes.Label);
end

