xe% ======
% ROB521_assignment1.m
% ======
%
% This assignment will introduce you to the idea of motion planning for  
% holonomic robots that can move in any direction and change direction of 
% motion instantaneously.  Although unrealistic, it can work quite well for
% complex large scale planning.  You will generate mazes to plan through 
% and employ the PRM algorithm presented in lecture as well as any 
% variations you can invent in the later sections.
% 
% There are three questions to complete (5 marks each):
%
%    Question 1: implement the PRM algorithm to construct a graph
%    connecting start to finish nodes.
%    Question 2: find the shortest path over the graph by implementing the
%    Dijkstra's or A* algorithm.
%    Question 3: identify sampling, connection or collision checking 
%    strategies that can reduce runtime for mazes.
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plots, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file.
%
% requires: basic Matlab, 
%
% S L Waslander, January 2022
%
clear; close all; clc;

% set random seed for repeatability if desired
% rng(1);

% ==========================
% Maze Generation
% ==========================
%
% The maze function returns a map object with all of the edges in the maze.
% Each row of the map structure draws a single line of the maze.  The
% function returns the lines with coordinates [x1 y1 x2 y2].
% Bottom left corner of maze is [0.5 0.5], 
% Top right corner is [col+0.5 row+0.5]
%

row = 5; % Maze rows
col = 7; % Maze columns
map = maze(row,col); % Creates the maze
start = [0.5, 1.0]; % Start at the bottom left
finish = [col+0.5, row]; % Finish at the top right

h = figure(1);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

% ======================================================
% Question 1: construct a PRM connecting start and finish
% ======================================================
%
% Using 500 samples, construct a PRM graph whose milestones stay at least 
% 0.1 units away from all walls, using the MinDist2Edges function provided for 
% collision detection.  Use a nearest neighbour connection strategy and the 
% CheckCollision function provided for collision checking, and find an 
% appropriate number of connections to ensure a connection from  start to 
% finish with high probability.


% variables to store PRM components
nS = 500;  % number of samples to try for milestone creation
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

disp("Time to create PRM graph")
tic;
% ------insert your PRM generation code here-------

%sample configurations randomly with 0.1 distance away from all walls
%Populate milestones

%Counter for milestone generation
n = 0;

while n < nS
    %generate random x and y coord, with 0.5, 0.5 being the starting point of the maze 
    x = col*rand()+0.5;
    y = row*rand()+0.5;
    
    %Wall check
    if (0.1 <= MinDist2Edges([x, y],map))
        %Add to milestones/configurations if valid
        milestones(n+1,:) = [x, y];        
        n = n + 1;
    end
end

%Add connection between milestones
%Loop over all points and use built in knnsearch
%Append start and finish points to milestones for Graph
milestones = [start; milestones];
milestones = [milestones; finish;];

%edge milestone index column vector for knn, column vector
%edgesind = [];

for i = 1:nS+2 %500 + start and end pts
    % milestone 1
    ptA = milestones(i,:); %query pt
    
    %Find 12 closest neighbors to ptA to ensure a connection
    knn = knnsearch(milestones, ptA, 'K', 10); %includes self, use 3 for testing basic
    %outputs indices of closest neighbors in an array
    
    %iterature through knn here
    for j = 1:length(knn)
        % load milestone 2 from knn
        ptB = milestones(knn(j),:);
        % Check if theres no collision between points
        if CheckCollision(ptA, ptB, map) == 0
            % Append to edges
            edges(end+1,:) = [ptA, ptB];
        end
    end
end


% ------end of your PRM generation code -------
toc;

figure(1);
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta') % line uses [x1 x2 y1 y2]
end
str = sprintf('Q1 - %d X %d Maze PRM', row, col);
title(str);
drawnow;

print -dpng assignment1_q1.png


% =================================================================
% Question 2: Find the shortest path over the PRM graph
% =================================================================
%
% Using an optimal graph search method (Dijkstra's or A*) , find the 
% shortest path across the graph generated.  Please code your own 
% implementation instead of using any built in functions.

disp('Time to find shortest path');
tic;

% Variable to store shortest path
spath = []; % shortest path, stored as a milestone row index sequence


% ------insert your shortest path finding algorithm here-------


%Implement A* and use graph made in part 1
%Assign heuristic to milestones/nodes using euclidean distance to endpoint
%Lower bounded
%Set heuristic value for start and finish to 0

heur = [];
heur(1,:) = 0;
heur(nS+2,:) = 0;

for i = 2:nS+1
    %add heuristic to randomly sampled milestones
    heur(i,:) = norm(finish - milestones(i,:)); %euclidean distance
end

%openSet = [(x,y), index of milestone location]
openSet = [start, 1];

%list of came froms from iterations
%[current(x,y) index]
%cameFrom = [start];
cameFrom = [0]; %start index know its 1

%g(n)
%Set scores of milestones to end/finish goal including start and end goal
g = ones(nS+2,1)*Inf;
g(1) = 0; %Set start score to 0

%f(n) = g(n) + h(n)
%Set scores of milestone to end/finish goal a current best guess
f = ones(nS+2,1)*Inf;
f(1) = 0; %Set start score to 0

while ~isempty(openSet) %if open set is not empty continue
    %OpenSet node (location) with lowest f(n) value
    %Setup that nodes are organized in the same order in arrays

    %Find indices of OpenSet nodes in milestones
    %Make array and find minimum of f(n)
    ind = openSet(:,3); %indices of milestones within openSet
    fOpen = f(ind); %values of f(n) in open set, resets fOpen
    [foVal, foin] = min(fOpen); %find minimum value of f(n) in openSet and index in fOpen
    %foin corresponds to the specific index in ind
    minfOpen = ind(foin); %minfOpen is the index of the current node in milestones
    current = milestones(minfOpen,:); %whole row of coord location (x,y)
    %this should be the same as openSet(minfOpen,1:2)

    if current == finish
        %reconstruct path(cameFrom, current)
        %total path is spath
        %start from finish node
        spath = [];
        inp = nS + 2; 
        while inp ~= 1
            %disp(inp)
            %append to spath (so its start -> end)
            spath = [inp spath];
            %take new index from inp
            inp = cameFrom(inp);
        end
        spath = [inp spath];
        break
    end

    %Remove current from openSet (whole row)
    %Current index in openSet is 
    openSet(foin,:) = []; 

    %Find neighbours of current node
    %loop over edges to find neighbors in both columns
    %append points to neighbours in both columns
    %reset neighbors for each loop since current changes
    neighbors = [];

    for i = 1:size(edges,1) %rows
        if edges(i, 1:2) == current
            neighbors(end+1,:) = edges(i,3:4);
        end
        if edges(i, 3:4) == current
            neighbors(end+1,:) = edges(i,1:2);
        end      
    end

    %remove duplicate rows since assuming bidirectional movement
    neighbors = unique(neighbors, 'rows'); %correct

    temp = [];

%     %remove current node from neighbors, didnt add
%     for i = 1:size(neighbors, 1)
%         if neighbors(i,:) ~= current
%             temp(end+1,:) = neighbors(i,:);
%         end
%     end


    %loop over each neighbour of current node
    for i = 1:size(neighbors, 1) %rows of neighbors
        pt = neighbors(i,:);

        %g(n) of current node using index from min f
        gcurrent = g(minfOpen);

        %tentative g(n) = g(current) + euclidean to neighbor
        tentg = gcurrent + norm(pt - current);

        %find index of neighbor pt in milestones
        [q, idx] = ismember(pt, milestones, 'rows');


        %if tentative score is less previous g score, better and record
        if tentg < g(idx)

            %add camefrom node as the current node in the corresponding index
            %to milestone
            %cameFrom(idx,:) = current;
            cameFrom(idx) = minfOpen; %current node index in milestones

            %update g and f scores
            g(idx) = tentg;
            f(idx) = tentg + heur(idx);

            %if neighbor not in open set add it
            %use openset first 2 cols and compare row data
            if ~ismember(pt,openSet(:,1:2), 'rows')
                %include index of milestone location idx
                openSet(end+1,:) = [pt idx];
                %looks like its working
            end
        end
    end
end

%if the current node is not the finish node out of loop its an error
if current ~= finish
    disp("Error: openSet empty, current != finish")
end

% ------end of shortest path finding algorithm------- 
toc;    

% plot the shortest path
figure(1);
for i=1:length(spath)-1
    plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
end
str = sprintf('Q2 - %d X %d Maze Shortest Path', row, col);
title(str);
drawnow;

print -dpng assingment1_q2.png

% ================================================================
% Question 3: find a faster way
% ================================================================
%
% Modify your milestone generation, edge connection, collision detection 
% and/or shortest path methods to reduce runtime.  What is the largest maze 
% for which you can find a shortest path from start to goal in under 20 
% seconds on your computer? (Anything larger than 40x40 will suffice for 
% full marks)


row = 41;
col = 41;
map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
milestones = [start; finish];  % each row is a point [x y] in feasible space
edges = [];  % each row is should be an edge of the form [x1 y1 x2 y2]

h = figure(2);clf; hold on;
plot(start(1), start(2),'go')
plot(finish(1), finish(2),'rx')
show_maze(map,row,col,h); % Draws the maze
drawnow;

fprintf("Attempting large %d X %d maze... \n", row, col);
tic;        
% ------insert your optimized algorithm here------

spath = []; %initialize path

%Populate milstones
%setup graph to be a uniform grid in the middle of each row and col of the
%maze

milestones = [];

for i = 0.5:1:(col+0.5)
    for j = 1:1:(row+0.5)
       milestones = [milestones; [i+0.5, j]];
    end
end

nS = size(milestones,1);

%Add connection between milestones
%Loop over all points and use built in knnsearch
%Append start and finish points to milestones for Graph
milestones = [start; milestones];
milestones = [milestones; finish;];

%edge milestone index column vector for knn, column vector
%edgesind = [];

for i = 1:nS + 2 %X without start and end + start and end pts
    % milestone 1
    ptA = milestones(i,:); %query pt
    
    %Find 12 closest neighbors to ptA to ensure a connection
    knn = knnsearch(milestones, ptA, 'K', 5); %includes self, use 3 for testing basic
    %outputs indices of closest neighbors in an array
    
    %iterature through knn here
    for j = 1:length(knn)
        % load milestone 2 from knn
        ptB = milestones(knn(j),:);
        % Check if theres no collision between points
        if CheckCollision(ptA, ptB, map) == 0
            % Append to edges
            edges(end+1,:) = [ptA, ptB];
        end
    end
end

%Implement A* and use graph made in part 1
%Assign heuristic to milestones/nodes using euclidean distance to endpoint
%Lower bounded
%Set heuristic value for start and finish to 0

heur = [];
heur(1,:) = 0;
heur(nS+2,:) = 0;

for i = 2:nS+1
    %add heuristic to randomly sampled milestones
    heur(i,:) = norm(finish - milestones(i,:)); %euclidean distance
end

%openSet = [(x,y), index of milestone location]
openSet = [start, 1];

%list of came froms from iterations
%[current(x,y) index]
%cameFrom = [start];
cameFrom = [0]; %start index know its 1

%g(n)
%Set scores of milestones to end/finish goal including start and end goal
g = ones(nS+2,1)*Inf;
g(1) = 0; %Set start score to 0

%f(n) = g(n) + h(n)
%Set scores of milestone to end/finish goal a current best guess
f = ones(nS+2,1)*Inf;
f(1) = 0; %Set start score to 0

while ~isempty(openSet) %if open set is not empty continue
    %OpenSet node (location) with lowest f(n) value
    %Setup that nodes are organized in the same order in arrays

    %Find indices of OpenSet nodes in milestones
    %Make array and find minimum of f(n)
    ind = openSet(:,3); %indices of milestones within openSet
    fOpen = f(ind); %values of f(n) in open set, resets fOpen
    [foVal, foin] = min(fOpen); %find minimum value of f(n) in openSet and index in fOpen
    %foin corresponds to the specific index in ind
    minfOpen = ind(foin); %minfOpen is the index of the current node in milestones
    current = milestones(minfOpen,:); %whole row of coord location (x,y)
    %this should be the same as openSet(minfOpen,1:2)

    if current == finish
        %reconstruct path(cameFrom, current)
        %total path is spath
        %start from finish node
        spath = [];
        inp = nS + 2; 
        while inp ~= 1
            %disp(inp)
            %append to spath (so its start -> end)
            spath = [inp spath];
            %take new index from inp
            inp = cameFrom(inp);
        end
        spath = [inp spath];
        break
    end

    %Remove current from openSet (whole row)
    %Current index in openSet is 
    openSet(foin,:) = []; 

    %Find neighbours of current node
    %loop over edges to find neighbors in both columns
    %append points to neighbours in both columns
    %reset neighbors for each loop since current changes
    neighbors = [];

    for i = 1:size(edges,1) %rows
        if edges(i, 1:2) == current
            neighbors(end+1,:) = edges(i,3:4);
        end
        if edges(i, 3:4) == current
            neighbors(end+1,:) = edges(i,1:2);
        end      
    end

    %remove duplicate rows since assuming bidirectional movement
    neighbors = unique(neighbors, 'rows'); %correct

    %loop over each neighbour of current node
    for i = 1:size(neighbors, 1) %rows of neighbors
        pt = neighbors(i,:);

        %g(n) of current node using index from min f
        gcurrent = g(minfOpen);

        %tentative g(n) = g(current) + euclidean to neighbor
        tentg = gcurrent + norm(pt - current);

        %find index of neighbor pt in milestones
        [q, idx] = ismember(pt, milestones, 'rows');


        %if tentative score is less previous g score, better and record
        if tentg < g(idx)

            %add camefrom node as the current node in the corresponding index
            %to milestone
            %cameFrom(idx,:) = current;
            cameFrom(idx) = minfOpen; %current node index in milestones

            %update g and f scores
            g(idx) = tentg;
            f(idx) = tentg + heur(idx);

            %if neighbor not in open set add it
            %use openset first 2 cols and compare row data
            if ~ismember(pt,openSet(:,1:2), 'rows')
                %include index of milestone location idx
                openSet(end+1,:) = [pt idx];
                %looks like its working
            end
        end
    end
end

%if the current node is not the finish node out of loop its an error
if current ~= finish
    disp("Error: openSet empty, current != finish")
end

% ------end of your optimized algorithm-------
dt = toc

figure(2); hold on;
plot(milestones(:,1),milestones(:,2),'m.');
if (~isempty(edges))
    line(edges(:,1:2:3)', edges(:,2:2:4)','Color','magenta')
end
if (~isempty(spath))
    for i=1:length(spath)-1
        plot(milestones(spath(i:i+1),1),milestones(spath(i:i+1),2), 'go-', 'LineWidth',3);
    end
end
str = sprintf('Q3 - %d X %d Maze solved in %f seconds', row, col, dt);
title(str);

print -dpng assignment1_q3.png

