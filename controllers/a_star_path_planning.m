function waypoints = a_star_path_planning(start, goal, map)
    % A* Path Planning Algorithm
    % start: starting position [x, y]
    % goal: goal position [x, y]
    % map: binary occupancy grid map (0: free space, 1: obstacle)

    % Initialize open and closed lists
    openList = [];
    closedList = false(size(map));

    % Add the start node to the open list
    openList = [openList; start, 0, heuristic(start, goal), 0 + heuristic(start, goal)];

    % Main loop
    while ~isempty(openList)
        % Find the node with the lowest f score
        [~, idx] = min(openList(:, 5));
        currentNode = openList(idx, :);
        openList(idx, :) = [];

        % Check if the goal is reached
        if isequal(currentNode(1:2), goal)
            waypoints = reconstruct_path(currentNode);
            return;
        end

        % Add the current node to the closed list
        closedList(currentNode(1), currentNode(2)) = true;

        % Get neighbors
        neighbors = get_neighbors(currentNode(1:2), map);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);

            % Skip if neighbor is in the closed list
            if closedList(neighbor(1), neighbor(2))
                continue;
            end

            % Calculate tentative g score
            tentative_gScore = currentNode(4) + 1; % Assuming uniform cost

            % Check if neighbor is in the open list
            inOpenList = ismember(neighbor, openList(:, 1:2), 'rows');
            if ~inOpenList || tentative_gScore < openList(inOpenList, 4)
                % Update the open list
                openList = [openList; neighbor, tentative_gScore, heuristic(neighbor, goal), tentative_gScore + heuristic(neighbor, goal)];
            end
        end
    end

    % If no path is found
    waypoints = [];
end

function h = heuristic(node, goal)
    % Heuristic function (Euclidean distance)
    h = sqrt((node(1) - goal(1))^2 + (node(2) - goal(2))^2);
end

function neighbors = get_neighbors(node, map)
    % Get neighbors of a node
    directions = [0 1; 1 0; 0 -1; -1 0];
    neighbors = [];
    for i = 1:size(directions, 1)
        neighbor = node + directions(i, :);
        if neighbor(1) > 0 && neighbor(1) <= size(map, 1) && neighbor(2) > 0 && neighbor(2) <= size(map, 2) && map(neighbor(1), neighbor(2)) == 0
            neighbors = [neighbors; neighbor];
        end
    end
end

function path = reconstruct_path(node)
    % Reconstruct the path from the goal to the start
    path = node(1:2);
    % Implement backtracking to reconstruct the path
    % Assuming that we store parent nodes in the node array
end
