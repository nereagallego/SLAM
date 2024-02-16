function H = SINGLES (prediction, observations, compatibility)
%-------------------------------------------------------
% University of Zaragoza
% Authors:  J. Neira, J. Tardos
%-------------------------------------------------------
%-------------------------------------------------------
global chi2;
global configuration;

H = zeros(1, observations.m);

% You have observations.m observations, and prediction.n
% predicted features.
%
% For every observation i, check whether it has only one neighbour,
% say feature j, and whether that feature j  has only that one neighbour
% observation i.  If so, H(i) = j.
%
% You will need to check the compatibility.ic matrix
% for this:
%
% compatibility.ic(i,j) = 1 if observation i is a neighbour of
% feature j.
            
for i = 1:observations.m
    % Check if observation i has only one neighbor
    neighbors = find(compatibility.ic(i,:)); % Find indices of neighbors of observation i
    if numel(neighbors) == 1
        % Check if the neighbor has only one neighbor (which is observation i)
        neighbor_index = neighbors(1);
        if sum(compatibility.ic(:, neighbor_index)) == 1
            % Set H(i) = neighbor_index
            H(i) = neighbor_index;
        end
    end
end
