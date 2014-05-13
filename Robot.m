classdef Robot < handle

    properties(Access=public)
        X = zeros(3,1); % [x y theta]
        U = zeros(2,1); % [v w]
        
        controller = {};
    end
    
    
   
end
