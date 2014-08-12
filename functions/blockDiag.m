function M = blockDiag(varargin)
% Creates a block diagonal matrix
    %
    % M = blockDiag(M1, M2)
    % M is a block diagonal matrix composed of M1, M2
    %     [ M1 |  0 ]
    % M = [ ------- ]
    %     [ 0  | M2 ]
    %
    % M = blockDiag(M1, M2, M3, ..., Mn)
    %   M can be a block diagonal consisting of n matrices.
    
    n = nargin;
    if n < 1 
        M = []; 
        return; 
    end
    
    % Quickly go through input and verify dimensionality along
    % with preparing for memory allocation
    d1 = zeros(1,n);
    d2 = zeros(1,n);
    for i=1:n
        if length(size(varargin{i})) > 2
            error('All matrices must be 2D');
        end
        
        d1(i) = size(varargin{i},1);
        d2(i) = size(varargin{i},2);
    end
    
    % Finally, set complete block diagonal matrix.
    D1 = 1;
    D2 = 1;
    M = zeros(sum(d1),sum(d2));
    for i=1:n
        M(D1:D1+d1(i)-1,D2:D2+d2(i)-1) = varargin{i};
        D1 = sum(d1(1:i))+1;
        D2 = sum(d2(1:i))+1;
    end
end