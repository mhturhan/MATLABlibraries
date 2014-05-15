function sol = lieDerivative(h,f,q)
% The lie derivative of h with respect to f, over the variables q
% Returns
%   The Lie derivative

% Initialize
sol = sym(zeros(size(h,1),size(f,2)));

% For each column of f
for a = 1:size(f,2)
    % For each row of h
    for b = 1:size(h,1)
        % Sum over all of the states
        for c = 1:length(q)
            sol(b,a) = sol(b,a) + diff(h(b),q(c))*f(c,a);
        end % for c
    end % for b
end % for a


end % function sol
