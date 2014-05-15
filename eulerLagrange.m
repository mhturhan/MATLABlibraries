function eqs = eulerLagrange(L,q)
% Input
%   L: The lagrangian
%   q: The state variables
% Output
%   eqs: The equations of motion


% Variables needed later for manipulation
dq      = strcat({'d'},q);
ddq     = strcat({'dd'},q);
qt      = strcat(q,{'(t)'});
dqt     = strcat(dq,{'(t)'});
diffqt  = strcat({'diff('},qt,{', t)'});
diffdqt = strcat({'diff('},dqt,{', t)'});
syms t real

% The Euler-Lagrange equation
% d/dt*pL/pdq - pL/pq = 0

% For each state variable
for n = 1:length(q)
    % Differentiate w.r.t. the state variable derivative
    eqs(n) = diff(L,dq(n));
    % Add t
    eqs(n) = subs(eqs(n),[q dq],[qt dqt]);
    % Differentiate w.r.t. time
    eqs(n) = diff(eqs(n),t);
    % Remove time from the equation
    eqs(n) = subs(eqs(n),[diffqt diffdqt],[dq ddq]);
    eqs(n) = subs(eqs(n),[qt dqt],[q dq]);
    % Subtract the second term
    eqs(n) = eqs(n) - diff(L,q(n));
    % Simplify
    eqs(n) = simplify(eqs(n),'IgnoreAnalyticConstraints',true);
end % for n

end % function eqs
