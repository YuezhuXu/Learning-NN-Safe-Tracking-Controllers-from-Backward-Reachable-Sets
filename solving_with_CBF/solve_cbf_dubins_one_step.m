function [u_opt, exitflag] = solve_cbf_dubins_one_step(xk, xref_next, gamma_k, Xs, Xu, er, Ul, Uu, dynamics_fcn)    % Solve one-step CBF tracking problem
    
    xk = xk(:);
    xref_next = xref_next(:);
    Ul = Ul(:);
    Uu = Uu(:);
    
    % Initial guess from least-squares tracking
    A = [cos(xk(3)), 0;
         sin(xk(3)), 0;
         0,          1];
    u0 = pinv(A) * (xref_next - xk);
    u0 = min(max(u0, Ul), Uu);
    
    % Tracking objective
    obj = @(u) norm(dynamics_fcn(xk,u) - xref_next, Inf);
    
    % CBF constraint
    nonlcon = @(u) cbf_nonlcon(u, xk, gamma_k, Xs, Xu, er, dynamics_fcn);
    
    options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'Display', 'none', ...
        'MaxIterations', 200, ...
        'MaxFunctionEvaluations', 5000);
    
    
    [u_opt, ~, exitflag] = fmincon(obj, u0, [], [], [], [], Ul, Uu, nonlcon, options);
    end
    
function [c, ceq] = cbf_nonlcon(u, xk, gamma_k, Xs, Xu, er, dynamics_fcn)
    
    hk = bar_h_dubins(xk, Xs, Xu, er);
    hnext = bar_h_dubins(dynamics_fcn(xk,u), Xs, Xu, er);
    
    c = [
    (1 - gamma_k) * hk - hnext;
    u(1) - 0.4;
   -u(1) - 0.4;
    u(2) - 0.25;
   -u(2) - 0.25
    ];
    ceq = [];
end