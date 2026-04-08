function h = bar_h_dubins(x, Xs, Xu, a)
    % Compute h(x) = dist_inf(x, (Xad_bar)^c)
    
    x = x(:);
    a = a(:);
    
    % Tightened outer box
    I_s = interval(Xs);
    outer_lb = infimum(I_s) + a;
    outer_ub = supremum(I_s) - a;
    
    % Outside tightened outer box
    if any(x < outer_lb) || any(x > outer_ub)
        h = 0;
        return;
    end

    d_obs_min = inf;
    
    for i = 1:length(Xu)
        % Enlarged obstacle
        I_u = interval(Xu{i});
        obs_lb = infimum(I_u) - a;
        obs_ub = supremum(I_u) + a;
    
        % Inside enlarged obstacle
        if all(x >= obs_lb) && all(x <= obs_ub)
            h = 0;
            return;
        end
    
        % inf-norm distance from point to box
        d_obs_i = max(max([obs_lb - x, zeros(size(x)), x - obs_ub], [], 2));
        d_obs_min = min(d_obs_min, d_obs_i);
    end

    % Distance to outer boundary
    d_outer = min([x - outer_lb; outer_ub - x]);
    
    if isempty(Xu)
        h = d_outer;    
    else
        h = min(d_outer, d_obs_min);
    end
end