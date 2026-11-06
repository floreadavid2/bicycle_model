function dxdt = bicycle_ode(t, x, delta_func, p)
    % BICYCLE_ODE Computes state derivatives dxdt = [dot_v; dot_r]
    % Inputs:
    %   t          - Current time [s]
    %   x          - State vector [v; r]
    %   delta_func - Function handle for steering angle input delta(t) [rad]
    %   p          - Vehicle parameter struct

    delta = delta_func(t);
    
    % Linear state equation: dot(x) = A*x + B*delta
    dxdt = p.A * x + p.B * delta;
end