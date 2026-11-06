function p = vehicle_params()
    % VEHICLE_PARAMS Returns a struct with Minerva FSAE baseline parameters
    % and linear 2-DOF Bicycle Model state-space matrices.

    % --- Vehicle physical parameters ---
    p.m  = 355;             % Vehicle mass [kg]
    p.I  = 182;             % Yaw moment of inertia (Izz) [kg*m^2]
    p.a  = 0.886;           % Distance from CG to front axle [m]
    p.b  = 0.669;           % Distance from CG to rear axle [m]
    p.L  = p.a + p.b;       % Wheelbase [m]
    p.g  = 9.81;            % Gravitational acceleration [m/s^2]

    % --- Tire static cornering stiffness (axle values) ---
    p.Cf = 46000;           % Front axle cornering stiffness [N/rad]
    p.Cr = 62000;           % Rear axle cornering stiffness [N/rad]

    % --- Operating condition ---
    p.u  = 15;              % Longitudinal velocity [m/s]

    % --- Stability metrics ---
    p.eta = (p.m * p.g * (p.b * p.Cr - p.a * p.Cf)) / (p.Cf * p.Cr * p.L); % Understeer gradient [rad]
    p.u_crit = sqrt(-(p.g * p.L) / p.eta);                                 % Critical speed [m/s]

    % --- State-Space Matrices (x = [v; r], u_in = delta) ---
    p.A = [ -(p.Cf + p.Cr) / (p.m * p.u),        (-p.a * p.Cf + p.b * p.Cr) / (p.m * p.u) - p.u;
            -(p.a * p.Cf - p.b * p.Cr) / (p.I * p.u), -(p.a^2 * p.Cf + p.b^2 * p.Cr) / (p.I * p.u) ];

    p.B = [ p.Cf / p.m;
           (p.a * p.Cf) / p.I ];

    % Output vector y = [r; v; ay; beta]
    p.C = [ 0, 1;
            1, 0;
           -(p.Cf + p.Cr) / (p.m * p.u), (p.b * p.Cr - p.a * p.Cf) / (p.m * p.u);
            1 / p.u, 0 ];

    p.D = [ 0;
            0;
            p.Cf / p.m;
            0 ];
end