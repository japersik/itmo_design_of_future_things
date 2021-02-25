%% init data
Phi = 1;                       % Desired motor position
%% Motors parameters
Umax = 8.2;                     % DC motor voltage [V]
Jd = 0.00237;					% DC motor inertia moment [kgm^2]
Jm = 0.001;                     % Body inertia moment [kgm^2]
J = Jd + Jm;                    % Full inertia moment [kgm^2]
R = 3;                          % DC motor resistance [Om]
km = 0.274;                     % Motor coefficient
ke = 0.274;                     % Motor coefficient
kf = 0.01;                      % Сoefficient of friction
L = 0.0047;                     % Inductance
%% Matrix for State Space
A = [0  1   0
    0   0   1
   0 -(R*kf+km*ke)/(L*J) -(R*J+L*kf)/(L*J) ];
B = [0
    0
    km/(L*J)];
C = [1 0 0];