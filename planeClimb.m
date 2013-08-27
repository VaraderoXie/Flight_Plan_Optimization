function fuelburn = planeClimb(planeNumber, climbAngle, climbAltitude)
% Compute the fuel consumption during takeoff based on aircraft type, 
% takeoff climb angle, and the final desired altitude.
% PLANECLIMBE(plane_number, climbAngle, climbAltitude) returns the fuel 
% consumption in lbs. 

%% Plane number options are:
% 1 - 737-300
% 2 - 737-800
% 3 - A320
% 4 - A340
% 5 - 777-200
switch plane_number
    case 1
        % 737-300 aircraft parameters
        S = 105.4;          % Wing area [m^2]
        m0 = 62800.0;       % Maximum takeoff weight [kg]
        lf = 20100;         % Maximum fuel capacity [l]
        alpha = 0.0735;     % Parameter defining the dimensional velocity
        SFC = 0.03977;      % Coefficient fuel burn
    case 2
        % 737-800 aircraft parameters
        S = 125.58;         % Wing area [m^2]
        m0 = 79100;         % Maximum takeoff weight [kg]
        lf = 26020;         % Maximum fuel capacity [l]
        alpha = 0.0697;     % Parameter defining the dimensional velocity
        SFC = 0.03875;      % Coefficient fuel burn
    case 3
        % A320 aircraft parameters
        S = 125;    % Wing area [m^2]
        m0 = 70000; % Maximum takeoff weight [kg]
        lf = 24050; % Maximum fuel capacity [l]
        alpha = 0.07;   % Parameter defining the dimensional velocity
        SFC = 0.03467;  % Coefficient fuel burn
    case 4
        % Characteristics of the aircraft A340
        S = 361.6; % Wing area [m^2]
        m0 = 275000; % Maximum takeoff weight [kg]
        lf = 155040;    % Maximum fuel capacity [l]
        alpha = 0.0663; % Parameter defining the dimensional velocity
        SFC = 0.03263;  % Coefficient fuel burn
    case 5
        % Characteristics of the aircraft 777-200
        S = 427.8;  % Wing area [m^2]
        m0 = 247200;    % Maximum takeoff weight [kg]
        lf = 117348;    % Maximum fuel capacity [l]
        alpha = 0.0648; % Parameter defining the dimensional velocity
        SFC = 0.03365;  % Coefficient fuel burn
    otherwise
        warning('Invalid plane number.');
end

%% Parameters and constants

beta = 60000;   % Velocity/altitude parameter
gamma = climbAngle; % A number between 150-300
zc = climbAltitude;   % Cruise altitude [m]
rho0 = 1.225;   % Air density at sea level [kg/m^3]
epsilon = 0.0001;   % Constant in the density function [l/m]
k = 0.045;  % Constant selected for the included drag coefficient
CDo = 0.015;    % Constant selected for the zero-lift drag coefficient

%% Simulate Climb
n = length(0.01:0.001:0.99);
nz = n;

% Initialization at t = 0 (takeoff)
% The index 1 is for the climb values
V1 = zeros(nz, 1); v1 = zeros(nz, 1); theta1 = zeros(nz, 1); c1 = zeros(nz, 1);
M1 = zeros(nz, 1); rho1 = zeros(nz,1); CL1 = zeros(nz, 1); F1 = zeros(nz, 1);
G1 = zeros(nz, 1); D1 = zeros(nz, 1); T1 = zeros(nz, 1);
fuelburnCL = zeros(nz+1,1); fuelburnCL(1) = 0;

% Fuel capacity in the aircraft at takeoff
rhof = 817.15;  % Fuel density [kg/m3]
fuelCL = zeros(nz, 1);  % Fuel burn vector [kg]
mf1 = zeros(nz+1,1);    % Weight of fuel vector
mf1(1) = lf*10^-3*rhof; % Weight of fuel at t=0 [kg]

% Maximum takeoff weight [kg]
m1 = zeros(nz+1,1);     % Weight of the aircraft vector
m1(1) = m0;             % Maximum takeoff weight [kg]

% Calculation of the time of climb
vmean1 = 0.0333 * gamma;    % Mean vertical speed [m/s]
Timeclimb = (0.99*zc)/vmean1/3600;  % Time of climb [hours]
tclimb = Timeclimb/n;   % Time increment [hours]

% Climb trajectory
% Calculate the velocities, forces, weight of the fuel burn and aircraft
% for each increment until the cruise altitude

i = 1;
for Z = 0.01:0.001:0.99;
    V1(i)=(-1/alpha)*log((1/beta)*((1/Z)-1));   % Velocity along flight path
    v1(i) = gamma*Z^2*(1-Z)^2;  % Vertical velocity (m/s)
    % v1(i) = -6*vm1*Z^2+6*vm1*Z;   % the parabolic vertical speed
    theta1(i) = asin(v1(i)/V1(i));  % Climb angle [rad]
    c1(i) = 340.245 - 54.15*Z;  % Speed of sound [m/s]
    M1(i) = V1(i) / c1(i);      % Mach number [-]
    rho1(i) = rho0*exp(-epsilon*zc*Z);  % Air density [kg/m^3]
    CL1(i) = (m1(i)*9.81*cos(theta1(i)))/(0.5*S*rho1(i)*V1(i)^2);   % Lift coef.
    F1(i) = (m1(i)*gamma*Z*(1-Z))/(alpha*zc);   % Acceleration force F[N]
    G1(i) = m1(i)*9.81*sin(theta1(i));  % Drag due to gravity G[N]
    D1(i) = 0.5*S*rho1(i)*V1(i)^2*(CDo+k*CL1(i)^2); % Aerodynamic drag D[N]
    T1(i) = F1(i)+G1(i)+D1(i);  % Thrust force T[N]
    fuelCL(i) = SFC*T1(i)*tclimb;   % Fuel burn at each increment [kg]
    fuelburnCL(i+1) = fuelburnCL(i) + fuelCL(i);    % Total fuel burn [kg]
    mf1(i+1) = mf1(i) - fuelCL(i);  % Weight of fuel [kg]
    m1(i+1) = m1(i) - fuelCL(i);    % Weight of the aircraft [kg]
    i = i + 1;
end

% Maximum thrust required during the climb
Tmax = max(T1);

% Distance climb
Vmean1 = mean(V1);  % Mean velocity along the flight path [m/s]
vhmean1 = sqrt(Vmean1^2 - vmean1^2);    % Mean horizontal velocity [m/s]
dHclimb = vhmean1 * Timeclimb * 3600;   % Horizontal climb distance [m]

fuelburnCL = fuelburnCL(end);

