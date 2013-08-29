function [fuelburn, TIME] = planeLand(planeNumber,descentTime,planeWeight,fuelWeight, fuelburnCL, fuelburnCR, Timedescent,Tmax)
% Compute the fuel consumption during takeoff based on aircraft type,
% descent angle, and the altitude from which it is descending from.
% PLANELAND(planeNumber, desscentAngle, descentAltitude) return the fuel
% consumption in kg.
if (nargin < 8) 
    error('on','Usage: planeLang(planeNumber, descentTime, planeWeight, fuelWeight, fuelburnCL, fuelburnCR, Timedescent, Tmax)');
end

% planeType returns S, m0, lf, alpha, SFC
[S, ~, ~, alpha, SFC] = planeType(planeNumber);

%% Parameters and constants

beta = 60000;   % Velocity/altitude parameter
if (descentTime >= 150 && descentTiem <= 300) 
    gamma = descentTime; % A number between 150-300
else
    warning('Invalid descentTime. Input range is 150 - 300');
end
zc = climbAltitude;   % Cruise altitude [m]
rho0 = 1.225;   % Air density at sea level [kg/m^3]
epsilon = 0.0001;   % Constant in the density function [l/m]
k = 0.045;  % Constant selected for the included drag coefficient
CDo = 0.015;    % Constant selected for the zero-lift drag coefficient

%% Simulate Descent
% Initialization
% Fuel burn before the descent [kg]
nz = length(0.01:0.001:0.99);
fuelD = zeros(nz, 1);
fuelburnD = zeros(nz+1,1);  fuelburnD(1) = 0;

% Weight aircraft and fuel before the descent [kg]

% Maximum take off weight [kg]

m3 = zeros(nz+1,1); m3(1) = planeWeight;
mf3 = zeros(nz+1,1); mf3(1) = fuelWeight;

V3 = zeros(nz,1); v3 = zeros(nz,1); theta3 = zeros(nz,1); rho3 = zeros(nz,1);
c3 = zeros(nz,1); M3 = zeros(nz,1);
CL3 = zeros(nz,1); F3 = zeros(nz,1); G3 = zeros(nz,1); D3 = zeros(nz,1);
T3 = zeros(nz, 1);

% Calculation of the time increment
tdescent = Timedescent/nz;   % Time increment [hours]

% Descent trajectory 
% Calculate the velocities, forces, weight of the fuel burn and airplane
% for each increment until the landing

i = 1;
for Z = 0.01:0.001:0.99;
    Zd = 1 - Z;     % dimensionless altitude
    V3(i) = (-1/alpha)*log((1/beta)*((1/Zd)-1));    % Velocity along flight path
    v3(i) = -gamma*Zd^2*(1-Zd)^2;   % Vertical speed (m/s)
    % v3(i) = -(-6*vm1*Z^2+6*vm1*Z);    % the parabolic vertical speed
    theta3(i) = asin(v3(i)/V3(i));  % Climb angle [rad]
    c3(i) = 340.254 - 45.15*Zd;     % Speed of sound [m/s]
    M3(i) = V3(i) / c3(i);          % Mach number [-]
    rho3(i) = rho0 * exp(-epsilon*zc*Zd);   % Air density [kg/m^3]
    CL3(i) = (m3(i) * 9.81 * cos(-theta3(i)))/(0.5*S*rho3(i)*V3(i)^2);  % Lift coef.
    F3(i) = (m3(i)*gamma*Zd*(1-Zd))/(alpha*zc); % Acceleration force F [N]
    G3(i) = m3(i) * 9.81 * sin(-theta3(i)); % Drag due to gravity Gd [N]
    D3(i) = 0.5 * S * rho3(i) * V3(i)^2 * (CDo + k * CL3(i)^2); % Aerodynamic drag D [N]
    T3(i) = D3(i) - G3(i) - F3(i);  % Thrust force [N] can't be negative
    if T3(i) <= (10/100)*Tmax
        T3(i) = (10/100)*Tmax;
    elseif T3(i) > (10/100)*Tmax
        T3(i) = T3(i);
    end
    fuelD(i) = SFC*T3(i)*tdescent;  % Fuel burn at each increment [kg]
    fuelburnD(i+1) = fuelburnD(i) + fuelD(i);   % Total fuel burn [kg]
    mf3(i+1) = mf3(i) - fuelD(i);   % Weight of fuel [kg]
    m3(i+1) = m3(i) - fuelD(i); % Weight of the aircraft [kg]
    i = i + 1;
end

% Tmean = mean(T3)

% Horizontal descent distance
Vmean3 = mean(V3);  % Mean velocity along the flight path [m/s]
vhmean3 = sqrt(Vmean3^2 - vmean3^2);    % Mean horizontal descent velocity [m/s]
% dHdescent = vhmean3 * Timedescent * 3600;   % Horizontal descent distance [m]

%% Results

% Time of the trajectory [hours]
TIME = Timeclimb + Timecruise + Timedescent;

% The fuel burn [kg]
fuelburn = fuelburnCL(end) + fuelburnCR(end) + fuelburnD(end);







