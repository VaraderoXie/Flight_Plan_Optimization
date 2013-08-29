function [fuelburn, planeWeight, fuelWeight, machCruise,SpeedOfSound,liftCoef,planeDrag,horizClimbDist,Tmax,Vmean1] = planeClimb(planeNumber, descentTime, climbAltitude)
% Compute the fuel consumption during takeoff based on aircraft type, 
% takeoff climb angle, and the final desired altitude.
% PLANECLIMB(planeNumber, climbAngle, climbAltitude) returns the fuel 
% consumption in kg. 

if (nargin < 3)
    error('Usage: [fuelburn, planeWeight, fuelWeight, machCruise, SpeedOfSound, liftCoef, planeDrag, horizClimbDist, Tmax,Vmean1]=planeClimb(planeNumber, descentTime, climbAltitude)');
end

% planeType returns S, m0, lf, alpha, SFC
[S, m0, lf, alpha, SFC] = planeType(planeNumber);


%% Parameters and constants

beta = 60000;   % Velocity/altitude parameter
if (descentTime >= 150 && descentTime <= 300) 
    gamma = descentTime; % A number between 150-300
else
    warning('Invalid descentTime. Input range is 150-300');
end
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

fuelburn = fuelburnCL(end);
planeWeight = m1(end);
fuelWeight = mf1(end);
machCruise = M1(end);
SpeedOfSound = c1(end);
liftCoef = CL1(end);
planeDrag = D1(end);
horizClimbDist = dHclimb;

