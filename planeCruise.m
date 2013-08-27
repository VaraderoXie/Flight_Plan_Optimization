function [fuelburn, planeWeight, fuelWeight] = planeCruise(planeNumber, cruiseSpeed, cruiseAltitude, liftCoef, totalDistance, fuelburnCL)
% PLANECRUISE(planeNumber, cruiseSpeed,cruiseAltitude) calculates the fuel 
% consumption during the cruising phase of the flight. 

%% Plane number options are:
% 1 - 737-300
% 2 - 737-800
% 3 - A320
% 4 - A340
% 5 - 777-200
switch planeNumber
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

V2 = cruiseSpeed;   % Cruise velocity [m/s] (constant)

% Calculate the air density and the aircraft weight at zc (after climb)
zc = cruiseAltitude;
rhoc = rho0*exp(-epsilon*zc);   % Air density [kg/m^3]
mc = ((liftCoef*S*V2^2)/(2*9.81))*rhoc; % Weight of the aircraft [kg]


% Cruise time and distance
% Define the time and the distance for the descent
gamma = 100.0;          % Pick a number between 100 and 200
vmean3 = 0.0333*gamma;  % Mean vertical speed [m/s]
vhmean3 = sqrt(Vmean1^2 - vmean3^2);    % Mean horizontal velocity [m/s]
Timedescent = (zc * 0.99)/vmean3/3600;  % Time of descent [hours]
dHdescent = vhmean3 * Timedescent * 36000;  % Horizontal descent distance [m]

% Time and distance for the cruise phase
dHcruise = totalDistance - dHdescent - dHclimb;     % Horizontal cruise distance [m]
Timecruise = dHcruise / V2 / 3600;  % Time of cruise [hours]

% Thrust force [N]
T2 = planeDrag;   % Thrust force is constant [N]

% Fuel burn
fuelburnCR = SFC * Timecruise*T2;   % Fuel burn during the cruise [kg]
m2 = mc - fuelburnCR;   % Weight of aircraft after the cruise [kg]
mf2 = fuelweightCL - fuelburnCR;    % Weight of fuel [kg]

fuelburn = fuelburnCR;
planeWeight = m2;
fuelWeight = mf2;
