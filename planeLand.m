function [fuelburn] = planeLand(planeNumber, descentAngle, descentAltitude, fuelburnCL, fuelburnCR)
% Compute the fuel consumption during takeoff based on aircraft type,
% descent angle, and the altitude from which it is descending from.
% PLANELAND(planeNumber, desscentAngle, descentAltitude) return the fuel
% consumption in kg.

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

% Initialization
% Fuel burn before the descent [kg]
fuelD = zeros(nz, 1);
fuelburnD = zeros(nz+1,1);  fuelburnD(1) = 0;

% Weight aircraft and fuel before the descent [kg]

% Maximum take off weight [kg]

m3 = zeros(nz+1,1); m3(1) = m2(end);
mf3 = zeros(nz+1,1); mf3(1) = mf2(end);

V3 = zeros(nz,1); v3 = zeros(nz,1); theta3 = zeros(nz,1); rho3 = zeros(nz,1);
c3 = zeros(nz,1); M3 = zeros(nz,1);
CL3 = zeros(nz,1); F3 = zeros(nz,1); G3 = zeros(nz,1); D3 = zeros(nz,1);
T3 = zeros(nz, 1);

% Calculation of the time increment
tdescent = Timedescent/n;   % Time increment [hours]

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

Tmean = mean(T3)

% Horizontal descent distance
Vmean3 = mean(V3);  % Mean velocity along the flight path [m/s]
vhmean3 = sqrt(Vmean3^2 - vmean3^2);    % Mean horizontal descent velocity [m/s]
dHdescent = vhmean3 * Timedescent * 3600;   % Horizontal descent distance [m]

% Plot

% Forces
figure(3);
subplot(2,2,1),
plot(F3, 0.01:0.001:0.99,G3,0.01:0.001:0.99,D3,0.01:0.001:0.99,T3,0.01:0.001:0.99)
title('Descent forces')
legend('Acceleration force F', 'Drag due to gravity G', 'Aerodynamic drag D', 'Thrust force T')
xlabel('Forces and required thrust [N]')
ylabel('Dimensionless altitude Z [-]')

% Vertical velocity and descent angle
subplot(2,2,2), plot(v3, 0.01:0.001:0.99, theta3*(180/pi),0.01:0.001:0.99)
title('Descent vertical velocity and descent angle')
legend('Vertical velocity', 'Descent angle')
xlabel('Vertical velocity v [m/s] and descent angle theta [o]')
ylabel('Dimensionless altitude Z [-]')

% Fuel burn at each increment
subplot(2,2,3), plot(fuelD, 0.01:0.001:0.99)
xlabel('Fuel burn [kg]')
ylabel('Dimensionless altitude Z [-]')
title('Fuel consumption at each step')

% Fuel burn
subplot(2,2,4), plot(fuelburnD, 0.99:-0.001:0.009)
xlabel('Fuel consumption [kg]')
ylabel('Dimensionless altitude Z [-]')
title('Fuel consumption during the descent')

% Values of the descent
step3 = ('DESCENT')
disp(['Value of fuel burn during the descent [kg]: ', num2str(fuelburnD(end))]);
disp(['Time of descent [hr]: ', num2str(Timedescent)]);
disp(['Horizontal descent distance [m]: ', num2str(dHdescent)]);

%% Results

disp('RESULTS')

% Time of the trajectory [hours]
TIME = Timeclimb + Timecruise + Timedescent;
disp(['Time trajectory [h]: ', num2str(TIME)]);

% The fuel burn [kg]
fuelburn = fuelburnCL(end) + fuelburnCR(end) + fuelburnD(end);







