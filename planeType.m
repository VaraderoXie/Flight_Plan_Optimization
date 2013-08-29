function [S, m0, lf, alpha, SFC] = planeType(planeNumber)
%% Plane number options are:
% 1 - 737-300
% 2 - 737-800
% 3 - A320
% 4 - A340
% 5 - 777-200
if (nargin < 1)
    error('on','Usage: planeType(planeNumber), planeNumber = 1 - 5');
end

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
        warning('on','Invalid plane number.');
        disp('Plane number options are:');
        disp('% 1 - 737-300');
        disp('% 2 - 737-800');
        disp('% 3 - A320');
        disp('% 4 - A340');
        disp('% 5 - 777-200');
end
