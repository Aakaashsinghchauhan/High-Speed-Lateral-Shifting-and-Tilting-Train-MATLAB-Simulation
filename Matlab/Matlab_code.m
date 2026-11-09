% SHIFT_TILT_DOUBLE_CLOTHOID_VIDEO_SIMULATION
% This script follows a simple top-down design.
%

%%
%
% Mathematical notation used in comments:
%   y_v, y_s, y_b              Lateral positions of frame, carriage, and bogie.
%   y_v_dot, y_s_dot, y_b_dot  Lateral velocities of the same three bodies.
%   y_v_ddot, y_s_ddot, y_b_ddot  Lateral accelerations of the three bodies.
%   theta_v, theta_s, theta_b  Roll angles of frame, carriage, and bogie.
%   theta_v_dot                First time derivative of theta_v.
%   theta_v_ddot               Second time derivative of theta_v.
% Underscores show subscripts, while _dot and _ddot show time derivatives.

%% Better check out the white paper to understand the model and sign conventions.

clear;      
close all;  
clc;        

%% STEP 1 - SELECT THE OPERATING CASE

% Enter exactly one of the following four case names:
% 'passive' uses no lateral-shift actuator and no tilt actuator.
% 'shift_only' uses the lateral-shift actuator only.
% 'tilt_only' uses the tilt actuator only.
% 'shift_and_tilt' uses both the lateral-shift and tilt actuators.
selectedCase = 'shift_and_tilt';       % Operating case simulated by this script.

%% STEP 2 - ROUTE

% Gravitational acceleration is used in the vehicle and load equations.
p.g = 9.81;                            % Gravitational acceleration [m/s^2].

% Track sign convention:
% A positive radius turns left and a negative radius turns right.
% Cant has the same sign as the inward rotation of the rail plane.
p.track.gauge = 1.435;                 % Distance between the rails [m].
p.track.halfGauge = p.track.gauge/2;   % Distance from the centreline to one rail [m].

% The route contains a tangent, a left curve, a middle tangent, a right
% curve, and a final tangent. Each curve has an entry and exit clothoid.
p.track.tangentIn = 90;                % Straight approach before curve 1 [m].
p.track.radius1 = 1000;                % Signed radius of curve 1 [m].
p.track.cant1 = 0.155;                 % Signed rail-height difference on curve 1 [m].
p.track.transitionIn1 = 500;           % Entry clothoid length for curve 1 [m].
p.track.circular1 = 280;               % Constant-radius length for curve 1 [m].
p.track.transitionOut1 = 500;          % Exit clothoid length for curve 1 [m].
p.track.middleTangent = 140;           % Straight distance between the curves [m].
p.track.radius2 = -1000;               % Signed radius of curve 2 [m].
p.track.cant2 = -0.155;                % Signed rail-height difference on curve 2 [m].
p.track.transitionIn2 = 500;           % Entry clothoid length for curve 2 [m].
p.track.circular2 = 280;               % Constant-radius length for curve 2 [m].
p.track.transitionOut2 = 500;          % Exit clothoid length for curve 2 [m].
p.track.tangentOut = 90;               % Straight distance after curve 2 [m].

%% STEP 3 - SPEED AND NUMERICAL SIMULATION SETTINGS

% A constant speed is used over the complete route.
p.speed.constantKmh = 380;             % Vehicle speed [km/h].
p.speed.minimumKmh = 20;               % Minimum speed used to protect my sanity [km/h].

% ode45 uses these values to control integration accuracy and step size.
% dont touch these values they are irrelavent for you.
p.ode.relTol = 0.00001;                % Relative integration-error tolerance.
p.ode.absTol = 0.0000001;              % Absolute integration-error tolerance.
p.ode.maxStep = 0.025;                 % Largest permitted integration step [s].

% These conditions can stop an invalid or unsafe numerical run.
p.sim.stopAtGaugeLimit = false;        % Set true to stop when the resultant leaves the gauge.
p.sim.maxStateAbs = 100;               % Maximum permitted absolute state magnitude.

%% STEP 4 - VEHICLE MASS AND ROLL INERTIA

% The model contains a vehicle frame, a sliding carriage, and one bogie.
p.mv = 15000;                          % Vehicle-frame mass [kg].
p.ms = 4000;                           % Sliding-carriage mass [kg].
p.mb = 2500;                           % Bogie mass [kg].
p.Ivr = 25000;                         % Vehicle-frame roll inertia [kg m^2].
p.Isr = 4500;                          % Sliding-carriage roll inertia [kg m^2].
p.Ibr = 1500;                          % Bogie roll inertia [kg m^2].

%% STEP 5 - SUSPENSION AND VEHICLE GEOMETRY

% Each stiffness and damping value below represents one side of the vehicle.
p.Kfy = 5000000;                       % Frame-to-carriage lateral stiffness [N/m].
p.Cfy = 250000;                        % Frame-to-carriage lateral damping [N s/m].
p.Kfz = 620000;                        % Frame vertical stiffness used in roll [N/m].
p.Cfz = 33000;                         % Frame vertical damping used in roll [N s/m].
p.Ksy = 260000;                        % Carriage-to-bogie lateral stiffness [N/m].
p.Csy = 33000;                         % Carriage-to-bogie lateral damping [N s/m].
p.Kby = 35000000;                      % Bogie-to-track lateral stiffness [N/m].
p.Cby = 16000;                         % Bogie-to-track lateral damping [N s/m].
p.Kbz = 2000000;                       % Bogie vertical stiffness used in roll [N/m].
p.Cbz = 20000;                         % Bogie vertical damping used in roll [N s/m].
p.Klgy = 308641.9753;                  % Lateral-guide stiffness used in roll [N/m].
p.Clgy = 15432.09877;                  % Lateral-guide damping used in roll [N s/m].

% These lengths define suspension lever arms and centre-of-mass heights.
p.d1 = 0.90;                           % Frame suspension half-spacing [m].
p.d2 = 1.00;                           % Bogie suspension half-spacing [m].
p.h1 = 0.90;                           % First frame-to-carriage lever dimension [m].
p.h2 = 0.25;                           % Second frame-to-carriage lever dimension [m].
p.h3 = 0.10;                           % Carriage-to-bogie lever dimension [m].
p.h4 = 0.45;                           % Carriage lateral-suspension height [m].
p.h5 = 0.09;                           % Primary lateral-suspension height [m].
p.hg1 = 0.37;                          % Bogie centre-of-mass height [m].
p.hg2 = 0.85;                          % Sliding-carriage centre-of-mass height [m].
p.hg3 = 1.52;                          % Vehicle-frame centre-of-mass height [m].

%% STEP 6 - WHEEL-INPUT FILTER

% The filter is a simplified second-order representation of wheel input.
p.wheel.omega = 2*pi*5;                % Natural angular frequency [rad/s].
p.wheel.zeta = 0.20;                   % Dimensionless damping ratio.

%% STEP 7 - DEFINE THE LATERAL-SHIFT AND TILT CONTROLLER

p.ctrl.enableShift = true;             % State that a lateral-shift actuator is available for selectable cases.
p.ctrl.enableTilt = true;              % State that a tilt actuator is available for selectable cases.
p.ctrl.shiftTravelLimit = 0.6675;      % Maximum relative lateral shift [m].
p.ctrl.tiltLimitDeg = 8.0;             % Maximum relative tilt [deg].
p.ctrl.shiftKp = 350000;               % Shift proportional gain [N/m].
p.ctrl.shiftKd = 50000;                % Shift derivative gain [N s/m].
p.ctrl.tiltKp = 1000000;               % Tilt proportional gain [N m/rad].
p.ctrl.tiltKd = 120000;                % Tilt derivative gain [N m s/rad].
p.ctrl.tiltRateSyncGain = 200000;      % Gain that matches normalized tilt rate to normalized lateral-shift rate [N m s/rad].
p.ctrl.forceLimit = 500000;            % Lateral actuator-force limit [N].
p.ctrl.momentLimit = 250000;           % Tilt actuator-moment limit [N m].
p.ctrl.forceTau = 0.08;                % Lateral actuator time constant [s].
p.ctrl.momentTau = 0.08;               % Tilt actuator time constant [s].
p.ctrl.useStaticFeedforward = true;    % Add the static-inverse plant feed-forward defined in the control-strategy equations.

%% STEP 8 - DEFINE MOVEMENT LIMITS AND MECHANICAL STOPS

% A bump stop applies a spring and damping reaction after its limit is crossed.
% The stops normally react against excessive movement without ending the run.
p.stop.enableBumpStops = true;                         % Enable all mechanical stops.
p.stop.frameCarriageDeflectionLimit = 0.10;            % Frame-to-carriage lateral limit [m].
p.stop.carriageBogieDeflectionLimit = p.ctrl.shiftTravelLimit; % Carriage-to-bogie lateral limit [m].
p.stop.relativeTiltLimitNoTilt = deg2radLocal(2.0);    % Passive relative-tilt limit [rad].
p.stop.relativeTiltLimitWithTilt = deg2radLocal(p.ctrl.tiltLimitDeg); % Active relative-tilt limit [rad].
p.stop.carriageBogieRollLimit = deg2radLocal(2.0);     % Carriage-to-bogie roll limit [rad].
p.stop.bogieCantRelativeRollLimit = deg2radLocal(0.15); % Bogie-to-track roll limit [rad].
p.stop.shiftStiffness = 50000000;                      % Shift-stop stiffness [N/m].
p.stop.shiftDamping = 1000000;                         % Shift-stop damping [N s/m].
p.stop.frameCarriageStiffness = 50000000;              % Frame lateral-stop stiffness [N/m].
p.stop.frameCarriageDamping = 1000000;                 % Frame lateral-stop damping [N s/m].
p.stop.relativeTiltStiffness = 50000000;               % Relative-tilt stop stiffness [N m/rad].
p.stop.relativeTiltDamping = 1000000;                  % Relative-tilt stop damping [N m s/rad].
p.stop.carriageBogieRollStiffness = 50000000;          % Carriage roll-stop stiffness [N m/rad].
p.stop.carriageBogieRollDamping = 1000000;             % Carriage roll-stop damping [N m s/rad].
p.stop.bogieCantRelativeRollStiffness = 50000000;      % Bogie roll-stop stiffness [N m/rad].
p.stop.bogieCantRelativeRollDamping = 1000000;         % Bogie roll-stop damping [N m s/rad].

%% STEP 9 - DEFINE PLOT AND LIVE-ANIMATION APPEARANCE

% The live animation is displayed in a MATLAB figure. No video file is saved.
p.video.makeAnimation = true;          % Set false to skip the live animation.
p.video.viewMode = 'front';            % Select 'front' or '3d'.
p.video.frameRate = 20;                % Number of displayed animation frames per second.
p.video.duration = 20;                 % Intended visual playback duration [s].
p.video.cameraMode = 'follow';         % Select 'follow' or 'wholeRoute' for 3-D view.
p.video.followWindow = 115;            % Half-length of the following camera window [m].
p.video.pauseBetweenFrames = 0.001;    % Small pause that lets MATLAB refresh each frame [s].

% The following values control only the drawing dimensions and colours.
p.visual.nRoute = 1500;                % Number of route points used for drawing.
p.visual.trackHalfWidth = 2.20;        % Half-width of the drawn track bed [m].
p.visual.carLength = 27.0;             % Drawn vehicle-frame length [m].
p.visual.bodyWidth = 2.95;             % Drawn vehicle-frame width [m].
p.visual.bodyHeight = 1.35;            % Drawn vehicle-frame height [m].
p.visual.carriageLength = 20.0;        % Drawn sliding-carriage length [m].
p.visual.carriageWidth = 2.55;         % Drawn sliding-carriage width [m].
p.visual.carriageHeight = 0.32;        % Drawn sliding-carriage height [m].
p.visual.bogieLength = 9.5;            % Drawn bogie length [m].
p.visual.bogieWidth = 2.20;            % Drawn bogie width [m].
p.visual.bogieHeight = 0.42;           % Drawn bogie height [m].
p.visual.bogieLift = 0.20;             % Drawn bogie height above the rail plane [m].
p.visual.bodyColor = [1.00 0.90 0.18]; % Vehicle-frame colour.
p.visual.carriageColor = [0.30 0.78 0.42]; % Sliding-carriage colour.
p.visual.bogieColor = [0.90 0.28 0.22]; % Bogie colour.
p.visual.railColor = [0.05 0.05 0.06]; % Rail colour.
p.visual.trackColor = [0.78 0.82 0.84]; % Track-bed colour.
p.visual.resultantColor = [0.82 0.08 0.08]; % Resultant-force colour.

%% STEP 10 - CALCULATE DERIVED SYSTEM VALUES

% Complete quantities such as curvature, route length, masses, and limits.
p = completeParameters(p); % p stores the complete parameter structure after all dependent values are calculated.

% Calculate the constant feed-forward plant matrices once before the ODE simulation begins.
p = calculateStaticFeedforwardMatrices(p); % p.ctrl receives A_v, B_v, C_r, G_0, and G_0_inverse.

% Build the operating case selected at the beginning of this script.
caseDef = defineShiftTiltCase(p, selectedCase); % caseDef stores the selected actuator configuration and its movement limits.

%% STEP 11 - SIMULATE THE VEHICLE AND CALCULATE RESULTS

% Apply a common visual style before any figures are created.
setupPlotStyle();

% Print the entered system definition so the run can be checked.
printSimulationDescription(p, caseDef);

% Solve the equations of motion over the complete route.
sim = simulateRoute(p, caseDef); % sim stores the complete solved time history and processed outputs.

% Calculate peak values and safety margins from the solved state history.
metrics = routeMetrics(sim, p); % metrics stores the peak values and route-assessment results.

% Print the calculated results in the Command Window.
printSimulationSummary(sim, metrics, p);

%% STEP 12 - PLOTS

% Plot the route definition before displaying the live animation.
plotRouteProfiles(p);

%% STEP 13 - LIVE ANIMATION

% The animation is the last program action. It is displayed but never saved.
if p.video.makeAnimation
    makeMovingAnimation(sim, p, caseDef);
end

fprintf('\nSimulation and live display are complete.\n');

%% LOCAL FUNCTIONS 

%% FUNCTION GROUP 1 - COMPLETE THE ENTERED SYSTEM PARAMETERS
% These functions calculate values that depend on the user-entered settings.


%% Function completeParameters stores the hard parameters defined for the system. 
% it calculates the derived parameters using functions such as signedCurvatureFromRadius.
% INPUTS AND OUTPUTS: completeParameters
% Input p is the structure containing all user-entered system parameters.
% Output p is the same structure with the calculated route, speed, mass, and controller fields added.
function p = completeParameters(p)

p.track.kappa1 = signedCurvatureFromRadius(p.track.radius1); %  curvature of curve 1 [1/m].
p.track.kappa2 = signedCurvatureFromRadius(p.track.radius2); %  curvature of curve 2 [1/m].
p.track.thetaCant1 = atan(p.track.cant1/p.track.gauge); % rail-plane cant angle on curve 1 [rad].
p.track.thetaCant2 = atan(p.track.cant2/p.track.gauge); % rail-plane cant angle on curve 2 [rad].

p.track.segmentLength = [ ... % route-section lengths in travel order [m].
    p.track.tangentIn, ... % straight approach before curve 1 [m].
    p.track.transitionIn1, ... %  entry-clothoid length for curve 1 [m].
    p.track.circular1, ... %  constant-radius length of curve 1 [m].
    p.track.transitionOut1, ... %  exit-clothoid length for curve 1 [m].
    p.track.middleTangent, ... % route-section lengths in travel order [m].
    p.track.transitionIn2, ... %  entry-clothoid length for curve 2 [m].
    p.track.circular2, ... %  constant-radius length of curve 2 [m].
    p.track.transitionOut2, ... %  exit-clothoid length for curve 2 [m].
    p.track.tangentOut]; %  straight distance after curve 2 [m].

% p.track.segmentLength = [90, 180, 280, 180, 140, 180, 280, 180, 90];


p.track.breakS = [0, cumsum(p.track.segmentLength)]; % cumulative sum of entier segment [90, 270, 550, 730, 870, 1050, 1330, 1510, 1600] [m].


p.track.totalLength = p.track.breakS(end); % final route station and complete route length [m].

p.speed.station_m = [0, p.track.totalLength]; %  start and end stations of the speed definition [m].

p.speed.speedKmh = p.speed.constantKmh*[1, 1]; % assigns the same constant speed at both speed stations [km/h].

p.speed.speed_mps = kmhToMs(p.speed.speedKmh); % stores the speed-station values converted to metres per second [m/s].

p.speed.minimum_mps = kmhToMs(p.speed.minimumKmh); %  numerical lower speed bound converted to metres per second [m/s].

p.totalMass = p.mv + p.ms + p.mb; % the combined frame, carriage, and bogie mass [kg].


p.ctrl.tiltLimit = deg2radLocal(p.ctrl.tiltLimitDeg); %  controller tilt limit converted from degrees to radians [rad].


p.ctrl.effectiveHeight = (p.mv*p.hg3 + p.ms*p.hg2 + p.mb*p.hg1) / p.totalMass;  %  mass-weighted centre-of-mass height [m].


p.ctrl.shiftComGain = (p.mv + p.ms)/p.totalMass; % carriage shift to total centre-of-mass shift 


p.ctrl.tiltComGain = p.mv*(p.h1 + p.h2)/p.totalMass; % frame tilt to centre-of-mass lateral shift [m/rad].

end


% signedCurvatureFromRadius(radius) function is nothing but curvature = 1/radius.
% calculates the signed curvature from the route radius.
% A positive radius turns left and a negative radius turns right.

% INPUTS AND OUTPUTS: signedCurvatureFromRadius
% Input radius is the signed curve radius [m]. Positive and negative values represent opposite curve directions.
% Output kappa is the signed curvature calculated as one divided by radius [1/m].
function kappa = signedCurvatureFromRadius(radius)
if abs(radius) < eps
    error('Route radius cannot be zero.');
end
kappa = 1/radius; % kappa is signed track curvature [1/m].
end


% defineShiftTiltCase uses the case name entered at the beginning of the
% script to select which actuators are enabled for the simulation.

% INPUTS AND OUTPUTS: defineShiftTiltCase
% Input p is the complete system-parameter structure.
% Input selectedCase is the case-name text entered at the beginning of the script.
% Output caseDef is the structure containing the selected shift, tilt, and movement-limit settings.
function caseDef = defineShiftTiltCase(p, selectedCase)
caseName = lower(strtrim(selectedCase)); % caseName converts the entered case text to lowercase and removes spaces at its ends.

switch caseName
    case 'passive'
        caseDef.id = 'passive_double_clothoid'; % Internal identifier for the passive case.
        caseDef.name = 'Passive'; % Readable case name used in figures and printed descriptions.
        caseDef.enableShift = false; % Disable the lateral-shift actuator for the passive case.
        caseDef.enableTilt = false; % Disable the tilt actuator for the passive case.

    case 'shift_only'
        caseDef.id = 'shift_only_double_clothoid'; % Internal identifier for the controlled lateral-shift case.
        caseDef.name = 'Controlled lateral shift only'; % Readable case name used in figures and printed descriptions.
        caseDef.enableShift = p.ctrl.enableShift; % Enable the lateral-shift actuator when it is available.
        caseDef.enableTilt = false; % Disable the tilt actuator for the shift-only case.

    case 'tilt_only'
        caseDef.id = 'tilt_only_double_clothoid'; % Internal identifier for the controlled tilt case.
        caseDef.name = 'Controlled tilt only'; % Readable case name used in figures and printed descriptions.
        caseDef.enableShift = false; % Disable the lateral-shift actuator for the tilt-only case.
        caseDef.enableTilt = p.ctrl.enableTilt; % Enable the tilt actuator when it is available.

    case 'shift_and_tilt'
        caseDef.id = 'shift_and_tilt_double_clothoid'; % Internal identifier for the combined controlled case.
        caseDef.name = 'Controlled lateral shift and tilt'; % Readable case name used in figures and printed descriptions.
        caseDef.enableShift = p.ctrl.enableShift; % Enable the lateral-shift actuator when it is available.
        caseDef.enableTilt = p.ctrl.enableTilt; % Enable the tilt actuator when it is available.

    otherwise
        error('Unknown selectedCase. Use ''passive'', ''shift_only'', ''tilt_only'', or ''shift_and_tilt''.');
end

caseDef.lockShift = ~caseDef.enableShift; % State whether the lateral-shift actuator is inactive for this case.
caseDef.limits.deltaFy = p.stop.frameCarriageDeflectionLimit; % allowed frame-to-carriage spring deflection for out system [m].
caseDef.limits.deltaSy = p.stop.carriageBogieDeflectionLimit; % allowed carriage-to-bogie spring deflection [m].
caseDef.limits.ysMinusYb = p.ctrl.shiftTravelLimit; %  allowed carriage shift y_s - y_b [m].
if caseDef.enableTilt
    caseDef.limits.thetaVMinusThetaS = p.ctrl.tiltLimit; % Use the controlled tilt limit when the tilt actuator is enabled [rad].
else
    caseDef.limits.thetaVMinusThetaS = p.stop.relativeTiltLimitNoTilt; % Use the passive relative-roll limit when tilt is disabled [rad].
end
caseDef.limits.thetaSMinusThetaB = p.stop.carriageBogieRollLimit; % allowed carriage-to-bogie roll theta_s - theta_b [rad].
end




%%---------------------------------------------------------------------------------------------------


%% FUNCTION GROUP 2 - ROUTE DESIGN
% These functions create curvature, cant, distance, and speed at each route station.

% sampleRoute(p, s) function creates a route structure containing curvature, cant angle, and global coordinates at each sampled station along the route. 
% It uses the trackProfileFromDistance function to evaluate curvature and cant angle at each station.

% INPUTS AND OUTPUTS: sampleRoute
% Input p is the complete system-parameter structure.
% Input s contains the route stations at which the track is sampled [m].
% Output route contains curvature, cant, heading, and plan-view coordinates at every supplied station.
function route = sampleRoute(p, s)
s = reshape(s, 1, []); % s is distance along the route [m]. This reshape shit is to convert the vector into a row vector regardless of what was in it.
route.s = s; % route.s stores the sampled route stations [m].
route.kappa = zeros(size(s)); % stores curvature at every sampled station [1/m].
route.thetaCant = zeros(size(s)); %  stores cant angle at every sampled station [rad].

for i = 1:numel(s)  % numel(s) calculates the number of sampled stations. for s = [0, 100, 200, 300]; numel(s) = 4. for i = 1:4, i = 1, 2, 3, 4

    % here we are getting out all the detailes of the route defined in the previous steps. 
    % we are getting the curvature and cant angle at every sampled station as it is dynamically changing along the route.
    % specially at the clothoid sections where the curvature and cant angle are changing from 0 to the final value of the curve.

    prof = trackProfileFromDistance(s(i), p); % prof contains curvature and cant values evaluated at the current sampled station.
    route.kappa(i) = prof.kappa; % stores signed curvature at sampled station i [1/m].
    route.thetaCant(i) = prof.theta; % stores the rail-plane cant angle at sampled station i [rad].

end

route.psi = cumtrapz(s, route.kappa); % stores the integrated route heading angle [rad].
route.x = cumtrapz(s, cos(route.psi)); % stores global route X coordinates [m].
route.y = cumtrapz(s, sin(route.psi)); % stores global route Y coordinates [m].
end



% trackProfileFromDistance(s, p) function
% prof = trackProfileFromDistance(s, p) calculates the curvature and cant angle at a given distance along the route. 
% It uses the defined track segments and their properties to determine the appropriate values based on the current station. 
% The function returns a structure containing the curvature, cant angle, their derivatives, and the segment name for the current station.


% INPUTS AND OUTPUTS: trackProfileFromDistance
% Input s is one current distance along the route [m].
% Input p is the complete system-parameter structure.
% Output prof contains the local curvature, cant, derivatives, station, and route-section name.
function prof = trackProfileFromDistance(s, p)
s = min(max(s, 0), p.track.totalLength); % s is distance along the route [m].   0<= s <= p.track.totalLength.
b = p.track.breakS; % b contains the cumulative boundary stations of the route sections [m].

%p.track.breakS = [0, 90, 270, 550, 730, 870, 1050, 1330, 1510, 1600 ]; each value in b corresponds to the end of a route section, defining the boundaries for curvature and cant angle calculations.
                % 0 =- > entry tangent
                % 90 =- > entry clothoid of curve 1
                % 270 =- > circular section of curve 1
                % 550 =- > exit clothoid of curve 1
                % 730 =- > middle tangent
                % 870 =- > entry clothoid of curve 2
                % 1050 =- > circular section of curve 2
                % 1330 =- > exit clothoid of curve 2
                % 1510 =- > exit tangent of curve 2
                % 1600 =- > end of route    


if s < b(2)
    [kappa, kappaDs, kappaDDs] = constantProfile(0); % kappa, kappa_Ds, and kappa_DDs are curvature and its first two spatial derivatives [1/m, 1/m^2, 1/m^3].
    [theta, thetaDs, thetaDDs] = constantProfile(0); % theta, theta_Ds, and theta_DDs are cant angle and its first two spatial derivatives [rad, rad/m, rad/m^2].
    segmentName = 'tangent in'; % segmentName identifies the route section containing the current station.
elseif s < b(3)
    [kappa, kappaDs, kappaDDs] = linearProfile(s, b(2), ... 
        p.track.transitionIn1, 0, p.track.kappa1);
    [theta, thetaDs, thetaDDs] = smoothProfile(s, b(2), ... 
        p.track.transitionIn1, 0, p.track.thetaCant1);
    segmentName = 'curve 1 entry clothoid'; % segmentName identifies the route section containing the current station.
elseif s < b(4)
    [kappa, kappaDs, kappaDDs] = constantProfile(p.track.kappa1); 
    [theta, thetaDs, thetaDDs] = constantProfile(p.track.thetaCant1); 
    segmentName = 'curve 1 circular';
elseif s < b(5)
    [kappa, kappaDs, kappaDDs] = linearProfile(s, b(4), ... 
        p.track.transitionOut1, p.track.kappa1, 0);
    [theta, thetaDs, thetaDDs] = smoothProfile(s, b(4), ... 
        p.track.transitionOut1, p.track.thetaCant1, 0);
    segmentName = 'curve 1 exit clothoid'; 
elseif s < b(6)
    [kappa, kappaDs, kappaDDs] = constantProfile(0); 
    [theta, thetaDs, thetaDDs] = constantProfile(0); 
    segmentName = 'middle tangent'; 
elseif s < b(7)
    [kappa, kappaDs, kappaDDs] = linearProfile(s, b(6), ... 
        p.track.transitionIn2, 0, p.track.kappa2);
    [theta, thetaDs, thetaDDs] = smoothProfile(s, b(6), ... 
        p.track.transitionIn2, 0, p.track.thetaCant2);
    segmentName = 'curve 2 entry clothoid'; 
elseif s < b(8)
    [kappa, kappaDs, kappaDDs] = constantProfile(p.track.kappa2); 
    [theta, thetaDs, thetaDDs] = constantProfile(p.track.thetaCant2); 
    segmentName = 'curve 2 circular'; 
elseif s < b(9)
    [kappa, kappaDs, kappaDDs] = linearProfile(s, b(8), ... 
        p.track.transitionOut2, p.track.kappa2, 0);
    [theta, thetaDs, thetaDDs] = smoothProfile(s, b(8), ... 
        p.track.transitionOut2, p.track.thetaCant2, 0);
    segmentName = 'curve 2 exit clothoid'; 
else
    [kappa, kappaDs, kappaDDs] = constantProfile(0); 
    [theta, thetaDs, thetaDDs] = constantProfile(0); 
    segmentName = 'tangent out'; 
end

prof.s = s; % the bounded route station [m].
prof.kappa = kappa; % curvature at the current station [1/m].
prof.kappaDs = kappaDs; % first spatial derivative of curvature [1/m^2].
prof.kappaDDs = kappaDDs; % second spatial derivative of curvature [1/m^3].
prof.kappaDot = 0; % first time derivative of curvature [1/(m s)].
prof.kappaDDot = 0; % second time derivative of curvature [1/(m s^2)].
prof.theta = theta; % track cant angle [rad].
prof.thetaDs = thetaDs; % track cant angle spatial derivative [rad/m].
prof.thetaDDs = thetaDDs; % track cant angle second spatial derivative [rad/m^2].
prof.thetaDot = 0; % track cant angle angular rate [rad/s].
prof.thetaDDot = 0; % track cant angle angular acceleration [rad/s^2].
prof.segmentName = segmentName; % segment name identifies the current route section.
end



% TIME derivative calculation function for the track curvature and cant angle. 
% It uses the vehicle speed and longitudinal acceleration to compute the time derivatives of curvature and cant angle based on their spatial derivatives.
% INPUTS AND OUTPUTS: addTimeDerivatives
% Input prof contains the local track geometry and its spatial derivatives.
% Input V is the vehicle speed [m/s].
% Input aLong is the longitudinal vehicle acceleration [m/s^2].
% Output prof is the same structure with curvature and cant time derivatives added.
function prof = addTimeDerivatives(prof, V, aLong) % V is vehicle speed [m/s] and aLong is longitudinal acceleration [m/s^2].
prof.kappaDot = prof.kappaDs*V; % first time derivative of curvature [1/(m s)].
prof.kappaDDot = prof.kappaDDs*V^2 + prof.kappaDs*aLong; % second time derivative of curvature [1/(m s^2)].
prof.thetaDot = prof.thetaDs*V; % track cant angle angular rate [rad/s].
prof.thetaDDot = prof.thetaDDs*V^2 + prof.thetaDs*aLong; % track cant angle angular acceleration [rad/s^2].
end



% constantProfile(value) function returns track properties that remain constant over a route section.
% INPUTS AND OUTPUTS: constantProfile
% Input value is the constant curvature or cant value used over one route section.
% Output value is the unchanged constant profile value.
% Outputs valueDs and valueDDs are its first and second spatial derivatives, which are both zero.
function [value, valueDs, valueDDs] = constantProfile(value)
valueDs = 0; % value_Ds is the first derivative of the profile with respect to route distance.
valueDDs = 0; % value_DDs is the second derivative of the profile with respect to route distance.
end


% linearProfile(s, s0, L, y0, y1) function returns the linear interpolation of a profile over a transition length.
% You get the defined values for the linaerProfile which has the following parameters you specify:
% s = current route station, 
% s0 = start of the transition, 
% L = transition length, 
% y0 = start value, 
% y1 = end value.

% INPUTS AND OUTPUTS: linearProfile
% Input s is the current route station and s0 is the transition start station [m].
% Input L is the transition length [m]. Inputs y0 and y1 are the start and end profile values.
% Outputs value, valueDs, and valueDDs are the profile value and its first two spatial derivatives.
function [value, valueDs, valueDDs] = linearProfile(s, s0, L, y0, y1)
if L <= 0
    [value, valueDs, valueDDs] = constantProfile(y1); % value and its two spatial derivatives take the constant end value when transition length L is zero.
    return;
end
u = min(max((s - s0)/L, 0), 1); % u is the normalized position inside the current transition, bounded from 0 to 1.
value = y0 + (y1 - y0)*u; % value is the interpolated profile magnitude at the current route station.
valueDs = (y1 - y0)/L; % value_Ds is the first derivative of the profile with respect to route distance.
valueDDs = 0; % value_DDs is the second derivative of the profile with respect to route distance.
end



%smoothProfile(s, s0, L, y0, y1) function returns the smooth interpolation of a profile over a transition length.
% INPUTS AND OUTPUTS: smoothProfile
% Input s is the current route station and s0 is the transition start station [m].
% Input L is the transition length [m]. Inputs y0 and y1 are the start and end profile values.
% Outputs value, valueDs, and valueDDs are the smooth profile value and its first two spatial derivatives.
function [value, valueDs, valueDDs] = smoothProfile(s, s0, L, y0, y1)
if L <= 0
    [value, valueDs, valueDDs] = constantProfile(y1); % value and its two spatial derivatives take the constant end value when transition length L is zero.
    return;
end
u = min(max((s - s0)/L, 0), 1); % u is the normalized position inside the current transition, bounded from 0 to 1.
[shape, shapeDs, shapeDDs] = smoothStep(u, 1/L); % shape, shape_Ds, and shape_DDs define the smooth transition and its first two spatial derivatives.
value = y0 + (y1 - y0)*shape; % value is the interpolated profile magnitude at the current route station.
valueDs = (y1 - y0)*shapeDs; % value_Ds is the first derivative of the profile with respect to route distance.
valueDDs = (y1 - y0)*shapeDDs; % value_DDs is the second derivative of the profile with respect to route distance.
end


% smoothStep(u, invLength) function defines the  transition shape 
% INPUTS AND OUTPUTS: smoothStep
% Input u is the normalized transition position from zero to one.
% Input invLength is one divided by the transition length [1/m].
% Outputs shape, shapeDs, and shapeDDs are the smooth-step magnitude and its first two spatial derivatives.
function [shape, shapeDs, shapeDDs] = smoothStep(u, invLength) % invLength = 1/L, L is the transition length.
shape = 3*u^2 - 2*u^3; % shape is the smooth transition magnitude.
shapeDs = (6*u - 6*u^2)*invLength; 
shapeDDs = (6 - 12*u)*invLength^2; 
end



% INPUTS AND OUTPUTS: speedAtDistance
% The first input is unused because this study applies one constant speed over the complete route.
% Input p is the complete system-parameter structure.
% Outputs V and speedKmh are the speed in m/s and km/h. Output aLong is longitudinal acceleration [m/s^2].
function [V, speedKmh, aLong] = speedAtDistance(~, p)
V = kmhToMs(p.speed.constantKmh); % V is vehicle speed [m/s].
V = max(V, p.speed.minimum_mps); 
speedKmh = msToKmh(V); 
aLong = 0; % a_long is longitudinal acceleration [m/s^2].
end



% Route geometry end ---------------------------------------------------








%% FUNCTION GROUP 3 - VEHICLE DYNAMIC MODEL
% These functions define the physical vehicle equations, movement limits, and mechanical stops.

% The vehicle equations are written directly as six coupled differential
% acceleration equations. The wheel-filter differential equation is appended
% to form the fourteen-state derivative vector.


% INPUTS AND OUTPUTS: forceInputSystemEquations
% The closed-loop function shiftTiltRouteEquations prepares these six inputs
% before it calls forceInputSystemEquations.
%
%
% Input 1: x is the fourteen-element mechanical-state column vector.
% This function will be called later by the ODE solver with the current state vector x,
%  and it will return the time derivative of the state vector dx.
% The ode solver will integrate the state vector over time to simulate the vehicle's dynamic response to the track geometry and actuator inputs.
% x(1)  = y_v, vehicle-frame lateral position [m].
% x(2)  = y_s, sliding-carriage lateral position [m].
% x(3)  = y_b, bogie lateral position [m].
% x(4)  = theta_v, vehicle-frame roll angle [rad].
% x(5)  = theta_s, sliding-carriage roll angle [rad].
% x(6)  = theta_b, bogie roll angle [rad].
% x(7)  = y_v_dot, vehicle-frame lateral velocity [m/s].
% x(8)  = y_s_dot, sliding-carriage lateral velocity [m/s].
% x(9)  = y_b_dot, bogie lateral velocity [m/s].
% x(10) = theta_v_dot, vehicle-frame roll rate [rad/s].
% x(11) = theta_s_dot, sliding-carriage roll rate [rad/s].
% x(12) = theta_b_dot, bogie roll rate [rad/s].
% x(13) = y_w, wheel-filter lateral displacement [m].
% x(14) = y_w_dot, wheel-filter lateral velocity [m/s].
%
% Input 2: p is the complete parameter structure created by
% completeParameters. Its named fields contain masses, inertias, suspension
% properties, geometry, gravity, wheel-filter settings, controller settings,
% movement limits, and mechanical-stop settings.
%
% Input 3: V is the current vehicle speed returned by speedAtDistance [m/s].
%
% Input 4: track is the local track-condition structure created by
% trackProfileFromDistance and completed by addTimeDerivatives.
% track = [kappa; kappaDot; kappaDDot; theta; thetaDot; thetaDDot] is the local track geometry.
% track.kappa is the local track curvature [1/m].
% track.theta is the local track cant angle [rad].
% track.thetaDot is the local track cant angular rate [rad/s].
% track.thetaDDot is the local track cant angular acceleration [rad/s^2].
% track.kappaDot is the local track curvature rate [1/(m s)].
% track.kappaDDot is the local track curvature acceleration [1/(m s^2)].
%
% Input 5: u is the two-element actuator-input column vector.
% u(1) = F_x_total, actual lateral actuator force supplied to the model [N].
% u(2) = M_tilt, actual tilt actuator moment supplied to the model [N m].
%
% Input 6: caseDef is the operating-case structure created by
% defineShiftTiltCase. It contains the enabled motions and case movement limits.
%
% Output: dx is the fourteen-element mechanical-state derivative vector.
% dx(1:6) contains the six position and angle rates.
% dx(7:12) contains the six lateral and roll accelerations.
% dx(13) contains y_w_dot, the wheel-filter lateral velocity [m/s].
% dx(14) contains y_w_ddot, the wheel-filter lateral acceleration [m/s^2].


function dx = forceInputSystemEquations(x, p, V, track, u, caseDef)

    % X = [q; q_dot; y_w; y_w_dot] is the complete mechanical state vector.
x = x(:); % x is reshaped into one state column vector.

q = x(1:6); % q = [y_v; y_s; y_b; theta_v; theta_s; theta_b].
qd = x(7:12); % q_dot = [y_v_dot; y_s_dot; y_b_dot; theta_v_dot; theta_s_dot; theta_b_dot].

    % q variables
yv = q(1); % vehicle-frame lateral position [m].
ys = q(2); % sliding-carriage lateral position [m].
yb = q(3); % bogie lateral position [m].
thetav = q(4); % vehicle-frame roll angle [rad].
thetas = q(5); % sliding-carriage roll angle [rad].
thetab = q(6); % bogie roll angle [rad].

    % qd variables
yvd = qd(1); % vehicle-frame lateral velocity [m/s].
ysd = qd(2); % sliding-carriage lateral velocity [m/s].
ybd = qd(3); % bogie lateral velocity [m/s].
thetavd = qd(4); % vehicle-frame roll rate [rad/s].
thetasd = qd(5); % sliding-carriage roll rate [rad/s].
thetabd = qd(6); % bogie roll rate [rad/s].

    % wheel-filter variables
yw = x(13); % wheel-filter displacement [m].
ywd = x(14); % wheel-filter velocity [m/s].

    % U with its actuator input boys
u = u(:); % reshaped into one actuator-input column vector.
FxTotal = u(1); % supplied lateral actuator force [N].
Mtilt = u(2); % supplied tilt actuator moment [N m].


 % Mechanical permitted limits applied to the relative displacements and angles.
 % Check out the calculations of mechanical stops right below the current function you are reading

 % Lateral Acceleration very important, note this:
curveAccel = V^2*track.kappa; % a_curve lateral curve acceleration [m/s^2].


stop = mechanicalStopInputs(q, qd, p, caseDef, track);
FxApplied = FxTotal + stop.shiftForce; % this includes mechanical stops and lateral actuator force [N].
MtiltApplied = Mtilt + stop.relativeTiltMoment; % M_tilt_applied includes actuator and tilt-stop moments [N m].


% Vehicle Dynamics Equations of Motion....check the white paper to understand better.

yvdd = ( ... % y_v_ddot is the vehicle-frame lateral acceleration [m/s^2].
    2*(p.Kfy*ys + p.Cfy*ysd) ...
  + 2*(p.h1+p.h2)*(p.Kfy*thetav + p.Cfy*thetavd) ...
  - 2*(p.Kfy*yv + p.Cfy*yvd) ...
  - 2*p.h2*(p.Kfy*thetas + p.Cfy*thetasd) ...
  + p.mv*p.g*track.theta ...
  - p.mv*curveAccel ...
  - p.mv*p.hg3*track.thetaDDot ...
  + stop.frameCarriageForce) / p.mv;

ysdd = ( ... % y_s_ddot is the sliding-carriage lateral acceleration [m/s^2].
    2*(p.Ksy*yb + p.Csy*ybd) ...
  + 2*(p.h2+p.h3)*(p.Ksy*thetas + p.Csy*thetasd) ...
  + 2*p.h4*(p.Ksy*thetab + p.Csy*thetabd) ...
  + 2*(p.Kfy*yv + p.Cfy*yvd) ...
  + 2*p.h2*(p.Kfy*thetas + p.Cfy*thetasd) ...
  - 2*(p.h1+p.h2)*(p.Kfy*thetav + p.Cfy*thetavd) ...
  - 2*(p.Kfy*ys + p.Cfy*ysd) ...
  - 2*(p.Ksy*ys + p.Csy*ysd) ...
  + p.ms*p.g*track.theta ...
  - p.ms*curveAccel ...
  - p.ms*p.hg2*track.thetaDDot ...
  + FxApplied ...
  - stop.frameCarriageForce) / p.ms;

ybdd = ( ... % y_b_ddot is the bogie lateral acceleration [m/s^2].
    2*(p.Ksy*ys + p.Csy*ysd) ...
  + 2*p.h5*(p.Kby*thetab + p.Cby*thetabd) ...
  + 2*(p.Kby*yw + p.Cby*ywd) ...
  - 2*(p.Kby*yb + p.Cby*ybd) ...
  - 2*(p.Ksy*yb + p.Csy*ybd) ...
  - 2*(p.h2+p.h3)*(p.Ksy*thetas + p.Csy*thetasd) ...
  - 2*p.h4*(p.Ksy*thetab + p.Csy*thetabd) ...
  + p.mb*p.g*track.theta ...
  - p.mb*p.hg1*track.thetaDDot ...
  - p.mb*curveAccel ...
  - FxApplied) / p.mb;

thetavdd = ( ... % theta_v_ddot is the vehicle-frame roll acceleration [rad/s^2].
    2*(p.h1+p.h2)*(p.Kfy*yv + p.Cfy*yvd) ...
  + 2*(p.h1+p.h2)*p.h2*(p.Kfy*thetas + p.Cfy*thetasd) ...
  + 2*p.d1^2*(p.Kfz*thetas + p.Cfz*thetasd) ...
  - 2*(p.h1+p.h2)^2*(p.Kfy*thetav + p.Cfy*thetavd) ...
  - 2*(p.h1+p.h2)*(p.Kfy*ys + p.Cfy*ysd) ...
  - 2*p.d1^2*(p.Kfz*thetav + p.Cfz*thetavd) ...
  + p.mv*p.g*(yv-ys) ...
  - p.Ivr*track.thetaDDot ...
  - (p.h1+p.h2)*stop.frameCarriageForce ...
  + MtiltApplied) / p.Ivr;

thetasdd = ( ... % theta_s_ddot is the sliding-carriage roll acceleration [rad/s^2].
    2*p.h2*(p.h1+p.h2)*(p.Kfy*thetav + p.Cfy*thetavd) ...
  + 2*p.h2*(p.Kfy*ys + p.Cfy*ysd) ...
  + 2*p.d1^2*(p.Kfz*thetav + p.Cfz*thetavd) ...
  + 2*(p.h2+p.h3)*(p.Ksy*ys + p.Csy*ysd) ...
  - 2*p.h2*(p.Kfy*yv + p.Cfy*yvd) ...
  - 2*p.h2^2*(p.Kfy*thetas + p.Cfy*thetasd) ...
  - 2*p.d1^2*(p.Kfz*thetas + p.Cfz*thetasd) ...
  - 2*(p.h2+p.h3)*(p.Ksy*yb + p.Csy*ybd) ...
  - 2*(p.h2+p.h3)^2*(p.Ksy*thetas + p.Csy*thetasd) ...
  - 2*(p.h2+p.h3)*p.h4*(p.Ksy*thetab + p.Csy*thetabd) ...
  + p.ms*p.g*(ys-yb) ...
  + 2*p.d1^2*(p.Klgy*thetab + p.Clgy*thetabd) ...
  - 2*p.d1^2*(p.Klgy*thetas + p.Clgy*thetasd) ...
  - p.Isr*track.thetaDDot ...
  - (p.h2+p.h3)*FxApplied ...
  - MtiltApplied ...
  + p.h2*stop.frameCarriageForce ...
  + stop.carriageBogieRollMoment) / p.Isr;

thetabdd = ( ... % theta_b_ddot is the bogie roll acceleration [rad/s^2].
    2*p.h4*(p.Ksy*ys + p.Csy*ysd) ...
  - 2*p.h5^2*(p.Kby*thetab + p.Cby*thetabd) ...
  - 2*p.h5*(p.Kby*yw + p.Cby*ywd) ...
  - 2*p.d2^2*(p.Kbz*thetab + p.Cbz*thetabd) ...
  - 2*p.h4*(p.Ksy*yb + p.Csy*ybd) ...
  - 2*p.h4*(p.h2+p.h3)*(p.Ksy*thetas + p.Csy*thetasd) ...
  - 2*p.h4^2*(p.Ksy*thetab + p.Csy*thetabd) ...
  + 2*p.h5*(p.Kby*yb + p.Cby*ybd) ...
  + 2*p.d1^2*(p.Klgy*thetas + p.Clgy*thetasd) ...
  - 2*p.d1^2*(p.Klgy*thetab + p.Clgy*thetabd) ...
  - p.Ibr*track.thetaDDot ...
  - p.h4*FxApplied ...
  - stop.carriageBogieRollMoment ...
  + stop.bogieCantRelativeRollMoment) / p.Ibr;

% Wheel Rail contact filter. Currently im using very basic second order filter design which will be improved
% in the future work with a more detailed wheel-rail contact model.
% if you wanna play around with the wheel rail contact designs, make the changes here.

yo = 0; % y_o is the external wheel-filter input and is zero in this script [m].

ywdd = -2*p.wheel.zeta*p.wheel.omega*ywd ... % y_w_ddot is the wheel-filter acceleration [m/s^2].
     - p.wheel.omega^2*yw ...
     + p.wheel.omega^2*yo;


% initializing the dx vector
dx = zeros(14, 1); % dx is the fourteen-state mechanical derivative vector.

% Placing the values in dx
dx(1:6) = qd; % dx(1:6) = q_dot.
dx(7:12) = [yvdd; ysdd; ybdd; thetavdd; thetasdd; thetabdd]; % dx(7:12) = q_ddot.
dx(13) = ywd; % dx(13) is y_w_dot [m/s].
dx(14) = ywdd; % dx(14) is y_w_ddot [m/s^2].

end




%% mechanicalStopInputs function logic
% Input q contains the six lateral positions and roll angles. Input qd contains their six rates.
% Input p contains the vehicle properties.

% Input track contains the local cant angle and cant angular rate.
% Output stop contains all calculated mechanical-stop forces and moments.

function stop = mechanicalStopInputs(q, qd, p, caseDef, track)
stop.frameCarriageForce = 0; % F_stop_f is the frame-to-carriage lateral stop force [N].
stop.shiftForce = 0; % F_stop_s is the carriage-to-bogie lateral stop force [N].
stop.relativeTiltMoment = 0; % M_stop_vs is the theta_v - theta_s stop moment [N m].
stop.carriageBogieRollMoment = 0; % M_stop_sb is the theta_s - theta_b stop moment [N m].
stop.bogieCantRelativeRollMoment = 0; % M_stop_bt is the bogie-to-track roll stop moment [N m].

if ~p.stop.enableBumpStops
    return;
end

frameCarriageDeflection = q(1) - q(2) ... % delta_y_f = y_v - y_s - (h_1 + h_2)*theta_v + h_2*theta_s [m].
    - (p.h1+p.h2)*q(4) ...
    + p.h2*q(5);
frameCarriageDeflectionDot = qd(1) - qd(2) ... % delta_y_f_dot = y_v_dot - y_s_dot - (h_1 + h_2)*theta_v_dot + h_2*theta_s_dot [m/s].
    - (p.h1+p.h2)*qd(4) ...
    + p.h2*qd(5);
carriageBogieDeflection = q(2) - q(3) ... % delta_y_s = y_s - y_b - (h_2 + h_3)*theta_s - h_4*theta_b [m].
    - (p.h2+p.h3)*q(5) ...
    - p.h4*q(6);
carriageBogieDeflectionDot = qd(2) - qd(3) ... % delta_y_s_dot = y_s_dot - y_b_dot - (h_2 + h_3)*theta_s_dot - h_4*theta_b_dot [m/s].
    - (p.h2+p.h3)*qd(5) ...
    - p.h4*qd(6);


% Raw value calcualtions for the relative tilt and roll angles and their rates. 

relativeTilt = q(4) - q(5); % relative_tilt = theta_v - theta_s [rad].
relativeTiltDot = qd(4) - qd(5); % relative_tilt_dot = theta_v_dot - theta_s_dot [rad/s].
carriageBogieRoll = q(5) - q(6); % carriage_bogie_roll = theta_s - theta_b [rad].
carriageBogieRollDot = qd(5) - qd(6); % carriage_bogie_roll_dot = theta_s_dot - theta_b_dot [rad/s].
bogieCantRelativeRoll = q(6) - track.theta; % bogie_track_roll = theta_b - theta_track [rad].
bogieCantRelativeRollDot = qd(6) - track.thetaDot; % bogie_track_roll_dot = theta_b_dot - theta_track_dot [rad/s].
limits = caseTravelLimits(p, caseDef); % limits stores the movement limits applied to the selected control case.



% Assigning the values to their respective stop forces and moments using the bumpStopReaction function.
stop.frameCarriageForce = bumpStopReaction( ... 
    frameCarriageDeflection, frameCarriageDeflectionDot, ...
    limits.deltaFy, p.stop.frameCarriageStiffness, ...
    p.stop.frameCarriageDamping);
stop.shiftForce = bumpStopReaction( ... 
    carriageBogieDeflection, carriageBogieDeflectionDot, ...
    limits.deltaSy, p.stop.shiftStiffness, p.stop.shiftDamping);
stop.relativeTiltMoment = bumpStopReaction(relativeTilt, relativeTiltDot, ... 
    limits.thetaVMinusThetaS, p.stop.relativeTiltStiffness, ...
    p.stop.relativeTiltDamping);
stop.carriageBogieRollMoment = bumpStopReaction( ... 
    carriageBogieRoll, carriageBogieRollDot, ...
    limits.thetaSMinusThetaB, ...
    p.stop.carriageBogieRollStiffness, ...
    p.stop.carriageBogieRollDamping);
stop.bogieCantRelativeRollMoment = bumpStopReaction( ... 
    bogieCantRelativeRoll, bogieCantRelativeRollDot, ...
    p.stop.bogieCantRelativeRollLimit, ...
    p.stop.bogieCantRelativeRollStiffness, ...
    p.stop.bogieCantRelativeRollDamping);
end





% INPUTS AND OUTPUTS: caseTravelLimits
% Input p contains the default movement limits.
% Optional input caseDef contains the movement limits selected for the current operating case.
% Output limits contains the final limits used by the controller and mechanical stops.

function limits = caseTravelLimits(p, caseDef)
limits.deltaFy = p.stop.frameCarriageDeflectionLimit; % frame-to-carriage spring-deflection limit [m].
limits.deltaSy = p.stop.carriageBogieDeflectionLimit; % carriage-to-bogie spring-deflection limit [m].
limits.ysMinusYb = p.ctrl.shiftTravelLimit; %  carriage-shift limit for y_s - y_b [m].
limits.thetaVMinusThetaS = p.stop.relativeTiltLimitWithTilt; %  relative frame-tilt limit for theta_v - theta_s [rad].
limits.thetaSMinusThetaB = p.stop.carriageBogieRollLimit; % carriage-to-bogie roll limit for theta_s - theta_b [rad].


% Sanity check for the optional caseDef input. If it is missing or empty, the default limits are returned.
if nargin < 2 || isempty(caseDef)
    return;
end
% If the caseDef input is present, it may contain a field named enableTilt. 
% If that field exists and is false, the relative frame-tilt limit is replaced with the no-tilt value.
if isfield(caseDef, 'enableTilt') && ~caseDef.enableTilt
    limits.thetaVMinusThetaS = p.stop.relativeTiltLimitNoTilt; % limits.thetaVMinusThetaS is the active relative frame-tilt limit for theta_v - theta_s [rad].
end

% If the caseDef input is present, it may contain a field named limits.
% If that field exists, it may contain any of the movement-limit field names.
% The matching default limit is replaced with the case-specific value.
if isfield(caseDef, 'limits')
    names = fieldnames(caseDef.limits); % names contains any case-specific movement-limit field names.
    for iName = 1:numel(names) % iName is the loop index for the following repeated statements.
        limits.(names{iName}) = caseDef.limits.(names{iName}); % The case-specific value replaces the matching default movement limit.
    end
end

end









% INPUTS AND OUTPUTS: bumpStopReaction
% Inputs relativePosition and relativeVelocity are the displacement and rate across one mechanical stop.
% Input limitValue is the permitted travel. Inputs stiffness and damping define the stop properties.
% Output reaction is the force or moment generated after the permitted travel is exceeded.
function reaction = bumpStopReaction(relativePosition, relativeVelocity, ...
    limitValue, stiffness, damping)
reaction = 0; % reaction is the spring-damper force or moment produced by one bump stop.
if limitValue <= 0 || stiffness <= 0
    return;
end

if relativePosition > limitValue
    penetration = relativePosition - limitValue; % penetration is displacement or angle beyond the permitted stop limit.
    velocityIntoStop = max(relativeVelocity, 0); % velocity_into_stop is the motion rate directed farther into the stop.
    reaction = -stiffness*penetration - damping*velocityIntoStop; % reaction is the spring-damper force or moment produced by one bump stop.
elseif relativePosition < -limitValue
    penetration = relativePosition + limitValue; % penetration is displacement or angle beyond the permitted stop limit.
    velocityIntoStop = min(relativeVelocity, 0); % velocity_into_stop is the motion rate directed farther into the stop.
    reaction = -stiffness*penetration - damping*velocityIntoStop; % reaction is the spring-damper force or moment produced by one bump stop.
end
end







% Vehicle Dynamics end ---------------------------------------------------

















%% FUNCTION GROUP 4 - RESULTANT LOAD AND CENTRE-OF-MASS CHARACTERISTICS

% This function group calculates physical characteristics describing how
% the track geometry, vehicle speed, gravity, and vehicle-body positions
% affect the direction and location of the vehicle resultant.
%
% resultantAngleFromTrack calculates the direction of the combined
% gravitational and curve-induced centripetal acceleration relative to the canted rail
% plane. The resulting angle lambda_track indicates the direction in which
% centre-of-mass compensation is required.
%
% massWeightedComInwardShift calculates the current lateral position of the
% complete vehicle centre of mass from the frame, carriage, and bogie
% positions and their respective masses.
%
%
% The curve-induced lateral acceleration is:
%
%       a_curve = V^2*kappa
%
% The acceleration components normal to and across the canted rail plane are:
%
%       a_vertical = g*cos(theta_track) + a_curve*sin(theta_track)
%
%       a_outward = a_curve*cos(theta_track) - g*sin(theta_track)
%
% The resultant-force angle relative to the rail plane is:
%
%       lambda_track = atan2(a_outward, a_vertical)
%
%
% The controller converts lambda_track into the required centre-of-mass
% displacement:
%
%       y_COM_demand = h_effective*tan(lambda_track)
%
%
% This group also calculates the current mass-weighted centre-of-mass
% position using the lateral positions of the vehicle frame, sliding
% carriage, and bogie:  its better to check the white paper cause its pretty straight forward
% 
%
% y_COM_actual = (m_v*y_v + m_s*y_s + m_b*y_b)
%                /(m_v + m_s + m_b)
%
% y_COM_actual gives the controller feedback about the centre-of-mass
% movement already produced by the vehicle. The controller compares this
% value with y_COM_demand to determine the remaining compensation required.


% resultantAngleFromTrack

% make sure the Resultant stays between the track gauge.
function lambdaTrack = resultantAngleFromTrack(track, p, V)
curveAccel = V^2*track.kappa; % a_curve = V^2*kappa [m/s^2].

verticalLoadPerMass = p.g*cos(track.theta) ... % a_vertical = g*cos(theta_track) + a_curve*sin(theta_track) [m/s^2].
    + curveAccel*sin(track.theta);

outwardLoadPerMass = curveAccel*cos(track.theta) ... % a_outward = a_curve*cos(theta_track) - g*sin(theta_track) [m/s^2].
    - p.g*sin(track.theta);


lambdaTrack = atan2(outwardLoadPerMass, verticalLoadPerMass);

end




% INPUTS AND OUTPUTS: massWeightedComInwardShift
% this lets us know the position of the com relative to the track gauge. 
% If the com is outside the track gauge, it will be a problem.... Big Big problem.
% Input xMechanical contains the mechanical states. Same as x that is used in forceInputSystemEquations.
% Output comIn is the mass-weighted inward centre-of-mass shift [m].
function comIn = massWeightedComInwardShift(xMechanical, p)
mass = [p.mv; p.ms; p.mb]; % mass contains the frame, carriage, and bogie masses [kg].
comIn = sum(mass.*xMechanical(1:3))/sum(mass); % xMechanical(1:3) = [y_v; y_s; y_b].
end






% Resultant load and centre-of-mass characteristics end ------------------------------------










%% FUNCTION GROUP 5 - CONTROLLER TARGETS AND COMMANDS


% The controller is a proportional-derivative controller with static-inverse
% plant feed-forward 
%
% CENTRE-OF-MASS COMPENSATION DEMAND
%
% y_COM_demand tells me how much the entire vehicles COM moved laterally
% y_COM_demand = h_effective*tan(lambda_track) [m].   
%
% h_effective = (m_v*h_g3 + m_s*h_g2 + m_b*h_g1) 
%               /(m_v + m_s + m_b) [m].   
%
%
% This gain will make sense later on but for now know that it will show the  
% Association between lateral shifting of carriage and frame with the COM of entier body.
% Just fuking read the white paper and you will understand it better.
%
% G_shift = (m_v + m_s)/(m_v + m_s + m_b) [-]. COM gain caused by lateral shifting 
%
% G_tilt = m_v*(h_1 + h_2)/(m_v + m_s + m_b) [m/rad]. COM gain caused by frame tilt
%
% The conversion from motion to centre-of-mass correction is:
%
% y_COM_from_shift = G_shift*y_shift_target [m].   
% y_COM_from_tilt = G_tilt*theta_tilt_target [m].
%
% PASSIVE CASE
%
% y_shift_target = 0 [m].
% theta_tilt_target = 0 [rad].
% F_x_command = 0 [N].
% M_tilt_command = 0 [N m].
%
% SHIFT-ONLY CASE
%
% Stay between the limits
%
% y_shift_target = clamp(y_COM_demand/G_shift, -y_shift_limit, +y_shift_limit) [m]. 
%
% The current mass-weighted centre-of-mass position is:
%
% y_COM_actual = (m_v*y_v + m_s*y_s + m_b*y_b)
%                /(m_v + m_s + m_b) [m].
%
% The remaining centre-of-mass error is:
%
% e_COM = y_COM_demand - y_COM_actual [m].
%
% The state-corrected shift target is:
%
% y_shift_target = clamp(y_shift_actual + e_COM/G_shift,        note: y_shift_actual = y_s - y_b [m].
%                        -y_shift_limit, +y_shift_limit) [m].  
%
% TILT-ONLY CASE
%
% theta_tilt_target = clamp(y_COM_demand/G_tilt,
%                           -theta_tilt_limit,
%                           +theta_tilt_limit) [rad].
%
% COMBINED SHIFT-AND-TILT CASE
%
% y_COM_shift_capacity = G_shift*y_shift_limit [m].
% y_COM_tilt_capacity = G_tilt*theta_tilt_limit [m].
% y_COM_combined_capacity = y_COM_shift_capacity
%                           + y_COM_tilt_capacity [m].
%
% f_demand = clamp(y_COM_demand/y_COM_combined_capacity, -1, +1).
%
% y_shift_target = f_demand*y_shift_limit [m].
% theta_tilt_target = f_demand*theta_tilt_limit [rad].
%
%
%
% LATERAL-SHIFT ACTUATOR COMMAND
%
% The steady input-output relationship of the vehicle plant is:
%
% [y_shift_steady; theta_tilt_steady] = G_0*[F_x; M_tilt].

% NOTE: G_0 is the static plant gain matrix. It is calculated by the function 
% calculateStaticPlantGainMatrix. To understand the calcualtions refer to the comments on that function.
% Dont get confused this gain with mass gains G_shift and G_tilt. 
%
% For the shift-only case:
%
% F_x_feedforward = y_shift_target/G_0_yy [N].      G_0_yy is the gain for which lateral shift is concerned
%
% e_shift = y_shift_target - y_shift_actual [m].
%
% F_x_unlimited = F_x_feedforward
%                 + K_p_shift*e_shift
%                 - K_d_shift*y_shift_actual_dot [N].
%
% F_x_command = clamp(F_x_unlimited,
%                     -F_x_limit, +F_x_limit) [N].
%
% TILT ACTUATOR COMMAND
%
% For the tilt-only case:
%
% M_tilt_feedforward = theta_tilt_target/G_0_theta_theta [N m].
% G_0_theta_theta is the gain for which body tilt is concerned. 
%



% For the combined shift-and-tilt case:
% NOTE: The combined shift and tilt is a bit different from the one given in the white paper.
% here both shift and tilt are in sync with each other.

%
% [F_x_feedforward; M_tilt_feedforward]
%     = G_0_inverse*[y_shift_target; theta_tilt_target].
%
% e_tilt = theta_tilt_target - theta_tilt_actual [rad].
%
% M_tilt_unlimited = M_tilt_feedforward
%                    + K_p_tilt*e_tilt
%                    - K_d_tilt*theta_tilt_actual_dot
%                    + M_rate_sync [N m].
%
% For the combined case, the synchronized tilt rate is:
%
% r_shift_normalized = y_shift_actual_dot/y_shift_limit [1/s]. 

% theta_tilt_sync_dot = r_shift_normalized*theta_tilt_limit [rad/s].

% e_rate_sync = theta_tilt_sync_dot - theta_tilt_actual_dot [rad/s].

% M_rate_sync = K_rate_sync*e_rate_sync [N m].
%
% M_tilt_command = clamp(M_tilt_unlimited,
%                        -M_tilt_limit, +M_tilt_limit) [N m].
%




function target = controllerTarget(track, p, V, caseDef)


lambdaTrack = resultantAngleFromTrack(track, p, V); 
comDemand = p.ctrl.effectiveHeight*tan(lambdaTrack); % y_COM_demand = h_effective*tan(lambda_track) [m].
limits = caseTravelLimits(p, caseDef); % mechanical travel limits based on the selected case.

target.shift = 0; % requested carriage shift y_s - y_b [m].
target.tilt = 0; % requested relative frame tilt theta_v - theta_s [rad].
target.lambdaTrack = lambdaTrack; %  resultant-force angle measured from the canted rail plane [rad].
target.comDemand = comDemand; % inward centre-of-mass displacement requested for curve compensation [m].
target.comFromShift = 0; % centre-of-mass correction assigned to lateral shift [m].
target.comFromTilt = 0; % centre-of-mass correction assigned to body tilt [m].

% Target generation based on selected case.

if caseDef.enableShift && caseDef.enableTilt % for both shift and tilt
    shiftComCapacity = p.ctrl.shiftComGain*limits.ysMinusYb; % 
    tiltComCapacity = p.ctrl.tiltComGain*limits.thetaVMinusThetaS; % maximum centre-of-mass correction available from body tilt [m].
    combinedComCapacity = shiftComCapacity + tiltComCapacity; %  total correction available when shift and tilt move together [m].
    if combinedComCapacity > eps % eps is a small number to prevent division by zero.
        synchronizedDemandFraction = clampAbs( ... % shared signed movement fraction from -1 to +1.
            comDemand/combinedComCapacity, 1);
    else
        synchronizedDemandFraction = 0; % A zero fraction prevents division by zero when both available movements are zero.
    end
    target.shift = synchronizedDemandFraction*limits.ysMinusYb; % follows the shared movement fraction of the available lateral travel [m].
    target.tilt = synchronizedDemandFraction*limits.thetaVMinusThetaS; % follows the same shared movement fraction of the available tilt [rad].
    target.comFromShift = p.ctrl.shiftComGain*target.shift; %  centre-of-mass correction assigned to lateral shift [m].
    target.comFromTilt = p.ctrl.tiltComGain*target.tilt; % centre-of-mass correction assigned to body tilt [m].


elseif caseDef.enableShift   % shift-only case
    target.shift = clampAbs(comDemand/p.ctrl.shiftComGain, ... 
        limits.ysMinusYb);
    target.comFromShift = p.ctrl.shiftComGain*target.shift; 

elseif caseDef.enableTilt  % tilt-only case
    target.tilt = clampAbs(comDemand/p.ctrl.tiltComGain, ... 
        limits.thetaVMinusThetaS);
    target.comFromTilt = p.ctrl.tiltComGain*target.tilt; 
end
end




% controlTargetWithState : Target generation based on selected case and vehicle state


function target = controlTargetWithState(x, p, caseDef, target)
xMechanical = x(1:14); % x_mechanical contains q, q_dot, y_w, and y_w_dot.
relativeShift = xMechanical(2) - xMechanical(3); % relative_shift = y_s - y_b [m].
relativeTilt = xMechanical(4) - xMechanical(5); % relative_tilt = theta_v - theta_s [rad].
limits = caseTravelLimits(p, caseDef); % limits stores the movement limits applied to the selected control case.

target.comFromShift = 0; % target.comFromShift is the centre-of-mass correction assigned to lateral shift [m].
target.comFromTilt = 0; % target.comFromTilt is the centre-of-mass correction assigned to body tilt [m].

if caseDef.enableShift && caseDef.enableTilt
    target.shift = clampAbs(target.shift, limits.ysMinusYb); % target.shift retains the synchronized lateral-shift request inside its movement limit [m].
    target.tilt = clampAbs(target.tilt, limits.thetaVMinusThetaS); % target.tilt retains the synchronized tilt request inside its movement limit [rad].
    target.comFromShift = p.ctrl.shiftComGain*(target.shift - relativeShift); % target.comFromShift is the remaining centre-of-mass correction required from lateral shift [m].
    target.comFromTilt = p.ctrl.tiltComGain*(target.tilt - relativeTilt); % target.comFromTilt is the centre-of-mass correction assigned to body tilt [m].
elseif caseDef.enableShift
    actualComIn = massWeightedComInwardShift(xMechanical, p); % actual_COM_in is the inward centre-of-mass shift [m].
    comError = target.comDemand - actualComIn; % COM_error is the remaining centre-of-mass displacement error [m].
    target.shift = clampAbs(relativeShift + comError/p.ctrl.shiftComGain, ... % target.shift is the requested carriage shift y_s - y_b [m].
        limits.ysMinusYb);
    target.comFromShift = p.ctrl.shiftComGain*(target.shift - relativeShift); % target.comFromShift is the centre-of-mass correction assigned to lateral shift [m].
elseif caseDef.enableTilt
    target.tilt = clampAbs(target.comDemand/p.ctrl.tiltComGain, ... % target.tilt is the requested relative frame tilt theta_v - theta_s [rad].
        limits.thetaVMinusThetaS);
    target.comFromTilt = p.ctrl.tiltComGain*target.tilt; % target.comFromTilt is the centre-of-mass correction assigned to body tilt [m].
end
end






% controllerCommand: Force and moment commands.

function command = controllerCommand(x, p, caseDef, target)
relativeShift = x(2) - x(3); % relative_shift = y_s - y_b [m].
relativeShiftDot = x(8) - x(9); % relative_shift_dot = y_s_dot - y_b_dot [m/s].
relativeTilt = x(4) - x(5); % relative_tilt = theta_v - theta_s [rad].
relativeTiltDot = x(10) - x(11); % relative_tilt_dot = theta_v_dot - theta_s_dot [rad/s].
limits = caseTravelLimits(p, caseDef); % limits contains the active shift and tilt movement limits.
synchronizedTiltRate = 0; % synchronized_tilt_rate is the tilt rate requested from the measured lateral-shift rate [rad/s].
if caseDef.enableShift && caseDef.enableTilt
    if abs(limits.ysMinusYb) > eps && abs(limits.thetaVMinusThetaS) > eps
        normalizedShiftRate = relativeShiftDot/limits.ysMinusYb; % normalized_shift_rate is lateral-shift rate divided by available shift travel [1/s].
        synchronizedTiltRate = normalizedShiftRate*limits.thetaVMinusThetaS; % synchronized_tilt_rate gives tilt the same normalized rate of change [rad/s].
    end
end

Fx = 0; % F_x is the commanded lateral actuator force [N].
Mtilt = 0; % M_tilt is the commanded tilt actuator moment [N m].
feedforward = staticFeedforwardCommand(p, caseDef, target); % feedforward = [F_feedforward; M_feedforward] from the steady plant gain.

if caseDef.enableShift
    Fx = feedforward(1) + p.ctrl.shiftKp*(target.shift - relativeShift) ... % 
        - p.ctrl.shiftKd*relativeShiftDot;
    Fx = clampAbs(Fx, p.ctrl.forceLimit); % F_x is limited to the permitted lateral actuator force [N].
end

if caseDef.enableTilt
    Mtilt = feedforward(2) + p.ctrl.tiltKp*(target.tilt - relativeTilt) ... 
        - p.ctrl.tiltKd*relativeTiltDot;
    if caseDef.enableShift
        tiltRateSyncError = synchronizedTiltRate - relativeTiltDot; % tilt_rate_sync_error compares normalized tilt and lateral-shift rates [rad/s].
        Mtilt = Mtilt + p.ctrl.tiltRateSyncGain*tiltRateSyncError; % M_tilt adds a correcting moment so tilt follows the lateral-shift rate [N m].
    end
    Mtilt = clampAbs(Mtilt, p.ctrl.momentLimit); % M_tilt is limited to the permitted tilt actuator moment [N m].
end

command.Fx = Fx; % command.F_x stores the final limited lateral-force command [N].
command.Mtilt = Mtilt; % command.M_tilt stores the final limited tilt-moment command [N m].
end



% MATHEMATICAL LOGIC: calculateStaticFeedforwardMatrices
%
%
% The complete nonlinear differential-equation model can be written as:
%
% x_dot = f(x, u),
%
% where:
%
% x is the fourteen-state mechanical vector,
% u = [F_x; M_tilt] is the two-element actuator-input vector,
%
% The model is evaluated around the selected operating point:
%
% x_0 = zeros(14, 1),
% u_0 = zeros(2, 1),
% kappa_0 = 0,
% theta_track_0 = 0,
% theta_track_dot_0 = 0,
% theta_track_ddot_0 = 0.
%
%
% Close to this operating point, the differential equations are approximated
% by the linear equation:
%
% delta_x_dot = A_full*delta_x + B_full*delta_u.
%
%
% For state j, the code calculates column j of A_full from:
% This shit is nothing but forming the state space equation from the dynamical equations in the vehicle dynamics code.
% just like for a spring F = k*x, K being the gain, here the gain for linearization is calculated by:
% making delta_x_dot = 0 which gives:
%  delta_x = -A_full_inverse*B_full*delta_u.
%
% Now this gain is for every value in delta_x but i need to find for values i can control which are y_s - y_b and theta_v - theta_s.
% To get these specific gains in this fasion i multiply the gain matrix with 
% Cr = [0 1 -1 0 0 0 0 0 0 0 0 0 0 0; .... ys - yb
%       0 0 0 1 -1 0 0 0 0 0 0 0 0 0 0] ... theta_v - theta_s
% 
% Delta_r = Cr*delta_x = -Cr*A_full_inverse*B_full*delta_u.
%
% This is the gain matrix i need to find which is G_0 = -Cr*A_full_inverse*B_full.
%
% and then I bifurcate the gain matrix to g_yy and g_theta_theta for lateral and tilt actuator respectively.
%
%
% The following code shows how the complete state and input matrices are calculated 
% without explicetly calling a matlab function.
%
% A_full(:, j) = [f(x_0 + h_j*e_j, u_0)
%                 - f(x_0 - h_j*e_j, u_0)]/(2*h_j).
%
%
% For actuator input k, the code calculates column k of B_full from:
%
% B_full(:, k) = [f(x_0, u_0 + h_k*e_k)
%                 - f(x_0, u_0 - h_k*e_k)]/(2*h_k).
%
%
% The last two states belong to the wheel filter. The feed-forward controller
% regulates the twelve vehicle states, so the function selects:
%
% x_v = x(1:12),
% A_v = A_full(1:12, 1:12),
% B_v = B_full(1:12, 1:2).
%
% The controlled output vector is:
%
% r = [y_s - y_b; theta_v - theta_s].
%
% This output is written in matrix form as:
%
% r = C_r*x_v,
%
% where the first row of C_r forms the actual carriage-to-bogie shift and
% the second row forms the actual frame-to-carriage relative tilt.
%
% At a steady condition, every vehicle-state derivative is zero. Therefore:
%
% 0 = A_v*x_v_steady + B_v*u_steady.
%
% Solving this equation for the steady vehicle state gives:
%
% x_v_steady = -A_v_inverse*B_v*u_steady.
%
% Substituting this result into r_steady = C_r*x_v_steady gives:
%
% r_steady = -C_r*A_v_inverse*B_v*u_steady.
%
% The static plant-gain matrix is consequently defined as:
%
% G_0 = -C_r*A_v_inverse*B_v,
%
% so the steady input-output relationship becomes:
%
% r_steady = G_0*u_steady.
%
% MATLAB evaluates A_v_inverse*B_v as A_v\B_v. Solving the linear system in
% this form avoids explicitly calculating inv(A_v).
%
% For the combined shift-and-tilt case, the required feed-forward inputs are:
%
% [F_x_feedforward; M_tilt_feedforward]
%     = G_0_inverse*[y_shift_target; theta_tilt_target].
%
% For a single enabled actuator, the relevant diagonal gain is used:
%
% F_x_feedforward = y_shift_target/G_0(1, 1),
% M_tilt_feedforward = theta_tilt_target/G_0(2, 2).
%


function p = calculateStaticFeedforwardMatrices(p)
if ~p.ctrl.useStaticFeedforward
    return;
end

V = kmhToMs(p.speed.constantKmh); 
linearTrack.kappa = 0; 
linearTrack.theta = 0; 
linearTrack.thetaDot = 0; 
linearTrack.thetaDDot = 0; 
combinedCase = defineShiftTiltCase(p, 'shift_and_tilt'); % Enable both actuator channels while calculating the plant matrices.
xOperating = zeros(14, 1); 
uOperating = zeros(2, 1); 
stateEquation = @(xNow) forceInputSystemEquations(xNow, p, V, linearTrack, uOperating, combinedCase); % Vary the state while holding the inputs fixed.
inputEquation = @(uNow) forceInputSystemEquations(xOperating, p, V, linearTrack, uNow, combinedCase); % Vary the inputs while holding the states fixed.

numberOfStateEquations = 14; 
numberOfStates = numel(xOperating); 
fullStateMatrix = zeros(numberOfStateEquations, numberOfStates); 

for iState = 1:numberOfStates
    statePerturbation = 0.000001*(1 + abs(xOperating(iState))); 
    stateAbove = xOperating; 
    stateBelow = xOperating; 
    stateAbove(iState) = stateAbove(iState) + statePerturbation; 
    stateBelow(iState) = stateBelow(iState) - statePerturbation; 
    derivativeAbove = stateEquation(stateAbove); 
    derivativeBelow = stateEquation(stateBelow); 
    fullStateMatrix(:, iState) = (derivativeAbove - derivativeBelow)/(2*statePerturbation); 
end

numberOfActuatorInputs = numel(uOperating); % The two actuator inputs are F_x and M_tilt.
fullInputMatrix = zeros(numberOfStateEquations, numberOfActuatorInputs); % Preallocate the complete 14-by-2 input matrix.

for iInput = 1:numberOfActuatorInputs
    inputPerturbation = 0.000001*(1 + abs(uOperating(iInput))); % Calculate a small change for the current actuator input.
    inputAbove = uOperating; % input_above begins at the zero input vector.
    inputBelow = uOperating; % input_below begins at the zero input vector.
    inputAbove(iInput) = inputAbove(iInput) + inputPerturbation; % Increase only the current input.
    inputBelow(iInput) = inputBelow(iInput) - inputPerturbation; % Decrease only the current input.
    derivativeAbove = inputEquation(inputAbove); % Evaluate all derivatives above the operating input.
    derivativeBelow = inputEquation(inputBelow); % Evaluate all derivatives below the operating input.
    fullInputMatrix(:, iInput) = (derivativeAbove - derivativeBelow)/(2*inputPerturbation); % Form one column of the complete input matrix.
end

vehicleStateIndex = 1:12; % Select the six vehicle coordinates and their six velocities.
Av = fullStateMatrix(vehicleStateIndex, vehicleStateIndex); % Remove the two wheel-filter states to obtain the 12-by-12 A_v matrix.
Bv = fullInputMatrix(vehicleStateIndex, :); % Retain both actuator columns to obtain the 12-by-2 B_v matrix.

Cr = zeros(2, 12); % C_r converts vehicle states into relative shift and relative tilt.
Cr(1, 2) = 1; % Include positive y_s in r_y.
Cr(1, 3) = -1; % Define r_y = y_s - y_b.
Cr(2, 4) = 1; % Include positive theta_v in r_theta.
Cr(2, 5) = -1; % Define r_theta = theta_v - theta_s.

if rcond(Av) < 0.0000000001
    error('The calculated A_v matrix is too poorly conditioned for a reliable static feed-forward calculation.');
end

G0 = -Cr*(Av\Bv); % G_0 = -C_r*A_v_inverse*B_v.

if rcond(G0) < 0.0000000001
    G0Inverse = pinv(G0); % Use a pseudoinverse if G_0 is poorly conditioned.
else
    G0Inverse = G0\eye(2); % Solve G_0*G_0_inverse = I.
end

p.ctrl.vehicleStateMatrix = Av; % Store A_v for inspection and later use.
p.ctrl.vehicleInputMatrix = Bv; % Store B_v for inspection and later use.
p.ctrl.relativeOutputMatrix = Cr; % Store C_r for inspection and later use.
p.ctrl.staticDcGain = G0; % Store G_0 for the single-actuator feed-forward calculations.
p.ctrl.staticFeedforwardInverse = G0Inverse; % Store G_0_inverse for the combined feed-forward calculation.
p.ctrl.shiftDcGain = G0(1, 1); % Store G_0_yy [m/N].
p.ctrl.tiltDcGain = G0(2, 2); % Store G_0_theta_theta [rad/(N m)].
end






% staticFeedforwardCommand: basic shit, take the constants from above and have this function 
% to calculate the feedforward command dynamically. Helps us not to have to calculate gain matrix every time we need to calculate the feedforward command.


function feedforward = staticFeedforwardCommand(p, caseDef, target)
feedforward = [0; 0]; % Start with zero force and zero moment.

if ~p.ctrl.useStaticFeedforward
    return;
end

if caseDef.enableShift && caseDef.enableTilt
    targetVector = [target.shift; target.tilt]; % target_vector contains the current shift and tilt targets.
    feedforward = p.ctrl.staticFeedforwardInverse*targetVector; % Calculate the current combined feed-forward force and moment.
elseif caseDef.enableShift
    feedforward(1) = target.shift/p.ctrl.shiftDcGain; % Calculate the current shift-only feed-forward force [N].
elseif caseDef.enableTilt
    feedforward(2) = target.tilt/p.ctrl.tiltDcGain; % Calculate the current tilt-only feed-forward moment [N m].
end
end







% Controller done ----------------------------------------------------
% Feel free to put your own controller and play around 







%% FUNCTION GROUP 6 - CONNECT AND SOLVE THE CLOSED-LOOP SYSTEM
% These functions connect the controller to the vehicle model and integrate the complete state equations.
% Dont bother with these functions, they are just for simulation purposes. 
% Play with the code above the rest will follow.

% INPUTS AND OUTPUTS: simulateRoute
% Input p is the complete system-parameter structure. Input caseDef is the selected operating case.
% Output sim contains time, states, calculated outputs, case data, and the termination result.
function sim = simulateRoute(p, caseDef)
minSpeed = max(min(p.speed.speed_mps), p.speed.minimum_mps);
tEndGuess = 1.8*p.track.totalLength/minSpeed + 20;
x0 = zeros(17, 1);
x0(17) = 0;

opts = odeset('RelTol', p.ode.relTol, ...
    'AbsTol', p.ode.absTol, ...
    'MaxStep', p.ode.maxStep, ...
    'Events', @(t, x) routeStopEvent(t, x, p));

[t, x, te, ~, ie] = ode45(@(t, x) shiftTiltRouteEquations( ...
    t, x, p, caseDef), [0, tEndGuess], x0, opts);

sim.t = t;
sim.x = x;
sim.p = p;
sim.caseDef = caseDef;
sim.outputs = postProcess(t, x, p, caseDef);
sim.terminatedByEvent = ~isempty(te);
sim.terminationTime = NaN;
sim.terminationReason = 'time span ended before route completion';
if ~isempty(te)
    sim.terminationTime = te(1);
    sim.terminationReason = eventReason(ie(1), p);
end
end

% INPUTS AND OUTPUTS: shiftTiltRouteEquations
% The first input is simulation time and is unused because the equations depend on state and route position.
% Input x is the complete seventeen-element state vector. Input p contains all system parameters.
% Input caseDef is the selected operating case.
% Output dx is the complete seventeen-element state-derivative vector returned to ode45.
function dx = shiftTiltRouteEquations(~, x, p, caseDef)
s = min(max(x(17), 0), p.track.totalLength);
[V, ~, aLong] = speedAtDistance(s, p);
track = trackProfileFromDistance(s, p);
track = addTimeDerivatives(track, V, aLong);

xMechanical = x(1:14);
FxActual = x(15);
MActual = x(16);

target = controllerTarget(track, p, V, caseDef);
target = controlTargetWithState(x, p, caseDef, target);
command = controllerCommand(x, p, caseDef, target);

uActual = [FxActual; MActual];
dxMechanical = forceInputSystemEquations(xMechanical, p, V, track, ...
    uActual, caseDef);

dx = zeros(17, 1);
dx(1:14) = dxMechanical;
dx(15) = (command.Fx - FxActual)/p.ctrl.forceTau;
dx(16) = (command.Mtilt - MActual)/p.ctrl.momentTau;
dx(17) = V;
end

% INPUTS AND OUTPUTS: routeStopEvent
% The first input is simulation time and is unused. Input x is the complete current state vector.
% Input p contains route length, gauge, and numerical stop settings.
% Output value contains event margins, isterminal selects stopping events, and direction selects crossing direction.
function [value, isterminal, direction] = routeStopEvent(~, x, p)
s = x(17);
routeRemaining = p.track.totalLength - s;
stateMargin = p.sim.maxStateAbs - max(abs(x(1:14)));

value = [routeRemaining; stateMargin];
isterminal = [1; 1];
direction = [-1; -1];

if p.sim.stopAtGaugeLimit
    [V, ~, aLong] = speedAtDistance(s, p);
    track = trackProfileFromDistance(s, p);
    track = addTimeDerivatives(track, V, aLong);
    offset = resultantOffset(x(1:14), p, V, track, x(15:16));
    cogOut = -x(1:3);
    gaugeMargin = p.track.halfGauge - abs(offset);
    cogMargin = p.track.halfGauge - max(abs(cogOut));
    value = [routeRemaining; gaugeMargin; cogMargin; stateMargin];
    isterminal = [1; 1; 1; 1];
    direction = [-1; -1; -1; -1];
end
end

% INPUTS AND OUTPUTS: eventReason
% Input eventIndex identifies the ode45 event that stopped integration. Input p contains event settings.
% Output reason is the readable text describing the termination event.
function reason = eventReason(eventIndex, p)
if p.sim.stopAtGaugeLimit
    labels = {'route completed', 'rail gauge limit', ...
        'COG gauge limit', 'state runaway'};
else
    labels = {'route completed', 'state runaway'};
end
if eventIndex >= 1 && eventIndex <= numel(labels)
    reason = labels{eventIndex};
else
    reason = 'unknown event';
end
end

%% FUNCTION GROUP 7 - CALCULATE OUTPUTS AND ASSESSMENT VALUES
% These functions convert solved states into physical outputs, loads, and summary metrics.

% INPUTS AND OUTPUTS: postProcess
% Input t contains simulation times and input x contains the solved state history.
% Input p contains all system parameters. Input caseDef is the selected operating case.
% Output out contains the calculated physical histories used for assessment and display.
function out = postProcess(t, x, p, caseDef)
n = numel(t);
out.s = x(:, 17);
out.speedKmh = zeros(n, 1);
out.kappa = zeros(n, 1);
out.radius = zeros(n, 1);
out.cantDeg = zeros(n, 1);
out.lambdaTrackDeg = zeros(n, 1);
out.shiftTarget = zeros(n, 1);
out.tiltTargetDeg = zeros(n, 1);
out.relativeShift = zeros(n, 1);
out.relativeTiltDeg = zeros(n, 1);
out.frameRollDeg = zeros(n, 1);
out.carriageRollDeg = zeros(n, 1);
out.bogieRollDeg = zeros(n, 1);
out.resultantOffset = zeros(n, 1);
out.comInwardShift = zeros(n, 1);
out.lateralForceArm = zeros(n, 1);
out.comfortAccel = zeros(n, 1);
out.Fx = x(:, 15);
out.Mtilt = x(:, 16);
out.yv = x(:, 1);
out.ys = x(:, 2);
out.yb = x(:, 3);
out.thetaV = x(:, 4);
out.thetaS = x(:, 5);
out.thetaB = x(:, 6);
out.frameSpringDeflection = zeros(n, 1);
out.carriageBogieSpringDeflection = zeros(n, 1);

for i = 1:n
    sNow = min(max(x(i, 17), 0), p.track.totalLength);
    [V, speedKmh, aLong] = speedAtDistance(sNow, p);
    track = trackProfileFromDistance(sNow, p);
    track = addTimeDerivatives(track, V, aLong);
    dxMechanical = forceInputSystemEquations(x(i, 1:14).', p, V, ...
        track, x(i, 15:16).', caseDef);

    target = controllerTarget(track, p, V, caseDef);
    target = controlTargetWithState(x(i, :).', p, caseDef, target);
    [offset, lambdaTrack, components] = resultantOffset( ...
        x(i, 1:14).', p, V, track, x(i, 15:16).');

    out.speedKmh(i) = speedKmh;
    out.kappa(i) = track.kappa;
    if abs(track.kappa) > eps
        out.radius(i) = 1/track.kappa;
    else
        out.radius(i) = Inf;
    end
    out.cantDeg(i) = rad2degLocal(track.theta);
    out.lambdaTrackDeg(i) = rad2degLocal(lambdaTrack);
    out.shiftTarget(i) = target.shift;
    out.tiltTargetDeg(i) = rad2degLocal(target.tilt);
    out.relativeShift(i) = x(i, 2) - x(i, 3);
    out.relativeTiltDeg(i) = rad2degLocal(x(i, 4) - x(i, 5));
    out.frameRollDeg(i) = rad2degLocal(track.theta + x(i, 4));
    out.carriageRollDeg(i) = rad2degLocal(track.theta + x(i, 5));
    out.bogieRollDeg(i) = rad2degLocal(track.theta + x(i, 6));
    out.resultantOffset(i) = offset;
    out.comInwardShift(i) = components.comInwardShift;
    out.lateralForceArm(i) = components.lateralForceArm;
    out.comfortAccel(i) = dxMechanical(7) + V^2*track.kappa ...
        - p.g*(track.theta + x(i, 4));
    out.frameSpringDeflection(i) = frameCarriageSpringDeflection( ...
        x(i, 1:14).', p);
    out.carriageBogieSpringDeflection(i) = ...
        carriageBogieSpringDeflection(x(i, 1:14).', p);
end
end

% INPUTS AND OUTPUTS: routeMetrics
% Input sim contains the solved simulation and calculated outputs. Input p contains route and limit data.
% Output metrics contains the summarized maximum values and remaining margins for the complete route.
function metrics = routeMetrics(sim, p)
out = sim.outputs;
gaugeMargin = p.track.halfGauge - abs(out.resultantOffset);
cogOut = -[out.yv, out.ys, out.yb];
cogMargin = p.track.halfGauge - max(abs(cogOut), [], 2);

[metrics.peakOffset, idxPeak] = max(abs(out.resultantOffset));
metrics.minGaugeMargin = min(gaugeMargin);
metrics.minCogMargin = min(cogMargin);
metrics.sAtPeakOffset = out.s(idxPeak);
metrics.speedAtPeakOffsetKmh = out.speedKmh(idxPeak);
metrics.kappaAtPeakOffset = out.kappa(idxPeak);
metrics.cantDegAtPeakOffset = out.cantDeg(idxPeak);
metrics.shiftAtPeakOffset = out.relativeShift(idxPeak);
metrics.shiftTargetAtPeakOffset = out.shiftTarget(idxPeak);
metrics.tiltAtPeakOffsetDeg = out.relativeTiltDeg(idxPeak);
metrics.tiltTargetAtPeakOffsetDeg = out.tiltTargetDeg(idxPeak);
metrics.peakShift = max(abs(out.relativeShift));
metrics.peakTiltDeg = max(abs(out.relativeTiltDeg));
metrics.peakFx = max(abs(out.Fx));
metrics.peakMtilt = max(abs(out.Mtilt));
metrics.peakComfortAccel = max(abs(out.comfortAccel));
metrics.rmsComfortAccel = sqrt(mean(out.comfortAccel.^2));
metrics.routeCompleted = strcmp(sim.terminationReason, 'route completed');
end

% INPUTS AND OUTPUTS: resultantOffset
% Input xMechanical contains the fourteen mechanical states. Input p contains system parameters.
% Input V is speed, input track contains local track conditions, and input uActual contains actual actuator inputs.
% Outputs offset and lambdaTrack are the resultant offset and angle. Output components stores contributing terms.
function [offset, lambdaTrack, components] = resultantOffset( ...
    xMechanical, p, V, track, u)
if nargin < 5
    u = [0; 0];
end

q = xMechanical(1:6);
yCentreOutward = -[q(1); q(2); q(3)];
mass = [p.mv; p.ms; p.mb];
height = [p.hg3; p.hg2; p.hg1];
totalMass = sum(mass);

curveAccel = V^2*track.kappa;
verticalLoadPerMass = p.g*cos(track.theta) ...
    + curveAccel*sin(track.theta);
outwardLoadPerMass = curveAccel*cos(track.theta) ...
    - p.g*sin(track.theta);
lambdaTrack = atan2(outwardLoadPerMass, verticalLoadPerMass);

normalLoad = totalMass*verticalLoadPerMass;
externalMoment = sum(mass.*(verticalLoadPerMass*yCentreOutward ...
    + outwardLoadPerMass*height));

offsetExternal = externalMoment/normalLoad;
offsetDirectActuator = directActuatorRailMoment(u);
offset = offsetExternal + offsetDirectActuator/normalLoad;

comOutwardShift = sum(mass.*yCentreOutward)/totalMass;
comInwardShift = -comOutwardShift;
effectiveHeight = sum(mass.*height)/totalMass;
lateralForceArm = effectiveHeight*tan(lambdaTrack);

components.externalOffset = offsetExternal;
components.actuatorOffset = offsetDirectActuator/normalLoad;
components.comInwardShift = comInwardShift;
components.lateralForceArm = lateralForceArm;
components.effectiveHeight = effectiveHeight;
components.normalLoad = normalLoad;
end

% INPUTS AND OUTPUTS: directActuatorRailMoment
% The actuator-input argument is unused in this simplified rail-moment calculation.
% Output moment is the direct actuator moment transferred to the rail and is zero in this model [N m].
function moment = directActuatorRailMoment(~)
% Internal shift/tilt actuators are not added directly to the whole-vehicle
% rail resultant; their effect enters through the simulated body states.
moment = 0;
end

% INPUTS AND OUTPUTS: frameCarriageSpringDeflection
% Input xMechanical contains the fourteen mechanical states. Input p contains suspension geometry.
% Output delta is the frame-to-carriage lateral spring deflection [m].
function delta = frameCarriageSpringDeflection(xMechanical, p)
delta = xMechanical(1) ...
    - xMechanical(2) ...
    - (p.h1 + p.h2)*xMechanical(4) ...
    + p.h2*xMechanical(5);
end

% INPUTS AND OUTPUTS: carriageBogieSpringDeflection
% Input xMechanical contains the fourteen mechanical states. Input p contains suspension geometry.
% Output delta is the carriage-to-bogie lateral spring deflection [m].
function delta = carriageBogieSpringDeflection(xMechanical, p)
delta = xMechanical(2) ...
    - xMechanical(3) ...
    - (p.h2 + p.h3)*xMechanical(5) ...
    - p.h4*xMechanical(6);
end

%% FUNCTION GROUP 8 - PRINT RESULTS AND DRAW ENGINEERING PLOTS
% These functions report the simulation and produce the non-animated engineering figures.

% INPUTS AND OUTPUTS: printSimulationDescription
% Input p contains route and system settings. Input caseDef contains the selected case description.
% This function prints text to the Command Window and does not return an output.
function printSimulationDescription(p, caseDef)
fprintf('=== Shift + tilt train on opposite double-clothoid route ===\n');
fprintf('Case: %s\n', caseDef.name);
fprintf('Total route length = %.1f m\n', p.track.totalLength);
fprintf('Track gauge = %.3f m, gauge/2 = %.4f m\n', ...
    p.track.gauge, p.track.halfGauge);
fprintf('\nRoute sections:\n');
fprintf('  tangent in        %.1f m\n', p.track.tangentIn);
fprintf('  curve 1: R = %+8.1f m, cant = %+6.1f mm, clothoids %.1f/%.1f m, circular %.1f m\n', ...
    p.track.radius1, 1000*p.track.cant1, p.track.transitionIn1, ...
    p.track.transitionOut1, p.track.circular1);
fprintf('  middle tangent    %.1f m\n', p.track.middleTangent);
fprintf('  curve 2: R = %+8.1f m, cant = %+6.1f mm, clothoids %.1f/%.1f m, circular %.1f m\n', ...
    p.track.radius2, 1000*p.track.cant2, p.track.transitionIn2, ...
    p.track.transitionOut2, p.track.circular2);
fprintf('  tangent out       %.1f m\n', p.track.tangentOut);
fprintf('\nConstant velocity = %.1f km/h for the complete route\n', ...
    p.speed.constantKmh);
fprintf('\nController limits: shift %.3f m, tilt %.2f deg, Fx %.1f kN, M %.1f kN m\n', ...
    p.ctrl.shiftTravelLimit, p.ctrl.tiltLimitDeg, ...
    p.ctrl.forceLimit/1000, p.ctrl.momentLimit/1000);
if p.sim.stopAtGaugeLimit
    fprintf('Gauge-limit event stopping is ON.\n');
else
    fprintf('Gauge-limit event stopping is OFF, so the visual run completes the route.\n');
end
end

% INPUTS AND OUTPUTS: printSimulationSummary
% Input sim contains the simulation result, metrics contains summarized results, and p contains reference limits.
% This function prints text to the Command Window and does not return an output.
function printSimulationSummary(sim, metrics, p)
fprintf('\n=== Simulation summary ===\n');
fprintf('Stop reason = %s, final time = %.2f s, final distance = %.1f m\n', ...
    sim.terminationReason, sim.t(end), sim.outputs.s(end));
fprintf('Peak rail-resultant offset = %.4f m at s = %.1f m, V = %.1f km/h\n', ...
    metrics.peakOffset, metrics.sAtPeakOffset, metrics.speedAtPeakOffsetKmh);
fprintf('Minimum rail gauge margin = %.4f m, minimum COG margin = %.4f m\n', ...
    metrics.minGaugeMargin, metrics.minCogMargin);
fprintf('At peak offset: kappa = %+9.6f 1/m, cant = %+6.3f deg\n', ...
    metrics.kappaAtPeakOffset, metrics.cantDegAtPeakOffset);
fprintf('Shift actual/target at peak offset = %+7.4f / %+7.4f m\n', ...
    metrics.shiftAtPeakOffset, metrics.shiftTargetAtPeakOffset);
fprintf('Tilt actual/target at peak offset = %+6.3f / %+6.3f deg\n', ...
    metrics.tiltAtPeakOffsetDeg, metrics.tiltTargetAtPeakOffsetDeg);
fprintf('Peak shift = %.4f m, peak tilt = %.3f deg\n', ...
    metrics.peakShift, metrics.peakTiltDeg);
fprintf('Peak actuator force = %.1f kN, peak tilt moment = %.1f kN m\n', ...
    metrics.peakFx/1000, metrics.peakMtilt/1000);
fprintf('Peak/RMS comfort acceleration diagnostic = %.4f / %.4f m/s^2\n', ...
    metrics.peakComfortAccel, metrics.rmsComfortAccel);
end

% INPUTS AND OUTPUTS: plotRouteProfiles
% Input p contains route geometry and plotting settings.
% This function creates the route-profile figure and does not return an output.
function plotRouteProfiles(p)
s = linspace(0, p.track.totalLength, p.visual.nRoute);
route = sampleRoute(p, s);
speedKmh = zeros(size(s));
for i = 1:numel(s)
    [~, speedKmh(i)] = speedAtDistance(s(i), p);
end

figure('Name', 'Double clothoid route and speed profile', 'Color', 'w');

subplot(4, 1, 1);
plot(s, route.kappa, 'LineWidth', 1.3);
routeXLines(p);
grid on;
ylabel('\kappa (1/m)');
title('Signed curvature: first and second curves have opposite radius signs');

subplot(4, 1, 2);
plot(s, rad2degLocal(route.thetaCant), 'LineWidth', 1.3);
routeXLines(p);
grid on;
ylabel('\theta_0 (deg)');
title('Signed cant angle');

subplot(4, 1, 3);
plot(s, speedKmh, 'LineWidth', 1.3);
routeXLines(p);
grid on;
ylabel('Speed (km/h)');
title('Constant speed used for the complete route');

subplot(4, 1, 4);
plot(route.x, route.y, 'k', 'LineWidth', 1.3);
axis equal;
grid on;
xlabel('Global X (m)');
ylabel('Global Y (m)');
title('Plan-view centreline from integrated curvature');
finishFigure();
end

% INPUTS AND OUTPUTS: routeXLines
% Input p contains the route boundary stations.
% This function draws route-boundary lines on the current axes and does not return an output.
function routeXLines(p)
hold on;
yl = ylim;
for i = 2:(numel(p.track.breakS)-1)
    x = p.track.breakS(i);
    plot([x x], yl, ':', 'Color', [0.25 0.25 0.25], ...
        'HandleVisibility', 'off');
end
ylim(yl);
end

% INPUTS AND OUTPUTS: setupPlotStyle
% This function has no inputs and no returned output.
% It applies the common default plotting style used by this script.
function setupPlotStyle()
set(groot, 'DefaultFigureColor', 'w');
set(groot, 'DefaultAxesColor', 'w');
set(groot, 'DefaultAxesXColor', 'k');
set(groot, 'DefaultAxesYColor', 'k');
set(groot, 'DefaultAxesZColor', 'k');
set(groot, 'DefaultTextColor', 'k');
set(groot, 'DefaultLineLineWidth', 1.2);
end

% INPUTS AND OUTPUTS: finishFigure
% This function has no inputs and no returned output.
% It applies final colours and formatting to the current figure.
function finishFigure()
fig = gcf;
set(fig, 'Color', 'w');
axesList = findobj(fig, 'Type', 'axes');
for iAxes = 1:numel(axesList)
    set(axesList(iAxes), ...
        'Color', 'w', ...
        'XColor', 'k', ...
        'YColor', 'k', ...
        'ZColor', 'k', ...
        'GridColor', [0.15 0.15 0.15]);
end
drawnow;
end

%% FUNCTION GROUP 9 - GENERAL NUMERICAL HELPERS
% These short functions perform limiting and unit conversions used by several blocks.

% INPUTS AND OUTPUTS: clampAbs
% Input value is the number to limit. Input limitValue is its permitted absolute magnitude.
% Output value is restricted to the interval from negative limitValue to positive limitValue.
function value = clampAbs(value, limitValue)
if limitValue < 0
    return;
end
value = min(max(value, -limitValue), limitValue);
end

% INPUTS AND OUTPUTS: kmhToMs
% Input speedKmh is speed in kilometres per hour.
% Output V is the same speed in metres per second.
function V = kmhToMs(speedKmh)
V = speedKmh/3.6;
end

% INPUTS AND OUTPUTS: msToKmh
% Input V is speed in metres per second.
% Output speedKmh is the same speed in kilometres per hour.
function speedKmh = msToKmh(V)
speedKmh = 3.6*V;
end

% INPUTS AND OUTPUTS: rad2degLocal
% Input rad is an angle in radians.
% Output deg is the same angle in degrees.
function deg = rad2degLocal(rad)
deg = 180*rad/pi;
end

% INPUTS AND OUTPUTS: deg2radLocal
% Input deg is an angle in degrees.
% Output rad is the same angle in radians.
function rad = deg2radLocal(deg)
rad = pi*deg/180;
end

%% FINAL FUNCTION GROUP - LIVE ANIMATION DISPLAY
% Everything below this heading is used only to display the live front or 3-D animation.
% No function in this section creates or saves a video file.

% INPUTS AND OUTPUTS: makeMovingAnimation
% Input sim contains the solved state history, p contains animation settings, and caseDef identifies the case.
% This function displays the selected live animation and does not return an output.
function makeMovingAnimation(sim, p, caseDef)
if strcmpi(p.video.viewMode, 'front')
    animateFrontView(sim, p, caseDef);
    return;
end

try
    animate3D(sim, p, caseDef);
catch ME
    warning('3-D animation failed: %s', ME.message);
    warning('Falling back to the front-view lateral dynamics animation.');
    animateFrontView(sim, p, caseDef);
end
end

% INPUTS AND OUTPUTS: animationTimes
% Input sim contains simulation time and input p contains the requested animation frame rate.
% Output frameTimes contains the simulation times displayed as animation frames [s].
function frameTimes = animationTimes(sim, p)
nFrames = max(2, round(p.video.duration*p.video.frameRate));
frameTimes = linspace(sim.t(1), sim.t(end), nFrames);
end

% INPUTS AND OUTPUTS: frameTitle
% Input xFrame is one complete state vector, p contains system data, and caseDef identifies the case.
% Output titleText is the live animation heading for that frame.
function titleText = frameTitle(xFrame, p, caseDef)
sNow = min(max(xFrame(17), 0), p.track.totalLength);
[V, speedKmh, aLong] = speedAtDistance(sNow, p);
track = trackProfileFromDistance(sNow, p);
track = addTimeDerivatives(track, V, aLong);
target = controllerTarget(track, p, V, caseDef);
target = controlTargetWithState(xFrame, p, caseDef, target);
shiftActual = xFrame(2) - xFrame(3);
tiltActualDeg = rad2degLocal(xFrame(4) - xFrame(5));
titleText = sprintf(['s = %.1f m / %.1f m, V = %.1f km/h, ' ...
    'kappa = %+7.5f 1/m, cant = %+5.2f deg\n' ...
    'shift %.3f/%.3f m, tilt %.2f/%.2f deg'], ...
    sNow, p.track.totalLength, speedKmh, track.kappa, ...
    rad2degLocal(track.theta), shiftActual, target.shift, ...
    tiltActualDeg, rad2degLocal(target.tilt));
end

% INPUTS AND OUTPUTS: animateFrontView
% Input sim contains solved states, p contains display settings, and caseDef identifies the case.
% This function displays the replayable front-view animation and does not return an output.
function animateFrontView(sim, p, caseDef)
frameTimes = animationTimes(sim, p);
fig = figure('Name', 'Front-view lateral shift and tilt dynamics', ...
    'Color', 'w');
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

ax = nexttile(layout, 1);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlim(ax, [-2.45 2.45]);
ylim(ax, [-0.20 3.70]);
xlabel(ax, 'Lateral position, positive to local left (m)');
ylabel(ax, 'Height above rail plane (m)');

axProfile = nexttile(layout, 2);
setupFrontProfileAxis(axProfile, sim, p);

handles = gobjects(0);
markerHandles = gobjects(0);
replayAnimation = true;

while replayAnimation
    for iFrame = 1:numel(frameTimes)
        if ~isgraphics(fig)
            return;
        end
        if ~isempty(handles)
            delete(handles(ishandle(handles)));
        end
        if ~isempty(markerHandles)
            delete(markerHandles(ishandle(markerHandles)));
        end
        xFrame = interp1(sim.t, sim.x, frameTimes(iFrame), 'linear').';
        handles = drawFrontTrain(ax, xFrame, p);
        markerHandles = drawFrontProfileMarker(axProfile, xFrame(17));
        title(ax, frameTitle(xFrame, p, caseDef));
        drawnow;
        pause(p.video.pauseBetweenFrames);
    end
    replayChoice = questdlg( ...
        'Play the live simulation again?', ...
        'Replay simulation', 'Replay', 'Finish', 'Replay');
    replayAnimation = strcmp(replayChoice, 'Replay');
end
end

% INPUTS AND OUTPUTS: setupFrontProfileAxis
% Input ax is the MATLAB axes handle. Input sim contains route histories and p contains route boundaries.
% This function prepares the front-animation profile axes and does not return an output.
function setupFrontProfileAxis(ax, sim, p)
out = sim.outputs;
profile.s = out.s;
profile.kappa = normalizeByPeak(out.kappa);
profile.cant = normalizeByPeak(out.cantDeg);
profile.shift = normalizeByLimit(out.relativeShift, p.ctrl.shiftTravelLimit);
profile.tilt = normalizeByLimit(out.relativeTiltDeg, p.ctrl.tiltLimitDeg);
setappdata(ax, 'FrontProfile', profile);

hold(ax, 'on');
plot(ax, profile.s, profile.kappa, 'LineWidth', 1.4);
plot(ax, profile.s, profile.cant, 'LineWidth', 1.4);
plot(ax, profile.s, profile.shift, 'LineWidth', 1.4);
plot(ax, profile.s, profile.tilt, 'LineWidth', 1.4);
yline(ax, 0, 'k:');
ylim(ax, [-1.15 1.15]);
xlim(ax, [0 p.track.totalLength]);
routeXLinesOnAxis(ax, p);
grid(ax, 'on');
xlabel(ax, 'Distance along route, s (m)');
ylabel(ax, 'Normalised value');
title(ax, 'Route and body response while the front view moves');
legend(ax, {'curvature', 'cant', 'actual shift', 'actual tilt'}, ...
    'Location', 'southoutside');
end

% INPUTS AND OUTPUTS: drawFrontProfileMarker
% Input ax is the MATLAB axes handle and input sNow is the current route station [m].
% Output markerHandles contains handles for the current-position markers.
function markerHandles = drawFrontProfileMarker(ax, sNow)
profile = getappdata(ax, 'FrontProfile');
markerHandles = gobjects(0);
yl = ylim(ax);
markerHandles(end+1) = plot(ax, [sNow sNow], yl, 'k-', ...
    'LineWidth', 1.0, 'HandleVisibility', 'off');

values = [ ...
    interp1(profile.s, profile.kappa, sNow, 'linear', 'extrap'), ...
    interp1(profile.s, profile.cant, sNow, 'linear', 'extrap'), ...
    interp1(profile.s, profile.shift, sNow, 'linear', 'extrap'), ...
    interp1(profile.s, profile.tilt, sNow, 'linear', 'extrap')];
colors = get(ax, 'ColorOrder');
for iValue = 1:numel(values)
    color = colors(1 + mod(iValue - 1, size(colors, 1)), :);
    markerHandles(end+1) = plot(ax, sNow, values(iValue), 'o', ...
        'Color', color, 'MarkerFaceColor', color, ...
        'MarkerSize', 6, 'HandleVisibility', 'off');
end
end

% INPUTS AND OUTPUTS: drawFrontTrain
% Input ax is the MATLAB axes handle, xFrame is one state vector, and p contains vehicle geometry.
% Output handles contains the graphics objects drawn for the front-view train.
function handles = drawFrontTrain(ax, xFrame, p)
sNow = min(max(xFrame(17), 0), p.track.totalLength);
[V, ~, aLong] = speedAtDistance(sNow, p);
track = trackProfileFromDistance(sNow, p);
track = addTimeDerivatives(track, V, aLong);
[offset, ~, components] = resultantOffset(xFrame(1:14), p, V, track, ...
    xFrame(15:16));

q = xFrame(1:6);
handles = gobjects(0);

railHalf = p.track.halfGauge;
[y1, z1] = stationCrossSectionPoint(track.theta, -railHalf, 0);
[y2, z2] = stationCrossSectionPoint(track.theta, railHalf, 0);
handles(end+1) = plot(ax, [y1 y2], [z1 z2], 'k-', 'LineWidth', 2.0);
for rail = [-railHalf railHalf]
    [yr, zr] = stationCrossSectionPoint(track.theta, rail, 0.05);
    handles(end+1) = plot(ax, yr, zr, 'ks', 'MarkerFaceColor', 'k', ...
        'MarkerSize', 7);
end

[yb, zb] = stationCrossSectionPoint(track.theta, q(3), ...
    p.hg1 + p.visual.bogieLift);
[ys, zs] = stationCrossSectionPoint(track.theta, q(2), p.hg2);
[yv, zv] = stationCrossSectionPoint(track.theta, q(1), p.hg3);

handles(end+1) = drawFrontBox(ax, yb, zb, track.theta + q(6), ...
    p.visual.bogieWidth, p.visual.bogieHeight, p.visual.bogieColor, 0.88);
handles(end+1) = drawFrontBox(ax, ys, zs, track.theta + q(5), ...
    p.visual.carriageWidth, p.visual.carriageHeight, ...
    p.visual.carriageColor, 0.82);
handles(end+1) = drawFrontBox(ax, yv, zv, track.theta + q(4), ...
    p.visual.bodyWidth, p.visual.bodyHeight, p.visual.bodyColor, 0.62);

[yShiftStart, zShiftStart] = stationCrossSectionPoint(track.theta, q(3), ...
    p.hg2 - 0.30);
[yShiftEnd, zShiftEnd] = stationCrossSectionPoint(track.theta, q(2), ...
    p.hg2 - 0.30);
handles(end+1) = quiver(ax, yShiftStart, zShiftStart, ...
    yShiftEnd - yShiftStart, zShiftEnd - zShiftStart, 0, ...
    'Color', p.visual.carriageColor, 'LineWidth', 2.0, ...
    'MaxHeadSize', 0.85);

[yCom, zCom] = stationCrossSectionPoint(track.theta, ...
    components.comInwardShift, p.ctrl.effectiveHeight);
[yFoot, zFoot] = stationCrossSectionPoint(track.theta, -offset, 0.08);
handles(end+1) = plot(ax, [yCom yFoot], [zCom zFoot], ...
    'Color', p.visual.resultantColor, 'LineWidth', 2.0);
handles(end+1) = plot(ax, yCom, zCom, 'o', ...
    'Color', p.visual.resultantColor, ...
    'MarkerFaceColor', p.visual.resultantColor, 'MarkerSize', 6);
end

% INPUTS AND OUTPUTS: drawFrontBox
% Input ax is the axes handle. Inputs y0, z0, roll, width, and height define the box position and shape.
% Inputs color and alphaValue define its appearance. Output h is the created graphics handle.
function h = drawFrontBox(ax, y0, z0, roll, width, height, color, alphaValue)
latAxis = [cos(roll), -sin(roll)];
vertAxis = [sin(roll), cos(roll)];
corners = [-1 -1; 1 -1; 1 1; -1 1];
verts = zeros(4, 2);
for i = 1:4
    verts(i, :) = [y0 z0] ...
        + 0.5*width*corners(i, 1)*latAxis ...
        + 0.5*height*corners(i, 2)*vertAxis;
end
h = patch(ax, verts(:, 1), verts(:, 2), color, ...
    'FaceAlpha', alphaValue, 'EdgeColor', 0.35*color, ...
    'LineWidth', 0.9);
end

% INPUTS AND OUTPUTS: stationCrossSectionPoint
% Input theta is rail-plane cant. Inputs yLocal and zLocal are local cross-section coordinates.
% Outputs Y and Z are the rotated cross-section coordinates.
function [Y, Z] = stationCrossSectionPoint(theta, yLocal, zLocal)
Y = yLocal.*cos(theta) + zLocal.*sin(theta);
Z = -yLocal.*sin(theta) + zLocal.*cos(theta);
end

% INPUTS AND OUTPUTS: routeXLinesOnAxis
% Input ax is the MATLAB axes handle and input p contains route boundaries.
% This function draws route-boundary lines on the supplied axes and does not return an output.
function routeXLinesOnAxis(ax, p)
yl = ylim(ax);
for i = 2:(numel(p.track.breakS)-1)
    x = p.track.breakS(i);
    plot(ax, [x x], yl, ':', 'Color', [0.25 0.25 0.25], ...
        'HandleVisibility', 'off');
end
ylim(ax, yl);
end

% INPUTS AND OUTPUTS: normalizeByPeak
% Input x is a numerical history.
% Output y is x divided by its largest absolute value, or zero when x has no nonzero peak.
function y = normalizeByPeak(x)
scale = max(abs(x));
if scale < eps
    y = zeros(size(x));
else
    y = x/scale;
end
end

% INPUTS AND OUTPUTS: normalizeByLimit
% Input x is a numerical history and input limitValue is the reference absolute limit.
% Output y is x divided by the absolute limit, or zero when the limit is zero.
function y = normalizeByLimit(x, limitValue)
scale = abs(limitValue);
if scale < eps
    y = zeros(size(x));
else
    y = x/scale;
end
end

% INPUTS AND OUTPUTS: animate3D
% Input sim contains solved states, p contains geometry and display settings, and caseDef identifies the case.
% This function displays the replayable three-dimensional animation and does not return an output.
function animate3D(sim, p, caseDef)
sRoute = linspace(0, p.track.totalLength, p.visual.nRoute);
route = sampleRoute(p, sRoute);
frameTimes = animationTimes(sim, p);

fig = figure('Name', 'Moving 3-D shift and tilt train on double clothoid route', ...
    'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
drawTrack3D(ax, route, p);
grid(ax, 'on');
xlabel(ax, 'Global X (m)');
ylabel(ax, 'Global Y (m)');
zlabel(ax, 'Height (m)');
view(ax, 38, 24);
zlim(ax, [-0.35 4.10]);
camlight(ax, 'headlight');
lighting(ax, 'gouraud');

if strcmpi(p.video.cameraMode, 'wholeRoute')
    axis(ax, 'tight');
    routePad = 35;
    xlim(ax, [min(route.x)-routePad, max(route.x)+routePad]);
    ylim(ax, [min(route.y)-routePad, max(route.y)+routePad]);
end

trainHandles = gobjects(0);
replayAnimation = true;

while replayAnimation
    for iFrame = 1:numel(frameTimes)
        if ~isgraphics(fig)
            return;
        end
        if ~isempty(trainHandles)
            delete(trainHandles(ishandle(trainHandles)));
        end

        xFrame = interp1(sim.t, sim.x, frameTimes(iFrame), 'linear').';
        trainHandles = drawTrain3D(ax, xFrame, p, caseDef, route);
        set3DCameraWindow(ax, xFrame(17), route, p);
        title(ax, frameTitle(xFrame, p, caseDef));
        drawnow;
        pause(p.video.pauseBetweenFrames);
    end
    replayChoice = questdlg( ...
        'Play the live simulation again?', ...
        'Replay simulation', 'Replay', 'Finish', 'Replay');
    replayAnimation = strcmp(replayChoice, 'Replay');
end
end

% INPUTS AND OUTPUTS: drawTrack3D
% Input ax is the axes handle, route contains sampled track coordinates, and p contains track geometry.
% This function draws the three-dimensional rails and does not return an output.
function drawTrack3D(ax, route, p)
[xA, yA, zA] = trackPointGlobal(route.x, route.y, route.psi, ...
    route.thetaCant, p.visual.trackHalfWidth*ones(size(route.s)), ...
    -0.025*ones(size(route.s)));
[xB, yB, zB] = trackPointGlobal(route.x, route.y, route.psi, ...
    route.thetaCant, -p.visual.trackHalfWidth*ones(size(route.s)), ...
    -0.025*ones(size(route.s)));
surf(ax, [xA; xB], [yA; yB], [zA; zB], zeros(2, numel(route.s)), ...
    'FaceColor', p.visual.trackColor, 'FaceAlpha', 0.20, ...
    'EdgeColor', 'none', 'HandleVisibility', 'off');

[xInner, yInner, zInner] = trackPointGlobal(route.x, route.y, route.psi, ...
    route.thetaCant, p.track.halfGauge*ones(size(route.s)), ...
    0.05*ones(size(route.s)));
[xOuter, yOuter, zOuter] = trackPointGlobal(route.x, route.y, route.psi, ...
    route.thetaCant, -p.track.halfGauge*ones(size(route.s)), ...
    0.05*ones(size(route.s)));
plot3(ax, xInner, yInner, zInner, 'Color', p.visual.railColor, ...
    'LineWidth', 2.0, 'DisplayName', 'rails');
plot3(ax, xOuter, yOuter, zOuter, 'Color', p.visual.railColor, ...
    'LineWidth', 2.0, 'HandleVisibility', 'off');
plot3(ax, route.x, route.y, zeros(size(route.x)), ':', ...
    'Color', [0.25 0.25 0.25], 'HandleVisibility', 'off');
end

% INPUTS AND OUTPUTS: drawTrain3D
% Input ax is the axes handle, xFrame is one state vector, p contains geometry, and caseDef identifies the case.
% Input route contains sampled track coordinates. Output handles contains the train graphics objects.
function handles = drawTrain3D(ax, xFrame, p, caseDef, route)
sNow = min(max(xFrame(17), 0), p.track.totalLength);
[V, ~, aLong] = speedAtDistance(sNow, p);
track = trackProfileFromDistance(sNow, p);
track = addTimeDerivatives(track, V, aLong);
target = controllerTarget(track, p, V, caseDef);
target = controlTargetWithState(xFrame, p, caseDef, target);
[offset, ~, components] = resultantOffset(xFrame(1:14), p, V, track, ...
    xFrame(15:16));

[x0, y0, psi0, theta0] = routeFrameAtS(route, sNow);
q = xFrame(1:6);
rollB = theta0 + q(6);
rollS = theta0 + q(5);
rollV = theta0 + q(4);

[xb, yb, zb] = trackPointGlobal(x0, y0, psi0, theta0, q(3), ...
    p.hg1 + p.visual.bogieLift);
[xs, ys, zs] = trackPointGlobal(x0, y0, psi0, theta0, q(2), p.hg2);
[xv, yv, zv] = trackPointGlobal(x0, y0, psi0, theta0, q(1), p.hg3);

handles = gobjects(0);
handles(end+1) = drawCurvedBox(ax, xb, yb, zb, psi0, rollB, ...
    p.visual.bogieLength, p.visual.bogieWidth, p.visual.bogieHeight, ...
    p.visual.bogieColor, 0.88);
handles(end+1) = drawCurvedBox(ax, xs, ys, zs, psi0, rollS, ...
    p.visual.carriageLength, p.visual.carriageWidth, ...
    p.visual.carriageHeight, p.visual.carriageColor, 0.82);
handles(end+1) = drawCurvedBox(ax, xv, yv, zv, psi0, rollV, ...
    p.visual.carLength, p.visual.bodyWidth, p.visual.bodyHeight, ...
    p.visual.bodyColor, 0.62);

[xCom, yCom, zCom] = trackPointGlobal(x0, y0, psi0, theta0, ...
    components.comInwardShift, p.ctrl.effectiveHeight);
[xFoot, yFoot, zFoot] = trackPointGlobal(x0, y0, psi0, theta0, ...
    -offset, 0.08);
handles(end+1) = plot3(ax, [xCom xFoot], [yCom yFoot], [zCom zFoot], ...
    'Color', p.visual.resultantColor, 'LineWidth', 2.0, ...
    'HandleVisibility', 'off');
handles(end+1) = scatter3(ax, xCom, yCom, zCom, 42, ...
    p.visual.resultantColor, 'filled', 'HandleVisibility', 'off');

shiftY = target.shift;
[xTarget, yTarget, zTarget] = trackPointGlobal(x0, y0, psi0, theta0, ...
    shiftY, p.hg2 - 0.24);
[xBase, yBase, zBase] = trackPointGlobal(x0, y0, psi0, theta0, ...
    0, p.hg2 - 0.24);
handles(end+1) = plot3(ax, [xBase xTarget], [yBase yTarget], ...
    [zBase zTarget], '-', 'Color', p.visual.carriageColor, ...
    'LineWidth', 2.1, 'HandleVisibility', 'off');
end

% INPUTS AND OUTPUTS: set3DCameraWindow
% Input ax is the axes handle, sNow is current station, route contains coordinates, and p contains view settings.
% This function positions the three-dimensional camera and does not return an output.
function set3DCameraWindow(ax, sNow, route, p)
if strcmpi(p.video.cameraMode, 'wholeRoute')
    return;
end
[x0, y0] = routeFrameAtS(route, sNow);
w = p.video.followWindow;
xlim(ax, [x0-w, x0+w]);
ylim(ax, [y0-w, y0+w]);
zlim(ax, [-0.35 4.10]);
daspect(ax, [1 1 0.07]);
end

% INPUTS AND OUTPUTS: drawCurvedBox
% Input ax is the axes handle. Inputs x0, y0, z0, psi, and roll define position and orientation.
% Inputs boxLength, boxWidth, boxHeight, color, and alphaValue define the box shape and appearance.
% Output h is the created three-dimensional graphics handle.
function h = drawCurvedBox(ax, x0, y0, z0, psi, roll, boxLength, ...
    boxWidth, boxHeight, color, alphaValue)
tangentAxis = [cos(psi) sin(psi) 0];
normalAxis = [-sin(psi) cos(psi) 0];
latAxis = [normalAxis(1)*cos(roll), normalAxis(2)*cos(roll), -sin(roll)];
vertAxis = [normalAxis(1)*sin(roll), normalAxis(2)*sin(roll), cos(roll)];
center = [x0 y0 z0];

signs = [-1 -1 -1;
          1 -1 -1;
         -1  1 -1;
          1  1 -1;
         -1 -1  1;
          1 -1  1;
         -1  1  1;
          1  1  1];
verts = zeros(8, 3);
for i = 1:8
    verts(i, :) = center ...
        + signs(i, 1)*0.5*boxLength*tangentAxis ...
        + signs(i, 2)*0.5*boxWidth*latAxis ...
        + signs(i, 3)*0.5*boxHeight*vertAxis;
end

faces = [1 2 4 3;
         5 7 8 6;
         1 5 6 2;
         3 4 8 7;
         1 3 7 5;
         2 6 8 4];

h = patch(ax, 'Vertices', verts, 'Faces', faces, ...
    'FaceColor', color, 'FaceAlpha', alphaValue, ...
    'EdgeColor', 0.35*color, 'LineWidth', 0.8, ...
    'HandleVisibility', 'off');
end

% INPUTS AND OUTPUTS: trackPointGlobal
% Inputs x, y, psi, and theta define the route frame. Inputs yLocal and zLocal define a local point.
% Outputs X, Y, and Z are the point coordinates in the global three-dimensional frame.
function [X, Y, Z] = trackPointGlobal(x, y, psi, theta, yLocal, zLocal)
normalX = -sin(psi);
normalY = cos(psi);
normalOffset = yLocal.*cos(theta) + zLocal.*sin(theta);
X = x + normalX.*normalOffset;
Y = y + normalY.*normalOffset;
Z = -yLocal.*sin(theta) + zLocal.*cos(theta);
end

% INPUTS AND OUTPUTS: routeFrameAtS
% Input route contains sampled route data and input sNow is the requested route station [m].
% Outputs x0, y0, psi0, and theta0 are the interpolated route position, heading, and cant angle.
function [x0, y0, psi0, theta0] = routeFrameAtS(route, sNow)
x0 = interp1(route.s, route.x, sNow, 'linear', 'extrap');
y0 = interp1(route.s, route.y, sNow, 'linear', 'extrap');
psi0 = interp1(route.s, route.psi, sNow, 'linear', 'extrap');
theta0 = interp1(route.s, route.thetaCant, sNow, 'linear', 'extrap');
end

