function [scenario, egoVehicle] = DrivingScenarioDesigner()
% Returns the drivingScenario defined in the Designer

%get scenario ans
% Command summary goes here
% prepare_simulator();
% run_Sim();
% model_name = 'MOBATSim';
% t = get_param(model_name,'StopTime');
% Construct a drivingScenario object.
scenario = drivingScenario;
step_length = 10;

% Add all road segments
roadCenters = [0 0 0;
    -50 0 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '40');

roadCenters = [-90 40 0;
    -90 140 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '41');

roadCenters = [-78 168 0;
    -8 239 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '42');

roadCenters = [-90 140 0;
    -90 220 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '43');

roadCenters = [-150 280 0;
    -250 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '44');

roadCenters = [-300 230 0;
    -300 160 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '45');

roadCenters = [-250 110 0;
    -170 110 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', '46');

roadCenters = [-250 280 0;
    -390 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road7');

roadCenters = [-480 370 0;
    -480 190 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road8');

roadCenters = [-480 190 0;
    -480 40 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road9');

roadCenters = [-480 40 0;
    -480 -40 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road10');

roadCenters = [-480 -40 0;
    -480 -120 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road11');

roadCenters = [-410 -190 0;
    -310 -190 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road12');

roadCenters = [-250 -250 0;
    -250 -320 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road13');

roadCenters = [-200 -370 0;
    -130 -370 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road14');

roadCenters = [-90 -330 0;
    -90 -170 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road15');

roadCenters = [-90 -170 0;
    -90 40 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road16');

roadCenters = [-250 -320 0;
    -250 -420 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road17');

roadCenters = [-300 -470 0;
    -350 -470 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road18');

roadCenters = [-300 160 0;
    -300 80 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road19');

roadCenters = [-320 60 0;
    -460 60 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road20');

roadCenters = [-170 110 0;
    -120 110 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road21');

roadCenters = [-140 80 0;
    -140 -20 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road22');

roadCenters = [-170 -50 0;
    -210 -50 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road23');

roadCenters = [-140 -20 0;
    -140 -180 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road24');

roadCenters = [-170 -210 0;
    -210 -210 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road25');

roadCenters = [-130 -370 0;
    50 -370 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road26');

roadCenters = [100 -220 0;
    100 -120 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road27');

roadCenters = [100 -320 0;
    100 -220 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road28');

roadCenters = [300 80 0;
    300 200 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road29');

roadCenters = [100 140 0;
    100 240 0];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road30');

roadCenters = [100 -120 0;
    100 140 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road31');

roadCenters = [240 -370 0;
    370 -370 0];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road32');

roadCenters = [140 -370 0;
    240 -370 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road33');

roadCenters = [50 -370 0;
    140 -370 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road34');

roadCenters = [370 -170 0;
    230 -170 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road35');

roadCenters = [300 -100 0;
    300 80 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road36');

roadCenters = [450 200 0;
    450 -90 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road37');

roadCenters = [120 50 0;
    120 -20 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road38');

roadCenters = [120 340 0;
    120 240 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road39');

roadCenters = [120 530 0;
    120 340 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road40');

roadCenters = [100 240 0;
    100 340 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road41');

roadCenters = [-250 300 0;
    60 300 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road42');

roadCenters = [60 300 0;
    160 300 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road43');

roadCenters = [160 280 0;
    60 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road44');

roadCenters = [260 300 0;
    350 300 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road45');

roadCenters = [220 280 0;
    160 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road46');

roadCenters = [160 300 0;
    260 300 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road47');

roadCenters = [-130 280 0;
    -150 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road48');

roadCenters = [-30 280 0;
    -130 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road49');

roadCenters = [60 280 0;
    -30 280 0];
laneSpecification = lanespec(1);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road50');

roadCenters = [100 440 0;
    100 530 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road51');

roadCenters = [100 340 0;
    100 440 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road52');

roadCenters = [120 140 0;
    120 50 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road53');

roadCenters = [120 240 0;
    120 140 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road54');

roadCenters = [-50 0 0;
    -78.284 11.716 0;
    -90 40 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road55');

roadCenters = [-90 140 0;
    -86.7848 155.194 0;
    -78 168 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road56');

roadCenters = [-90 220 0;
    -107.574 262.426 0;
    -150 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road57');

roadCenters = [-250 280 0;
    -285.355 265.355 0;
    -300 230 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road58');

roadCenters = [-300 160 0;
    -285.353 124.645 0;
    -250 110 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road59');

roadCenters = [-390 280 0;
    -453.64 253.64 0;
    -480 190 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road60');

roadCenters = [-480 -120 0;
    -459.498 -169.498 0;
    -410 -190 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road61');

roadCenters = [-310 -190 0;
    -267.574 -207.574 0;
    -250 -250 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road62');

roadCenters = [-250 -320 0;
    -235.355 -355.355 0;
    -200 -370 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road63');

roadCenters = [-130 -370 0;
    -101.716 -358.284 0;
    -90 -330 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road64');

roadCenters = [-250 -420 0;
    -264.645 -455.355 0;
    -300 -470 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road65');

roadCenters = [-300 80 0;
    -305.858 65.8579 0;
    -320 60 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road66');

roadCenters = [-460 60 0;
    -474.142 54.1421 0;
    -480 40 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road67');

roadCenters = [-170 110 0;
    -148.787 101.213 0;
    -140 80 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road68');

roadCenters = [-120 110 0;
    -98.7868 118.787 0;
    -90 140 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road69');

roadCenters = [-140 -20 0;
    -148.787 -41.2132 0;
    -170 -50 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road70');

roadCenters = [-140 -180 0;
    -148.787 -201.213 0;
    -170 -210 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road71');

roadCenters = [-210 -210 0;
    -238.284 -221.716 0;
    -250 -250 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road72');

roadCenters = [50 -370 0;
    85.3553 -355.355 0;
    100 -320 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road73');

roadCenters = [100 -120 0;
    117.574 -77.5736 0;
    160 -60 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road74');

roadCenters = [160 -60 0;
    258.995 -18.9949 0;
    300 80 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road75');

roadCenters = [300 200 0;
    276.568 256.568 0;
    220 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road76');

roadCenters = [370 -370 0;
    440.71 -340.71 0;
    470 -270 0;
    440.71 -199.29 0;
    370 -170 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road77');

roadCenters = [370 -170 0;
    320.502 -149.498 0;
    300 -100 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road78');

roadCenters = [-250 280 0;
    -257.07 282.93 0;
    -260 290 0;
    -257.07 297.07 0;
    -250 300 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road79');

roadCenters = [350 300 0;
    420.711 270.711 0;
    450 200 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road80');

roadCenters = [450 -90 0;
    426.568 -146.569 0;
    370 -170 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road81');

roadCenters = [380 120 0;
    323.432 143.431 0;
    300 200 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road82');

roadCenters = [24 -305 0;
    80.3994 -220.059 0;
    100 -120 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road83');

roadCenters = [120 -20 0;
    131.716 -48.2843 0;
    160 -60 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road84');

roadCenters = [100 530 0;
    102.93 537.07 0;
    110 540 0;
    117.07 537.07 0;
    120 530 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road85');

roadCenters = [160 280 0;
    131.716 268.284 0;
    120 240 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road86');

roadCenters = [100 240 0;
    117.574 282.426 0;
    160 300 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road87');

roadCenters = [120 340 0;
    131.716 311.716 0;
    160 300 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road88');

roadCenters = [160 280 0;
    117.574 297.574 0;
    100 340 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road89');

roadCenters = [60 300 0;
    88.2843 311.716 0;
    100 340 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road90');

roadCenters = [120 340 0;
    102.426 297.574 0;
    60 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road91');

roadCenters = [100 240 0;
    88.2843 268.284 0;
    60 280 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road92');

roadCenters = [60 300 0;
    102.426 282.426 0;
    120 240 0];
laneSpecification = lanespec(1, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road93');

roadCenters = [370 -370 0;
    380 -372.68 0;
    387.32 -380 0;
    390 -390 0;
    388.79 -396.84 0;
    387.32 -400 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

roadCenters = [387.32 -400 0;
    320 -516.6 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road1');

roadCenters = [320 -516.6 0;
    316.94 -520.98 0;
    310 -527.12 0;
    296.14 -531.804 0;
    270 -520.98 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road2');

roadCenters = [270 -520.98 0;
    249.2 -510.15 0;
    226.84 -517.2 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road3');

roadCenters = [226.84 -517.2 0;
    81.9901 -556.013 0;
    -40.85 -470 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road4');

roadCenters = [-40.85 -470 0;
    -72.77 -451.57 0;
    -104.7 -470 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road5');

roadCenters = [-104.7 -470 0;
    -136.625 -488.432 0;
    -168.55 -470 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road6');

roadCenters = [-168.55 -470 0;
    -197.26 -451.71 0;
    -228.71 -464.74 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road94');

roadCenters = [-228.71 -464.74 0;
    -233.77 -468.62 0;
    -240 -470 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road95');

roadCenters = [-240 -470 0;
    -300 -470 0];
laneSpecification = lanespec(2, 'Width', 3.7);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road96');




% Add the ego vehicle
% ans = ans;
% load('matlab.mat')
load('test.mat');


% temp1 = [ans.logsout{1}.Values.Data(1,1) -ans.logsout{1}.Values.Data(1,3) 0];
% temp2 = [ans.logsout{1}.Values.Data(:,1) -ans.logsout{1}.Values.Data(:,3) ans.logsout{1}.Values.Data(:,2)]
%if you want to change the ego vehicle, just put the code of the corresponding vehicle to the first place.
egoVehicle = vehicle(scenario, ...
    'ClassID', 2, ...
    'Position', [-V2_translation.Data(1,3) -V2_translation.Data(1,1) 0]);
    waypoints = [-V2_translation.Data((1:step_length:end),3) -V2_translation.Data((1:step_length:end),1) V2_translation.Data((1:step_length:end),2)];
    speed = V2_speed.Data(1:step_length:end);
    trajectory(egoVehicle, waypoints, speed);
% Add the non-ego actors
% car1 = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [str2double(get_param(append(model_name,'/Vehicle Model 1 - Manual'),'X_init')) str2double(get_param(append(model_name,'/Vehicle Model 1 - Manual'),'Y_init')) 0]);
%     waypoints = [ans.v1_info.InertFrm.Cg.Disp.X.Data ans.v1_info.InertFrm.Cg.Disp.Y.Data ans.v1_info.InertFrm.Cg.Disp.Z.Data];
%     speed = sqrt(ans.v1_info.InertFrm.Cg.Vel.Xdot.Data.^2+ans.v1_info.InertFrm.Cg.Vel.Ydot.Data.^2);
%     trajectory(car1, waypoints, speed);
car1 = vehicle(scenario, ...
     'ClassID', 1, ...
     'Position', [-V1_translation.Data(1,3) -V1_translation.Data(1,1) 0]);
     waypoints = [-V1_translation.Data((1:step_length:end),3) -V1_translation.Data((1:step_length:end),1) V1_translation.Data((1:step_length:end),2)];
     speed = V1_speed.Data(1:step_length:end);
     trajectory(car1, waypoints, speed);
    
car3 = vehicle(scenario, ...
    'ClassID', 3, ...
    'Position', [-V3_translation.Data(1,3) -V3_translation.Data(1,1) 0]);
    waypoints = [-V3_translation.Data((1:step_length:end),3) -V3_translation.Data((1:step_length:end),1) V3_translation.Data((1:step_length:end),2)];
    speed = V3_speed.Data(1:step_length:end);
    trajectory(car3, waypoints, speed);
    
car4 = vehicle(scenario, ...
    'ClassID', 4, ...
    'Position', [-V4_translation.Data(1,3) -V4_translation.Data(1,1) 0]);
    waypoints = [-V4_translation.Data((1:step_length:end),3) -V4_translation.Data((1:step_length:end),1) V4_translation.Data((1:step_length:end),2)];
    speed = V4_speed.Data(1:step_length:end);
    trajectory(car4, waypoints, speed);
    
car5 = vehicle(scenario, ...
    'ClassID', 5, ...
    'Position', [-V5_translation.Data(1,3) -V5_translation.Data(1,1) 0]);
    waypoints = [-V5_translation.Data((1:step_length:end),3) -V5_translation.Data((1:step_length:end),1) V5_translation.Data((1:step_length:end),2)];
    speed = V5_speed.Data(1:step_length:end);
    trajectory(car5, waypoints, speed);
    
 car6 = vehicle(scenario, ...
    'ClassID', 6, ...
    'Position', [-V6_translation.Data(1,3) -V6_translation.Data(1,1) 0]);
    waypoints = [-V6_translation.Data((1:step_length:end),3) -V6_translation.Data((1:step_length:end),1) V6_translation.Data((1:step_length:end),2)];
    speed = V6_speed.Data(1:step_length:end);
    trajectory(car6, waypoints, speed);
    
 car7 = vehicle(scenario, ...
    'ClassID', 7, ...
    'Position', [-V7_translation.Data(1,3) -V7_translation.Data(1,1) 0]);
    waypoints = [-V7_translation.Data((1:step_length:end),3) -V7_translation.Data((1:step_length:end),1) V7_translation.Data((1:step_length:end),2)];
    speed = V7_speed.Data(1:step_length:end);
    trajectory(car7, waypoints, speed);
    
 car8 = vehicle(scenario, ...
    'ClassID', 8, ...
    'Position', [-V8_translation.Data(1,3) -V8_translation.Data(1,1) 0]);
    waypoints = [-V8_translation.Data((1:step_length:end),3) -V8_translation.Data((1:step_length:end),1) V8_translation.Data((1:step_length:end),2)];
    speed = V8_speed.Data(1:step_length:end);
    trajectory(car8, waypoints, speed);
    
car9 = vehicle(scenario, ...
    'ClassID', 9, ...
    'Position', [-V9_translation.Data(1,3) -V9_translation.Data(1,1) 0]);
    waypoints = [-V9_translation.Data((1:step_length:end),3) -V9_translation.Data((1:step_length:end),1) V9_translation.Data((1:step_length:end),2)];
    speed = V9_speed.Data(1:step_length:end);
    trajectory(car9, waypoints, speed);
    
car10 = vehicle(scenario, ...
    'ClassID', 10, ...
    'Position', [-V10_translation.Data(1,3) -V10_translation.Data(1,1) 0]);
    waypoints = [-V10_translation.Data((1:step_length:end),3) -V10_translation.Data((1:step_length:end),1) V10_translation.Data((1:step_length:end),2)];
    speed = V10_speed.Data(1:step_length:end);
    trajectory(car10, waypoints, speed);
% car3 = vehicle(scenario, ...
%     'ClassID', 3, ...
%     'Position', [-temp.V3_translation.Data(1,3) -temp.V3_translation.Data(1,1) 0]);
%     waypoints = [-temp.V3_translation.Data((1:step_length:end),3) -temp.V3_translation.Data((1:step_length:end),1) temp.V3_translation.Data((1:step_length:end),2)];
%     speed = temp.V3_speed.Data(1:step_length:end);
%     trajectory(car3, waypoints, speed);
% car4 = vehicle(scenario, ...
%     'ClassID', 4, ...
%     'Position', [-temp.V4_translation.Data(1,3) -temp.V4_translation.Data(1,1) 0]);
%     waypoints = [-temp.V4_translation.Data((1:step_length:end),3) -temp.V4_translation.Data((1:step_length:end),1) temp.V4_translation.Data((1:step_length:end),2)];
%     speed = temp.V4_speed.Data(1:step_length:end);
%     trajectory(car4, waypoints, speed);
% car5 = vehicle(scenario, ...
%     'ClassID', 5, ...
%     'Position', [-temp.V5_translation.Data(1,3) -temp.V5_translation.Data(1,1) 0]);
%     waypoints = [-temp.V5_translation.Data((1:step_length:end),3) -temp.V5_translation.Data((1:step_length:end),1) temp.V5_translation.Data((1:step_length:end),2)];
%     speed = temp.V5_speed.Data(1:step_length:end);
%     trajectory(car5, waypoints, speed);
% car6 = vehicle(scenario, ...
%     'ClassID', 6, ...
%     'Position', [-temp.V6_translation.Data(1,3) -temp.V6_translation.Data(1,1) 0]);
%     waypoints = [-temp.V6_translation.Data((1:step_length:end),3) -temp.V6_translation.Data((1:step_length:end),1) temp.V6_translation.Data((1:step_length:end),2)];
%     speed = temp.V6_speed.Data(1:step_length:end);
%     trajectory(car6, waypoints, speed);
% car7 = vehicle(scenario, ...
%     'ClassID', 7, ...
%     'Position', [-temp.V7_translation.Data(1,3) -temp.V7_translation.Data(1,1) 0]);
%     waypoints = [-temp.V7_translation.Data((1:step_length:end),3) -temp.V7_translation.Data((1:step_length:end),1) temp.V7_translation.Data((1:step_length:end),2)];
%     speed = temp.V7_speed.Data(1:step_length:end);
%     trajectory(car7, waypoints, speed);
% car8 = vehicle(scenario, ...
%     'ClassID', 8, ...
%     'Position', [-temp.V8_translation.Data(1,3) -temp.V8_translation.Data(1,1) 0]);
%     waypoints = [-temp.V8_translation.Data((1:step_length:end),3) -temp.V8_translation.Data((1:step_length:end),1) temp.V8_translation.Data((1:step_length:end),2)];
%     speed = temp.V8_speed.Data(1:step_length:end);
%     trajectory(car8, waypoints, speed);
% car9 = vehicle(scenario, ...
%     'ClassID', 9, ...
%     'Position', [-temp.V9_translation.Data(1,3) -temp.V9_translation.Data(1,1) 0]);
%     waypoints = [-temp.V9_translation.Data((1:step_length:end),3) -temp.V9_translation.Data((1:step_length:end),1) temp.V9_translation.Data((1:step_length:end),2)];
%     speed = temp.V9_speed.Data(1:step_length:end);
%     trajectory(car9, waypoints, speed);
% car10 = vehicle(scenario, ...
%     'ClassID', 10, ...
%     'Position', [-temp.V10_translation.Data(1,3) -temp.V10_translation.Data(1,1) 0]);
%     waypoints = [-temp.V10_translation.Data((1:step_length:end),3) -temp.V10_translation.Data((1:step_length:end),1) temp.V10_translation.Data((1:step_length:end),2)];
%     speed = temp.V10_speed.Data(1:step_length:end);
%     trajectory(car10, waypoints, speed);
