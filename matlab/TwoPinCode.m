clear 
close all
clc

Knee = "Test2_2022_11_18_144427";

Right = 1;

Tests = ["FE1" ,"FE2"];

ProbeName = 'New Biomech 4 LED Stylus Calibration Se x'; 

for T = 1:length(Tests)
    
clearvars -except T Tests EXT State Knee Right ProbeName
% close all
clc

%% Set up Certus sampling frequency and state whether specimen is left or right knee

Freq = 100; % Certus sampling freq (Hz)

% Right = false; % True for right knee, false for left knee

angle = @(u_,v_) acosd(dot(u_,v_)/(norm(u_,2)*norm(v_,2))); %define a function to calculate the angle between two vectors
ucross=@(u_,v_) cross(u_,v_)/norm(cross(u_,v_),2); %define function to find unit cross product
uvector=@(a,b) (b-a)/norm(b-a,2); %define a function to find a unit vector from a to b


%% Import marker and probe tip locations for inital digitising position

% Points are later used to define body specific coordinate systems,
% calculated from anatomical landmarks

% Probe points are from stationary condition, only the probe
% columns should differ between data files.

% Find folder path for knee tracking data files:

% FolderPath = ['\Knee']; 
% Folders = struct2cell(dir(FolderPath));
% Folder = string(Folders(1,:));
% Folder = find(contains(Folder, ShortRig));
% Folders = char(Folders(1,Folder));
% Path = [FolderPath,'\', Folders];
FolderPath = fullfile(pwd, '/', char(Knee));

% TIBIA

% TM
FileName = [FolderPath, '/TM.csv'];
XYZdataT{1} = readcell(FileName);

% TL
FileName = [FolderPath, '/TL.csv'];
XYZdataT{2} = readcell(FileName);

% % TP
% FileName = [FolderPath, '/TP.csv'];
% [~, ~, raw] = readcell(FileName);
% XYZdataT{3} = raw;

% TD
FileName = [FolderPath, '/TD.csv'];
XYZdataT{4} = readcell(FileName);

% % TPT
% FileName = [FolderPath, '\TPT.csv'];
% [~, ~, raw] = readcell(FileName);
% XYZdataT{5} = raw;


% Find position of digitised tibial landmarks by 'finding' the probe
% columns in the respect digitised data files and extracting the points of
% the tip of the probe

% Tibia Medial
MarkerNames = string(char(XYZdataT{1}(4,:)));
ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
ProbeRowIndex = 5;
TMed = mean(cell2mat(XYZdataT{1}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% TMed = mean(cell2mat(XYZdataT{1}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% TMed = get_tip(TMed(1), TMed(2), TMed(3), TMed(4), TMed(5), TMed(6), TMed(7));
% TMed = TMed';


% Tibia Lateral
MarkerNames = string(char(XYZdataT{2}(4,:)));
ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
ProbeRowIndex = 5;
TLat = mean(cell2mat(XYZdataT{2}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% TLat = mean(cell2mat(XYZdataT{2}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% TLat = get_tip(TLat(1), TLat(2), TLat(3), TLat(4), TLat(5), TLat(6), TLat(7));
% TLat = TLat';

% 
% % Tibia Proximal
% MarkerNames = string(char(XYZdataT{3}(5,:)));
% ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
% ProbeRowIndex = 5;
% TProx = mean(cell2mat(XYZdataT{3}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% % TProx = mean(cell2mat(XYZdataT{3}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% % TProx = get_tip(TProx(1), TProx(2), TProx(3), TProx(4), TProx(5), TProx(6), TProx(7));
% % TProx = TProx';


% Tibia Distal
MarkerNames = string(char(XYZdataT{4}(4,:)));
ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
ProbeRowIndex = 5;
TDis = mean(cell2mat(XYZdataT{4}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% TDis = mean(cell2mat(XYZdataT{4}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% TDis = get_tip(TDis(1), TDis(2), TDis(3), TDis(4), TDis(5), TDis(6), TDis(7));
% TDis = TDis';


% % Tibia Patella Tendon
% MarkerNames = string(char(XYZdataT{5}(5,:)));
% ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
% ProbeRowIndex = 5;
% TIns = mean(cell2mat(XYZdataT{5}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% % TIns = mean(cell2mat(XYZdataT{5}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% % TIns = get_tip(TIns(1), TIns(2), TIns(3), TIns(4), TIns(5), TIns(6), TIns(7));
% % TIns = TIns';


% FEMUR

% FM
FileName = [FolderPath, '/FM.csv'];
XYZdataF{1} = readcell(FileName);

% FL
FileName = [FolderPath, '/FL.csv'];
XYZdataF{2} = readcell(FileName);

% FP
FileName = [FolderPath, '/FP.csv'];
XYZdataF{3} = readcell(FileName);

% % FP2
% FileName = [FolderPath, '\FPL.csv'];
% [~, ~, raw] = readcell(FileName);
% XYZdataF{4} = raw;

% Find position of digitised femoral landmarks by 'finding' the probe
% columns in the respect digitised data files and extracting the points of
% the tip of the probe

% Femur Medial
MarkerNames = string(char(XYZdataF{1}(4,:)));
ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
ProbeRowIndex = 5;
FMed = mean(cell2mat(XYZdataF{1}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% FMed = mean(cell2mat(XYZdataF{1}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% FMed = get_tip(FMed(1), FMed(2), FMed(3), FMed(4), FMed(5), FMed(6), FMed(7));
% FMed = FMed';


% Femur Lateral
MarkerNames = string(char(XYZdataF{2}(4,:)));
ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
ProbeRowIndex = 5;
FLat = mean(cell2mat(XYZdataF{2}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% FLat = mean(cell2mat(XYZdataF{2}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% FLat = get_tip(FLat(1), FLat(2), FLat(3), FLat(4), FLat(5), FLat(6), FLat(7));
% FLat = FLat';


% Femur Proximal 
MarkerNames = string(char(XYZdataF{3}(4,:)));
ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
ProbeRowIndex = 5;
FProx = mean(cell2mat(XYZdataF{3}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% FProx = mean(cell2mat(XYZdataF{3}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% FProx = get_tip(FProx(1), FProx(2), FProx(3), FProx(4), FProx(5), FProx(6), FProx(7));
% FProx = FProx';

% % Femur Proximal 2
% MarkerNames = string(char(XYZdataF{4}(5,:)));
% ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
% ProbeRowIndex = 5;
% FProx2 = mean(cell2mat(XYZdataF{4}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% 
% FProx = (FProx1+FProx2)/2;


% % PATELLA
% 
% % PMP
% FileName = [FolderPath, '\PMP.csv'];
% [~, ~, raw] = readcell(FileName);
% XYZdataP{1} = raw;
% 
% % PMD
% FileName = [FolderPath, '\PMD.csv'];
% [~, ~, raw] = readcell(FileName);
% XYZdataP{2} = raw;
% 
% % PLP
% FileName = [FolderPath, '\PLP.csv'];
% [~, ~, raw] = readcell(FileName);
% XYZdataP{3} = raw;
% 
% % PLD
% FileName = [FolderPath, '\PLD.csv'];
% [~, ~, raw] = readcell(FileName);
% XYZdataP{4} = raw;
% 
% % PPT
% FileName = [FolderPath, '\PPT.csv'];
% [~, ~, raw] = readcell(FileName);
% XYZdataP{5} = raw;
% 
% 
% % Find position of digitised patellar landmarks by 'finding' the probe
% % columns in the respect digitised data files and extracting the points of
% % the tip of the probe
% 
% % Patalla Medial Proximal
% MarkerNames = string(char(XYZdataP{1}(5,:)));
% ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
% ProbeRowIndex = 5;
% PMedP = mean(cell2mat(XYZdataP{1}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% % PMedP = mean(cell2mat(XYZdataP{1}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% % PMedP = get_tip(PMedP(1), PMedP(2), PMedP(3), PMedP(4), PMedP(5), PMedP(6), PMedP(7));
% % PMedP = PMedP';
% 
% 
% % Patella Medial Distal
% MarkerNames = string(char(XYZdataP{2}(5,:)));
% ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
% ProbeRowIndex = 5;
% PMedD = mean(cell2mat(XYZdataP{2}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% % PMedD = mean(cell2mat(XYZdataP{2}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% % PMedD = get_tip(PMedD(1), PMedD(2), PMedD(3), PMedD(4), PMedD(5), PMedD(6), PMedD(7));
% % PMedD = PMedD';
% % NB: To get one patella digitisation point for the medial side, average
% % the positions of PMedP and PMedD:
% 
% PMed = (PMedP + PMedD)/2;
% 
% % Patella Lateral Proximal
% MarkerNames = string(char(XYZdataP{3}(5,:)));
% ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
% ProbeRowIndex = 5;
% PLatP = mean(cell2mat(XYZdataP{3}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% % PLatP = mean(cell2mat(XYZdataP{3}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% % PLatP = get_tip(PLatP(1), PLatP(2), PLatP(3), PLatP(4), PLatP(5), PLatP(6), PLatP(7));
% % PLatP = PLatP';
% 
% 
% 
% % Patella Lateral Distal
% MarkerNames = string(char(XYZdataP{4}(5,:)));
% ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
% ProbeRowIndex = 5;
% PLatD = mean(cell2mat(XYZdataP{4}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% % PLatD = mean(cell2mat(XYZdataP{4}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% % PLatD = get_tip(PLatD(1), PLatD(2), PLatD(3), PLatD(4), PLatD(5), PLatD(6), PLatD(7));
% % PLatD = PLatD';
% % NB: To get one patella digitisation point for the lateral side, average
% % the positions of PLatP and PLatD:
% 
% PLat = (PLatP + PLatD)/2;
% 
% 
% % Patella Patella Tendon
% MarkerNames = string(char(XYZdataP{5}(5,:)));
% ProbeColumnIndex = find(contains(MarkerNames, ProbeName));
% ProbeRowIndex = 5;
% PIns = mean(cell2mat(XYZdataP{5}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+2)));
% % PIns = mean(cell2mat(XYZdataP{5}(ProbeRowIndex+1:end-1,ProbeColumnIndex:ProbeColumnIndex+6)));
% % % Convert XYZ of NDI tracker (i.e. not the probe tip) to XYZ of probe tip
% % PIns = get_tip(PIns(1), PIns(2), PIns(3), PIns(4), PIns(5), PIns(6), PIns(7));
% % PIns = PIns';

%%%%%%% END OF IMPORT OF ANATOMIC REFERENCE POINTS %%%%%%%%%%%%%%



%% Import position of tracking markers at inital digitisation position

% Same process as digitising the bony landmarks but instead of the probe
% points, the trackers themselves are used.

% NB: Pins can be named whatever is deemed fit. If something general, like
% 'Pin1' is used, then be sure to keep a record of which bone this tracker
% relates to for each experiment.

% As the knee should not have moved during the digitisation process, any of
% the XYZdata sets may be used.

% Find position and quaternions, using XYZdataT:

MarkerNames = string(char(XYZdataT{1}(4,:)));

% Pin 1
Pin1ColumnIndex = find(contains(MarkerNames, 'Pin1 x'));
Pin1RowIndex = 5;
Pin1_XYZ = mean(cell2mat(XYZdataT{1}(Pin1RowIndex+1:end-1, Pin1ColumnIndex:Pin1ColumnIndex+2)));
Pin1_Q = mean(cell2mat(XYZdataT{1}(Pin1RowIndex+1:end-1, Pin1ColumnIndex-4:Pin1ColumnIndex-1)));
Pin1_R = quaternion2euler(Pin1_Q);

% % Pin 2
% Pin2ColumnIndex = find(contains(MarkerNames, 'Pin2 x'));
% Pin2RowIndex = 5;
% Pin2_XYZ = mean(cell2mat(XYZdataT{1}(Pin2RowIndex+1:end-1, Pin2ColumnIndex:Pin2ColumnIndex+2)));
% Pin2_Q = mean(cell2mat(XYZdataT{1}(Pin2RowIndex+1:end-1, Pin2ColumnIndex-4:Pin2ColumnIndex-1)));
% Pin2_R = quaternion2euler(Pin2_Q);
% 
% % Patella 3
% Patella3ColumnIndex = find(contains(MarkerNames, 'Patella3 x'));
% Patella3RowIndex = 5;
% Patella3_XYZ = mean(cell2mat(XYZdataT{1}(Patella3RowIndex+1:end-1, Patella3ColumnIndex:Patella3ColumnIndex+2)));
% Patella3_Q = mean(cell2mat(XYZdataT{1}(Patella3RowIndex+1:end-1, Patella3ColumnIndex-4:Patella3ColumnIndex-1)));
% Patella3_R = quaternion2euler(Patella3_Q);
% 
% % Patella 4
% Patella4ColumnIndex = find(contains(MarkerNames, 'Patella4 x'));
% Patella4RowIndex = 5;
% Patella4_XYZ = mean(cell2mat(XYZdataT{1}(Patella4RowIndex+1:end-1, Patella4ColumnIndex:Patella4ColumnIndex+2)));
% Patella4_Q = mean(cell2mat(XYZdataT{1}(Patella4RowIndex+1:end-1, Patella4ColumnIndex-4:Patella4ColumnIndex-1)));
% Patella4_R = quaternion2euler(Patella4_Q);
% 
% % Pin 5
% Pin5ColumnIndex = find(contains(MarkerNames, 'Pin5 x'));
% Pin5RowIndex = 5;
% Pin5_XYZ = mean(cell2mat(XYZdataT{1}(Pin5RowIndex+1:end-1, Pin5ColumnIndex:Pin5ColumnIndex+2)));
% Pin5_Q = mean(cell2mat(XYZdataT{1}(Pin5RowIndex+1:end-1, Pin5ColumnIndex-4:Pin5ColumnIndex-1)));
% Pin5_R = quaternion2euler(Pin5_Q);

% Pin 6
Pin6ColumnIndex = find(contains(MarkerNames, 'Pin2 x'));
Pin6RowIndex = 5;
Pin6_XYZ = mean(cell2mat(XYZdataF{1}(Pin6RowIndex+1:end-1, Pin6ColumnIndex:Pin6ColumnIndex+2)));
Pin6_Q = mean(cell2mat(XYZdataF{1}(Pin6RowIndex+1:end-1, Pin6ColumnIndex-4:Pin6ColumnIndex-1)));
Pin6_R = quaternion2euler(Pin6_Q);




%% Define coordinate systems for each bone using digitised points

% For tibia
[gTt0, originT] = defineBodyFixedFrameTibia_v2(TMed, TLat, TDis, Right);
grt0 = [originT, 1]';

% For femur
[gTf0, originF] = defineBodyFixedFrameFemur_v2(FMed, FLat, FProx, Right);
grf0 = [originF, 1]';

% % For patella
% [gTp0, originP] = defineBodyFixedFramePatella_v2(PMed, PLat, PIns, Right);
% grp0 = [originP, 1]';

% % For patella tendon insertion points
% grtPT0 = [TIns, 1]'; % Position of patella tendon insertion on TIBIA in global frame
% grpPT0 = [PIns,1]'; % Position of patella tendon insertion on PATELLA in global frame


%% Define fixed frame for each of the trackers in global coordinates

% Pin 1 in global coordinates
[gT_Pin1_t0] = defineTrackerFixedFrame_v2(Pin1_R, Pin1_XYZ);

% % Pin 2 in global coordinates
% [gT_Pin2_t0] = defineTrackerFixedFrame_v2(Pin2_R, Pin2_XYZ);
% 
% % Patella 3 in global coordinates
% [gT_Patella3_t0] = defineTrackerFixedFrame_v2(Patella3_R, Patella3_XYZ);
% 
% % Patella 4 in global coordinates
% [gT_Patella4_t0] = defineTrackerFixedFrame_v2(Patella4_R, Patella4_XYZ);
% 
% % Pin 5 in global coordinates
% [gT_Pin5_t0] = defineTrackerFixedFrame_v2(Pin5_R, Pin5_XYZ);

% Pin 6 in global coordinates
[gT_Pin6_t0] = defineTrackerFixedFrame_v2(Pin6_R, Pin6_XYZ);



%% Generate constant transforms between the bony and tracker fixed coordinate systems

% This step generates a transform between each of the bone to their
% relevant trackers (rigid body assumed)


% For tibia:

% For Pin 1 to tibia frame
Pin1_T_tc = gT_Pin1_t0\gTt0;
% For Pin 1 to tibia origin
Pin1_r_tc = gT_Pin1_t0\grt0;
% % For Pin 1 to patella tendon insertion on tibia
% Pin1_r_ptc = gT_Pin1_t0\grtPT0;
% 
% % For Pin 2 to tibia frame
% Pin2_T_tc = gT_Pin2_t0\gTt0;
% % For Pin 2 to tibia origin
% Pin2_r_tc = gT_Pin2_t0\grt0;
% % For Pin 2 to patella tendon insertion on tibia
% Pin2_r_ptc = gT_Pin2_t0\grtPT0;
% 
% 
% 
% % For Patella:
% 
% % For Patella 3 to patella frame
% Patella3_T_tc = gT_Patella3_t0\gTp0;
% % For Patella 3 to patella origin
% Patella3_r_tc = gT_Patella3_t0\grp0;
% % For Patella 3 to patella tendon insertion on patella
% Patella3_r_ptc = gT_Patella3_t0\grpPT0;
% 
% % For Patella 4 to patella frame
% Patella4_T_tc = gT_Patella4_t0\gTp0;
% % For Patella 4 to patella origin
% Patella4_r_tc = gT_Patella4_t0\grp0;
% % For Patella 4 to patella tendon insertion on patella
% Patella4_r_ptc = gT_Patella4_t0\grpPT0;
% 
% 
% % For Femur:
% 
% % For Pin 5
% Pin5_T_tc = gT_Pin5_t0\gTf0;
% % For Pin 5 to femur origin
% Pin5_r_tc = gT_Pin5_t0\grf0;

% For Pin 6
Pin6_T_tc = gT_Pin6_t0\gTf0;
% For Pin 6 to femur origin
Pin6_r_tc = gT_Pin6_t0\grf0;





%% Load tracked points
Test = char(Tests(T));
% FolderPath = fullfile(pwd, Path );
FileName = [FolderPath,'/', Test, '.csv'];

% PinsDigi = [Pin1_XYZ; Pin2_XYZ; Patella3_XYZ; Patella4_XYZ; Pin5_XYZ; Pin6_XYZ]
PinsDigi = [Pin1_XYZ; Pin6_XYZ];

RawData = readcell(FileName);
RawDataOriginalSize = length(RawData);

TrackerNames = RawData(4,:);

%Find pin position, quaternion and error columns

% For Pin 1:
Pin1_X = find(contains(TrackerNames, 'Pin1 x'));
Pin1_Q0 = find(contains(TrackerNames, 'Pin1 q0'));
Pin1_Error = find(contains(TrackerNames, 'Pin1 Error'));

% % For Pin 2
% Pin2_X = find(contains(TrackerNames, 'Pin2 x'));
% Pin2_Q0 = find(contains(TrackerNames, 'Pin2 q0'));
% Pin2_Error = find(contains(TrackerNames, 'Pin2 Error'));
% 
% % For Patella 3
% Patella3_X = find(contains(TrackerNames, 'Patella3 x'));
% Patella3_Q0 = find(contains(TrackerNames, 'Patella3 q0'));
% Patella3_Error = find(contains(TrackerNames, 'Patella3 Error'));
% 
% % For Patella 4
% Patella4_X = find(contains(TrackerNames, 'Patella4 x'));
% Patella4_Q0 = find(contains(TrackerNames, 'Patella4 q0'));
% Patella4_Error = find(contains(TrackerNames, 'Patella4 Error'));
% 
% % For Pin 5
% Pin5_X = find(contains(TrackerNames, 'Pin5 x'));
% Pin5_Q0 = find(contains(TrackerNames, 'Pin5 q0'));
% Pin5_Error = find(contains(TrackerNames, 'Pin5 Error'));

% For Pin 6
Pin6_X = find(contains(TrackerNames, 'Pin2 x'));
Pin6_Q0 = find(contains(TrackerNames, 'Pin2 q0'));
Pin6_Error = find(contains(TrackerNames, 'Pin2 Error'));


% % Extract data to be used, remove rows with drop out. Find indexes of rows
% % where both trackers for tibia, patella, or femur drop out simultaneously
% 
% Tibia_Out = find(  isnan(RawData(1:end,Pin1_Error))==1 &  isnan(RawData(1:end,Pin2_Error))==1);
% Patella_Out = find(  isnan(RawData(1:end,Patella3_Error))==1 &  isnan(RawData(1:end,Patella4_Error))==1);
% Femur_Out = find(  isnan(RawData(1:end,Pin5_Error))==1 &  isnan(RawData(1:end,Pin6_Error))==1);
% 
% % Removes duplicate rows where 1 or more bodies have dropped out
% Bodies_Out = [Tibia_Out', Patella_Out', Femur_Out'];
% Bodies_Out = unique(Bodies_Out);
% 
% % Rewrites raw data as before, minus rows where drop out occurred
% RawData([Bodies_Out],:) = [];
TrackedData = RawData;


% if RawDataOriginalSize == length(TrackedData)
%     MarkerStatus = ['No marker drop out. RawDataOriginal = ', num2str(length(RawData)), ' TrackedData = ', num2str(length(TrackedData))]
% else
%     MarkerStatus = ['Marker drop out. RawDataOriginal = ', num2str(RawDataOriginalSize), ' TrackedData = ', num2str(length(TrackedData)) ...
%        ' Tibia points dropped = ', num2str(length(Tibia_Out)), ' Patella points dropped = ', num2str(length(Patella_Out)), ' Femur points dropped = ', num2str(length(Femur_Out)) ]
% end

% disp(T);

% Separate tracked data by pin

% Pin 1:
Pin1_XYZs = cell2mat(TrackedData(5:end,Pin1_X:Pin1_X + 2));
Pin1_Qs = cell2mat(TrackedData(5:end,Pin1_Q0:Pin1_Q0 + 3));
    for i = 1:size(Pin1_Qs,1)
        Pin1_Rs(i,:) = quaternion2euler(Pin1_Qs(i,:));
    end
    
% % Pin 2:
% Pin2_XYZs = TrackedData(1:end,Pin2_X:Pin2_X + 2);
% Pin2_Qs = TrackedData(1:end,Pin2_Q0:Pin2_Q0 + 3);
%     for i = 1:size(Pin2_Qs,1)
%         Pin2_Rs(i,:) = quaternion2euler(Pin2_Qs(i,:));
%     end
% 
% % Patella 3:
% Patella3_XYZs = TrackedData(1:end,Patella3_X:Patella3_X + 2);
% Patella3_Qs = TrackedData(1:end,Patella3_Q0:Patella3_Q0 + 3);
%     for i = 1:size(Patella3_Qs,1)
%         Patella3_Rs(i,:) = quaternion2euler(Patella3_Qs(i,:));
%     end
% 
% % Patella 4:
% Patella4_XYZs = TrackedData(1:end,Patella4_X:Patella4_X + 2);
% Patella4_Qs = TrackedData(1:end,Patella4_Q0:Patella4_Q0 + 3);
%     for i = 1:size(Patella4_Qs,1)
%         Patella4_Rs(i,:) = quaternion2euler(Patella4_Qs(i,:));
%     end
% 
% % Pin 5:
% Pin5_XYZs = TrackedData(1:end,Pin5_X:Pin5_X + 2);
% Pin5_Qs = TrackedData(1:end,Pin5_Q0:Pin5_Q0 + 3);
%     for i = 1:size(Pin5_Qs,1)
%         Pin5_Rs(i,:) = quaternion2euler(Pin5_Qs(i,:));
%     end

% Pin 6:
Pin6_XYZs = cell2mat(TrackedData(5:end,Pin6_X:Pin6_X + 2));
Pin6_Qs = cell2mat(TrackedData(5:end,Pin6_Q0:Pin6_Q0 + 3));
    for i = 1:size(Pin6_Qs,1)
        Pin6_Rs(i,:) = quaternion2euler(Pin6_Qs(i,:));
    end


% Create matrices of tracker marker position and rotations in time

% Pin 1:
[gT_Pin1_ti] = findTrackerFixedFrames_v2(Pin1_Rs, Pin1_XYZs);
% % Pin 2:
% [gT_Pin2_ti] = findTrackerFixedFrames_v2(Pin2_Rs, Pin2_XYZs);
% % Patella 3:
% [gT_Patella3_ti] = findTrackerFixedFrames_v2(Patella3_Rs, Patella3_XYZs);
% % Patella 4:
% [gT_Patella4_ti] = findTrackerFixedFrames_v2(Patella4_Rs, Patella4_XYZs);
% % Pin 5:
% [gT_Pin5_ti] = findTrackerFixedFrames_v2(Pin5_Rs, Pin5_XYZs);
% % Pin 6:
[gT_Pin6_ti] = findTrackerFixedFrames_v2(Pin6_Rs, Pin6_XYZs);







%% Calculate transformation matricies from body fixed to global fixed frames and motion relative to initial position


 for i = 1:length(gT_Pin1_ti)

                %pin1error(i,1) = isnan(TrackedData(i,Pin1_Error));

% %                      Using either Pin 1 or Pin 2 for tibia movement
%                      if isnan(TrackedData(i,Pin1_Error)) == 1

%                          Body fixed relative to global
%                          gTti{i,1} = gT_Pin2_ti{i,1}*Pin2_T_tc;
% %                          Position vectors of tibia origin in global
%                          grti{i,1} = gT_Pin2_ti{i,1}*Pin2_r_tc;
% %                          Patella tendon insertion point in global
%                          grtPTi{i,1} = gT_Pin2_ti{i,1}*Pin2_r_ptc;
%                          flagTibia(i,1) = 5;
               
%                      else
% 
%                          Body fixed relative to global
                         gTti{i,1} = gT_Pin1_ti{i,1}*Pin1_T_tc;
%                          Position vectors of tibia origin in global
                         grti{i,1} = gT_Pin1_ti{i,1}*Pin1_r_tc;
% %                          Patella tendon insertion point in global
%                          grtPTi{i,1} = gT_Pin1_ti{i,1}*Pin1_r_ptc;
                         flagTibia(i,1) = 2;
                         
%                      end



% %                      Using either Patella 3 or 4 for patella movement
%                      if isnan(TrackedData(i,Patella4_Error)) == 1
%                          
%                           Body fixed relative to global
%                          gTpi{i,1} = gT_Patella3_ti{i,1}*Patella3_T_tc;
%                          Position vectors of tibia origin in global
%                          grpi{i,1} = gT_Patella3_ti{i,1}*Patella3_r_tc;
%                          Patella tendon insertion point in global
%                          grpPTi{i,1} = gT_Patella3_ti{i,1}*Patella3_r_ptc;
%                          
%                       
%                         flagPatella(i,1) = 20;
% 
%                      else
%                         
%                          Body fixed relative to global
%                          gTpi{i,1} = gT_Patella4_ti{i,1}*Patella4_T_tc;
%                          Position vectors of tibia origin in global
%                          grpi{i,1} = gT_Patella4_ti{i,1}*Patella4_r_tc;
%                          Patella tendon insertion point in global
%                          grpPTi{i,1} = gT_Patella4_ti{i,1}*Patella4_r_ptc;
%                          
%                          flagPatella(i,1) = 25;
%                      end

                     
%                      if contains(FileName,'FE') == 1
% %                          [gT_Pin6_ti] = findTrackerFixedFrames_v2(Pin6_R, Pin6_XYZ);
%                          
%                                  gTfi{i,1} = gT_Pin6_ti{1,1}*Pin6_T_tc;
%                                  % Position vectors of tibia origin in global
%                                  grfi{i,1} = gT_Pin6_ti{1,1}*Pin6_r_tc;
%                                  flagFemur(i,1) = 100;

%                             [gT_Pin5_ti] = findTrackerFixedFrames_v2(Pin5_R, Pin5_XYZ);
%                          
%                                  gTfi{i,1} = gT_Pin5_ti{1,1}*Pin5_T_tc;
%                                  % Position vectors of tibia origin in global
%                                  grfi{i,1} = gT_Pin5_ti{1,1}*Pin5_r_tc;
%                                  flagFemur(i,1) = 100;
%                      else
                         
                        % Using either pin 5 or 6 for femur movement
%                               if isnan(TrackedData(i,Pin6_Error)) == 1

%                                  % Body fixed relative to global
%                                  gTfi{i,1} = gT_Pin5_ti{i,1}*Pin5_T_tc;
%                                  % Position vectors of tibia origin in global
%                                  grfi{i,1} = gT_Pin5_ti{i,1}*Pin5_r_tc;
%                                  flagFemur(i,1) = 1;


%                              else

                                    % Body fixed relative to global
                                 gTfi{i,1} = gT_Pin6_ti{i,1}*Pin6_T_tc;
                                 % Position vectors of tibia origin in global
                                 grfi{i,1} = gT_Pin6_ti{i,1}*Pin6_r_tc;
                                 flagFemur(i,1) = 10;

%                               end
                             
%                      end





                % Calc motion of bones relative to each other

                      % Tibia relative to femur:
                      fTt{i,1} = gTfi{i,1}\gTti{i,1};
                      fRt{i,1}=fTt{i,1}(1:3,1:3);

%                       % Patella relative to femur:
%                       fTp{i,1} = gTfi{i,1}\gTpi{i,1};


                 % Convert points into femoral reference plane

%                     % Tibial patella tendon insertion point in femoral frame of reference 
%                     frtPTi{i,1} = gTfi{i,1}\grtPTi{i,1};
%                     % Patellar patella tendon insertion point in femoral frame of reference
%                     frpPTi{i,1} = gTfi{i,1}\grpPTi{i,1};
                    % Tibial origin point in femoral frame of reference
                    frti{i,1} = gTfi{i,1}\(grti{i,1} - grfi{i,1});

                    %Begin richard new version (v5)

                    I_=gTfi{i,1}(1:3,1);%Femoral X axis unit vector, Grood and Suntay definition
                    J_=gTfi{i,1}(1:3,2);%Femoral Y axis unit vector, Grood and Suntay definition
                    K_=gTfi{i,1}(1:3,3);%Femoral Z axis unit vector, Grood and Suntay definition
                    
                    i_=gTti{i,1}(1:3,1);%Tibial x axis unit vector, Grood and Suntay definition
                    j_=gTti{i,1}(1:3,2);%Tibial y axis unit vector, Grood and Suntay definition
                    k_=gTti{i,1}(1:3,3);%Tibial z axis unit vector, Grood and Suntay definition
%                     
%                     pi_=gTpi{i,1}(1:3,1);%Patellar x axis unit vector, Grood and Suntay definition
%                     pj_=gTpi{i,1}(1:3,2);%Patellar y axis unit vector, Grood and Suntay definition
%                     pk_=gTpi{i,1}(1:3,3);%Patellar z axis unit vector, Grood and Suntay definition


                    e1_=I_;%Femoral X axis in global reference frame, Grood and Suntay definition
                    e3_=k_;%Tibial z axis in global reference frame, Grood and Suntay definition
%                     pe3_=pk_;
                    e2_=ucross(e3_,e1_);%Floating axis in global reference frame, Grood and Suntay definition
%                     pe2_=ucross(pe3_,e1_);
                    
                    
                    
                    
                    for TIBIOFEMORAL = 1
                        [TFMatrixangles(i,:), TFMatrixtrans(i,:)] = rotationsAndTranslations(fTt{i}, Right);
%                     Flexion(i,1)=asind(-dot(e2_,K_));
%                     Flexion(i,1)=acosd(dot(J_,e2_));
                    beta(i,1)=acosd(dot(I_,k_));
                    if Right
                        ExtRotation(i,1)=asind(dot(-e2_,i_));
%                         ExtRotation(i,1)=acosd(dot(j_,e2_));
                        Varus(i,1)=90-beta(i,1);
                    else
                        ExtRotation(i,1)=asind(dot(e2_,i_));
%                         ExtRotation(i,1)=acosd(dot(j_,e2_));
                        Varus(i,1)=-(90-beta(i,1));
                    end


                %     H_=tibiaOrigin-femurOrigin;
                    H_(i,:)=[grti{i,1}(1:3)-grfi{i,1}(1:3)]';%translation vector of from femoral origin to tibial origin (in global coordinate frame)
                    if Right    
                        LatMed(i,1)=dot(H_(i,:),e1_);%projected onto the medial lateral axis e1
                    else
                        LatMed(i,1)=dot(H_(i,:),-e1_);
                    end
                    AntPost(i,1)=dot(H_(i,:),e2_);%projected onto the anterior posterior axis e2
                    DistComp(i,1)=-dot(H_(i,:),e3_);%projected onto the compression distraction axis e3, minus sign to make distraction +ve

                    TFangles(i,:) = [TFMatrixangles(i,1) Varus(i,1)  ExtRotation(i,1)];
                    TFtranslations(i,:) = [LatMed(i,1)  AntPost(i,1)  DistComp(i,1)];
                    
                    end
                    
%                     for PATELLOFERMORAL = 1
%                         [PFMatrixangles(i,:), PFMatrixtrans(i,:)] = rotationsAndTranslations(fTp{i}, Right);
% %                         Flexion(i,1)=asind(-dot(e2_,K_));
% %                     Flexion(i,1)=acosd(dot(J_,e2_));
%                     Pbeta(i,1)=acosd(dot(I_,pk_));
%                     if Right
%                         LatTilt(i,1)=asind(dot(-pe2_,pi_));
% %                         ExtRotation(i,1)=acosd(dot(j_,e2_));
%                         MedRot(i,1)=90-Pbeta(i,1);
%                     else
%                         LatTilt(i,1)=asind(dot(pe2_,pi_));
% %                         ExtRotation(i,1)=acosd(dot(j_,e2_));
%                         MedRot(i,1)=-(90-Pbeta(i,1));
%                     end
% 
% 
%                 %     pH_=patellaOrigin-femurOrigin;
%                     pH_(i,:)=[grpi{i,1}(1:3)-grfi{i,1}(1:3)]';%translation vector of from femoral origin to tibial origin (in global coordinate frame)
%                     if Right    
%                         Shift(i,1)=dot(pH_(i,:),e1_);%projected onto the medial lateral axis e1
%                     else
%                         Shift(i,1)=dot(pH_(i,:),-e1_);
%                     end
%                     pAntPost(i,1)=dot(pH_(i,:),pe2_);%projected onto the anterior posterior axis e2
%                     pDistComp(i,1)=-dot(pH_(i,:),pe3_);%projected onto the compression distraction axis e3, minus sign to make distraction +ve
% 
%                     PFangles(i,:) = [PFMatrixangles(i,1) MedRot(i,1)  LatTilt(i,1)];
%                     PFtranslations(i,:) = [Shift(i,1)  pAntPost(i,1)  pDistComp(i,1)];
%                     
%                     end
                    
                    
                    
%                  % Extract translations and rotations
%                  % For tibia/femur:
%                  [TFangles(i,:), TFtranslations(i,:)] = rotationsAndTranslations(fTt{i}, Right);
%                  TFq(i,:) = TFtranslations(i,1) + (TFtranslations(i,3) * cosd(beta(i,1)) );
%                  if ~Right
%                      TFq(i,1) = -TFq(i,1);
%                  end
%                 TFq(i,2) = TFtranslations(i,2);
%                 TFq(i,3) = -TFtranslations(i,3) - (TFtranslations(i,1) * cosd(beta(i,1)));
%                 
%                 For patella/femur:
           
%                  [PFangles(i,:), PFtranslations(i,:)] = rotationsAndTranslations(fTp{i}, Right);
%                  PFq(i,:) = PFtranslations(i,1) + (PFtranslations(i,3) * cosd(Pbeta(i,1)) );
%                  if ~Right
%                      PFq(i,1) = -PFq(i,1);
%                  end
%                 PFq(i,2) = PFtranslations(i,2);
%                 PFq(i,3) = -PFtranslations(i,3) - (PFtranslations(i,1) * cosd(Pbeta(i,1)));


 end
 
% Pin5toPin6 = find(diff(flagFemur) == 9);
% Pin6toPin5 = find(diff(flagFemur) == -9);
% Pin5only = find(flagFemur == 1);
% FemurSwitches = sort([Pin5toPin6; Pin6toPin5]);
% 
% Pin2toPin1 = find(diff(flagTibia) == -3);
% Pin1toPin2 = find(diff(flagTibia) == 3);
% Pin2only = find(flagTibia == 5);
% TibiaSwitches = sort([Pin2toPin1; Pin1toPin2]);
% 
% Patella3toPatella4 = find(diff(flagPatella) == 5);
% Patella4toPatella3 = find(diff(flagPatella) == -5);
% Patella3only = find(flagPatella == 20);
% PatellaSwitches = sort([Patella3toPatella4; Patella4toPatella3]);
% 
% TFSwitches = sort([FemurSwitches; TibiaSwitches]);
% PFSwitches = sort([FemurSwitches; PatellaSwitches]);
% 
% for tf = 1:length(TFSwitches)
%     
%     TFAngleDiffs = diff(TFangles);
%     TFAngleDiffs = TFAngleDiffs(TFSwitches(tf),:);
%     TFangles(TFSwitches(tf)+1:end,:) = TFangles(TFSwitches(tf)+1:end,:) - TFAngleDiffs;
%     
%     TFTransDiffs = diff(TFtranslations);
%     TFTransDiffs = TFTransDiffs(TFSwitches(tf),:);
%     TFtranslations(TFSwitches(tf)+1:end,:) = TFtranslations(TFSwitches(tf)+1:end,:) - TFTransDiffs;
%     
% end
% for pf = 1:length(PFSwitches)
% %     
%     PFAngleDiffs = diff(PFangles);
%     PFAngleDiffs = PFAngleDiffs(PFSwitches(pf),:);
%     PFangles(PFSwitches(pf)+1:end,:) = PFangles(PFSwitches(pf)+1:end,:) - PFAngleDiffs;
%     
%     PFTransDiffs = diff(PFtranslations);
%     PFTransDiffs = PFTransDiffs(PFSwitches(pf),:);
%     PFtranslations(PFSwitches(pf)+1:end,:) = PFtranslations(PFSwitches(pf)+1:end,:) - PFTransDiffs;
%     
% end




MinFlex = min(TFangles(:,1))
MaxFlex = max(TFangles(:,1))

% Flexion = TFangles(:,1) - MinFlex;






% 
% for K = 1:16
%     if K <10
%         Knees(K) = regexprep(join(["0",string(K)]),'\s+', '');
%         else
%         Knees(K) = string(K);
%     end
%     Lef(K) = ["L"];
%     Righ(K) = ["R"];
%     
% end
% KneesL = strcat(Knees,Lef);
% KneesR = strcat(Knees,Righ);
% Knees = [KneesL KneesR]';
% for J = 1:length(Knees)
%     KneeSearch = find(contains(FolderPath, Knees(J)));
%     if KneeSearch == 1
%         KneeNumber = char(Knees(J));
%     else
%     end
% end


%% PlotsFMed
% 
% 
figure
subplot(3, 1, 1)
plot(TFangles(:,1))
ylabel('Flexion')
subplot(3, 1, 2)
plot(TFangles(:,2))
ylabel('Tibial Varus')
subplot(3, 1, 3)
plot(TFangles(:,3))
ylabel('Tibial Ext Rotation')
sgtitle('Tibia angles relative to femur')

figure
subplot(3, 1, 1)
plot(TFtranslations(:,1))
ylabel('Lateral/medial')
subplot(3, 1, 2)
plot(TFtranslations(:,2))
ylabel('Anterior/posterior')
subplot(3, 1, 3)
plot(TFtranslations(:,3))
ylabel('DistComp')
sgtitle('Tibia translations relative to femur')

% Write output files
OutputPath = [pwd,'\',char(Knee), '\OutputFiles\' Test '.csv'];
Output = [TFangles TFtranslations];
% Output = fillmissing(Output,'spline');
% csvwrite(OutputPath, Output);





end

