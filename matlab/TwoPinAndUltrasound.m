clearvars -except M_
clc

Knee = "IK02_2022_12_12";

% ProbeName = 'New Biomech 4 LED Stylus Calibration Se x';
ProbeName = 'Isabelle Biomech 4 LED Stylus Calibrati x';

warning('off','MATLAB:illConditionedMatrix');

%% Set up Certus sampling frequency and state whether specimen is left or right knee

Freq = 100; % Certus sampling freq (Hz)

Right = true; % True for right knee, false for left knee

angle = @(u_,v_) acosd(dot(u_,v_)/(norm(u_,2)*norm(v_,2))); %define a function to calculate the angle between two vectors
ucross=@(u_,v_) cross(u_,v_)/norm(cross(u_,v_),2); %define function to find unit cross product
uvector=@(a,b) (b-a)/norm(b-a,2); %define a function to find a unit vector from a to b


%% Import marker and probe tip locations for inital digitising position

% Points are later used to define body specific coordinate systems,
% calculated from anatomical landmarks

% Probe points are from stationary condition, only the probe
% columns should differ between data files.

% Find folder path for knee tracking data files:

FolderPath = 'C:\Users\Isabelle\OneDrive - Imperial College London\Documents\Matlab\Certus_Tracking\IK02_2022_12_12';

% TIBIA

% CSP_T = {'TM1','TL1','TD1'};
% TP_T = {'TM1'};
CSP_T = {'TM_AD','TL_AD','TD_AD'}; %Coordinate System Points
TP1_T = {'TM_BD','TL_BD','TD_BD','TRP1_BD','TRP2_BD','TRP3_BD','TRP4_BD','TM_AD','TL_AD','TD_AD','TRP1_AD','TRP2_AD','TRP3_AD','TRP4_AD','TMCLAnterior','TMCLPosterior','TMCLAnterior2','TMCLPosterior2'}; %Tracking Points (Mean of Digitisation Recording)
TP2_T = {'MCLAnteriorFlexion1_AD','MCLAnteriorFlexion2_AD','MCLAnteriorExt1_AD','MCLAnteriorExt2_AD','MCLPosteriorFlexion1_AD','MCLPosteriorFlexion2_AD','MCLPosteriorExt1_AD','MCLPosteriorExt2_AD'}; %Tracking Points (All points in Digitisation Recording)
Marker = 'Pin1';

DigitisationPoints=[CSP_T TP1_T TP2_T];
for i=1:length(DigitisationPoints)
    
    d=dir(fullfile(FolderPath, [DigitisationPoints{i} '.*']));
    FileName = fullfile(FolderPath, d.name);
    [~,~,ext] = fileparts(FileName);
    if isequal(ext,'.csv')
        importoptions=delimitedTextImportOptions('EmptyLineRule','read','Delimiter',',');
        XYZdataT{i}= readcell(FileName,importoptions);
        XYZdataT{i}(cellfun(@(x) all(ismissing(x)), XYZdataT{i})) = {NaN};
    else
        XYZdataT{i}= readcell(FileName);
    end
    
    % Find position of digitised tibial landmarks by 'finding' the probe
    % columns in the respect digitised data files and extracting the points of
    % the tip of the probe
    
    RowIndex = 5;
    MarkerNames = string(XYZdataT{i}(RowIndex,:));
    ColumnIndex = find(contains(MarkerNames, ProbeName));
    
    %Digitised point in global frame of reference
    eval(['g_r_t' DigitisationPoints{i} '_tD = cell2mat(XYZdataT{i}(RowIndex+1:end,ColumnIndex:ColumnIndex+2));']);
    
    % Import position of tracking markers at digitisation time point
    % Same process as digitising the bony landmarks but instead of the probe
    % points, the trackers themselves are used.
    % Find position and quaternions, using XYZdata:
    
    ColumnIndex = find(contains(MarkerNames, [Marker ' x']));
    tMarker_XYZ = cell2mat(XYZdataT{i}(RowIndex+1:end, ColumnIndex:ColumnIndex+2));
    tMarker_Q = cell2mat(XYZdataT{i}(RowIndex+1:end, ColumnIndex-4:ColumnIndex-1));
    
    if i<=length(CSP_T)+length(TP1_T)
        eval(['g_r_t' DigitisationPoints{i} '_tD = mean(g_r_t' DigitisationPoints{i} '_tD,1);']);
        tMarker_XYZ = mean(tMarker_XYZ,1);
        tMarker_Q = mean(tMarker_Q,1);
        
    end
    
    clear tMarker_R
    for j = 1:size(tMarker_Q,1)
        tMarker_R(j,:) = quaternion2euler(tMarker_Q(j,:));
    end
    
    g_T_tMarker_tD = findTrackerFixedFrames_v2(tMarker_R, tMarker_XYZ);
    
    for j=1:length(g_T_tMarker_tD)
        % Digitised point in marker frame of reference
        eval(['tMarker_r_t' DigitisationPoints{i} '(:,j)= g_T_tMarker_tD{j}\transpose([g_r_t' DigitisationPoints{i} '_tD(j,:) 1]);']);
    end
    
    
end

%Define coordinate systems for tibia in the tibia marker frame of reference using digitised points
eval(['[tMarker_T_t, origin_t] = defineBodyFixedFrameTibia_v2(transpose(tMarker_r_t' CSP_T{1} '(1:3)), transpose(tMarker_r_t' CSP_T{2} '(1:3)), transpose(tMarker_r_t' CSP_T{3} '(1:3)), Right);']);


% FEMUR

% CSP_F = {'FM1','FL1','FP1'};
% TP_F = {'FM1'};
CSP_F = {'FM_AD','FL_AD','FP_AD'}; %Coordinate System Points
TP1_F = {'FM_BD','FL_BD','FP_BD','FRP1_BD','FRP2_BD','FRP3_BD','FRP4_BD','FM_AD','FL_AD','FP_AD','FRP1_AD','FRP2_AD','FRP3_AD','FRP4_AD','FMCLAnterior','FMCLPosterior'}; %Tracking Points (Mean of Digitisation Recording)
TP2_F = {}; %Tracking Points (All points in Digitisation Recording)
Marker = 'Pin2';
DigitisationPoints=[CSP_F TP1_F TP2_F];

for i=1:length(DigitisationPoints)
    d=dir(fullfile(FolderPath, [DigitisationPoints{i} '.*']));
    FileName = fullfile(FolderPath, d.name);
    [~,~,ext] = fileparts(FileName);
    if isequal(ext,'.csv')
        importoptions=delimitedTextImportOptions('EmptyLineRule','read','Delimiter',',');
        XYZdataF{i}= readcell(FileName,importoptions);
        XYZdataF{i}(cellfun(@(x) all(ismissing(x)), XYZdataF{i})) = {NaN};
    else
        XYZdataF{i}= readcell(FileName);
    end
    
    % Find position of digitised tibial landmarks by 'finding' the probe
    % columns in the respect digitised data files and extracting the points of
    % the tip of the probe
    
    RowIndex = 5;
    MarkerNames = string(XYZdataF{i}(RowIndex,:));
    ColumnIndex = find(contains(MarkerNames, ProbeName));
    
    %Digitised point in global frame of reference
    eval(['g_r_f' DigitisationPoints{i} '_tD = cell2mat(XYZdataF{i}(RowIndex+1:end,ColumnIndex:ColumnIndex+2));']);
    
    % Import position of tracking markers at digitisation time point
    % Same process as digitising the bony landmarks but instead of the probe
    % points, the trackers themselves are used.
    % Find position and quaternions, using XYZdata:
    
    ColumnIndex = find(contains(MarkerNames, [Marker ' x']));
    fMarker_XYZ = cell2mat(XYZdataF{i}(RowIndex+1:end, ColumnIndex:ColumnIndex+2));
    fMarker_Q = cell2mat(XYZdataF{i}(RowIndex+1:end, ColumnIndex-4:ColumnIndex-1));
    
    if i<=length(CSP_F)+length(TP1_F)
        eval(['g_r_f' DigitisationPoints{i} '_tD = mean(g_r_f' DigitisationPoints{i} '_tD,1);']);
        fMarker_XYZ = mean(fMarker_XYZ,1);
        fMarker_Q = mean(fMarker_Q,1);
        
    end
    
    clear fMarker_R
    for j = 1:size(fMarker_Q,1)
        fMarker_R(j,:) = quaternion2euler(fMarker_Q(j,:));
    end
    
    
    %Define fixed frame for each of the markers in global coordinates at
    %digitisation time point
    g_T_fMarker_tD = findTrackerFixedFrames_v2(fMarker_R, fMarker_XYZ);
    
    for j=1:length(g_T_fMarker_tD)
        % Digitised point in marker frame of reference
        eval(['fMarker_r_f' DigitisationPoints{i} '(:,j)= g_T_fMarker_tD{j}\transpose([g_r_f' DigitisationPoints{i} '_tD(j,:) 1]);']);
    end
    
end

%Define coordinate systems for femur in the femur marker frame of reference using digitised points
eval(['[fMarker_T_f, origin_f] = defineBodyFixedFrameFemur_v2(transpose(fMarker_r_f' CSP_F{1} '(1:3)), transpose(fMarker_r_f' CSP_F{2} '(1:3)), transpose(fMarker_r_f' CSP_F{3} '(1:3)), Right);']);

% ULTRASOUND PROBE

% DigitisationPoints = {'USB3','USL2','UST1','UST2','UST3','USR2'};
% DigitisationPoints = {'USB1','USB2','USL1','USL2','UST1','UST2','USR1','USR2'};
TP_US={'Corners','Line','OriginalDPs'};
Marker = 'US';

CSP_US = {}; %Coordinate System Points
TP1_US = {}; %Tracking Points (Mean of Digitisation Recording)
TP2_US = {'USB1_AD','USB2_AD','UST1_AD','UST2_AD','USR1_BD','USR2_BD','USL1_AD','USL2_AD','USL1_BD','USL2_BD'}; %Tracking Points (All points in Digitisation Recording)
% TP2_US = {'USB1_AD','USB2_AD','USB1_BD','USB2_BD','UST1_AD','UST2_AD','UST1_BD','UST2_BD','USR1_BD','USR2_BD','USR1_AD','USR2_AD','USL1_AD','USL2_AD','USL1_BD','USL2_BD'}; %Tracking Points (All points in Digitisation Recording)
% TP1_US ={};
% TP2_US = {'USB1','USB2','USB3','USB_Surface_1','USB_Surface_2','UST1','UST2','UST3','UST4','UST_Surface_1','UST_Surface_2','USL1','USR1','USP1','USP2','USP3','USP4','USP5','USP6'};
Marker = 'US';
DigitisationPoints=[CSP_US TP1_US TP2_US];
figure
for i=1:length(DigitisationPoints)
    d=dir(fullfile(FolderPath, [DigitisationPoints{i} '.*']));
    FileName = fullfile(FolderPath, d.name);
    [~,~,ext] = fileparts(FileName);
    if isequal(ext,'.csv')
        importoptions=delimitedTextImportOptions('EmptyLineRule','read','Delimiter',',');
        XYZdataU{i}= readcell(FileName,importoptions);
        XYZdataU{i}(cellfun(@(x) all(ismissing(x)), XYZdataU{i})) = {NaN};
    else
        XYZdataU{i}= readcell(FileName);
    end
    
    % Find position of digitised tibial landmarks by 'finding' the probe
    % columns in the respect digitised data files and extracting the points of
    % the tip of the probe
    
    RowIndex = 5;
    MarkerNames = string(XYZdataU{i}(RowIndex,:));
    ColumnIndex = find(contains(MarkerNames, ProbeName));
    
    %Digitised point in global frame of reference
    eval(['g_r_u' DigitisationPoints{i} '_tD = cell2mat(XYZdataU{i}(RowIndex+1:end,ColumnIndex:ColumnIndex+2));']);
    
    % Import position of tracking markers at digitisation time point
    % Same process as digitising the bony landmarks but instead of the probe
    % points, the trackers themselves are used.
    % Find position and quaternions, using XYZdata:
    
    ColumnIndex = find(contains(MarkerNames, [Marker ' x']));
    uMarker_XYZ = cell2mat(XYZdataU{i}(RowIndex+1:end, ColumnIndex:ColumnIndex+2));
    uMarker_Q = cell2mat(XYZdataU{i}(RowIndex+1:end, ColumnIndex-4:ColumnIndex-1));
    
    if i<=length(CSP_US)+length(TP1_US)
        eval(['g_r_u' DigitisationPoints{i} '_tD = nanmean(g_r_u' DigitisationPoints{i} '_tD,1);']);
        uMarker_XYZ = mean(uMarker_XYZ,1);
        uMarker_Q = mean(uMarker_Q,1);
        
    end
    
    clear uMarker_R
    for j = 1:size(uMarker_Q,1)
        uMarker_R(j,:) = quaternion2euler(uMarker_Q(j,:));
    end
    
    
    %Define fixed frame for each of the markers in global coordinates at
    %digitisation time point
    g_T_uMarker_tD = findTrackerFixedFrames_v2(uMarker_R, uMarker_XYZ);
    
    for j=1:length(g_T_uMarker_tD)
        % Digitised point in marker frame of reference
        eval(['uMarker_r_u' DigitisationPoints{i} '(:,j)= g_T_uMarker_tD{j}\transpose([g_r_u' DigitisationPoints{i} '_tD(j,:) 1]);']);
    end
    
    eval(['scatter3(uMarker_r_u' DigitisationPoints{i} '(1,:), uMarker_r_u' DigitisationPoints{i} '(2,:), uMarker_r_u' DigitisationPoints{i} '(3,:), ".");']);
    hold on;
end

TP2_US(end+1:end+3)={'Corners','Line','OriginalDPs'};

wkspc=who;
UST=eval(['[' sprintf('transpose(%s); ',wkspc{contains(wkspc,'uMarker_r_uUST')}) ']']);
USB=eval(['[' sprintf('transpose(%s); ',wkspc{contains(wkspc,'uMarker_r_uUSB')}) ']']);
USR=eval(['[' sprintf('transpose(%s); ',wkspc{contains(wkspc,'uMarker_r_uUSR')}) ']']);
USL=eval(['[' sprintf('transpose(%s); ',wkspc{contains(wkspc,'uMarker_r_uUSL')}) ']']);

[uMarker_r_uCorners, uMarker_r_uLine] = findUltrasoundTrackingPoints({UST,USB,USL,USR});
uMarker_r_uOriginalDPs=[UST;USB;USR;USL]';

[uMarker_T_u, origin_u]=defineBodyFixedFrameUS(uMarker_r_uCorners);
% u_t_uMarker=inv(uMarker_T_u);
% v=uMarker_T_u(:,3);
% uMarker_r_uCorners=uMarker_r_uCorners+1.5*repmat(v,1,size(uMarker_r_uCorners,2));
% uMarker_r_uLine=uMarker_r_uLine+1.5*repmat(v,1,size(uMarker_r_uLine,2));
% uMarker_T_u(:,4)=uMarker_T_u(:,4)+1.5*v;
% uMarker_r_uOriginalDPs=uMarker_r_uOriginalDPs+1.5*repmat(v,1,size(uMarker_r_uOriginalDPs,2));

USProbePosition=[];
T_=[];
w=who;
fig1=[];

load(fullfile(FolderPath,'MCL_Insertion_Points'))
% load(fullfile(FolderPath,'OutputFolder','findMCLInsertionPoints2'));
% % tMarker_r_tMCLinsertion= [-105;69;-2;1];
% tMarker_r_tMCLinsertion= [-85;61;6;1];
% % fMarker_r_fMCLinsertion= [70;-125;-90;1];
% fMarker_r_fMCLinsertion= [70;-135;-100;1];
% [X Y Z]=ndgrid(-120:5:-80,55:2:70,-10:2:10);
% tMarker_r_tGrid= [X(:)'; Y(:)'; Z(:)'];
% tMarker_r_tGrid= [tMarker_r_tGrid; ones(1,size(tMarker_r_tGrid,2))];
% 
% [X Y Z]=ndgrid(40:5:70,-135:5:-100,-105:5:-80);
% fMarker_r_fGrid= [X(:)'; Y(:)'; Z(:)'];
% fMarker_r_fGrid= [fMarker_r_fGrid; ones(1,336)];

%%

% Tests = ["Test1" ,"Test2","Test3","Test4","Test5","Test6","Test7","Test8","Test9","Test10","Test11","Test12"];
d=dir(fullfile(FolderPath,'USFE*'));
a={d.name};
a=a([1:7 11:15 17:26]);
% a=a([1 4 12 14 15 17 20 23 24 26]);
% a=a(4);
% a=a(10);
[~,Tests,~]=cellfun(@fileparts,a,'UniformOutput',false);
% Tests={};
% Tests=["USonMCL3_AD"];
% Tests={'MCLPosteriorExt2_AD'};
% Tests={'MCLAnteriorExt2_AD'};

c1=jet(1000);
strain=[0 2.4075   -1.4458   -2.5561   -1.5182    1.2337    1.1031   -2.4060    1.5094   -1.2541];
strain=[0 1 2 3 4 5 6 7 8 9];
c=round((strain-min(strain))/max(strain-min(strain))*999)+1;


fig=figure('color','w','WindowState','maximized');

for T_ = 1:length(Tests)
        
    d=dir(fullfile(FolderPath, [Tests{T_} '.*']));
    FileName = fullfile(FolderPath, d.name);
    [~,~,ext] = fileparts(FileName);
    if isequal(ext,'.csv')
        importoptions=delimitedTextImportOptions('EmptyLineRule','read','Delimiter',',');
        XYZdata= readcell(FileName,importoptions);
        XYZdata(cellfun(@(x) all(ismissing(x)), XYZdata)) = {NaN};
    else
        XYZdata= readcell(FileName);
        XYZdata(cellfun(@(x) all(ismissing(x)), XYZdata)) = {''};
    end
    
    RowIndex = 5;
    MarkerNames = string(XYZdata(RowIndex,:));
    
    %Find pin position, quaternion and error columns
    
    % Tibia Marker
    Marker='Pin1';
    ColumnIndex = find(contains(MarkerNames, [Marker ' x']));
    tMarker_XYZ = cell2mat(XYZdata(RowIndex+1:end, ColumnIndex:ColumnIndex+2));
    tMarker_Q = cell2mat(XYZdata(RowIndex+1:end, ColumnIndex-4:ColumnIndex-1));
    clear tMarker_R
    for i = 1:size(tMarker_Q,1)
        tMarker_R(i,:) = quaternion2euler(tMarker_Q(i,:));
    end
    
    % Femur Marker
    Marker='Pin2';
    ColumnIndex = find(contains(MarkerNames, [Marker ' x']));
    fMarker_XYZ = cell2mat(XYZdata(RowIndex+1:end, ColumnIndex:ColumnIndex+2));
    fMarker_Q = cell2mat(XYZdata(RowIndex+1:end, ColumnIndex-4:ColumnIndex-1));
    clear fMarker_R
    for i = 1:size(fMarker_Q,1)
        fMarker_R(i,:) = quaternion2euler(fMarker_Q(i,:));
    end
    
    % Ultrasound Marker
    Marker='US';
    ColumnIndex = find(contains(MarkerNames, [Marker ' x']));
    uMarker_XYZ = cell2mat(XYZdata(RowIndex+1:end, ColumnIndex:ColumnIndex+2));
    uMarker_Q = cell2mat(XYZdata(RowIndex+1:end, ColumnIndex-4:ColumnIndex-1));
    clear uMarker_R
    for i = 1:size(uMarker_Q,1)
        uMarker_R(i,:) = quaternion2euler(uMarker_Q(i,:));
    end
    
    % Create matrices of tracker marker position and rotations in time
    
    % Tibia Marker:
    [g_T_tMarker_ti] = findTrackerFixedFrames_v2(tMarker_R, tMarker_XYZ);
    % Femur Marker
    [g_T_fMarker_ti] = findTrackerFixedFrames_v2(fMarker_R, fMarker_XYZ);
    % Ultrasound Marker
    [g_T_uMarker_ti] = findTrackerFixedFrames_v2(uMarker_R, uMarker_XYZ);
    
    frames=1:1000;
    [TFangles{T_}, TFtranslations{T_}]=calculateKinematics(g_T_tMarker_ti,g_T_fMarker_ti,tMarker_T_t,fMarker_T_f,Right,frames);
    [~,extension_ind]=min(TFangles{T_}(:,1));
%     [~, F_frames]=findpeaks(TFangles{T}(:,1),'MinPeakHeight',60,'MinPeakDistance',300);
%     [~, E_frames]=findpeaks(-TFangles{T}(:,1),'MinPeakHeight',-10,'MinPeakDistance',300);
%     figure
%     findpeaks(-TFangles{T}(:,1),'MinPeakHeight',-10,'MinPeakDistance',300);
    frames=1:1000;
    
    %     FOR='tMarker';
    %     TP={'tTD_BD','tTM_BD','tTL_BD','tTRP1_BD','tTRP2_BD','tTRP3_BD','tTRP4_BD','tTD_AD','tTM_AD','tTL_AD','tTRP1_AD','tTRP2_AD','tTRP3_AD','tTRP4_AD'};
    FOR='t';
%     FOR='t';
%     TP={'tMCLPosteriorExt2_AD','tFitPointsA','tFitPointsP','fFitPointsA','fFitPointsP'};
%     TP={'tMCLAnteriorFlexion1_AD','tMCLAnteriorFlexion2_AD','tMCLPosteriorFlexion1_AD','tMCLPosteriorFlexion2_AD','fFMCLAnterior','tTMCLAnterior','tTMCLPosterior','fFMCLPosterior'};
%     TP={'tMCLPosteriorExt1_AD','tMCLPosteriorExt2_AD','tMCLAnteriorExt1_AD','tMCLAnteriorExt2_AD','tTM_AD','tTM_BD','fFM_AD','fFM_BD','fGrid','tGrid','fMCLinsertion','tMCLinsertion'};
%     TP={'fFMCLAnterior','tTMCLAnterior','tTMCLPosterior','fFMCLPosterior','uCorners','uLine','tTM_AD','fFM_AD'};
    TP={'fFMCLAnterior','tTMCLAnterior','tTMCLPosterior','fFMCLPosterior'};

    TwoD=1;
    trackPoints(FOR,TP,frames,g_T_tMarker_ti,g_T_fMarker_ti,g_T_uMarker_ti,tMarker_T_t,fMarker_T_f,uMarker_T_u)
    findMCLIntermediatePoint(FOR);
    [AStrain PStrain]=findMCLStrain(FOR,extension_ind);
    AStrainMax(T_)=max(AStrain);
    PStrainMin(T_)=min(PStrain);
    TP=[TP,'MCLAnteriorInter','MCLPosteriorInter'];
%     [percentagePosition(:,T) ang(:,T)]=find_US_MCL_relativePosition(u_r_fFMCLAnterior_ti,u_r_tTMCLAnterior_ti,u_r_fFMCLPosterior_ti,u_r_tTMCLPosterior_ti,u_r_uLine_ti);
%     [percentagePosition(:,:,T), ang(:,:,T)]=find_US_MCL_relativePosition(u_r_fFMCLAnterior_ti,u_r_MCLAnteriorInter_ti,u_r_tTMCLAnterior_ti,u_r_fFMCLPosterior_ti,u_r_MCLPosteriorInter_ti,u_r_tTMCLPosterior_ti,u_r_uLine_ti);
    
    Points=cellfun(@(x) x([2 1 3 4],:,:),eval(['{' sprintf(['cat(3, ' FOR '_r_%s_ti{:}) '],TP{:}), '}']),'UniformOutput', false);
%     Points={Points{1},Points{2},Points{3},Points{4} [Points{5}, Points{9},Points{6}], [Points{7},Points{10}, Points{8}]};
%     plottype={'.','.','.','.','-','-'};
%     Points={Points{1}, [Points{2}, Points{6},Points{3}], [Points{4}, Points{7},Points{5}]};
    plottype={'-','-'};
    Points={[Points{1}, Points{5},Points{2}], [Points{3}, Points{6},Points{4}]};
    
%     Points={[Points{1}, Points{9},Points{2}], [Points{3}, Points{10},Points{4}],Points{5},Points{6},[Points{7}, Points{8}]};
%     plottype={'-.','-.','.','-','x'};
%     legend_=TP;
    legend_={'MCL Anterior', 'MCL Posterior', 'US Corners','US Line'};
    title_={};
%     title_=[Tests{T} ' (Frame of Reference = ' FOR ', AP Position = ' num2str(percentagePosition(:,T)*100,'%.1f') '%, AP Angle = ' num2str(ang(:,T),'%.1f') ')'];
%     [M fig]=plotTrackingPoints(Points,plottype,TwoD,title_,legend_,c1(c(T_),:));
%     
%     folder='C:\Users\Isabelle\OneDrive - Imperial College London\Documents\Matlab\Certus_Tracking\IK02_2022_12_12\OutputFolder\MCL_and_US_GIF';
%     savename=fullfile(folder,[Tests{T}]);
%     saveGIF(savename,M,0.1);
%     saveas(fig,savename);
%     exportgraphics(fig,[savename '.png'],'Resolution',600);


for i=1:1000
F=t_r_fFMCLAnterior_ti{i};
T=t_r_tTMCLAnterior_ti{i};
ang(i)=atand((F(2)-T(2))/(F(3)-T(3)));
end
angA(:,T_)=interp1(TFangles{T_}(:,1),ang,-20:0.5:80);
% plot(TFangles{T_}(:,1),ang)
% hold on
for i=1:1000
F=t_r_fFMCLPosterior_ti{i};
T=t_r_tTMCLPosterior_ti{i};
ang(i)=atand((F(2)-T(2))/(F(3)-T(3)));

end
angP(:,T_)=interp1(TFangles{T_}(:,1),ang,-20:0.5:80);
% plot(TFangles{T_}(:,1),ang)
% drawnow
end

    FlexionAngles=cellfun(@(x) x(:,1),TFangles,'UniformOutput',false);
    FlexionAngles=cat(2,FlexionAngles{:});
    FlexionAngles=FlexionAngles-mean(min(FlexionAngles));
    
%     fig1=plotFlexion(TFangles);


%%
for i = 1:length(g_T_tMarker_ti)
    
    % Body fixed relative to global
    g_T_t_ti{i,1} = g_T_tMarker_ti{i,1}*tMarker_T_t;
    g_T_f_ti{i,1} = g_T_fMarker_ti{i,1}*fMarker_T_f;
    g_T_u_ti{i,1} = g_T_uMarker_ti{i,1}*uMarker_T_u;
    
    % Tibia relative to femur:
    f_T_t_ti{i,1} = g_T_f_ti{i,1}\g_T_t_ti{i,1};
    f_R_t_ti{i,1}=f_T_t_ti{i,1}(1:3,1:3); %rotation matrix
    
    %   Ultrasound points relative to US, femur,tibia and global:
    TP_US=[TP1_US TP2_US];
    for j=1:length(TP_US)
        eval(['g_r_u' TP_US{j} '_ti{i,1} = g_T_uMarker_ti{i,1}*uMarker_r_u' TP_US{j} ';']);
        eval(['f_r_u' TP_US{j} '_ti{i,1} = g_T_f_ti{i,1}\g_r_u' TP_US{j} '_ti{i,1};']);
        eval(['t_r_u' TP_US{j} '_ti{i,1} = g_T_t_ti{i,1}\g_r_u' TP_US{j} '_ti{i,1};']);
        eval(['u_r_u' TP_US{j} '_ti{i,1} = uMarker_T_u\uMarker_r_u' TP_US{j} ';']);
    end
    
    %   Tibia points relative to US, femur and global:
    TP_T=[TP1_T TP2_T];
    for j=1:length(TP_T)
        eval(['g_r_t' TP_T{j} '_ti{i,1} = g_T_tMarker_ti{i,1}*tMarker_r_t' TP_T{j} ';']);
        eval(['f_r_t' TP_T{j} '_ti{i,1} = g_T_f_ti{i,1}\g_r_t' TP_T{j} '_ti{i,1};']);
        eval(['u_r_t' TP_T{j} '_ti{i,1} = g_T_u_ti{i,1}\g_r_t' TP_T{j} '_ti{i,1};']);
    end
    
    %   Femur points relative to US, femur and global :
    TP_F=[TP1_F TP2_F];
    for j=1:length(TP_F)
        eval(['g_r_f' TP_F{j} '_ti{i,1} = g_T_fMarker_ti{i,1}*fMarker_r_f' TP_F{j} ';']);
        eval(['f_r_f' TP_F{j} '_ti{i,1} = fMarker_T_f\fMarker_r_f' TP_F{j} ';']);
        eval(['u_r_f' TP_F{j} '_ti{i,1} = g_T_u_ti{i,1}\g_r_f' TP_F{j} '_ti{i,1};']);
    end
    
    
    %Begin richard new version (v5)
    
    I_=g_T_f_ti{i,1}(1:3,1);%Femoral X axis unit vector, Grood and Suntay definition
    J_=g_T_f_ti{i,1}(1:3,2);%Femoral Y axis unit vector, Grood and Suntay definition
    K_=g_T_f_ti{i,1}(1:3,3);%Femoral Z axis unit vector, Grood and Suntay definition
    
    i_=g_T_t_ti{i,1}(1:3,1);%Tibial x axis unit vector, Grood and Suntay definition
    j_=g_T_t_ti{i,1}(1:3,2);%Tibial y axis unit vector, Grood and Suntay definition
    k_=g_T_t_ti{i,1}(1:3,3);%Tibial z axis unit vector, Grood and Suntay definition
    
    
    e1_=I_;%Femoral X axis in global reference frame, Grood and Suntay definition
    e3_=k_;%Tibial z axis in global reference frame, Grood and Suntay definition
    %                     pe3_=pk_;
    e2_=ucross(e3_,e1_);%Floating axis in global reference frame, Grood and Suntay definition
    %                     pe2_=ucross(pe3_,e1_);
    
    for TIBIOFEMORAL = 1
        [TFMatrixangles(i,:), TFMatrixtrans(i,:)] = rotationsAndTranslations(f_T_t_ti{i}, Right);
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
        
        tibiaOrigin=g_T_t_ti{i,1}(1:3,end);
        femurOrigin=g_T_f_ti{i,1}(1:3,end);
        H_(i,:)=tibiaOrigin-femurOrigin;
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
    
    
end

MinFlex = min(TFangles(:,1));
MaxFlex = max(TFangles(:,1));

USProbePosition(:,:,T_)=t_r_uLine_ti{end};
% Flexion = TFangles(:,1) - MinFlex;

%%
function [TFangles, TFtranslations]=calculateKinematics(g_T_tMarker_ti,g_T_fMarker_ti,tMarker_T_t,fMarker_T_f,Right,frames)

angle = @(u_,v_) acosd(dot(u_,v_)/(norm(u_,2)*norm(v_,2))); %define a function to calculate the angle between two vectors
ucross=@(u_,v_) cross(u_,v_)/norm(cross(u_,v_),2); %define function to find unit cross product
uvector=@(a,b) (b-a)/norm(b-a,2); %define a function to find a unit vector from a to b

for i = 1:length(frames)
    
    % Body fixed relative to global
    g_T_t_ti{frames(i),1} = g_T_tMarker_ti{frames(i),1}*tMarker_T_t;
    g_T_f_ti{frames(i),1} = g_T_fMarker_ti{frames(i),1}*fMarker_T_f;
    
    % Tibia relative to femur:
    f_T_t_ti{frames(i),1} = g_T_f_ti{frames(i),1}\g_T_t_ti{frames(i),1};
    
    %Begin richard new version (v5)
    
    I_=g_T_f_ti{frames(i),1}(1:3,1);%Femoral X axis unit vector, Grood and Suntay definition
    J_=g_T_f_ti{frames(i),1}(1:3,2);%Femoral Y axis unit vector, Grood and Suntay definition
    K_=g_T_f_ti{frames(i),1}(1:3,3);%Femoral Z axis unit vector, Grood and Suntay definition
    
    i_=g_T_t_ti{frames(i),1}(1:3,1);%Tibial x axis unit vector, Grood and Suntay definition
    j_=g_T_t_ti{frames(i),1}(1:3,2);%Tibial y axis unit vector, Grood and Suntay definition
    k_=g_T_t_ti{frames(i),1}(1:3,3);%Tibial z axis unit vector, Grood and Suntay definition
    
    
    e1_=I_;%Femoral X axis in global reference frame, Grood and Suntay definition
    e3_=k_;%Tibial z axis in global reference frame, Grood and Suntay definition
    e2_=ucross(e3_,e1_);%Floating axis in global reference frame, Grood and Suntay definition
    
    [TFMatrixangles(i,:), TFMatrixtrans(i,:)] = rotationsAndTranslations(f_T_t_ti{frames(i)}, Right);
    beta(i,1)=acosd(dot(I_,k_));
    if Right
        ExtRotation(i,1)=asind(dot(-e2_,i_));
        Varus(i,1)=90-beta(i,1);
    else
        ExtRotation(i,1)=asind(dot(e2_,i_));
        Varus(i,1)=-(90-beta(i,1));
    end
    
    tibiaOrigin=g_T_t_ti{frames(i),1}(1:3,end);
    femurOrigin=g_T_f_ti{frames(i),1}(1:3,end);
    H_(i,:)=tibiaOrigin-femurOrigin;
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
TFangles=fillmissing(TFangles,'spline');
end

function fig1=plotFlexion(TFangles)

fig1=figure('color','w','visible','on');

for i=1:length(TFangles)
    plot(TFangles{i}(:,1))
    hold on
end
ylabel('Flexion (°)')
ylim([-30 110]);
grid on
title('Knee Flexion Angle')

end



function [M fig]=plotTrackingPoints(TP,plottype,TwoD,title_,legend_,c) %f_r_uLine_ti,f_r_uCorners_ti,f_r_fFM_ti,f_r_tTM_ti)

fig=[];
% c=hsv(length(TP));
%     c=c([1 5 7],:);

% fig=figure('color','w','WindowState','maximized');
drawnow
pause(0.5)
%     set(fig,'visible','off')

for i=1:length(TP)
    if isequal(plottype{i},'fill')
        p(i)=fill3(TP{i}(1,:,1),TP{i}(2,:,1),TP{i}(3,:,1),c,'FaceAlpha',0.7);
    else
        p(i)=plot3(TP{i}(1,:,1),TP{i}(2,:,1),TP{i}(3,:,1),plottype{i},'LineWidth',1.2,'Color',c,'MarkerSize',8);
        hold on
    end
end

axis equal
A=cat(2,TP{:});
axis([min(A(1,:,:),[],'all') max(A(1,:,:),[],'all') min(A(2,:,:),[],'all') max(A(2,:,:),[],'all') min(A(3,:,:),[],'all') max(A(3,:,:),[],'all')])
%     ylabel('Up/Down along probe (mm)');
%     xlabel('Anterior (-ve)/Posterior (+ve) (mm)');
grid on
title(title_, 'Interpreter', 'none');
legend(legend_, 'Interpreter', 'none','Location','southoutside');
if TwoD
%     view(-30,10)
% view(270,0)
view(0,0)
end
% set(gca,'xdir','reverse','zdir','reverse')
set(gca,'xdir','reverse')
drawnow
for i=1:size(TP{1},3)
    for j=1:length(p)
        try
            set(p(j),'XData',TP{j}(1,:,i),'YData',TP{j}(2,:,i),'ZData',TP{j}(3,:,i));
        catch
            set(p(j),'Vertices',(squeeze(TP{j}(1:3,:,i))'));
        end
    end
    M(i)=getframe(gcf);
end

end
