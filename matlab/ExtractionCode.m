clear all
close all
clc
%% Find min flexion angle for knee

Knee = 'X1-1-1';

FolderPath = [pwd,'\', Knee '\OutputFiles'];

Files = struct2cell(dir(FolderPath));
ExtFiles = string(Files(1,:));
ExtFiles = (ExtFiles(1, find(contains(ExtFiles, 'FE'))))';

for i = 1:length(ExtFiles)
    
    FileName = [FolderPath,ExtFiles(i,1)];
    FileName = join(FileName,'\');
    ExtFlex = xlsread(FileName);
    MinFlex(i) = min(ExtFlex(:,1));    
    
end

MeanMinFlex = mean(MinFlex);

% Adjust motions to whichever angle you want for full extension/flexion

% Choose FlexionDatum from literature
FlexionDatum = 43;

FlexionOffset = MeanMinFlex - FlexionDatum;

% Define plot increment size
Increment = 2;

TestNames = ExtFiles;

for j =1:length(ExtFiles)
    
       FileName = [FolderPath,'\', char(TestNames(j,1))];
       Motions{j,1} = xlsread(FileName);
 
       % Subtract mean min flex to 'zero' flexion angle column
       AlteredMotions{j,1} = [(Motions{j,1}(:,1) - FlexionOffset) , Motions{j,1}(:,2:end)];

       MaxFlexAngle = max(AlteredMotions{j,1}(:,1));
       MinFlexAngle = FlexionDatum;
       
       for K = [MinFlexAngle (((ceil((MinFlexAngle+0.0001)/Increment)))*Increment):Increment:(((floor((MaxFlexAngle+0.0001)/Increment)))*Increment)  floor(MaxFlexAngle)];
           
          % Find indices where flexion is equal to each chosen extraction angle below some tolerance (currently 0.75) 
          AngleIndices{K,1} = find(abs(AlteredMotions{j,1}(:,1) - K) < 0.75) ;
          AngleIndices{K,1} = [AngleIndices{K,1}; 1000000];
          DiffAngleIndices{K,1} = diff(AngleIndices{K,1});
          
          Groups{K,1} = find(DiffAngleIndices{K,1}(:,:) > 1);
          Groups{K,1} = Groups{K,1}(2:end) - (diff(Groups{K,1}))/2;
          Groups{K,1} = round(Groups{K,1});
          
          % Returns every index in altered motions where flexion angle is equal to k
          AngleIndices{K,1} = AngleIndices{K,1}(Groups{K,1});
          
          SelectedMotions{K,1} = mean(AlteredMotions{j,1}(AngleIndices{K,1},:));
           
       end
       
       SeparatedMotions{j,1} = SelectedMotions(~cellfun(@isempty,SelectedMotions));
       SeparatedMotions{j,1} = vertcat(SeparatedMotions{j,1}{:,1});
       
    
end

MotionTitles = ["Flexion", "Varus", "Ext Rot", "LatMed", "AntPost", "DistComp"];

for i = 1:length(SeparatedMotions)
    
    for j = 2:length(MotionTitles)
        figure
        plot( SeparatedMotions{i,1}(:,1) , SeparatedMotions{i,1}(:,j));
        title([ Knee, ' Test ', char(ExtFiles(i)), ' ', char(MotionTitles(j)) ])
        xlabel('Flexion Angle')
        ylabel(MotionTitles(j))
    end
    
    
    if i == 1
         SumMotions = SeparatedMotions{i,1};
    else
        SumMotions =  SumMotions + SeparatedMotions{i,1};
    end
    
end

AvgMotions = SumMotions/length(SeparatedMotions);

for j =2:size(AvgMotions,2)
    
        figure
        plot( AvgMotions(:,1) , AvgMotions(:,j));
        title(['Average Across Tests for Knee ', Knee,' ' char(MotionTitles(j)) ])
        xlabel('Flexion Angle')
        ylabel(MotionTitles(j))



end



