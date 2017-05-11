clear all;
close all;
clc

makePlots = true;

% Load Data
%load('RAL2017/constantVelFlightMomentumControl.mat');
load('RAL2017/constantVelFlightOrientationControl.mat');
%load('RAL2017/elicoidalFlightOrientationControlRobust.mat');

plotOrientationError = false;

plotOnlyOrientationError = false;


fontsize  = 12;
set(0,'defaultaxesfontsize',fontsize);

lineWidth = 1.5;

jointTorquesAndVelFigureDimension = [560, 600];

%%
time                         = jointTorques.time;

%Load joint torques
for j = 1:5
    jointTorquesParts{j}      = jointTorques.signals(j).values;
end
jointTorques   = jointTorquesParts;

%Load joint veocities star
for j = 1:5
    jointVelocitiesStarParts{j}      = jointVelocitiesStar.signals(j).values;
end
jointVelocitiesStar   = jointVelocitiesStarParts;
    

% Load com des

comDes = squeeze(comDes.Data);

% Load Jet intensities dot

jetsDot = squeeze(jetIntensitiesDot.Data);

% Load Jet intensities 

jets = squeeze(jetsIntensitiesByWeight.Data);

% Load com error

comError = squeeze(comError.Data);

% Load angular momentum

angMom   = squeeze(angularMomentum.Data);

% Load orientation error 

oriErr   = squeeze(orientationError.Data);

% norm vel error

normVerror  = squeeze(normVelError.Data);





%% PLOT DATA

if makePlots
    
    if ~plotOnlyOrientationError
    
        figure(1);

        % Plot com des and jets info
        subplot(3,1,1);
        plot(time(startingSample:end,:),comDes(:,startingSample:end),'LineWidth',lineWidth);
        title('Desired center of mass: $r$','Interpreter','latex');
        l = legend('$r_x$','$r_y$','$r_z$');
        set(l,'Interpreter','latex')
        ylabel('[$m$]','Interpreter','latex') % y-axis label
        %
        subplot(3,1,2);
        plot(time(startingSample:end,:),jetsDot(startingSample:end,:),'LineWidth',lineWidth);
        title('Jet intensity rate-of-change: $u^*_1$','Interpreter','latex');
        l = legend('\emph{Left hand}','\emph{Right hand}','\emph{Left foot}','\emph{Right foot}');
        set(l,'Interpreter','latex')
        ylabel('[$N/s$]','Interpreter','latex') % y-axis label
        %
        subplot(3,1,3);
        plot(time(startingSample:end,:),jets(startingSample:end,:),'LineWidth',lineWidth);
        title('Jet intensities by weight: $T/mg$','Interpreter','latex');
        l = legend('\emph{Left hand}','\emph{Right hand}','\emph{Left foot}','\emph{Right foot}');
        set(l,'Interpreter','latex')
        ylabel('[$-$]','Interpreter','latex') % y-axis label
        xlabel('Time [$s$]','Interpreter','latex') % x-axis label

        figure(2);

        % Plot com error and errors
        subplot(3,1,1);
        plot(time(startingSample:end,:),comError(:,startingSample:end),'LineWidth',lineWidth);
        title('Center of mass error: $^Ac - r$','Interpreter','latex');
        l = legend('$(^Ac - r)_x$','$(^Ac - r)_y$','$(^Ac - r)_z$');
        set(l,'Interpreter','latex')
        ylabel('[$m$]','Interpreter','latex') % y-axis label
        %
        subplot(3,1,2);
        if plotOrientationError
            plot(time(startingSample:end,:),oriErr(startingSample:end,:),'LineWidth',lineWidth);
            title('Norm of angular  error $.$','Interpreter','latex');
        else
            plot(time(startingSample:end,:),angMom(startingSample:end,:),'LineWidth',lineWidth);
            title('Norm of angular momentum error: $|\tilde{h}^\omega|$','Interpreter','latex');
        end
        ylabel('[$Nm$]','Interpreter','latex') % y-axis label
        %
        subplot(3,1,3);
        plot(time(startingSample:end,:),normVerror(startingSample:end,:),'LineWidth',lineWidth);
        title('Norm of velocity error: $|u_2^* -\dot{s}|$','Interpreter','latex');
        ylabel('[$^\circ/s$]','Interpreter','latex') % y-axis label
        xlabel('Time [$s$]','Interpreter','latex') % x-axis label

        % Plot joint torques
        FigHandle = figure;
        %get(FigHandle, 'Position')
        set(FigHandle, 'Position', [100, 100, jointTorquesAndVelFigureDimension]);

        startingSample = 4;
        for j = 1:5
            subplot(5,1,j);
            plot(time(startingSample:end,:),jointTorques{j}(startingSample:end,:),'LineWidth',lineWidth);
            title(fromIndexToPart(j),'Interpreter','latex');
            ylabel('[$Nm$]','Interpreter','latex') % y-axis label
            if j == 5
                xlabel('Time [s]','Interpreter','latex') % x-axis label
            end
        end

        FigHandle = figure;
        set(FigHandle, 'Position', [200, 100, jointTorquesAndVelFigureDimension]);
        % Plot joint velocities

        for j = 1:5
            subplot(5,1,j);
            plot(time(startingSample:end,:),jointVelocitiesStar{j}(startingSample:end,:),'LineWidth',lineWidth);
            title(fromIndexToPart(j),'Interpreter','latex');
            ylabel('[$^\circ/s$]','Interpreter','latex') % y-axis label
            if j == 5
                xlabel('Time [$s$]','Interpreter','latex') % x-axis label
            end
        end
    else
        FigHandle = figure;
        set(FigHandle, 'Position', [200, 100, 560, 135]);
        plot(time(startingSample:end,:),oriErr(startingSample:end,:),'LineWidth',lineWidth);
        title('Norm of angular  error $.$','Interpreter','latex');
        xlabel('Time [$s$]','Interpreter','latex') % x-axis label
    end
%     
end

cd('../../../papers/contributions/momentumControlRAL2017/imgs')