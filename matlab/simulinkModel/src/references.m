function [comDes,rotBaseDes,xDcomDes,tSettling]  = references(t,CoM0,w_H_b0,config)
%#codegen

    persistent tSwitch;
    persistent xSwitch;
    persistent state;
    
    tSettling   = 0;
    xDcomDes    = zeros(3,1);
    
    if isempty(tSwitch) || isempty(xSwitch) || isempty(state) 
        tSwitch = t;
        xSwitch = CoM0;
        state   = 1;
    end

    comDes         = [CoM0,zeros(3,3)];
    
    [rool,pitch,yaw]   = rpyFromRotation(w_H_b0(1:3,1:3));
    
    if t < config.references.orientation.stopSpin
        yaw                = yaw + config.references.orientation.spinVel*t;
    else
        yaw                = yaw + config.references.orientation.spinVel*config.references.orientation.stopSpin;
    end
    
    A_R_B_des          = rotx(rool)*roty(pitch)*rotz(yaw);
    
    if config.references.orientation.constantRdes
        rpy            = config.references.orientation.constantRdesrpy;
        A_R_B_des      = rotx(rpy(1))*roty(rpy(2))*rotz(rpy(3));
    end
    
    rotBaseDes         = [A_R_B_des,zeros(3,2)];
    
    
    if config.references.elicoidalFlight
        tElicoidal = config.references.elicoidal.timeReady;
        f          = config.references.elicoidal.frequency;
        dir        = config.references.elicoidal.dir;
        e1         = [1;0;0];
        e2         = [0;1;0];

        if t < tElicoidal
            A = config.references.elicoidal.amplitude*t/tElicoidal;
        else
            A = config.references.elicoidal.amplitude;
        end

        xcomDes    = config.references.elicoidal.constCom + ...
                      A*cos(2*pi*f*t)*e1 + ...
                      A*sin(2*pi*f*t)*e2 + ...
                      config.references.elicoidal.vel*t*dir;

        xDcomDes   =-A*2*pi*f*sin(2*pi*f*t)*e1 + ...
                     A*2*pi*f*cos(2*pi*f*t)*e2 + ...
                      config.references.elicoidal.vel*dir;


        xDDcomDes  =-A*(2*pi*f)^2*cos(2*pi*f*t)*e1 + ...
                    -A*(2*pi*f)^2*sin(2*pi*f*t)*e2;


        xDDDcomDes = A*(2*pi*f)^3*sin(2*pi*f*t)*e1 + ...
                    -A*(2*pi*f)^3*cos(2*pi*f*t)*e2;
                
        comDes = [xcomDes,xDcomDes,xDDcomDes,xDDDcomDes];
    elseif config.references.sinuisoidalFlight
        f              = config.references.sin.frequency;
        A              = config.references.sin.amplitude;
        dir            = config.references.sin.dir;
        
        xcomDes    =  config.references.sin.constCom ...
                    + A*sin(2*pi*f*t)*dir ;
        xDcomDes   =  A*(2*pi*f)*cos(2*pi*f*t)*dir;
        xDDcomDes  = -A*(2*pi*f)^2*sin(2*pi*f*t)*dir;
        xDDDcomDes = -A*(2*pi*f)^3*cos(2*pi*f*t)*dir;

        if t > config.references.sin.tChangeRef
            xcomDes    =  config.references.sin.constCom + A*sin(2*pi*f*config.references.sin.tChangeRef)*dir + ...
                          config.references.sin.vel*(t - config.references.sin.tChangeRef)*config.references.sin.dirIncVel;
            xDcomDes   =config.references.sin.vel*config.references.sin.dirIncVel;

            xDDcomDes =xDDcomDes*0;
            xDDDcomDes = xDDDcomDes*0;
        end
        comDes = [xcomDes+CoM0,xDcomDes,xDDcomDes,xDDDcomDes];
    elseif config.references.variousDirFlight

        
        dir         = config.references.varDir.series(state,3:end)';
        v           = config.references.varDir.series(state,2);
        
        xcomDes     = xSwitch + v*(t-tSwitch)*dir;
        xDcomDes    = v*dir;
        xDDcomDes   = zeros(3,1);
        xDDDcomDes  = zeros(3,1);
        
        comDes = [xcomDes,xDcomDes,xDDcomDes,xDDDcomDes];
        
        if t > config.references.varDir.series(state,1) && state < size(config.references.varDir.series,1)
            state   = state + 1;
            xSwitch = xcomDes;  
            tSwitch = t;
        end
     elseif config.references.minJerkVelFligt 
        dir         = config.references.minJerkVel.series(state,4:end)';
        v           = config.references.minJerkVel.series(state,3);
        
        tSettling   = config.references.minJerkVel.series(state,2);
        xDcomDes    = v*dir;
        
        if t > config.references.minJerkVel.series(state,1) && state < size(config.references.minJerkVel.series,1)
            state   = state + 1;
            tSwitch = t;
        end

    end
    
    if ~config.references.useRefVelAccJerk
        comDes         = [comDes(:,1),zeros(3,3)];
    end

end