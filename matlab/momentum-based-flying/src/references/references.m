function [comDes, rotBaseDes, xDcomDes, tSettling]  = references(t, CoM0, w_H_b0, Config)

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

    comDes             = [CoM0,zeros(3,3)];
    [roll,pitch,yaw]   = rpyFromRotation(w_H_b0(1:3,1:3));
    
    if t < Config.references.orientation.stopSpin
        yaw                = yaw + Config.references.orientation.spinVel*t;
    else
        yaw                = yaw + Config.references.orientation.spinVel*Config.references.orientation.stopSpin;
    end
    
    A_R_B_des          = rotx(roll)*roty(pitch)*rotz(yaw);
    
    if Config.references.orientation.constantRdes
        rpy            = Config.references.orientation.constantRdes_rpy;
        A_R_B_des      = rotx(rpy(1))*roty(rpy(2))*rotz(rpy(3));
    end
    
    rotBaseDes         = [A_R_B_des,zeros(3,2)];
    
    if Config.references.elicoidalFlight
        tElicoidal = Config.references.elicoidal.timeReady;
        f          = Config.references.elicoidal.frequency;
        dir        = Config.references.elicoidal.dir;
        e1         = [1;0;0];
        e2         = [0;1;0];

        if t < tElicoidal
            A = Config.references.elicoidal.amplitude*t/tElicoidal;
        else
            A = Config.references.elicoidal.amplitude;
        end

        xcomDes    = Config.references.elicoidal.constCom + ...
                      A*cos(2*pi*f*t)*e1 + ...
                      A*sin(2*pi*f*t)*e2 + ...
                      Config.references.elicoidal.vel*t*dir;

        xDcomDes   =-A*2*pi*f*sin(2*pi*f*t)*e1 + ...
                     A*2*pi*f*cos(2*pi*f*t)*e2 + ...
                      Config.references.elicoidal.vel*dir;

        xDDcomDes  =-A*(2*pi*f)^2*cos(2*pi*f*t)*e1 + ...
                    -A*(2*pi*f)^2*sin(2*pi*f*t)*e2;

        xDDDcomDes = A*(2*pi*f)^3*sin(2*pi*f*t)*e1 + ...
                    -A*(2*pi*f)^3*cos(2*pi*f*t)*e2;
                
        comDes     = [xcomDes,xDcomDes,xDDcomDes,xDDDcomDes];

    elseif Config.references.sinuisoidalFlight
        f          = Config.references.sin.frequency;
        A          = Config.references.sin.amplitude;
        dir        = Config.references.sin.dir;
        
        xcomDes    =  Config.references.sin.constCom + A*sin(2*pi*f*t)*dir ;
        xDcomDes   =  A*(2*pi*f)*cos(2*pi*f*t)*dir;
        xDDcomDes  = -A*(2*pi*f)^2*sin(2*pi*f*t)*dir;
        xDDDcomDes = -A*(2*pi*f)^3*cos(2*pi*f*t)*dir;

        if t > Config.references.sin.tChangeRef
            xcomDes    =  Config.references.sin.constCom + A*sin(2*pi*f*Config.references.sin.tChangeRef)*dir + ...
                          Config.references.sin.vel*(t - Config.references.sin.tChangeRef)*Config.references.sin.dirIncVel;
            xDcomDes   =  Config.references.sin.vel*Config.references.sin.dirIncVel;
            xDDcomDes  = xDDcomDes*0;
            xDDDcomDes = xDDDcomDes*0;
        end
        comDes     = [xcomDes+CoM0,xDcomDes,xDDcomDes,xDDDcomDes];
        
    elseif Config.references.variousDirFlight

        dir         = Config.references.varDir.series(state,3:end)';
        v           = Config.references.varDir.series(state,2);
 
        xcomDes     = xSwitch + v*(t-tSwitch)*dir;
        xDcomDes    = v*dir;
        xDDcomDes   = zeros(3,1);
        xDDDcomDes  = zeros(3,1);
        
        comDes      = [xcomDes,xDcomDes,xDDcomDes,xDDDcomDes];
        
        if t > Config.references.varDir.series(state,1) && state < size(Config.references.varDir.series,1)
            state   = state + 1;
            xSwitch = xcomDes;  
            tSwitch = t;
        end
        
     elseif Config.USE_MIN_JERK_FOR_COM_REF 
        dir         = Config.references.minJerkVel.series(state,4:end)';
        v           = Config.references.minJerkVel.series(state,3);
        
        tSettling   = Config.references.minJerkVel.series(state,2);
        xDcomDes    = v*dir;
        
        if t > Config.references.minJerkVel.series(state,1) && state < size(Config.references.minJerkVel.series,1)
            state   = state + 1;
            tSwitch = t;
        end
    end
    
    if ~Config.references.useRefVelAccJerk
        comDes         = [comDes(:,1),zeros(3,3)];
    end
end