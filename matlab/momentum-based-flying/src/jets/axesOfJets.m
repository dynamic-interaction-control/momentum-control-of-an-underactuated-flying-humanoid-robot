function [matrixOfJetsAxes, matrixOfJetsArms] = axesOfJets(w_H_J1, w_H_J2, w_H_J3, w_H_J4, posCoM, Config)

    directions       = sign(Config.jets.axes);
    axes             = abs(Config.jets.axes);
    matrixOfJetsAxes = [directions(1)*w_H_J1(1:3,axes(1)),directions(2)*w_H_J2(1:3,axes(2)),...
                        directions(3)*w_H_J3(1:3,axes(3)),directions(4)*w_H_J4(1:3,axes(4))];
    
    % Distances between the end effectors and the CoM position
    rtLHand          = w_H_J1(1:3,4)-posCoM;
    rtRHand          = w_H_J2(1:3,4)-posCoM;
    rtLFoot          = w_H_J3(1:3,4)-posCoM;
    rtRFoot          = w_H_J4(1:3,4)-posCoM;
                
    matrixOfJetsArms = [rtLHand,rtRHand,rtLFoot,rtRFoot];
end