function [matrixOfJetsAxes,matrixOfJetsArms] = axesOfJets(w_H_J1,w_H_J2,w_H_J3,w_H_J4,com,config)
%#codegen

    directions       = sign(config.jets.axes);
    axes             = abs(config.jets.axes);
    matrixOfJetsAxes = [directions(1)*w_H_J1(1:3,axes(1)),directions(2)*w_H_J2(1:3,axes(2)),...
                        directions(3)*w_H_J3(1:3,axes(3)),directions(4)*w_H_J4(1:3,axes(4))];
    
    rtLHand         = w_H_J1(1:3,4)-com;
    rtRHand         = w_H_J2(1:3,4)-com;
    rtLFoot         = w_H_J3(1:3,4)-com;
    rtRFoot         = w_H_J4(1:3,4)-com;
                
    matrixOfJetsArms = [rtLHand,rtRHand,rtLFoot,rtRFoot];

end