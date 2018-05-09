function jetIntensitiesAtZero = jetIntensitiesAtZero(matrixOfJetsAxes,Min,Jin,comRefs,Config)
%#codegen 

    persistent jetIntensitiesSym;
    
    jetIntensitiesAtZero = zeros(4,1);
    
    if isempty(jetIntensitiesSym)
        ndof              = size(Config.N_DOF_MATRIX,1);
        M                 = Min(:,1:ndof+6);

        Jc                = Jin(25:end,:);

        mg                = M(1,1)*Config.GRAVITY_ACC*[0;0;1;zeros(3,1)];
        rDD               = comRefs(:,3);
        angMomentumMat    = Jc(:,4:6)';

        Am                = matrixOfJetsAxes;
        Mm                = [angMomentumMat(:, 1: 3)*Am(:,1),...
                             angMomentumMat(:, 7: 9)*Am(:,2),...
                             angMomentumMat(:,13:15)*Am(:,3),...
                             angMomentumMat(:,19:21)*Am(:,4)];

        A                 = [Am;Mm];

        HdDot             = [M(1,1)*rDD;zeros(3,1)];
               
    if Config.jets.choiceOfJetsIntAt0 == 0    
            jetIntensitiesAtZero      = zeros(4,1);
    elseif Config.jets.choiceOfJetsIntAt0 == 1
            % Choose jet intensities so as to make them symmetric, i.e.
            %
            % left-hand-jet = right-hand-jet, left-foot-jet = right-foot-hand
            
            symmetrizMat              = [1 0;1 0;0 1;0 1];
            jetIntensitiesBiSym       = pinv(A*symmetrizMat)*(mg+HdDot);
            jetIntensitiesAtZero      = symmetrizMat*jetIntensitiesBiSym;
            
        elseif Config.jets.choiceOfJetsIntAt0 == 2
            % Choose jet intensities so as to make them symmetric, but hand
            % jets intensities are alpha times  foot jet intensities, i.e.
            %
            % left-hand-jet = right-hand-jet = alpha*left-foot-jet = alpha*right-foot-hand
            
            alpha                     = Config.jets.alpha;
            symmetrizMat              = [alpha;alpha;1;1];
            jetIntensitiesBiSym       = pinv(A*symmetrizMat)*(mg+HdDot);
            jetIntensitiesAtZero      = symmetrizMat*jetIntensitiesBiSym;
        end
    end    
end


