function f = fromJetsIntensitiesToForces(matrixOfJetsAxes,jetsIntensities)
%#codegen
    
    f = [jetsIntensities(1)*matrixOfJetsAxes(:,1);
         jetsIntensities(2)*matrixOfJetsAxes(:,2);
         jetsIntensities(3)*matrixOfJetsAxes(:,3);
         jetsIntensities(4)*matrixOfJetsAxes(:,4)];
    
end