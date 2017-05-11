function R = Raxes(th,axis)
    if axis == 1
        R = [ 1    0        0 
              0  cos(th)   -sin(th)
              0  sin(th)    cos(th)];
    end
    if axis == 2
        R = [  cos(th)   0  sin(th)  
                   0        1     0
              -sin(th)   0   cos(th)];
    end
    if axis == 3
        R = [ cos(th) -sin(th) 0 
              sin(th)  cos(th) 0
               0          0    1];
    end
end