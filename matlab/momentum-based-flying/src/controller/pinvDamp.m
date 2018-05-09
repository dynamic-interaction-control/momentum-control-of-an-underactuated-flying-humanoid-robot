function pinvA = pinvDamp(A,reg)
    pinvA = A'/(A*A' + reg*eye(size(A,1)));
end

