function  v  = fromSkewtoVector(S)
    v    = zeros(3,1);
    v(1) = -S(2,3);
    v(2) =  S(1,3);
    v(3) = -S(1,2);
end

