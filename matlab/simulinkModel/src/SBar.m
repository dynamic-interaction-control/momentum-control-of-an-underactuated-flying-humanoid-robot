function [ X ] = SBar( x )
    X=[eye(3);skew(x)];
end

