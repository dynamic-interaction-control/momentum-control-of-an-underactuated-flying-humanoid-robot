function [ trans, shape ] = iDynTreeModelPositionFromChi( chi )
%iDynTreeModelPositionFromChi Extract a couple (iDynTree.Transform,
%iDynTree.VectorDynSize) from a chi vector 
internalDofs = (length(chi)-7-6)/2;

pos = iDynTree.Position();
pos.fromMatlab(chi(1:3));
quat = iDynTree.Vector4();
quat.fromMatlab(chi(4:7));
rot = iDynTree.Rotation();
rot.fromQuaternion(quat);
trans = iDynTree.Transform(rot,pos);

shape = iDynTree.VectorDynSize();
shape.fromMatlab(chi(8:8+internalDofs-1));

end

