package us.ihmc.geometry.matrix.interfaces;

import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;

public interface RotationScaleMatrixReadOnly extends Matrix3DReadOnly
{
   public RotationMatrixReadOnly getRotationMatrix();

   public TupleReadOnly getScale();

   public double getScaleX();

   public double getScaleY();

   public double getScaleZ();
}
