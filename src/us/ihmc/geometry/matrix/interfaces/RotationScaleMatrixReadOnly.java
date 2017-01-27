package us.ihmc.geometry.matrix.interfaces;

import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;

public interface RotationScaleMatrixReadOnly<T extends RotationScaleMatrixReadOnly<T>> extends Matrix3DReadOnly<T>
{
   public RotationMatrixReadOnly<?> getRotationMatrix();

   public TupleReadOnly getScale();

   public double getScaleX();

   public double getScaleY();

   public double getScaleZ();
}
