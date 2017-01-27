package us.ihmc.geometry.matrix.interfaces;

public interface RotationMatrixReadOnly<T extends RotationMatrixReadOnly<T>> extends Matrix3DReadOnly<T>
{
   public void normalize();
}
