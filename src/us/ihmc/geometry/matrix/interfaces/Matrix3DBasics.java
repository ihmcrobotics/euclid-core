package us.ihmc.geometry.matrix.interfaces;

public interface Matrix3DBasics extends Matrix3DReadOnly
{
   public void set(Matrix3DReadOnly other);
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);
}