package us.ihmc.geometry.tuple4D.interfaces;

public interface Vector4DBasics<T extends Vector4DBasics<T>> extends Vector4DReadOnly<T>, Tuple4DBasics<T>
{
   public void setX(double x);

   public void setY(double y);

   public void setZ(double z);

   public void setS(double s);
}
