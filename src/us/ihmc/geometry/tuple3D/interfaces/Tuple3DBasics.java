package us.ihmc.geometry.tuple3D.interfaces;

import us.ihmc.geometry.interfaces.GeometryObject;

public interface Tuple3DBasics<T extends Tuple3DBasics<T>> extends Tuple3DReadOnly<T>, GeometryObject<T>
{
   public void set(Tuple3DReadOnly<?> tupleBasics);

   public void add(Tuple3DReadOnly<?> tupleBasics);

   public void sub(Tuple3DReadOnly<?> tupleBasics);

   public void set(double x, double y, double z);

   default void set(T other)
   {
      set(other.getX(), other.getY(), other.getZ());
   }

   public void add(double x, double y, double z);

   public void sub(double x, double y, double z);

   public void set(int index, double value);

   public void setToZero();

   public void setToNaN();

   public void setX(double x);

   public void setY(double y);

   public void setZ(double z);
}