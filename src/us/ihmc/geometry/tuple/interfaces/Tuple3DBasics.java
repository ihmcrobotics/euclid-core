package us.ihmc.geometry.tuple.interfaces;

public interface Tuple3DBasics extends Tuple3DReadOnly
{
   public void set(Tuple3DReadOnly tupleBasics);

   public void add(Tuple3DReadOnly tupleBasics);

   public void sub(Tuple3DReadOnly tupleBasics);

   public void set(double x, double y, double z);

   public void add(double x, double y, double z);

   public void sub(double x, double y, double z);

   public void set(int index, double value);

   public void setToZero();

   public void setToNaN();

   public void setX(double x);

   public void setY(double y);

   public void setZ(double z);
}