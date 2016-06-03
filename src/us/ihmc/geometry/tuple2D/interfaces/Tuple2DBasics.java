package us.ihmc.geometry.tuple2D.interfaces;

public interface Tuple2DBasics extends Tuple2DReadOnly
{
   public void set(Tuple2DReadOnly tuple2DBasics);

   public void add(Tuple2DReadOnly tuple2DBasics);

   public void sub(Tuple2DReadOnly tuple2DBasics);

   public void set(double x, double y);

   public void add(double x, double y);

   public void sub(double x, double y);

   public void set(int index, double value);

   public void setToZero();

   public void setToNaN();

   public void setX(double x);

   public void setY(double y);
}
