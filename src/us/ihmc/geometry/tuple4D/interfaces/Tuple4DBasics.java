package us.ihmc.geometry.tuple4D.interfaces;

public interface Tuple4DBasics extends Tuple4DReadOnly
{
   void negate();

   void setToZero();

   void setToNaN();

   void set(Tuple4DReadOnly tuple);

   void set(double x, double y, double z, double s);
}