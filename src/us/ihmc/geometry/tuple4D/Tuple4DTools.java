package us.ihmc.geometry.tuple4D;

import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class Tuple4DTools
{
   public static boolean containsNaN(Tuple4DReadOnly tuple4D)
   {
      return containsNaN(tuple4D.getX(), tuple4D.getY(), tuple4D.getZ(), tuple4D.getS());
   }

   public static boolean containsNaN(double x, double y, double z, double s)
   {
      return Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(z) || Double.isNaN(s);
   }

   public static boolean isUnitary(Tuple4DReadOnly tuple4D, double epsilon)
   {
      return isUnitary(tuple4D.getX(), tuple4D.getY(), tuple4D.getZ(), tuple4D.getS(), epsilon);
   }

   public static boolean isUnitary(double x, double y, double z, double s, double epsilon)
   {
      return Math.abs(1.0 - QuaternionTools.norm(x, y, z, s)) < epsilon;
   }

   public static double dot(Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2)
   {
      return tuple1.getX() * tuple2.getX() + tuple1.getY() * tuple2.getY() + tuple1.getZ() * tuple2.getZ() + tuple1.getS() * tuple2.getS();
   }

   public static boolean epsilonEquals(Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2, double epsilon)
   {
      double difference;

      difference = tuple1.getX() - tuple2.getX();
      if (Double.isNaN(difference) || Math.abs(difference) > epsilon)
         return false;

      difference = tuple1.getY() - tuple2.getY();
      if (Double.isNaN(difference) || Math.abs(difference) > epsilon)
         return false;

      difference = tuple1.getZ() - tuple2.getZ();
      if (Double.isNaN(difference) || Math.abs(difference) > epsilon)
         return false;

      difference = tuple1.getS() - tuple2.getS();
      if (Double.isNaN(difference) || Math.abs(difference) > epsilon)
         return false;

      return true;
   }

   public static String toString(Tuple4DReadOnly tuple)
   {
      return Tuple4DTools.toString(tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS());
   }

   public static String toString(double x, double y, double z, double s)
   {
      return "(" + x + ", " + y + ", " + z + ", " + s + ")";
   }
}
