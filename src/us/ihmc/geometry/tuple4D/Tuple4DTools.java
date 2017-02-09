package us.ihmc.geometry.tuple4D;

import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class Tuple4DTools
{
   public static boolean epsilonEquals(Tuple4DReadOnly<?> tuple1, Tuple4DReadOnly<?> tuple2, double epsilon)
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

   public static String toString(Tuple4DReadOnly<?> tuple)
   {
      return Tuple4DTools.toString(tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS());
   }

   public static String toString(double x, double y, double z, double s)
   {
      return "(" + x + ", " + y + ", " + z + ", " + s + ")";
   }
}
