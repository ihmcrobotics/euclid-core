package us.ihmc.geometry.tuple;

import us.ihmc.geometry.tuple.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public abstract class TupleTools
{
   public static boolean containsNaN(Tuple3DReadOnly tuple)
   {
      return Double.isNaN(tuple.getX()) || Double.isNaN(tuple.getY()) || Double.isNaN(tuple.getZ());
   }

   public static boolean epsilonEquals(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2, double epsilon)
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

      return true;
   }

   public static boolean epsilonEquals(Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2, double epsilon)
   {
      double difference;

      difference = tuple1.getX() - tuple2.getX();
      if (Double.isNaN(difference) || Math.abs(difference) > epsilon)
         return false;

      difference = tuple1.getY() - tuple2.getY();
      if (Double.isNaN(difference) || Math.abs(difference) > epsilon)
         return false;

      return true;
   }

   public static final double interpolate(double a, double b, double alpha)
   {
      return (1.0 - alpha) * a + alpha * b;
   }
}
