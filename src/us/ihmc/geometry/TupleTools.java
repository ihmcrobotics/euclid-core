package us.ihmc.geometry;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class TupleTools
{
   /**
    * Tests on a per component basis if the two given tuples are equal to an {@code epsilon}.
    * 
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   public static boolean epsilonEquals(Tuple2DReadOnly<?> tuple1, Tuple2DReadOnly<?> tuple2, double epsilon)
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

   /**
    * Tests on a per component basis if the two given tuples are equal to an {@code epsilon}.
    * 
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
   public static boolean epsilonEquals(Tuple3DReadOnly<?> tuple1, Tuple3DReadOnly<?> tuple2, double epsilon)
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

   /**
    * Tests on a per component basis if the two given tuples are equal to an {@code epsilon}.
    * 
    * @param tuple1 the first tuple. Not modified.
    * @param tuple2 the second tuple. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two tuples are equal, {@code false} otherwise.
    */
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

   /**
    * Performs a linear interpolation from {@code a} to {@code b} given the 
    * percentage {@code alpha}.
    * <p>
    * result = (1.0 - alpha) * a + alpha * b
    * </p>
    * 
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation.
    * A value of 0 will return {@code a}, while a value of 1 will return {@code b}.
    * @return the interpolated value.
    */
   public static double interpolate(double a, double b, double alpha)
   {
      return (1.0 - alpha) * a + alpha * b;
   }
}
