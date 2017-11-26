package us.ihmc.euclid.tools;

import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

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
   public static boolean epsilonEquals(Tuple2DReadOnly tuple1, Tuple2DReadOnly tuple2, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(tuple1.getX(), tuple2.getX(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getY(), tuple2.getY(), epsilon))
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
   public static boolean epsilonEquals(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(tuple1.getX(), tuple2.getX(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getY(), tuple2.getY(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getZ(), tuple2.getZ(), epsilon))
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
   public static boolean epsilonEquals(Tuple4DReadOnly tuple1, Tuple4DReadOnly tuple2, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(tuple1.getX(), tuple2.getX(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getY(), tuple2.getY(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getZ(), tuple2.getZ(), epsilon))
         return false;

      if (!EuclidCoreTools.epsilonEquals(tuple1.getS(), tuple2.getS(), epsilon))
         return false;

      return true;
   }

   /**
    * Performs a linear interpolation from {@code a} to {@code b} given the percentage
    * {@code alpha}.
    * <p>
    * result = (1.0 - alpha) * a + alpha * b
    * </p>
    *
    * @param a the first value used in the interpolation.
    * @param b the second value used in the interpolation.
    * @param alpha the percentage to use for the interpolation. A value of 0 will return {@code a},
    *           while a value of 1 will return {@code b}.
    * @return the interpolated value.
    * @deprecated Use {@link EuclidCoreTools#interpolate(double,double,double)} instead
    */
   public static double interpolate(double a, double b, double alpha)
   {
      return EuclidCoreTools.interpolate(a, b, alpha);
   }
}
