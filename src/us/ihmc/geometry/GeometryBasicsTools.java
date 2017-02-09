package us.ihmc.geometry;

public class GeometryBasicsTools
{
   public static final double EPS_NORM_FAST_SQRT = 2.107342e-08;

   public static double fastSquareRoot(double squaredValueClosedToOne)
   {
      if (Math.abs(1.0 - squaredValueClosedToOne) < EPS_NORM_FAST_SQRT)
         squaredValueClosedToOne = 0.5 * (1.0 + squaredValueClosedToOne);
      else
         squaredValueClosedToOne = Math.sqrt(squaredValueClosedToOne);

      return squaredValueClosedToOne;
   }

   /**
    * Tests if at least one of the two given elements is equal to {@linkplain Double#NaN}.
    * 
    * @param a the first element.
    * @param b the second element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false} otherwise.
    */
   public static boolean containsNaN(double a, double b)
   {
      return Double.isNaN(a) || Double.isNaN(b);
   }

   /**
    * Tests if at least one of the three given elements is equal to {@linkplain Double#NaN}.
    * 
    * @param a the first element.
    * @param b the second element.
    * @param c the third element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false} otherwise.
    */
   public static boolean containsNaN(double a, double b, double c)
   {
      return Double.isNaN(a) || Double.isNaN(b) || Double.isNaN(c);
   }

   /**
    * Tests if at least one of the four given elements is equal to {@linkplain Double#NaN}.
    * 
    * @param a the first element.
    * @param b the second element.
    * @param c the third element.
    * @param d the fourth element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false} otherwise.
    */
   public static boolean containsNaN(double a, double b, double c, double d)
   {
      return Double.isNaN(a) || Double.isNaN(b) || Double.isNaN(c) || Double.isNaN(d);
   }

   /**
    * Tests if at least one of the nine given elements is equal to {@linkplain Double#NaN}.
    * 
    * @param a0 the first element.
    * @param a1 the second element.
    * @param a2 the third element.
    * @param a3 the fourth element.
    * @param a4 the fifth element.
    * @param a5 the sixth element.
    * @param a6 the seventh element.
    * @param a7 the eighth element.
    * @param a8 the ninth element.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false} otherwise.
    */
   public static boolean containsNaN(double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8)
   {
      if (Double.isNaN(a0) || Double.isNaN(a1) || Double.isNaN(a2))
         return true;
      if (Double.isNaN(a3) || Double.isNaN(a4) || Double.isNaN(a5))
         return true;
      if (Double.isNaN(a6) || Double.isNaN(a7) || Double.isNaN(a8))
         return true;
      return false;
   }

   /**
    * Tests if at least one element in the given array is equal to {@link Double#NaN}.
    * 
    * @param array the array containing the elements to test for {@link Double#NaN}. Not modified.
    * @return {@code true} if at least one element is equal to {@link Double#NaN}, {@code false} otherwise.
    */
   public static boolean containsNaN(double[] array)
   {
      for (int i = 0; i < array.length; i++)
      {
         if (Double.isNaN(array[i]))
            return true;
      }
      return false;
   }
}
