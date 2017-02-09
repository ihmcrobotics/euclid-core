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
}
