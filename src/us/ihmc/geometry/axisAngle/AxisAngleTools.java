package us.ihmc.geometry.axisAngle;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;

public abstract class AxisAngleTools
{
   public static boolean containsNaN(AxisAngleReadOnly axisAngle)
   {
      return containsNaN(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle());
   }

   public static boolean containsNaN(double ux, double uy, double uz, double angle)
   {
      return Double.isNaN(ux) || Double.isNaN(uy) || Double.isNaN(uz) || Double.isNaN(angle);
   }

   public static double axisNormSquared(AxisAngleReadOnly axisAngle)
   {
      return axisNormSquared(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle());
   }

   public static double axisNormSquared(double ux, double uy, double uz, double angle)
   {
      return ux * ux + uy * uy + uz * uz;
   }

   public static double axisNorm(AxisAngleReadOnly axisAngle)
   {
      return axisNorm(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle());
   }

   public static double axisNorm(double ux, double uy, double uz, double angle)
   {
      return Math.sqrt(axisNormSquared(ux, uy, uz, angle));
   }

   public static boolean isAxisUnitary(AxisAngleReadOnly axisAngle, double epsilon)
   {
      return isAxisUnitary(axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle(), epsilon);
   }

   public static boolean isAxisUnitary(double ux, double uy, double uz, double angle, double epsilon)
   {
      return Math.abs(1.0 - axisNorm(ux, uy, uz, angle)) < epsilon;
   }
}
