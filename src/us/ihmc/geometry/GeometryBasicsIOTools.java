package us.ihmc.geometry;

import java.util.Random;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Tuple4DReadOnly;

public abstract class GeometryBasicsIOTools
{
   private static final int DEFAULT_NUMBER_OF_CHAR = 6;
   private static final int DEFAULT_PRECISION = 3;
   private static final String DEFAULT_FORMAT = getStringFormat(DEFAULT_NUMBER_OF_CHAR, DEFAULT_PRECISION);

   public static String getHomogeneousTransformString(Matrix3DReadOnly<?> matrix, Tuple3DReadOnly<?> translation)
   {
      return getHomogeneousTransformString(matrix, translation, DEFAULT_FORMAT);
   }

   public static String getHomogeneousTransformString(Matrix3DReadOnly<?> matrix, Tuple3DReadOnly<?> translation, String format)
   {
      String ret = "";

      for (int i = 0; i < 3; i++)
      {
         ret += getStringOf(null, " ", " ", matrix.getElement(i, 0), matrix.getElement(i, 1), matrix.getElement(i, 2));
         ret += "| " + String.format(format, translation.get(i)) + "\n";
      }

      ret += getStringOf(null, " ", " ", 0.0, 0.0, 0.0);
      ret += "| " + String.format(format, 1.0);

      return ret;
   }

   public static String getQuaternionBasedTransformString(QuaternionReadOnly<?> quaternion, Tuple3DReadOnly<?> translation)
   {
      String ret = "";

      ret += getStringOf("Quaternion:  (", " )\n", ", ", quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS());
      ret += getStringOf("Translation: (", " )", " ", translation.getX(), translation.getY(), translation.getZ());
      return ret;
   }

   public static String getTuple2DString(Tuple2DReadOnly<?> tuple)
   {
      return getTuple2DString(tuple, DEFAULT_FORMAT);
   }

   public static String getTuple2DString(Tuple2DReadOnly<?> tuple, String format)
   {
      return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY());
   }

   public static String getTuple3DString(Tuple3DReadOnly<?> tuple)
   {
      return getTuple3DString(tuple, DEFAULT_FORMAT);
   }

   public static String getTuple3DString(Tuple3DReadOnly<?> tuple, String format)
   {
      return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY(), tuple.getZ());
   }

   public static String getTuple4DString(Tuple4DReadOnly<?> tuple)
   {
      return getTuple4DString(tuple, DEFAULT_FORMAT);
   }

   public static String getTuple4DString(Tuple4DReadOnly<?> tuple, String format)
   {
      return getStringOf("(", " )", ", ", format, tuple.getX(), tuple.getY(), tuple.getZ(), tuple.getS());
   }

   public static String getAxisAngleString(AxisAngleReadOnly<?> axisAngle)
   {
      return getAxisAngleString(axisAngle, DEFAULT_FORMAT);
   }

   public static String getAxisAngleString(AxisAngleReadOnly<?> axisAngle, String format)
   {
      return getStringOf("(", " )", ", ", format, axisAngle.getX(), axisAngle.getY(), axisAngle.getZ(), axisAngle.getAngle());
   }

   public static String getMatrixString(Matrix3DReadOnly<?> matrix)
   {
      return getMatrixString(matrix, DEFAULT_FORMAT);
   }

   public static String getMatrixString(Matrix3DReadOnly<?> matrix, String format)
   {
      return getMatrixString(format, matrix.getM00(), matrix.getM01(), matrix.getM02(), matrix.getM10(), matrix.getM11(), matrix.getM12(), matrix.getM20(),
                             matrix.getM21(), matrix.getM22());
   }

   public static String getMatrixString(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      return getMatrixString(DEFAULT_FORMAT, m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   public static String getMatrixString(String format, double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21,
                                        double m22)
   {
      String ret = getStringOf("/", " \\\n", ", ", format, m00, m01, m02);
      ret += getStringOf("|", " |\n", ", ", format, m10, m11, m12);
      ret += getStringOf("\\", " /", ", ", format, m20, m21, m22);
      return ret;
   }

   public static String getStringOf(String prefix, String suffix, String separator, double... values)
   {
      return getStringOf(prefix, suffix, separator, DEFAULT_FORMAT, values);
   }

   public static String getStringOf(String prefix, String suffix, String separator, String format, double... values)
   {
      String ret = getStringOf(separator, format, values);

      if (prefix != null)
         ret = prefix + ret;

      if (suffix != null)
         ret += suffix;

      return ret;
   }

   public static String getStringOf(String separator, String format, double... values)
   {
      if (values.length == 0)
         return "";
      String ret = String.format(format, values[0]);
      for (int i = 1; i < values.length; i++)
         ret += separator + String.format(format, values[i]);
      return ret;
   }

   public static String getStringFormat(int numberOfChar, int precision)
   {
      return "%" + numberOfChar + "." + precision + "f";
   }

   public static void main(String[] args)
   {
      Random random = new Random();
      Matrix3DReadOnly<?> matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      Tuple2DReadOnly<?> tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
      Tuple3DReadOnly<?> tuple3D = GeometryBasicsRandomTools.generateRandomVector3D(random);
      Tuple4DReadOnly<?> tuple4D = GeometryBasicsRandomTools.generateRandomVector4D(random);
      AxisAngleReadOnly<?> axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random);

      System.out.println(getHomogeneousTransformString(matrix, tuple3D));
      System.out.println();
      System.out.println(getQuaternionBasedTransformString(new Quaternion(), tuple3D));
      System.out.println();
      System.out.println(getTuple2DString(tuple2D));
      System.out.println();
      System.out.println(getTuple3DString(tuple3D));
      System.out.println();
      System.out.println(getTuple4DString(tuple4D));
      System.out.println();
      System.out.println(getAxisAngleString(axisAngle));
      System.out.println();
      System.out.println(getMatrixString(matrix));
   }
}
