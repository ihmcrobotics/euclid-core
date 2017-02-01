package us.ihmc.geometry.transform;

import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public abstract class TransformTools
{
   private static int TO_STRING_PRECISION = 3;
   private static final boolean PRINT_LAST_ROW = false;

   static String toString(Matrix3DReadOnly<?> matrix, TupleReadOnly translation)
   {
      String ret = "";
      String format = "%" + (3 + TO_STRING_PRECISION) + "." + TO_STRING_PRECISION + "f";
      String matrixPart = format + " ";
      String vectorPart = "| " + format + "\n";

      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
            ret += String.format(matrixPart, matrix.getElement(i, j));
         ret += String.format(vectorPart, translation.get(i));
      }

      if (PRINT_LAST_ROW)
      {
         for (int j = 0; j < 3; j++)
            ret += String.format(matrixPart, 0.0);
         ret += String.format(vectorPart, 1.0);
      }
      return ret;
   }

   public static String toString(QuaternionReadOnly quaternion, TupleReadOnly translation)
   {
      String ret = "";
      String format = "%" + (3 + TO_STRING_PRECISION) + "." + TO_STRING_PRECISION + "f";

      ret += "Quaternion:  (" + String.format(format + " " + format + " " + format + " " + format + " ", quaternion.getX(), quaternion.getY(), quaternion.getZ(), quaternion.getS()) + ")\n";
      ret += "Translation: (" + String.format(format + " " + format + " " + format + " ", translation.getX(), translation.getY(), translation.getZ()) + ")\n";
      return ret;
   }
}
