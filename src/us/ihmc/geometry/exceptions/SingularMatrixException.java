package us.ihmc.geometry.exceptions;

import us.ihmc.geometry.matrix.Matrix3DReadOnlyTools;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;

public class SingularMatrixException extends RuntimeException
{
   private static final long serialVersionUID = 885554370291457429L;

   public SingularMatrixException()
   {
      super();
   }

   public SingularMatrixException(String message)
   {
      super(message);
   }

   public SingularMatrixException(Matrix3DReadOnly matrix)
   {
      super("The matrix is singular:\n" + matrix);
   }

   public SingularMatrixException(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      super("The matrix is singular:\n" + Matrix3DReadOnlyTools.toString(m00, m01, m02, m10, m11, m12, m20, m21, m22));
   }
}
