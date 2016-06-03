package us.ihmc.geometry.exceptions;

import us.ihmc.geometry.matrix.Matrix3DReadOnlyTools;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;

public class NotARotationScaleMatrixException extends RuntimeException
{
   private static final long serialVersionUID = -969795129004644572L;

   public NotARotationScaleMatrixException()
   {
      super();
   }

   public NotARotationScaleMatrixException(String message)
   {
      super(message);
   }

   public NotARotationScaleMatrixException(Matrix3DReadOnly matrix)
   {
      super("The matrix is not a rotation-scale matrix: \n" + matrix);
   }

   public NotARotationScaleMatrixException(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      super("The matrix is not a rotation-scale matrix: \n" + Matrix3DReadOnlyTools.toString(m00, m01, m02, m10, m11, m12, m20, m21, m22));
   }
}
