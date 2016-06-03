package us.ihmc.geometry.exceptions;

import us.ihmc.geometry.matrix.Matrix3DReadOnlyTools;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;

public class NotAMatrix2DException extends RuntimeException
{
   private static final long serialVersionUID = 8769481575502495447L;

   public NotAMatrix2DException()
   {
      super();
   }

   public NotAMatrix2DException(String message)
   {
      super(message);
   }

   public NotAMatrix2DException(Matrix3DReadOnly matrix)
   {
      super("The matrix is not in XY plane: \n" + matrix);
   }

   public NotAMatrix2DException(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      super("The matrix is not in XY plane: \n" + Matrix3DReadOnlyTools.toString(m00, m01, m02, m10, m11, m12, m20, m21, m22));
   }
}
