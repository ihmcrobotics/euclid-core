package us.ihmc.geometry.transform.interfaces;

import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.tuple.interfaces.PointBasics;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;

public interface Transform
{
   public void transform(PointBasics pointToTransform);

   public void transform(VectorBasics vectorToTransform);

   public void transform(QuaternionBasics quaternionToTransform);

   public void transform(Vector4DBasics vectorToTransform);

   public void transform(Point2DBasics point2DToTransform);

   public void transform(Point2DBasics point2DToTransform, boolean checkIfTransformInXYPlane);

   public void transform(Vector2DBasics vector2DToTransform);

   public void transform(Matrix3D matrixToTransform);
   
   public void transform(RotationMatrix matrixToTransform);
}
