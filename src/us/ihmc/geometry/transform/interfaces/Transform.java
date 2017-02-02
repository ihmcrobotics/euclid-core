package us.ihmc.geometry.transform.interfaces;

import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.tuple.interfaces.PointBasics;
import us.ihmc.geometry.tuple.interfaces.PointReadOnly;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public interface Transform
{
   default void transform(PointBasics pointToTransform)
   {
      transform(pointToTransform, pointToTransform);
   }

   void transform(PointReadOnly pointOriginal, PointBasics pointTransformed);

   default void transform(VectorBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   void transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed);

   default void transform(QuaternionBasics quaternionToTransform)
   {
      transform(quaternionToTransform, quaternionToTransform);
   }

   void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed);

   default void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   void transform(Vector4DReadOnly vector4DOriginal, Vector4DBasics vector4DTransformed);

   default void transform(Point2DBasics point2DToTransform)
   {
      transform(point2DToTransform, true);
   }

   default void transform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed)
   {
      transform(point2DOriginal, point2DTransformed, true);
   }

   default void transform(Point2DBasics point2DToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(point2DToTransform, point2DToTransform, checkIfTransformInXYPlane);
   }

   void transform(Point2DReadOnly point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane);

   default void transform(Vector2DBasics vector2DToTransform)
   {
      transform(vector2DToTransform, true);
   }

   default void transform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed)
   {
      transform(vector2DOriginal, vector2DTransformed, true);
   }

   default void transform(Vector2DBasics vector2DToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(vector2DToTransform, vector2DToTransform, checkIfTransformInXYPlane);
   }

   void transform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane);

   default void transform(Matrix3D matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   void transform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed);

   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   default void inverseTransform(PointBasics pointToTransform)
   {
      inverseTransform(pointToTransform, pointToTransform);
   }

   void transform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTranformed);

   void inverseTransform(PointReadOnly pointOriginal, PointBasics pointTransformed);

   default void inverseTransform(VectorBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   void inverseTransform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed);

   default void inverseTransform(Point2DBasics pointToTransform)
   {
      inverseTransform(pointToTransform, true);
   }

   default void inverseTransform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
   {
      inverseTransform(pointOriginal, pointTransformed, true);
   }

   default void inverseTransform(Point2DBasics pointToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(pointToTransform, pointToTransform, checkIfTransformInXYPlane);
   }

   void inverseTransform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed, boolean checkIfTransformInXYPlane);

   default void inverseTransform(Vector2DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, true);
   }

   default void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
   {
      inverseTransform(vectorOriginal, vectorTransformed, true);
   }

   default void inverseTransform(Vector2DBasics vectorToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(vectorToTransform, vectorToTransform, checkIfTransformInXYPlane);
   }

   void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed, boolean checkIfTransformInXYPlane);
}
