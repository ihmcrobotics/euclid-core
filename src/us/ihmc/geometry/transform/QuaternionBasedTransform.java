package us.ihmc.geometry.transform;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.geometry.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.interfaces.EpsilonComparable;
import us.ihmc.geometry.interfaces.Settable;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.transform.interfaces.Transform;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.PointBasics;
import us.ihmc.geometry.tuple.interfaces.PointReadOnly;
import us.ihmc.geometry.tuple.interfaces.TupleBasics;
import us.ihmc.geometry.tuple.interfaces.TupleReadOnly;
import us.ihmc.geometry.tuple.interfaces.VectorBasics;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Point2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.QuaternionTools;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;

public class QuaternionBasedTransform implements Transform, EpsilonComparable<QuaternionBasedTransform>, Settable<QuaternionBasedTransform>
{
   private final Quaternion quaternion = new Quaternion();
   private final Vector translationVector = new Vector();

   public QuaternionBasedTransform()
   {
      setIdentity();
   }

   public QuaternionBasedTransform(QuaternionBasedTransform other)
   {
      set(other);
   }

   public QuaternionBasedTransform(RigidBodyTransform rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   public QuaternionBasedTransform(DenseMatrix64F matrix)
   {
      set(matrix);
   }

   public QuaternionBasedTransform(double[] array)
   {
      set(array);
   }

   public QuaternionBasedTransform(RotationMatrix rotationMatrix, TupleReadOnly translation)
   {
      set(rotationMatrix, translation);
   }

   public QuaternionBasedTransform(QuaternionReadOnly quaternion, TupleReadOnly translation)
   {
      set(quaternion, translation);
   }

   public QuaternionBasedTransform(AxisAngleReadOnly<?> axisAngle, TupleReadOnly translation)
   {
      set(axisAngle, translation);
   }

   @Override
   public void setToZero()
   {
      setIdentity();
   }

   @Override
   public void setToNaN()
   {
      quaternion.setToNaN();
      translationVector.setToNaN();
   }

   @Override
   public boolean containsNaN()
   {
      return quaternion.containsNaN() || translationVector.containsNaN();
   }

   public void resetRotation()
   {
      quaternion.setToZero();
   }

   public void resetTranslation()
   {
      translationVector.setToZero();
   }

   public void set(double qx, double qy, double qz, double qs, double x, double y, double z)
   {
      quaternion.set(qx, qy, qz, qs);
      translationVector.set(x, y, z);
   }

   public void setUnsafe(double qx, double qy, double qz, double qs, double x, double y, double z)
   {
      quaternion.setUnsafe(qx, qy, qz, qs);
      translationVector.set(x, y, z);
   }

   @Override
   public void set(QuaternionBasedTransform other)
   {
      quaternion.set(other.quaternion);
      translationVector.set(other.translationVector);
   }

   public void set(RigidBodyTransform rigidBodyTransform)
   {
      quaternion.set(rigidBodyTransform.getRotationMatrix());
      translationVector.set(rigidBodyTransform.getTranslationVector());
   }

   public void set(DenseMatrix64F matrix)
   {
      quaternion.set(matrix);
      translationVector.set(matrix, 4);
   }

   public void set(double[] array)
   {
      quaternion.set(array);
      translationVector.set(array, 4);
   }

   public void set(RotationMatrix rotationMatrix, TupleReadOnly translation)
   {
      quaternion.set(rotationMatrix);
      translationVector.set(translation);
   }

   public void set(AxisAngleReadOnly<?> axisAngle, TupleReadOnly translation)
   {
      quaternion.set(axisAngle);
      translationVector.set(translation);
   }

   public void set(QuaternionReadOnly quaternion, TupleReadOnly translation)
   {
      this.quaternion.set(quaternion);
      translationVector.set(translation);
   }

   public void setIdentity()
   {
      quaternion.setToZero();
      translationVector.setToZero();
   }

   public void setRotation(AxisAngleReadOnly<?> axisAngle)
   {
      quaternion.set(axisAngle);
   }

   public void setRotation(QuaternionReadOnly quaternion)
   {
      this.quaternion.set(quaternion);
   }

   public void setRotation(RotationMatrix rotationMatrix)
   {
      quaternion.set(rotationMatrix);
   }

   public void setRotation(VectorReadOnly rotationVector)
   {
      quaternion.set(rotationVector);
   }

   public void setTranslation(double x, double y, double z)
   {
      translationVector.set(x, y, z);
   }

   public void setTranslation(TupleReadOnly translation)
   {
      translationVector.set(translation);
   }

   public void get(DenseMatrix64F matrixToPack)
   {
      quaternion.get(matrixToPack);
      translationVector.get(matrixToPack, 4);
   }

   public void get(DenseMatrix64F matrixToPack, int startRow, int column)
   {
      quaternion.get(matrixToPack, startRow, column);
      translationVector.get(matrixToPack, startRow + 4, column);
   }

   public void get(double[] transformArrayToPack)
   {
      quaternion.get(transformArrayToPack);
      translationVector.get(transformArrayToPack, 4);
   }

   public void get(QuaternionBasics quaternionToPack, TupleBasics translationToPack)
   {
      quaternion.get(quaternionToPack);
      translationToPack.set(translationVector);
   }

   public void get(RotationMatrix rotationMarixToPack, TupleBasics translationToPack)
   {
      rotationMarixToPack.set(quaternion);
      translationToPack.set(translationVector);
   }

   public void get(RotationScaleMatrix rotationMarixToPack, TupleBasics translationToPack)
   {
      rotationMarixToPack.set(quaternion, 1.0);
      translationToPack.set(translationVector);
   }

   public QuaternionReadOnly getQuaternion()
   {
      return quaternion;
   }

   public void getRotation(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(quaternion);
   }

   public void getRotation(QuaternionBasics quaternionToPack)
   {
      quaternion.get(quaternionToPack);
   }

   public void getRotation(AxisAngleBasics<?> axisAngleToPack)
   {
      quaternion.get(axisAngleToPack);
   }

   public void getRotation(VectorBasics rotationVectorToPack)
   {
      quaternion.get(rotationVectorToPack);
   }

   public VectorReadOnly getTranslationVector()
   {
      return translationVector;
   }

   public void getTranslation(TupleBasics translationToPack)
   {
      translationToPack.set(translationVector);
   }

   public void invert()
   {
      quaternion.conjugate();
      quaternion.transform(translationVector);
      translationVector.negate();
   }

   public void multiply(QuaternionBasedTransform other)
   {
      QuaternionTools.addTransform(quaternion, other.getTranslationVector(), translationVector);
      quaternion.multiply(other.getQuaternion());
   }

   public void multiply(RigidBodyTransform rigidBodyTransform)
   {
      QuaternionTools.addTransform(quaternion, rigidBodyTransform.getTranslationVector(), translationVector);
      quaternion.multiply(rigidBodyTransform.getRotationMatrix());
   }

   public void preMultiply(QuaternionBasedTransform other)
   {
      QuaternionTools.transform(other.getQuaternion(), translationVector, translationVector);
      translationVector.add(other.getTranslationVector());
      quaternion.preMultiply(other.getQuaternion());
   }

   public void preMultiply(RigidBodyTransform rigidBodyTransform)
   {
      rigidBodyTransform.transform(translationVector);
      translationVector.add(rigidBodyTransform.getTranslationVector());
      quaternion.preMultiply(rigidBodyTransform.getRotationMatrix());
   }

   @Override
   public void transform(PointReadOnly pointOriginal, PointBasics pointTransformed)
   {
      quaternion.transform(pointOriginal, pointTransformed);
      pointTransformed.add(translationVector);
   }

   @Override
   public void transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
   {
      quaternion.transform(vectorOriginal, vectorTransformed);
   }

   @Override
   public void transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      quaternion.transform(quaternionOriginal, quaternionTransformed);
   }

   @Override
   public void transform(Vector4DReadOnly vector4DOriginal, Vector4DBasics vector4DTransformed)
   {
      quaternion.transform(vector4DOriginal, vector4DTransformed);
   }

   @Override
   public void transform(Matrix3DReadOnly<?> matrixOriginal, Matrix3D matrixTransformed)
   {
      quaternion.transform(matrixOriginal, matrixTransformed);
   }

   @Override
   public void transform(RotationMatrixReadOnly<?> matrixOriginal, RotationMatrix matrixTranformed)
   {
      quaternion.transform(matrixOriginal, matrixTranformed);
   }

   @Override
   public void transform(Point2DBasics point2DOriginal, Point2DBasics point2DTransformed, boolean checkIfTransformInXYPlane)
   {
      quaternion.transform(point2DOriginal, point2DTransformed, checkIfTransformInXYPlane);
      point2DTransformed.add(translationVector.getX(), translationVector.getY());
   }

   @Override
   public void transform(Vector2DReadOnly vector2DOriginal, Vector2DBasics vector2DTransformed, boolean checkIfTransformInXYPlane)
   {
      quaternion.transform(vector2DOriginal, vector2DTransformed, checkIfTransformInXYPlane);
   }

   public void inverseTransform(PointBasics pointToTransform)
   {
      inverseTransform(pointToTransform, pointToTransform);
   }

   public void inverseTransform(PointReadOnly pointOriginal, PointBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector);
      quaternion.inverseTransform(pointTransformed);
   }

   public void inverseTransform(VectorBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   public void inverseTransform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
   {
      quaternion.inverseTransform(vectorOriginal, vectorTransformed);
   }

   public void inverseTransform(Point2DBasics pointToTransform)
   {
      inverseTransform(pointToTransform, pointToTransform);
   }

   public void inverseTransform(Point2DBasics pointOriginal, Point2DBasics pointTransformed)
   {
      pointTransformed.set(pointOriginal);
      pointTransformed.sub(translationVector.getX(), translationVector.getY());
      quaternion.inverseTransform(pointTransformed);
   }

   public void inverseTransform(Vector2DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   public void inverseTransform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
   {
      quaternion.inverseTransform(vectorOriginal, vectorTransformed);
   }

   @Override
   public boolean epsilonEquals(QuaternionBasedTransform other, double epsilon)
   {
      return quaternion.epsilonEquals(other.quaternion, epsilon) && translationVector.epsilonEquals(other.translationVector, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((QuaternionBasedTransform) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   public boolean equals(QuaternionBasedTransform other)
   {
      if (other == null)
         return false;
      else
         return quaternion.equals(other.quaternion) && translationVector.equals(other.translationVector);
   }

   @Override
   public String toString()
   {
      return TransformTools.toString(quaternion, translationVector);
   }
}
