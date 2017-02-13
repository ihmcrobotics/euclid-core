package us.ihmc.geometry.matrix;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.geometry.exceptions.NotAMatrix2DException;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.Vector4D;

public class RotationScaleMatrixToolsTest
{
   private static final double EPS = 1.0e-10;

   @Test
   public void testTransformTuple() throws Exception
   {
      Random random = new Random(34534L);
      RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);

      Vector3D originalVector = GeometryBasicsRandomTools.generateRandomVector3D(random);
      Vector3D actualVector = new Vector3D();
      Vector3D expectedVector = new Vector3D();

      expectedVector.set(originalVector);
      expectedVector.scale(rotationScaleMatrix.getScaleX(), rotationScaleMatrix.getScaleY(), rotationScaleMatrix.getScaleZ());
      rotationScaleMatrix.getRotationMatrix().transform(expectedVector, expectedVector);

      rotationScaleMatrix.transform(originalVector, actualVector);
      GeometryBasicsTestTools.assertTuple3DEquals(expectedVector, actualVector, EPS);
   }

   @Test
   public void testTransformTuple2D() throws Exception
   {
      Random random = new Random(34534L);
      RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
      rotationScaleMatrix.setToYawMatrix(2.0 * Math.PI * random.nextDouble());
      rotationScaleMatrix.setScale(1.0 + random.nextDouble(), 1.0 + random.nextDouble(), 1.0);

      Vector2D originalVector = GeometryBasicsRandomTools.generateRandomVector2D(random);
      Vector2D actualVector = new Vector2D();
      Vector2D expectedVector = new Vector2D();

      expectedVector.set(originalVector);
      expectedVector.scale(rotationScaleMatrix.getScaleX(), rotationScaleMatrix.getScaleY());
      rotationScaleMatrix.getRotationMatrix().transform(expectedVector, expectedVector, true);

      rotationScaleMatrix.transform(originalVector, actualVector, true);
      GeometryBasicsTestTools.assertTuple2DEquals(expectedVector, actualVector, EPS);

      try
      {
         RotationScaleMatrix randomRotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
         randomRotationScaleMatrix.transform(originalVector, actualVector, true);
         fail("Should have thrown an excetpion");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
   }

   @Test
   public void testTransformQuaternion() throws Exception
   {
      Random random = new Random(34534L);
      RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);

      Quaternion originalQuaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
      Quaternion actualQuaternion = new Quaternion();
      Quaternion expectedQuaternion = new Quaternion();

      rotationScaleMatrix.getRotationMatrix().transform(originalQuaternion, expectedQuaternion);
      rotationScaleMatrix.transform(originalQuaternion, actualQuaternion);
      GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
   }

   @Test
   public void testTransformVector4D() throws Exception
   {
      Random random = new Random(34534L);
      RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);

      Vector4D originalVector = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D actualVector = new Vector4D();
      Vector4D expectedVector = new Vector4D();

      expectedVector.set(originalVector);
      expectedVector.scale(rotationScaleMatrix.getScaleX(), rotationScaleMatrix.getScaleY(), rotationScaleMatrix.getScaleZ(), 1.0);
      rotationScaleMatrix.getRotationMatrix().transform(expectedVector, expectedVector);

      rotationScaleMatrix.transform(originalVector, actualVector);
      GeometryBasicsTestTools.assertTuple4DEquals(expectedVector, actualVector, EPS);
   }

   @Test
   public void testTransformRotationMatrix() throws Exception
   {
      Random random = new Random(34534L);
      RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);

      RotationMatrix originalRotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      RotationMatrix actualRotationMatrix = new RotationMatrix();
      RotationMatrix expectedRotationMatrix = new RotationMatrix();

      expectedRotationMatrix.set(originalRotationMatrix);
      expectedRotationMatrix.preMultiply(rotationScaleMatrix.getRotationMatrix());

      rotationScaleMatrix.transform(originalRotationMatrix, actualRotationMatrix);
      GeometryBasicsTestTools.assertMatrix3DEquals(expectedRotationMatrix, actualRotationMatrix, EPS);
   }

   @Test
   public void testTransformMatrix3D() throws Exception
   {
      Random random = new Random(34534L);
      RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);

      Matrix3D originalMatrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
      Matrix3D actualMatrix = new Matrix3D();
      Matrix3D expectedMatrix = new Matrix3D();

      // M' = (RS) * M * (RS)^-1
      expectedMatrix.set(originalMatrix);
      expectedMatrix.preMultiply(rotationScaleMatrix);
      expectedMatrix.multiplyInvertOther(rotationScaleMatrix);

      rotationScaleMatrix.transform(originalMatrix, actualMatrix);
      GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, actualMatrix, EPS);
   }

   @Test
   public void testInverseTransformTuple() throws Exception
   {
      Random random = new Random(24534L);
      RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);

      Vector3D originalVector = GeometryBasicsRandomTools.generateRandomVector3D(random);
      Vector3D actualVector = new Vector3D();
      Vector3D expectedVector = new Vector3D(originalVector);

      rotationScaleMatrix.transform(originalVector, actualVector);
      assertFalse(expectedVector.epsilonEquals(actualVector, EPS));
      rotationScaleMatrix.inverseTransform(actualVector, actualVector);
      GeometryBasicsTestTools.assertTuple3DEquals(expectedVector, actualVector, EPS);
   }

   @Test
   public void testInverseTransformTuple2D() throws Exception
   {
      Random random = new Random(24534L);
      RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
      rotationScaleMatrix.setToYawMatrix(2.0 * Math.PI * random.nextDouble());
      rotationScaleMatrix.setScale(1.0 + random.nextDouble(), 1.0 + random.nextDouble(), 1.0);

      Vector2D originalVector = GeometryBasicsRandomTools.generateRandomVector2D(random);
      Vector2D actualVector = new Vector2D();
      Vector2D expectedVector = new Vector2D(originalVector);

      rotationScaleMatrix.transform(originalVector, actualVector, true);
      assertFalse(expectedVector.epsilonEquals(actualVector, EPS));
      rotationScaleMatrix.inverseTransform(actualVector, actualVector, true);
      GeometryBasicsTestTools.assertTuple2DEquals(expectedVector, actualVector, EPS);

      try
      {
         RotationScaleMatrix randomRotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);
         randomRotationScaleMatrix.inverseTransform(originalVector, actualVector, true);
         fail("Should have thrown an excetpion");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
   }

   @Test
   public void testInverseTransformTuple4D() throws Exception
   {
      Random random = new Random(24534L);
      RotationScaleMatrix rotationScaleMatrix = GeometryBasicsRandomTools.generateRandomRotationScaleMatrix(random, 10.0);

      Vector4D originalVector = GeometryBasicsRandomTools.generateRandomVector4D(random);
      Vector4D actualVector = new Vector4D();
      Vector4D expectedVector = new Vector4D(originalVector);

      rotationScaleMatrix.transform(originalVector, actualVector);
      assertFalse(expectedVector.epsilonEquals(actualVector, EPS));
      rotationScaleMatrix.inverseTransform(actualVector, actualVector);
      GeometryBasicsTestTools.assertTuple4DEquals(expectedVector, actualVector, EPS);
   }
}
