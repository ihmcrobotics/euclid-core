package us.ihmc.geometry.tuple4D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.exceptions.NotAMatrix2DException;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.transform.AffineTransform;
import us.ihmc.geometry.transform.QuaternionBasedTransform;
import us.ihmc.geometry.transform.RigidBodyTransform;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.RotationVectorConversion;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public abstract class QuaternionBasicsTest<T extends QuaternionBasics> extends Tuple4DBasicsTest<T>
{
   // Read-only part
   @Test
   public void testIsUnitary()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T q1 = createRandomTuple(random);

         assertTrue(q1.isUnitary(getEpsilon())); // Quaternion should have norm = 1

         T q2 = createRandomTuple(random);
         q1 = createTuple(q2.getX(), q2.getY(), q2.getZ(), q2.getS());
         assertTrue(q1.isUnitary(getEpsilon()));

         double delta = 2.0 * Math.sqrt(getEpsilon());

         q1 = createTuple(delta + q2.getX(), q2.getY(), q2.getZ(), q2.getS());
         assertFalse(q1.isUnitary(getEpsilon()));
         q1 = createTuple(q2.getX(), delta + q2.getY(), q2.getZ(), q2.getS());
         assertFalse(q1.isUnitary(getEpsilon()));
         q1 = createTuple(q2.getX(), q2.getY(), delta + q2.getZ(), q2.getS());
         assertFalse(q1.isUnitary(getEpsilon()));
         q1 = createTuple(q2.getX(), q2.getY(), q2.getZ(), delta + q2.getS());
         assertFalse(q1.isUnitary(getEpsilon()));
      }
   }

   @Test
   public void testIsQuaternionZOnly() throws Exception
   {
      Random random = new Random(23905872L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double qx = random.nextDouble();
         double qy = random.nextDouble();
         double qz = random.nextDouble();
         double qs = random.nextDouble();
         T quaternion = createEmptyTuple();

         quaternion.set(qx, qy, qz, qs);
         assertFalse(quaternion.isZOnly(getEpsilon()));

         quaternion.set(0.0, qy, qz, qs);
         assertFalse(quaternion.isZOnly(getEpsilon()));

         quaternion.set(qx, 0.0, qz, qs);
         assertFalse(quaternion.isZOnly(getEpsilon()));

         quaternion.set(0.0, 0.0, qz, qs);
         assertTrue(quaternion.isZOnly(getEpsilon()));

         quaternion.set(2.0 * getEpsilon(), 0.0, qz, qs);
         assertFalse(quaternion.isZOnly(getEpsilon()));
         quaternion.set(0.0, 2.0 * getEpsilon(), qz, qs);
         assertFalse(quaternion.isZOnly(getEpsilon()));
      }
   }

   @Test
   public void testCheckIfIsZOnly() throws Exception
   {
      Random random = new Random(23905872L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double qx = random.nextDouble();
         double qy = random.nextDouble();
         double qz = random.nextDouble();
         double qs = random.nextDouble();
         T quaternion = createEmptyTuple();

         quaternion.set(qx, qy, qz, qs);

         try
         {
            quaternion.checkIfIsZOnly(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         quaternion.set(0.0, qy, qz, qs);
         try
         {
            quaternion.checkIfIsZOnly(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         quaternion.set(qx, 0.0, qz, qs);
         try
         {
            quaternion.checkIfIsZOnly(getEpsilon());
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a RuntimeException");
         }

         quaternion.set(0.0, 0.0, qz, qs);
         quaternion.checkIfIsZOnly(getEpsilon());
      }
   }

   @Test
   public void testGetAngle()
   {
      Random random = new Random(65445L);
      double expectedAngle = 2.0 * Math.PI * random.nextDouble(); // Sign issue when theta < 0.0
      double c = Math.cos(expectedAngle / 2.0);
      double s = Math.sin(expectedAngle / 2.0);
      Vector3D axis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      Quaternion q = new Quaternion();
      double qx = s * axis.getX();
      double qy = s * axis.getY();
      double qz = s * axis.getZ();
      double qs = c;
      q.setUnsafe(qx, qy, qz, qs);

      assertEquals(expectedAngle, q.getAngle(), getEpsilon());
   }

   @Test
   public void testGetRotationVector() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T quaternion = createRandomTuple(random);
         Vector3D expectedRotationVector = new Vector3D();
         Vector3D actualRotationVector = new Vector3D();

         RotationVectorConversion.convertQuaternionToRotationVector(quaternion, expectedRotationVector);
         quaternion.get(actualRotationVector);
         GeometryBasicsTestTools.assertTuple3DEquals(expectedRotationVector, actualRotationVector, getEpsilon());
      }
   }

   @Test
   public void testGetYawPitchRoll()
   {
      Random random = new Random(654651351L);
      Quaternion quaternion;
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         { // Test getYawPitchRoll(double[] yawPitchRollToPack)
            double[] yawPitchRoll = new double[4];
            quaternion.getYawPitchRoll(yawPitchRoll);
            double[] expectedYawPitchRoll = new double[4];
            YawPitchRollConversion.convertQuaternionToYawPitchRoll(quaternion, expectedYawPitchRoll);

            for (int j = 0; j < yawPitchRoll.length; j++)
               Assert.assertEquals(yawPitchRoll[j], expectedYawPitchRoll[j], getEpsilon());
         }

         { // Test getYaw()
            double yaw = quaternion.getYaw();
            double expectedYaw = YawPitchRollConversion.computeYaw(quaternion);
            Assert.assertEquals(yaw, expectedYaw, getEpsilon());
         }

         { // Test getPitch()
            double pitch = quaternion.getPitch();
            double expectedPitch = YawPitchRollConversion.computePitch(quaternion);
            Assert.assertEquals(pitch, expectedPitch, getEpsilon());
         }

         { // Test getRoll()
            double roll = quaternion.getRoll();
            double expectedRoll = YawPitchRollConversion.computeRoll(quaternion);
            Assert.assertEquals(roll, expectedRoll, getEpsilon());
         }
      }
   }

   @Test
   public void testTransform()
   {
      Random random = new Random(6787L);
      T quaternion = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleToTransform)
         Tuple3DReadOnly tuple = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = GeometryBasicsRandomTools.generateRandomVector3D(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.transform(quaternion, tuple, expectedTuple);
         quaternion.transform(actualTuple);

         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleOriginal, TupleBasics tupleTransformed)
         Tuple3DReadOnly tuple = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = GeometryBasicsRandomTools.generateRandomVector3D(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.transform(quaternion, tuple, expectedTuple);
         quaternion.transform(tuple, actualTuple);

         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         Tuple2DReadOnly tuple = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = GeometryBasicsRandomTools.generateRandomVector2D(random);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = createTuple(0.0f, 0.0f, qz, qs);

         QuaternionTools.transform(quaternion, tuple, expectedTuple, false);
         quaternion.transform(actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         quaternion.transform(actualTuple, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         quaternion.transform(actualTuple, false);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         Tuple2DReadOnly tuple = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = GeometryBasicsRandomTools.generateRandomVector2D(random);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = createTuple(0.0f, 0.0f, qz, qs);

         QuaternionTools.transform(quaternion, tuple, expectedTuple, false);
         quaternion.transform(tuple, actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         quaternion.transform(tuple, actualTuple, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         quaternion.transform(tuple, actualTuple, false);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      // Test exceptions
      try
      {
         quaternion = createRandomTuple(random);
         quaternion.transform(new Vector2D());
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("Should have thrown a NotAMatrix2DException.");
      }

      try
      {
         quaternion = createRandomTuple(random);
         quaternion.transform(new Vector2D(), new Vector2D());
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("Should have thrown a NotAMatrix2DException.");
      }
      try
      {
         quaternion = createRandomTuple(random);
         quaternion.transform(new Vector2D(), true);
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("Should have thrown a NotAMatrix2DException.");
      }

      try
      {
         quaternion = createRandomTuple(random);
         quaternion.transform(new Vector2D(), new Vector2D(), true);
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("Should have thrown a NotAMatrix2DException.");
      }

      {// Test transform quaternion
         quaternion = createRandomTuple(random);

         Quaternion qOriginal = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Quaternion qExpected = new Quaternion();
         Quaternion qActual = new Quaternion();

         qExpected.multiply(quaternion, qOriginal);

         quaternion.transform(qOriginal, qActual);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

         qActual.set(qOriginal);
         quaternion.transform(qActual);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());
      }

      {// Test transform Vector4D
         quaternion = createRandomTuple(random);

         Vector4D vectorOriginal = GeometryBasicsRandomTools.generateRandomVector4D(random);
         Vector4D vectorExpected = new Vector4D();
         Vector4D vectorActual = new Vector4D();

         QuaternionTools.transform(quaternion, vectorOriginal, vectorExpected);

         quaternion.transform(vectorOriginal, vectorActual);
         GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, getEpsilon());

         vectorActual.set(vectorOriginal);
         quaternion.transform(vectorActual);
         GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, getEpsilon());
      }

      { // Test transform Matrix3D
         quaternion = createRandomTuple(random);

         Matrix3D matrixOriginal = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         Matrix3D matrixExpected = new Matrix3D();
         Matrix3D matrixActual = new Matrix3D();

         QuaternionTools.transform(quaternion, matrixOriginal, matrixExpected);

         quaternion.transform(matrixOriginal, matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual.set(matrixOriginal);
         quaternion.transform(matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());
      }

      { // Test transform RotationMatrix
         quaternion = createRandomTuple(random);

         RotationMatrix matrixOriginal = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix matrixExpected = new RotationMatrix();
         RotationMatrix matrixActual = new RotationMatrix();

         QuaternionTools.transform(quaternion, matrixOriginal, matrixExpected);

         quaternion.transform(matrixOriginal, matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual.set(matrixOriginal);
         quaternion.transform(matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());
      }
   }

   @Test
   public void testInverseTransform()
   {
      Random random = new Random(6787L);
      T quaternion = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleToTransform)
         Tuple3DReadOnly tuple = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = GeometryBasicsRandomTools.generateRandomVector3D(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple);
         quaternion.inverseTransform(actualTuple);

         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleOriginal, TupleBasics tupleTransformed)
         Tuple3DReadOnly tuple = GeometryBasicsRandomTools.generateRandomVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = GeometryBasicsRandomTools.generateRandomVector3D(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple);
         quaternion.inverseTransform(tuple, actualTuple);

         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         Tuple2DReadOnly tuple = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = GeometryBasicsRandomTools.generateRandomVector2D(random);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = createTuple(0.0f, 0.0f, qz, qs);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple, false);
         quaternion.inverseTransform(actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         quaternion.inverseTransform(actualTuple, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         quaternion.inverseTransform(actualTuple, false);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         Tuple2DReadOnly tuple = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = GeometryBasicsRandomTools.generateRandomVector2D(random);
         double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);
         float qz = (float) Math.sin(0.5 * theta);
         float qs = (float) Math.cos(0.5 * theta);
         quaternion = createTuple(0.0f, 0.0f, qz, qs);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple, false);
         quaternion.inverseTransform(tuple, actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         quaternion.inverseTransform(tuple, actualTuple, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         quaternion.inverseTransform(tuple, actualTuple, false);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      // Test exceptions
      try
      {
         quaternion = createRandomTuple(random);
         quaternion.inverseTransform(new Vector2D());
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("Should have thrown a NotAMatrix2DException.");
      }

      try
      {
         quaternion = createRandomTuple(random);
         quaternion.inverseTransform(new Vector2D(), new Vector2D());
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("Should have thrown a NotAMatrix2DException.");
      }
      try
      {
         quaternion = createRandomTuple(random);
         quaternion.inverseTransform(new Vector2D(), true);
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("Should have thrown a NotAMatrix2DException.");
      }

      try
      {
         quaternion = createRandomTuple(random);
         quaternion.inverseTransform(new Vector2D(), new Vector2D(), true);
         fail("Should have thrown a NotAMatrix2DException.");
      }
      catch (NotAMatrix2DException e)
      {
         // good
      }
      catch (Exception e)
      {
         fail("Should have thrown a NotAMatrix2DException.");
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(QuaternionBasics quaternionToTransform)
         QuaternionReadOnly original = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         QuaternionBasics actual = new Quaternion(original);
         QuaternionBasics expected = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         quaternion.inverseTransform(actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
         QuaternionReadOnly original = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         QuaternionBasics actual = new Quaternion(original);
         QuaternionBasics expected = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         quaternion.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Vector4DBasics vectorToTransform)
         Vector4DReadOnly original = GeometryBasicsRandomTools.generateRandomVector4D(random);
         Vector4DBasics actual = new Vector4D(original);
         Vector4DBasics expected = GeometryBasicsRandomTools.generateRandomVector4D(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         quaternion.inverseTransform(actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4DReadOnly original = GeometryBasicsRandomTools.generateRandomVector4D(random);
         Vector4DBasics actual = new Vector4D(original);
         Vector4DBasics expected = GeometryBasicsRandomTools.generateRandomVector4D(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         quaternion.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Matrix3D matrixToTransform)
         Matrix3DReadOnly original = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         Matrix3D actual = new Matrix3D(original);
         Matrix3D expected = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         quaternion.inverseTransform(actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3DReadOnly original = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         Matrix3D actual = new Matrix3D(original);
         Matrix3D expected = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         quaternion.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrix matrixToTransform)
         RotationMatrixReadOnly original = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(original);
         RotationMatrix expected = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         quaternion.inverseTransform(actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
         RotationMatrixReadOnly original = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(original);
         RotationMatrix expected = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         quaternion = createRandomTuple(random);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         quaternion.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }
   }

   // Basics part

   @Override
   @Test
   public void testNegate() throws Exception
   {
      super.testNegate();

      // Redo the test to make sure setAndNegate(QuaternionReadOnly other) is called.
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (double signX = -1.0; signX <= 1.0; signX += 2.0)
      {
         for (double signY = -1.0; signY <= 1.0; signY += 2.0)
         {
            for (double signZ = -1.0; signZ <= 1.0; signZ += 2.0)
            {
               for (double signS = -1.0; signS <= 1.0; signS += 2.0)
               {
                  T original = createRandomTuple(random);
                  double xOriginal = signX * original.getX();
                  double yOriginal = signY * original.getY();
                  double zOriginal = signZ * original.getZ();
                  double sOriginal = signS * original.getS();
                  tuple1 = createTuple(xOriginal, yOriginal, zOriginal, sOriginal);

                  tuple2.setToNaN();
                  tuple2.setAndNegate(tuple1);
                  assertEquals(tuple2.getX(), -xOriginal, getEpsilon());
                  assertEquals(tuple2.getY(), -yOriginal, getEpsilon());
                  assertEquals(tuple2.getZ(), -zOriginal, getEpsilon());
                  assertEquals(tuple2.getS(), -sOriginal, getEpsilon());
                  assertEquals(tuple1.getX(), xOriginal, getEpsilon());
                  assertEquals(tuple1.getY(), yOriginal, getEpsilon());
                  assertEquals(tuple1.getZ(), zOriginal, getEpsilon());
                  assertEquals(tuple1.getS(), sOriginal, getEpsilon());

                  tuple1.negate();
                  assertEquals(tuple1.getX(), -xOriginal, getEpsilon());
                  assertEquals(tuple1.getY(), -yOriginal, getEpsilon());
                  assertEquals(tuple1.getZ(), -zOriginal, getEpsilon());
                  assertEquals(tuple1.getS(), -sOriginal, getEpsilon());
               }
            }
         }
      }
   }

   @Override
   @Test
   public void testNormalize()
   {
      super.testNormalize();

      Quaternion expected = new Quaternion();
      Quaternion actual = new Quaternion();
      actual.setUnsafe(0.0, 0.0, 0.0, 0.0);
      actual.normalize();
      GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(621541L);
      T quaternion = createRandomTuple(random);
      quaternion.setToZero();
      T zeroQ = createTuple(0.0, 0.0, 0.0, 1.0);

      GeometryBasicsTestTools.assertQuaternionEquals(quaternion, zeroQ, getEpsilon());
   }

   @Test
   public void testConjugate()
   {
      Random random = new Random(65445L);
      T quaternion;
      T quaternionCopy = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         quaternion = createRandomTuple(random);
         quaternionCopy.set(quaternion);

         { // Test conjugate()
            quaternion.conjugate();

            assertEquals(quaternion.getX(), -quaternionCopy.getX(), getEpsilon());
            assertEquals(quaternion.getY(), -quaternionCopy.getY(), getEpsilon());
            assertEquals(quaternion.getZ(), -quaternionCopy.getZ(), getEpsilon());
            assertEquals(quaternion.getS(), quaternionCopy.getS(), getEpsilon());
         }

         { // Test conjugate (QuaternionBasics other)
            T quaternion2 = createEmptyTuple();
            quaternion2.setAndConjugate(quaternionCopy);

            assertTrue(quaternion2.getX() == -quaternionCopy.getX());
            assertTrue(quaternion2.getY() == -quaternionCopy.getY());
            assertTrue(quaternion2.getZ() == -quaternionCopy.getZ());
            assertTrue(quaternion2.getS() == quaternionCopy.getS());
         }
      }
   }

   @Test
   public void testInverse()
   {
      Random random = new Random(15461L);

      Vector3D axis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

      double sinHalfTheta = Math.sin(theta / 2.0);
      double cosHalfTheta = Math.cos(theta / 2.0);

      double qx = axis.getX() * sinHalfTheta;
      double qy = axis.getY() * sinHalfTheta;
      double qz = axis.getZ() * sinHalfTheta;
      double qs = cosHalfTheta;

      // Test that it computes the inverse
      T qExpected = createEmptyTuple();
      qExpected.setUnsafe(-qx, -qy, -qz, qs);
      T qActual = createEmptyTuple();
      qActual.setUnsafe(qx, qy, qz, qs);
      qActual.inverse();

      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

      // Test that the quaternion is normalized
      double scale = random.nextDouble();
      qActual.setUnsafe(scale * qx, scale * qy, scale * qz, scale * qs);
      qActual.inverse();

      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

      // Test that the quaternion is not kept within [-Pi, Pi]
      theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI, 2.0 * Math.PI);
      sinHalfTheta = Math.sin(theta / 2.0);
      cosHalfTheta = Math.cos(theta / 2.0);

      qx = axis.getX() * sinHalfTheta;
      qy = axis.getY() * sinHalfTheta;
      qz = axis.getZ() * sinHalfTheta;
      qs = cosHalfTheta;

      qExpected.setUnsafe(-qx, -qy, -qz, qs);
      qActual.setUnsafe(qx, qy, qz, qs);
      qActual.inverse();
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

      // Test that setAndInverse() does "set" and "inverse"
      T qOriginal = createRandomTuple(random);
      qExpected.set(qOriginal);
      qExpected.inverse();
      qActual.setAndInverse(qOriginal);
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());
   }

   @Test
   public void testNormalizeAndLimitToPiMinusPi()
   {
      Random random = new Random(15461L);

      Vector3D axis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      double theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

      double sinHalfTheta = Math.sin(theta / 2.0);
      double cosHalfTheta = Math.cos(theta / 2.0);

      double qx = axis.getX() * sinHalfTheta;
      double qy = axis.getY() * sinHalfTheta;
      double qz = axis.getZ() * sinHalfTheta;
      double qs = cosHalfTheta;

      // Test that it does not mess up a quaternion already normalized
      T qExpected = createEmptyTuple();
      qExpected.setUnsafe(qx, qy, qz, qs);
      T qActual = createEmptyTuple();
      qActual.setUnsafe(qx, qy, qz, qs);
      qActual.normalizeAndLimitToPiMinusPi();

      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

      // Test that the quaternion is normalized
      double scale = random.nextDouble();
      qActual.setUnsafe(scale * qx, scale * qy, scale * qz, scale * qs);
      qActual.normalizeAndLimitToPiMinusPi();

      assertEquals(1.0, qActual.norm(), getEpsilon());
      GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

      // Test that the quaternion is kept within [-Pi, Pi]
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         theta = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI, 2.0 * Math.PI);
         sinHalfTheta = Math.sin(theta / 2.0);
         cosHalfTheta = Math.cos(theta / 2.0);

         qx = axis.getX() * sinHalfTheta;
         qy = axis.getY() * sinHalfTheta;
         qz = axis.getZ() * sinHalfTheta;
         qs = cosHalfTheta;

         qExpected.setUnsafe(qx, qy, qz, qs);
         qActual.setUnsafe(qx, qy, qz, qs);
         qActual.normalizeAndLimitToPiMinusPi();
         if (Math.abs(qExpected.getAngle()) < Math.PI)
         {
            GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());
         }
         else
         {
            assertTrue(Math.abs(qActual.getAngle()) < Math.PI);
            qExpected.negate();
            GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());
         }
      }
   }

   @Override
   public void testSetDoubles()
   {
      Random random = new Random(621541L);
      T quaternion = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(double x, double y, double z, double s);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         quaternion.set(x, y, z, s);

         // The method should normalize, so assertNotEquals is used.
         assertNotEquals(quaternion.getX(), x, getEpsilon());
         assertNotEquals(quaternion.getY(), y, getEpsilon());
         assertNotEquals(quaternion.getZ(), z, getEpsilon());
         assertNotEquals(quaternion.getS(), s, getEpsilon());
         assertEquals(1.0, quaternion.norm(), getEpsilon());

         T original = createRandomTuple(random);
         x = original.getX();
         y = original.getY();
         z = original.getZ();
         s = original.getS();
         quaternion.set(x, y, z, s);
         GeometryBasicsTestTools.assertQuaternionEquals(original, quaternion, getEpsilon());
      }
   }

   @Test
   public void testSetAxisAngle()
   {
      Random random = new Random(574631L);
      T actualQuaternion = createEmptyTuple();
      T expectedQuaternion = createEmptyTuple();
      AxisAngle axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(AxisAngleReadOnly axisAngle)
         axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random);

         QuaternionConversion.convertAxisAngleToQuaternion(axisAngle, expectedQuaternion);

         actualQuaternion.set(axisAngle);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());
      }
   }

   @Test
   public void testSetRotationMatrix()
   {
      Random random = new Random(574631L);
      T actualQuaternion = createEmptyTuple();
      T expectedQuaternion = createEmptyTuple();
      RotationMatrix rotationMatrix;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(RotationMatrixReadOnly rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, expectedQuaternion);

         actualQuaternion.set(rotationMatrix);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());
      }
   }

   @Test
   public void testSetRotationVector()
   {
      Random random = new Random(574631L);
      T actualQuaternion = createEmptyTuple();
      T expectedQuaternion = createEmptyTuple();
      Vector3D rotationVector;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test set(Vector3DReadOnly rotationVector)
         rotationVector = GeometryBasicsRandomTools.generateRandomRotationVector(random);

         QuaternionConversion.convertRotationVectorToQuaternion(rotationVector, expectedQuaternion);

         actualQuaternion.set(rotationVector);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());
      }
   }

   @Test
   public void testSetYawPitchRoll()
   {
      Random random = new Random(574631L);
      T actualQuaternion = createEmptyTuple();
      T expectedQuaternion = createEmptyTuple();
      double[] yawPitchRoll;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setYawPitchRoll(double[] yawPitchRoll)
         yawPitchRoll = GeometryBasicsRandomTools.generateRandomYawPitchRoll(random);

         QuaternionConversion.convertYawPitchRollToQuaternion(yawPitchRoll, expectedQuaternion);

         actualQuaternion.setYawPitchRoll(yawPitchRoll);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());

         actualQuaternion.setToZero();
         actualQuaternion.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());
      }
   }

   @Test
   public void testSetEuler()
   {
      Random random = new Random(574631L);
      T actualQuaternion = createEmptyTuple();
      T expectedQuaternion = createEmptyTuple();
      Vector3D eulerAngles;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setEuler(Vector3DReadOnly eulerAngles)
         eulerAngles = GeometryBasicsRandomTools.generateRandomRotationVector(random);

         QuaternionConversion.convertYawPitchRollToQuaternion(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), expectedQuaternion);

         actualQuaternion.setEuler(eulerAngles);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());

         actualQuaternion.setToZero();
         actualQuaternion.setEuler(eulerAngles.getX(), eulerAngles.getY(), eulerAngles.getZ());
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());
      }
   }

   @Test
   public void testSetYawQuaternion()
   {
      Random random = new Random(574631L);
      T actualQuaternion = createEmptyTuple();
      T expectedQuaternion = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setToYawQuaternion(double yaw)
         double yaw = GeometryBasicsRandomTools.generateRandomDouble(random, 2.0 * Math.PI);

         actualQuaternion.setToYawQuaternion(yaw);
         expectedQuaternion.set(new AxisAngle(0.0, 0.0, 1.0, yaw));
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());
      }
   }

   @Test
   public void testSetPitchQuaternion()
   {
      Random random = new Random(574631L);
      T actualQuaternion = createEmptyTuple();
      T expectedQuaternion = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setToPitchQuaternion(double pitch)
         double pitch = GeometryBasicsRandomTools.generateRandomDouble(random, 2.0 * Math.PI);

         actualQuaternion.setToPitchQuaternion(pitch);
         expectedQuaternion.set(new AxisAngle(0.0, 1.0, 0.0, pitch));
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());
      }
   }

   @Test
   public void testSetRollQuaternion()
   {
      Random random = new Random(574631L);
      T actualQuaternion = createEmptyTuple();
      T expectedQuaternion = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setToRollQuaternion(double roll)
         double roll = GeometryBasicsRandomTools.generateRandomDouble(random, 2.0 * Math.PI);

         actualQuaternion.setToRollQuaternion(roll);
         expectedQuaternion.set(new AxisAngle(1.0, 0.0, 0.0, roll));
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, getEpsilon());
      }
   }

   @Test
   public void testDifference()
   {
      Random random = new Random(65445L);
      T diff = createEmptyTuple();
      T expected = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T q1 = createRandomTuple(random);
         T q2 = createRandomTuple(random);

         diff.difference(q1, q2);
         QuaternionTools.multiplyConjugateLeft(q1, q2, expected);

         GeometryBasicsTestTools.assertQuaternionEquals(diff, expected, getEpsilon());
      }
   }

   @Test
   public void testMultiply()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T qOther1 = createRandomTuple(random);
         T qOther2 = createRandomTuple(random);
         T qActual = createRandomTuple(random);
         T qExpected = createEmptyTuple();

         { // Test multiply(QuaternionBasics other)
            qActual.set(qOther1);
            qExpected.set(qOther1);
            qActual.multiply(qOther2);
            QuaternionTools.multiply(qExpected, qOther2, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }

         { // Test multiply(QuaternionBasics q1, QuaternionBasics q2)
            qActual.multiply(qOther1, qOther2);
            QuaternionTools.multiply(qOther1, qOther2, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }

         { // Test multiply(RotationMatrixReadOnly matrix)
            RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

            qActual.set(qOther1);
            qExpected.set(qOther1);

            qActual.multiply(matrix);
            QuaternionTools.multiply(qExpected, matrix, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }
      }
   }

   @Test
   public void testMultiplyConjugate()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T qOther1 = createRandomTuple(random);
         T qOther2 = createRandomTuple(random);
         T qActual = createRandomTuple(random);
         T qExpected = createEmptyTuple();

         { // Test multiplyConjugateThis(QuaternionBasics other)
            qActual.set(qOther1);
            qExpected.set(qOther1);
            qActual.multiplyConjugateThis(qOther2);
            QuaternionTools.multiplyConjugateLeft(qExpected, qOther2, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }

         { // Test multiplyConjugateOther(QuaternionBasics other)
            qActual.set(qOther1);
            qExpected.set(qOther1);
            qActual.multiplyConjugateOther(qOther2);
            QuaternionTools.multiplyConjugateRight(qExpected, qOther2, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }
      }
   }

   @Test
   public void testAppendYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);
      
      T expected = createEmptyTuple();
      T actual = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendYawRotation(double yaw)
         T original = createRandomTuple(random);
         T yawRotation = createEmptyTuple();
         double yaw = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

         yawRotation.setToYawQuaternion(yaw);
         QuaternionTools.multiply(original, yawRotation, expected);

         actual.set(original);
         actual.appendYawRotation(yaw);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         T original = createRandomTuple(random);
         T pitchRotation = createEmptyTuple();
         double pitch = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

         pitchRotation.setToPitchQuaternion(pitch);
         QuaternionTools.multiply(original, pitchRotation, expected);

         actual.set(original);
         actual.appendPitchRotation(pitch);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendRollRotation(double roll)
         T original = createRandomTuple(random);
         T rollRotation = createEmptyTuple();
         double roll = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

         rollRotation.setToRollQuaternion(roll);
         QuaternionTools.multiply(original, rollRotation, expected);

         actual.set(original);
         actual.appendRollRotation(roll);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPrependYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);
      
      T expected = createEmptyTuple();
      T actual = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // prependYawRotation(double yaw)
         T original = createRandomTuple(random);
         T yawRotation = createEmptyTuple();
         double yaw = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

         yawRotation.setToYawQuaternion(yaw);
         QuaternionTools.multiply(yawRotation, original, expected);

         actual.set(original);
         actual.prependYawRotation(yaw);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // prependPitchRotation(double pitch)
         T original = createRandomTuple(random);
         T pitchRotation = createEmptyTuple();
         double pitch = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

         pitchRotation.setToPitchQuaternion(pitch);
         QuaternionTools.multiply(pitchRotation, original, expected);

         actual.set(original);
         actual.prependPitchRotation(pitch);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // prependRollRotation(double roll)
         T original = createRandomTuple(random);
         T rollRotation = createEmptyTuple();
         double roll = GeometryBasicsRandomTools.generateRandomDouble(random, Math.PI);

         rollRotation.setToRollQuaternion(roll);
         QuaternionTools.multiply(rollRotation, original, expected);

         actual.set(original);
         actual.prependRollRotation(roll);

         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testPreMultiply()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T qOther1 = createRandomTuple(random);
         T qOther2 = createRandomTuple(random);
         T qActual = createRandomTuple(random);
         T qExpected = createEmptyTuple();

         { // Test preMultiply(QuaternionBasics other)
            qActual.set(qOther1);
            qExpected.set(qOther1);
            qActual.preMultiply(qOther2);
            QuaternionTools.multiply(qOther2, qExpected, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }

         { // Test preMultiply(RotationMatrixReadOnly matrix)
            RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

            qActual.set(qOther1);
            qExpected.set(qOther1);
            qActual.preMultiply(matrix);
            QuaternionTools.multiply(matrix, qExpected, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }
      }
   }

   @Test
   public void testPreMultiplyConjugate()
   {
      Random random = new Random(65445L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T qOther1 = createRandomTuple(random);
         T qOther2 = createRandomTuple(random);
         T qActual = createRandomTuple(random);
         T qExpected = createEmptyTuple();

         { // Test preMultiplyConjugateThis(QuaternionBasics other)
            qActual.set(qOther1);
            qExpected.set(qOther1);
            qActual.preMultiplyConjugateThis(qOther2);
            QuaternionTools.multiplyConjugateRight(qOther2, qExpected, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }

         { // Test multiplyConjugateOther(QuaternionBasics other)
            qActual.set(qOther1);
            qExpected.set(qOther1);
            qActual.preMultiplyConjugateOther(qOther2);
            QuaternionTools.multiplyConjugateLeft(qOther2, qExpected, qExpected);

            GeometryBasicsTestTools.assertQuaternionEquals(qActual, qExpected, getEpsilon());
         }
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(723459L);
      T q0 = createEmptyTuple();
      T qf = createEmptyTuple();
      T qActual = createEmptyTuple();
      T qExpected = createEmptyTuple();
      double epsilon = 10.0 * getEpsilon();

      // Check that interpolating with two zero angle quaternions, we obtain a zero angle quaternion.
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double alpha = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         qActual.interpolate(q0, qf, alpha);
         qExpected.setToZero();
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, epsilon);
         GeometryBasicsTestTools.assertQuaternionIsSetToZero(q0);
         GeometryBasicsTestTools.assertQuaternionIsSetToZero(qf);

         qActual.set(q0);
         qActual.interpolate(qf, alpha);
         qExpected.setToZero();
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, epsilon);
         GeometryBasicsTestTools.assertQuaternionIsSetToZero(q0);
         GeometryBasicsTestTools.assertQuaternionIsSetToZero(qf);
      }

      // Check that when q0 == qf, qActual == q0 == qf
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         q0 = createRandomTuple(random);
         qf.set(q0);
         double alpha = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);
         qActual.interpolate(q0, qf, alpha);
         qExpected.set(q0);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, epsilon);

         qActual.set(q0);
         qActual.interpolate(qf, alpha);
         qExpected.set(q0);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, epsilon);
      }

      // Simplify the interpolation by making q0 and qf describe rotation of different angle but around the same axis.
      // Such that the interpolation becomes a simple interpolation over the angle.
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double angle0 = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0);
         double anglef = GeometryBasicsRandomTools.generateRandomDouble(random, 1.0);
         Vector3D axis = GeometryBasicsRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angle0, q0);
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), anglef, qf);
         double alpha = GeometryBasicsRandomTools.generateRandomDouble(random, 10.0);

         double angleInterpolated = (1.0 - alpha) * angle0 + alpha * anglef;
         QuaternionConversion.convertAxisAngleToQuaternion(axis.getX(), axis.getY(), axis.getZ(), angleInterpolated, qExpected);
         qActual.interpolate(q0, qf, alpha);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(qExpected, qActual, epsilon);

         qActual.set(q0);
         qActual.interpolate(qf, alpha);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(qExpected, qActual, epsilon);
      }

      // Test when swapping q0 and qf and 'inverting' alpha
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         q0 = createRandomTuple(random);
         qf = createRandomTuple(random);
         double alpha = GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 1.0);
         qExpected.interpolate(q0, qf, alpha);
         qActual.interpolate(qf, q0, 1.0 - alpha);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(qExpected, qActual, epsilon);

         qActual.set(qf);
         qActual.interpolate(q0, 1.0 - alpha);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(qExpected, qActual, epsilon);
      }

      // Test with a different algorithm
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         q0 = createRandomTuple(random);
         qf = createRandomTuple(random);
         double alpha = GeometryBasicsRandomTools.generateRandomDouble(random, 0.0, 1.0);

         Quaternion qDiff = new Quaternion();
         if (q0.dot(qf) < 0.0)
         {
            Quaternion qfCopy = new Quaternion(qf);
            qfCopy.negate();
            qDiff.difference(q0, qfCopy);
         }
         else
         {
            qDiff.difference(q0, qf);
         }

         AxisAngle axisAngleDiff = new AxisAngle(qDiff);
         axisAngleDiff.setAngle(axisAngleDiff.getAngle() * alpha);
         qDiff.set(axisAngleDiff);
         qExpected.multiply(q0, qDiff);

         qActual.interpolate(q0, qf, alpha);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(qExpected, qActual, epsilon);

         qActual.set(q0);
         qActual.interpolate(qf, alpha);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(qExpected, qActual, epsilon);
      }
   }

   @Test
   public void testApplyTransform()
   {
      Random random = new Random(23523L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RigidBodyTransform transform = GeometryBasicsRandomTools.generateRandomRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         expected.preMultiply(transform.getRotationMatrix());
         actual.set(original);
         actual.applyTransform(transform);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         expected.preMultiply(transform.getQuaternion());
         actual.set(original);
         actual.applyTransform(transform);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         AffineTransform transform = GeometryBasicsRandomTools.generateRandomAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         expected.preMultiply(transform.getRotationMatrix());
         actual.set(original);
         actual.applyTransform(transform);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }
   }

}