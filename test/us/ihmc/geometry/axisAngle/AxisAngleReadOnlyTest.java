package us.ihmc.geometry.axisAngle;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.EuclidCoreTools;
import us.ihmc.geometry.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.geometry.exceptions.NotAMatrix2DException;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.geometry.testingTools.EuclidCoreRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple3D.RotationVectorConversion;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.QuaternionTools;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.geometry.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public abstract class AxisAngleReadOnlyTest<T extends AxisAngleReadOnly>
{
   public static final int NUMBER_OF_ITERATIONS = 100;

   public abstract T createEmptyAxisAngle();

   public abstract T createAxisAngle(double ux, double uy, double uz, double angle);

   public abstract T createRandomAxisAngle(Random random);

   public abstract double getEpsilon();

   public abstract double getSmallestEpsilon();

   @Test
   public void testGetAngle()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double expectedAngle = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, expectedAngle);
         double actualAngle = axisAngle.getAngle();

         assertTrue(expectedAngle == actualAngle);
      }
   }

   @Test
   public void testGetX()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double expectedX = random.nextInt(100);
         axisAngle = createAxisAngle(expectedX, 0.0, 0.0, 0.0);
         double actualX = axisAngle.getX();

         assertTrue(expectedX == actualX);
      }
   }

   @Test
   public void testGetY()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double expectedY = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, expectedY, 0.0, 0.0);
         double actualY = axisAngle.getY();

         assertTrue(expectedY == actualY);
      }
   }

   @Test
   public void testGetZ()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double expectedZ = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, 0.0, expectedZ, 0.0);
         double actualZ = axisAngle.getZ();

         assertTrue(expectedZ == actualZ);
      }
   }

   @Test
   public void testGetAngle32()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float expectedAngle = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, expectedAngle);
         float actualAngle = axisAngle.getAngle32();

         assertTrue(expectedAngle == actualAngle);
      }
   }

   @Test
   public void testGetX32()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float expectedX = random.nextInt(100);
         axisAngle = createAxisAngle(expectedX, 0.0, 0.0, 0.0);
         float actualX = axisAngle.getX32();

         assertTrue(expectedX == actualX);
      }
   }

   @Test
   public void testGetY32()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float expectedY = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, expectedY, 0.0, 0.0);
         float actualY = axisAngle.getY32();

         assertTrue(expectedY == actualY);
      }
   }

   @Test
   public void testGetZ32()
   {
      Random random = new Random(564648L);
      T axisAngle;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float expectedZ = random.nextInt(100);
         axisAngle = createAxisAngle(0.0, 0.0, expectedZ, 0.0);
         float actualZ = axisAngle.getZ32();

         assertTrue(expectedZ == actualZ);
      }
   }

   @Test
   public void testContainsNaN()
   {
      T axisAngle;

      axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
      assertFalse(axisAngle.containsNaN());
      axisAngle = createAxisAngle(Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, Double.NaN, 0.0, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, 0.0, Double.NaN, 0.0);
      assertTrue(axisAngle.containsNaN());
      axisAngle = createAxisAngle(0.0, 0.0, 0.0, Double.NaN);
      assertTrue(axisAngle.containsNaN());
   }

   @Test
   public void testIsAxisUnitary() throws Exception
   {
      Random random = new Random(51651L);
      AxisAngle axisAngle = new AxisAngle();

      for (int i = 0; i < 20; i++)
      {
         double smallScale = EuclidCoreRandomTools.generateRandomDouble(random, 0.90, 0.95);
         double bigScale = EuclidCoreRandomTools.generateRandomDouble(random, 1.05, 1.10);

         double ux = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);
         double uy = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);
         double uz = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);
         double angle = EuclidCoreRandomTools.generateRandomDouble(random, 10.0);
         double norm = EuclidCoreTools.norm(ux, uy, uz);

         ux /= norm;
         uy /= norm;
         uz /= norm;

         axisAngle.set(ux, uy, uz, angle);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));

         axisAngle.set(ux * smallScale, uy, uz, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy * smallScale, uz, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz * smallScale, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz, angle * smallScale);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));

         axisAngle.set(ux * bigScale, uy, uz, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy * bigScale, uz, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz * bigScale, angle);
         assertFalse(axisAngle.isAxisUnitary(getEpsilon()));
         axisAngle.set(ux, uy, uz, angle * bigScale);
         assertTrue(axisAngle.isAxisUnitary(getEpsilon()));
      }
   }

   @Test
   public void testIsZOnly() throws Exception
   {
      Random random = new Random(23905872L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double ux = random.nextDouble();
         double uy = random.nextDouble();
         double uz = random.nextDouble();
         double angle = random.nextDouble();
         T axisAngle = createEmptyAxisAngle();
         assertTrue(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(ux, uy, uz, 0.0);
         assertTrue(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(ux, uy, uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(0.0, uy, uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(ux, 0.0, uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(0.0, 0.0, uz, angle);
         assertTrue(axisAngle.isZOnly(getEpsilon()));

         axisAngle = createAxisAngle(2.0 * getEpsilon(), 0.0, uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));
         axisAngle = createAxisAngle(0.0, 2.0 * getEpsilon(), uz, angle);
         assertFalse(axisAngle.isZOnly(getEpsilon()));
      }
   }

   @Test
   public void testCheckIfIsZOnly() throws Exception
   {
      Random random = new Random(23905872L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double ux = random.nextDouble();
         double uy = random.nextDouble();
         double uz = random.nextDouble();
         double angle = random.nextDouble();
         T axisAngle = createEmptyAxisAngle();
         axisAngle.checkIfIsZOnly(getEpsilon());

         axisAngle = createAxisAngle(ux, uy, uz, 0.0);
         axisAngle.checkIfIsZOnly(getEpsilon());

         axisAngle = createAxisAngle(ux, uy, uz, angle);

         try
         {
            axisAngle.checkIfIsZOnly(getEpsilon());
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

         axisAngle = createAxisAngle(0.0, uy, uz, angle);
         try
         {
            axisAngle.checkIfIsZOnly(getEpsilon());
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

         axisAngle = createAxisAngle(ux, 0.0, uz, angle);

         try
         {
            axisAngle.checkIfIsZOnly(getEpsilon());
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

         axisAngle = createAxisAngle(0.0, 0.0, uz, angle);
         axisAngle.checkIfIsZOnly(getEpsilon());
      }
   }

   @Test
   public void testGetRotationVector()
   {
      Random random = new Random(2343456L);
      T axisAngle;
      Vector3D actualVector = new Vector3D();
      Vector3D expectedVector = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         axisAngle = createRandomAxisAngle(random);

         actualVector.setToNaN();
         axisAngle.getRotationVector(actualVector);
         RotationVectorConversion.convertAxisAngleToRotationVector(axisAngle, expectedVector);

         GeometryBasicsTestTools.assertRotationVectorEquals(actualVector, expectedVector, getEpsilon());
      }
   }

   @Test
   public void testGetYawPitchRoll() throws Exception
   {
      Random random = new Random(2342L);
      T axisAngle;
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         axisAngle = createRandomAxisAngle(random);

         { // Test getYawPitchRoll(double[] yawPitchRollToPack)
            double[] yawPitchRoll = new double[4];
            axisAngle.getYawPitchRoll(yawPitchRoll);
            double[] expectedYawPitchRoll = new double[4];
            YawPitchRollConversion.convertAxisAngleToYawPitchRoll(axisAngle, expectedYawPitchRoll);

            for (int j = 0; j < yawPitchRoll.length; j++)
               Assert.assertEquals(yawPitchRoll[j], expectedYawPitchRoll[j], getEpsilon());
         }

         { // Test getYaw()
            double yaw = axisAngle.getYaw();
            double expectedYaw = YawPitchRollConversion.computeYaw(axisAngle);
            Assert.assertEquals(yaw, expectedYaw, getEpsilon());
         }

         { // Test getPitch()
            double pitch = axisAngle.getPitch();
            double expectedPitch = YawPitchRollConversion.computePitch(axisAngle);
            Assert.assertEquals(pitch, expectedPitch, getEpsilon());
         }

         { // Test getRoll()
            double roll = axisAngle.getRoll();
            double expectedRoll = YawPitchRollConversion.computeRoll(axisAngle);
            Assert.assertEquals(roll, expectedRoll, getEpsilon());
         }
      }
   }

   @Test
   public void testGetDoubleArray()
   {
      Random random = new Random(3513515L);
      T axisAngle;

      { // Test get(double[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double[] axisAngleArray = new double[4];
            axisAngle = createRandomAxisAngle(random);
            axisAngle.get(axisAngleArray);

            assertTrue(axisAngle.getX() == axisAngleArray[0]);
            assertTrue(axisAngle.getY() == axisAngleArray[1]);
            assertTrue(axisAngle.getZ() == axisAngleArray[2]);
            assertTrue(axisAngle.getAngle() == axisAngleArray[3]);
         }
      }

      { // Test get(double[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            int startIndex = random.nextInt(20);
            double[] axisAngleArray = new double[startIndex + 4 + random.nextInt(10)];
            axisAngle = createRandomAxisAngle(random);

            axisAngle.get(startIndex, axisAngleArray);

            assertTrue(axisAngle.getX() == axisAngleArray[startIndex + 0]);
            assertTrue(axisAngle.getY() == axisAngleArray[startIndex + 1]);
            assertTrue(axisAngle.getZ() == axisAngleArray[startIndex + 2]);
            assertTrue(axisAngle.getAngle() == axisAngleArray[startIndex + 3]);
         }
      }
   }

   @Test
   public void testGetFloatArray()
   {
      Random random = new Random(3513515L);
      T axisAngle;

      { // Test get(float[] axisAngleArray)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            float[] axisAngleArray = new float[4];
            axisAngle = createRandomAxisAngle(random);
            axisAngle.get(axisAngleArray);

            assertTrue(axisAngle.getX32() == axisAngleArray[0]);
            assertTrue(axisAngle.getY32() == axisAngleArray[1]);
            assertTrue(axisAngle.getZ32() == axisAngleArray[2]);
            assertTrue(axisAngle.getAngle32() == axisAngleArray[3]);
         }
      }

      { // Test get(float[] axisAngleArray, int startIndex)
         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            int startIndex = random.nextInt(20);
            float[] axisAngleArray = new float[startIndex + 4 + random.nextInt(10)];
            axisAngle = createRandomAxisAngle(random);

            axisAngle.get(startIndex, axisAngleArray);

            assertTrue(axisAngle.getX32() == axisAngleArray[startIndex + 0]);
            assertTrue(axisAngle.getY32() == axisAngleArray[startIndex + 1]);
            assertTrue(axisAngle.getZ32() == axisAngleArray[startIndex + 2]);
            assertTrue(axisAngle.getAngle32() == axisAngleArray[startIndex + 3]);
         }
      }
   }

   @Test
   public void testGetWithIndex() throws Exception
   {
      Random random = new Random(324234L);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T axisAngle = createRandomAxisAngle(random);
         assertTrue(axisAngle.getX() == axisAngle.get(0));
         assertTrue(axisAngle.getY() == axisAngle.get(1));
         assertTrue(axisAngle.getZ() == axisAngle.get(2));
         assertTrue(axisAngle.getAngle() == axisAngle.get(3));

         assertTrue(axisAngle.getX32() == axisAngle.get32(0));
         assertTrue(axisAngle.getY32() == axisAngle.get32(1));
         assertTrue(axisAngle.getZ32() == axisAngle.get32(2));
         assertTrue(axisAngle.getAngle32() == axisAngle.get32(3));
      }
   }

   @Test
   public void testTransform()
   {
      Random random = new Random(6787L);
      T axisAngle = createEmptyAxisAngle();
      Quaternion quaternion = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleToTransform)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, tuple, expectedTuple);
         axisAngle.transform(actualTuple);

         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(actualTuple);
         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleOriginal, TupleBasics tupleTransformed)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, tuple, expectedTuple);
         axisAngle.transform(tuple, actualTuple);

         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());

         actualTuple = new Vector3D();
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(tuple, actualTuple);
         GeometryBasicsTestTools.assertTuple3DEquals(tuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         double theta = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         axisAngle = createAxisAngle(0.0, 0.0, 1.0, theta);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, tuple, expectedTuple, false);
         axisAngle.transform(actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         axisAngle.transform(actualTuple, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         axisAngle.transform(actualTuple, false);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         double theta = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         axisAngle = createAxisAngle(0.0, 0.0, 1.0, theta);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, tuple, expectedTuple, false);
         axisAngle.transform(tuple, actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         axisAngle.transform(tuple, actualTuple, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         axisAngle.transform(tuple, actualTuple, false);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());

         actualTuple = new Vector2D();
         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(tuple, actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(tuple, actualTuple, getEpsilon());
      }

      // Test exceptions
      try
      {
         axisAngle = createRandomAxisAngle(random);
         axisAngle.transform(new Vector2D());
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
         axisAngle = createRandomAxisAngle(random);
         axisAngle.transform(new Vector2D(), new Vector2D());
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
         axisAngle = createRandomAxisAngle(random);
         axisAngle.transform(new Vector2D(), true);
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
         axisAngle = createRandomAxisAngle(random);
         axisAngle.transform(new Vector2D(), new Vector2D(), true);
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
      { // Test transform Matrix3D
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());

         Matrix3D matrixOriginal = EuclidCoreRandomTools.generateRandomMatrix3D(random);
         Matrix3D matrixExpected = new Matrix3D();
         Matrix3D matrixActual = new Matrix3D();
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, matrixOriginal, matrixExpected);

         axisAngle.transform(matrixOriginal, matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual.set(matrixOriginal);
         axisAngle.transform(matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual = new Matrix3D();
         axisAngle.transform(matrixOriginal, matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixOriginal, matrixActual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {// Test transform quaternion
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());

         Quaternion qOriginal = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Quaternion qExpected = new Quaternion();
         Quaternion qActual = new Quaternion();
         quaternion.set(axisAngle);

         qExpected.multiply(quaternion, qOriginal);

         axisAngle.transform(qOriginal, qActual);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

         qActual.set(qOriginal);
         axisAngle.transform(qActual);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(qActual);
         GeometryBasicsTestTools.assertQuaternionEquals(qExpected, qActual, getEpsilon());

         qActual = new Quaternion();
         axisAngle.transform(qOriginal, qActual);
         GeometryBasicsTestTools.assertQuaternionEquals(qOriginal, qActual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {// Test transform Vector4D
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());

         Vector4D vectorOriginal = EuclidCoreRandomTools.generateRandomVector4D(random);
         Vector4D vectorExpected = new Vector4D();
         Vector4D vectorActual = new Vector4D();
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, vectorOriginal, vectorExpected);

         axisAngle.transform(vectorOriginal, vectorActual);
         GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, getEpsilon());

         vectorActual.set(vectorOriginal);
         axisAngle.transform(vectorActual);
         GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(vectorActual);
         GeometryBasicsTestTools.assertTuple4DEquals(vectorExpected, vectorActual, getEpsilon());

         vectorActual = new Vector4D();
         axisAngle.transform(vectorOriginal, vectorActual);
         GeometryBasicsTestTools.assertTuple4DEquals(vectorOriginal, vectorActual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform RotationMatrix
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());

         RotationMatrix matrixOriginal = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix matrixExpected = new RotationMatrix();
         RotationMatrix matrixActual = new RotationMatrix();
         quaternion.set(axisAngle);

         QuaternionTools.transform(quaternion, matrixOriginal, matrixExpected);

         axisAngle.transform(matrixOriginal, matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual.set(matrixOriginal);
         axisAngle.transform(matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         axisAngle = createAxisAngle(0.0, 0.0, 0.0, 0.0);
         axisAngle.transform(matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, getEpsilon());

         matrixActual = new RotationMatrix();
         axisAngle.transform(matrixOriginal, matrixActual);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixOriginal, matrixActual, getEpsilon());
      }
   }

   @Test
   public void testInverseTransform()
   {
      Random random = new Random(6787L);
      T axisAngle = createEmptyAxisAngle();
      Quaternion quaternion = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleToTransform)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple);
         axisAngle.inverseTransform(actualTuple);

         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(TupleBasics tupleOriginal, TupleBasics tupleTransformed)
         Tuple3DReadOnly tuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         Tuple3DBasics actualTuple = new Vector3D(tuple);
         Tuple3DBasics expectedTuple = EuclidCoreRandomTools.generateRandomVector3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple);
         axisAngle.inverseTransform(tuple, actualTuple);

         GeometryBasicsTestTools.assertTuple3DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleToTransform)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         double theta = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         axisAngle = createAxisAngle(0.0, 0.0, 1.0, theta);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple, false);
         axisAngle.inverseTransform(actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         axisAngle.inverseTransform(actualTuple, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         actualTuple.set(tuple);
         axisAngle.inverseTransform(actualTuple, false);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test transform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleTransformed)
         Tuple2DReadOnly tuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         Tuple2DBasics actualTuple = new Vector2D(tuple);
         Tuple2DBasics expectedTuple = EuclidCoreRandomTools.generateRandomVector2D(random);
         double theta = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         axisAngle = createAxisAngle(0.0, 0.0, 1.0, theta);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, tuple, expectedTuple, false);
         axisAngle.inverseTransform(tuple, actualTuple);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         axisAngle.inverseTransform(tuple, actualTuple, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
         axisAngle.inverseTransform(tuple, actualTuple, false);
         GeometryBasicsTestTools.assertTuple2DEquals(expectedTuple, actualTuple, getEpsilon());
      }

      // Test exceptions
      try
      {
         axisAngle = createRandomAxisAngle(random);
         axisAngle.inverseTransform(new Vector2D());
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
         axisAngle = createRandomAxisAngle(random);
         axisAngle.inverseTransform(new Vector2D(), new Vector2D());
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
         axisAngle = createRandomAxisAngle(random);
         axisAngle.inverseTransform(new Vector2D(), true);
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
         axisAngle = createRandomAxisAngle(random);
         axisAngle.inverseTransform(new Vector2D(), new Vector2D(), true);
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
         QuaternionReadOnly original = EuclidCoreRandomTools.generateRandomQuaternion(random);
         QuaternionBasics actual = new Quaternion(original);
         QuaternionBasics expected = EuclidCoreRandomTools.generateRandomQuaternion(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
         QuaternionReadOnly original = EuclidCoreRandomTools.generateRandomQuaternion(random);
         QuaternionBasics actual = new Quaternion(original);
         QuaternionBasics expected = EuclidCoreRandomTools.generateRandomQuaternion(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Vector4DBasics vectorToTransform)
         Vector4DReadOnly original = EuclidCoreRandomTools.generateRandomVector4D(random);
         Vector4DBasics actual = new Vector4D(original);
         Vector4DBasics expected = EuclidCoreRandomTools.generateRandomVector4D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4DReadOnly original = EuclidCoreRandomTools.generateRandomVector4D(random);
         Vector4DBasics actual = new Vector4D(original);
         Vector4DBasics expected = EuclidCoreRandomTools.generateRandomVector4D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Matrix3D matrixToTransform)
         Matrix3DReadOnly original = EuclidCoreRandomTools.generateRandomMatrix3D(random);
         Matrix3D actual = new Matrix3D(original);
         Matrix3D expected = EuclidCoreRandomTools.generateRandomMatrix3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3DReadOnly original = EuclidCoreRandomTools.generateRandomMatrix3D(random);
         Matrix3D actual = new Matrix3D(original);
         Matrix3D expected = EuclidCoreRandomTools.generateRandomMatrix3D(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrix matrixToTransform)
         RotationMatrixReadOnly original = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(original);
         RotationMatrix expected = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
         RotationMatrixReadOnly original = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix actual = new RotationMatrix(original);
         RotationMatrix expected = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         axisAngle = createRandomAxisAngle(random);
         double scale = 0.5 + random.nextDouble();
         axisAngle = createAxisAngle(scale * axisAngle.getX(), scale * axisAngle.getY(), scale * axisAngle.getZ(), axisAngle.getAngle());
         quaternion.set(axisAngle);

         QuaternionTools.inverseTransform(quaternion, original, expected);
         axisAngle.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, getEpsilon());
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);

      T axisAngle = createRandomAxisAngle(random);

      assertFalse(axisAngle.equals(createEmptyAxisAngle()));
      assertFalse(axisAngle.equals((Object) createEmptyAxisAngle()));
      assertTrue(axisAngle.equals((Object) axisAngle));
      assertFalse(axisAngle.equals(null));
      assertFalse(axisAngle.equals(new double[5]));

      double x = axisAngle.getX();
      double y = axisAngle.getY();
      double z = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      assertTrue(axisAngle.equals(createAxisAngle(x, y, z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x + getSmallestEpsilon(), y, z, angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x - getSmallestEpsilon(), y, z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y + getSmallestEpsilon(), z, angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x, y - getSmallestEpsilon(), z, angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y, z + getSmallestEpsilon(), angle)));
      assertFalse(axisAngle.equals(createAxisAngle(x, y, z - getSmallestEpsilon(), angle)));

      assertFalse(axisAngle.equals(createAxisAngle(x, y, z, angle + getSmallestEpsilon())));
      assertFalse(axisAngle.equals(createAxisAngle(x, y, z, angle - getSmallestEpsilon())));
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      T axisAngle = createRandomAxisAngle(random);
      double x = axisAngle.getX();
      double y = axisAngle.getY();
      double z = axisAngle.getZ();
      double angle = axisAngle.getAngle();

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x + 0.999 * epsilon, y, z, angle), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x - 0.999 * epsilon, y, z, angle), epsilon));

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y + 0.999 * epsilon, z, angle), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y - 0.999 * epsilon, z, angle), epsilon));

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z + 0.999 * epsilon, angle), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z - 0.999 * epsilon, angle), epsilon));

      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle + 0.999 * epsilon), epsilon));
      assertTrue(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle - 0.999 * epsilon), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x + 1.001 * epsilon, y, z, angle), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x - 1.001 * epsilon, y, z, angle), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y + 1.001 * epsilon, z, angle), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y - 1.001 * epsilon, z, angle), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z + 1.001 * epsilon, angle), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z - 1.001 * epsilon, angle), epsilon));

      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle + 1.001 * epsilon), epsilon));
      assertFalse(axisAngle.epsilonEquals(createAxisAngle(x, y, z, angle - 1.001 * epsilon), epsilon));
   }
}