package us.ihmc.geometry.matrix;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple3D.RotationVectorConversion;
import us.ihmc.geometry.tuple3D.Vector3D;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.QuaternionConversion;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversion;

public class RotationMatrixTest
{
   public static final int NUMBER_OF_ITERATIONS = 100;
   public static final double EPS = 1.0e-10;

   @Test
   public void testRotationMatrix()
   {
      Random random = new Random(46876L);
      Matrix3D matrix, matrixCopy;
      RotationMatrix rotationMatrix = new RotationMatrix();
      RotationMatrix expectedRotationMatrix = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test RotationMatrix()
         rotationMatrix.setToNaN();

         GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(rotationMatrix);

         rotationMatrix.setIdentity();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test RotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         rotationMatrix.setToNaN();
         expectedRotationMatrix.setToNaN();

         try
         {
            rotationMatrix = expectedRotationMatrix = new RotationMatrix(matrix.getM00(), matrix.getM01(), matrix.getM02(), matrix.getM10(), matrix.getM11(), matrix.getM12(), matrix.getM20(),
                  matrix.getM21(), matrix.getM22());
            Assert.assertTrue(rotationMatrix.isRotationMatrix());

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);
         }
         catch (RuntimeException e)
         {
            GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(rotationMatrix);
            GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(expectedRotationMatrix);
         }

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test RotationMatrix(double[] rotationMatrixArray)
         matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         double[] rotationMatrixArray;
         double[] rotationMatrixArrayCopy;
         rotationMatrixArray = rotationMatrixArrayCopy = new double[] {matrix.getM00(), matrix.getM01(), matrix.getM02(), matrix.getM10(), matrix.getM11(), matrix.getM12(), matrix.getM20(),
               matrix.getM21(), matrix.getM22()};

         rotationMatrix.setToNaN();
         expectedRotationMatrix.setToNaN();

         try
         {
            rotationMatrix = new RotationMatrix(rotationMatrixArray);
            expectedRotationMatrix.set(rotationMatrixArray);

            Assert.assertTrue(rotationMatrix.getM00() == rotationMatrixArray[0]);
            Assert.assertTrue(rotationMatrix.getM01() == rotationMatrixArray[1]);
            Assert.assertTrue(rotationMatrix.getM02() == rotationMatrixArray[2]);
            Assert.assertTrue(rotationMatrix.getM10() == rotationMatrixArray[3]);
            Assert.assertTrue(rotationMatrix.getM11() == rotationMatrixArray[4]);
            Assert.assertTrue(rotationMatrix.getM12() == rotationMatrixArray[5]);
            Assert.assertTrue(rotationMatrix.getM20() == rotationMatrixArray[6]);
            Assert.assertTrue(rotationMatrix.getM21() == rotationMatrixArray[7]);
            Assert.assertTrue(rotationMatrix.getM22() == rotationMatrixArray[8]);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);
         }
         catch (RuntimeException e)
         {
            GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(rotationMatrix);
            GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(expectedRotationMatrix);
         }

         for (int j = 0; j < rotationMatrixArray.length; j++)
            Assert.assertTrue(rotationMatrixArray[j] == rotationMatrixArrayCopy[j]);

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test RotationMatrix(double[] rotationMatrixArray)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test RotationMatrix(Matrix3DBasics rotationMatrix)
         matrix = matrixCopy = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         double[] matrixArray;
         double[] matrixArrayCopy;
         matrixArray = matrixArrayCopy = new double[] {matrix.getM00(), matrix.getM01(), matrix.getM02(), matrix.getM10(), matrix.getM11(), matrix.getM12(), matrix.getM20(),
               matrix.getM21(), matrix.getM22()};

         rotationMatrix.setToNaN();
         expectedRotationMatrix.setToNaN();

         try
         {
            rotationMatrix = new RotationMatrix(matrix);
            expectedRotationMatrix = new RotationMatrix(matrixArray);
            expectedRotationMatrix.set(matrixArray);

            Assert.assertTrue(matrix.getM00() == matrixArray[0]);
            Assert.assertTrue(matrix.getM01() == matrixArray[1]);
            Assert.assertTrue(matrix.getM02() == matrixArray[2]);
            Assert.assertTrue(matrix.getM10() == matrixArray[3]);
            Assert.assertTrue(matrix.getM11() == matrixArray[4]);
            Assert.assertTrue(matrix.getM12() == matrixArray[5]);
            Assert.assertTrue(matrix.getM20() == matrixArray[6]);
            Assert.assertTrue(matrix.getM21() == matrixArray[7]);
            Assert.assertTrue(matrix.getM22() == matrixArray[8]);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);
            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, matrix, EPS);
            GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
         }
         catch (RuntimeException e)
         {
            GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(rotationMatrix);
            GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(expectedRotationMatrix);
         }

         for (int j = 0; j < matrixArray.length; j++)
            Assert.assertTrue(matrixArray[j] == matrixArrayCopy[j]);

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test RotationMatrix(AxisAngleBasics axisAngle)
         AxisAngle axisAngle = GeometryBasicsRandomTools.generateRandomAxisAngle(random);

         rotationMatrix = new RotationMatrix(axisAngle);
         RotationMatrixConversion.convertAxisAngleToMatrix(axisAngle, expectedRotationMatrix);

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test RotationMatrix(QuaternionBasics quaternion)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         rotationMatrix = new RotationMatrix(quaternion);
         RotationMatrixConversion.convertQuaternionToMatrix(quaternion, expectedRotationMatrix);

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test RotationMatrix(VectorBasics rotationVector)
         Vector3D rotationVector = GeometryBasicsRandomTools.generateRandomVector(random);

         rotationMatrix = new RotationMatrix(rotationVector);
         RotationMatrixConversion.convertRotationVectorToMatrix(rotationVector, expectedRotationMatrix);

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);
      }
   }

   @Test
   public void testSetToZero()
   {
      RotationMatrix rotationMatrix = new RotationMatrix();
      RotationMatrix identityMatrix = new RotationMatrix();
      identityMatrix.setToNaN();
      rotationMatrix.setToNaN();

      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(identityMatrix);
      GeometryBasicsTestTools.assertMatrix3DContainsOnlyNaN(rotationMatrix);

      identityMatrix.setIdentity();
      rotationMatrix.setToZero();

      GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, identityMatrix, EPS);
   }

   @Test
   public void testCheckIfMatrixProper() throws Exception
   {
      Random random = new Random(46876L);
      Matrix3D matrix, matrixCopy = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         matrixCopy.set(matrix);

         RotationMatrix rotationMatrix = new RotationMatrix();

         try
         {
            rotationMatrix = new RotationMatrix(matrix);
            rotationMatrix.checkIfRotationMatrix();
            Assert.assertTrue(matrix.isRotationMatrix());
         }
         catch (RuntimeException e)
         {
            if (matrix.isRotationMatrix())
               throw e;
            // else it is good
         }

         GeometryBasicsTestTools.assertMatrix3DEquals(matrix, matrixCopy, EPS);
      }
   }

   @Test
   public void testSet() throws Exception
   {
      { // Test set(RotationMatrix other)
         Random random = new Random(648967L);
         Matrix3D expectedMatrix;
         RotationMatrix rotationMatrix = new RotationMatrix(), expectedRotationMatrix;

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            expectedMatrix = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
            expectedRotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
            rotationMatrix.setToNaN();

            rotationMatrix.set(expectedRotationMatrix);
            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedRotationMatrix, EPS);

            try
            {
               rotationMatrix.set(expectedMatrix);
               GeometryBasicsTestTools.assertMatrix3DEquals(expectedMatrix, rotationMatrix, EPS);
            }
            catch (RuntimeException e)
            {
               if (expectedMatrix.isRotationMatrix())
                  throw e;
               // else it is good
            }
         }
      }

      { // Test set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
         Random random = new Random(648967L);

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double[] matrixArray = new double[9];
            double[] matrixArrayCopy = new double[9];

            for (int j = 0; j < matrixArray.length; j++)
               matrixArray[j] = matrixArrayCopy[j] = random.nextDouble() + 1;

            RotationMatrix rotationMatrix = new RotationMatrix();
            rotationMatrix.setToNaN();

            try
            {
               rotationMatrix.set(matrixArray[0], matrixArray[1], matrixArray[2], matrixArray[3], matrixArray[4], matrixArray[5], matrixArray[6],
                     matrixArray[7], matrixArray[8]);

               Assert.assertTrue(rotationMatrix.getM00() == matrixArray[0]);
               Assert.assertTrue(rotationMatrix.getM01() == matrixArray[1]);
               Assert.assertTrue(rotationMatrix.getM02() == matrixArray[2]);
               Assert.assertTrue(rotationMatrix.getM10() == matrixArray[3]);
               Assert.assertTrue(rotationMatrix.getM11() == matrixArray[4]);
               Assert.assertTrue(rotationMatrix.getM12() == matrixArray[5]);
               Assert.assertTrue(rotationMatrix.getM20() == matrixArray[6]);
               Assert.assertTrue(rotationMatrix.getM21() == matrixArray[7]);
               Assert.assertTrue(rotationMatrix.getM22() == matrixArray[8]);
            }
            catch (RuntimeException e)
            {
               if (rotationMatrix.isRotationMatrix())
                  throw e;
               // else it is good
            }

            for (int k = 0; k < matrixArray.length; k++)
               Assert.assertTrue(matrixArray[k] == matrixArrayCopy[k]);
         }
      }

      { // Test set(double[] rotationMatrixArray)
         Random random = new Random(46876L);

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            double[] matrixArray = new double[9];
            double[] matrixArrayCopy = new double[9];

            for (int j = 0; j < matrixArray.length; j++)
               matrixArray[j] = matrixArrayCopy[j] = random.nextDouble() + 1;

            RotationMatrix rotationMatrix = new RotationMatrix();
            rotationMatrix.setToNaN();

            try
            {
               rotationMatrix.set(matrixArray);

               Assert.assertTrue(rotationMatrix.getM00() == matrixArray[0]);
               Assert.assertTrue(rotationMatrix.getM01() == matrixArray[1]);
               Assert.assertTrue(rotationMatrix.getM02() == matrixArray[2]);
               Assert.assertTrue(rotationMatrix.getM10() == matrixArray[3]);
               Assert.assertTrue(rotationMatrix.getM11() == matrixArray[4]);
               Assert.assertTrue(rotationMatrix.getM12() == matrixArray[5]);
               Assert.assertTrue(rotationMatrix.getM20() == matrixArray[6]);
               Assert.assertTrue(rotationMatrix.getM21() == matrixArray[7]);
               Assert.assertTrue(rotationMatrix.getM22() == matrixArray[8]);
            }
            catch (RuntimeException e)
            {
               if (rotationMatrix.isRotationMatrix())
                  throw e;
               // else it is good
            }

            for (int k = 0; k < matrixArray.length; k++)
               Assert.assertTrue(matrixArray[k] == matrixArrayCopy[k]);
         }
      }

      { // Test set(DenseMatrix64F matrix)
         Random random = new Random(46876L);

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix actualMatrix = new RotationMatrix();
            RotationMatrix randomRotation = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
            DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
            for (int row = 0; row < 3; row++)
            {
               for (int column = 0; column < 3; column++)
               {
                  denseMatrix.set(row, column, randomRotation.getElement(row, column));
               }
            }

            actualMatrix.set(denseMatrix);

            for (int row = 0; row < 3; row++)
            {
               for (int column = 0; column < 3; column++)
               {
                  assertTrue(denseMatrix.get(row, column) == actualMatrix.getElement(row, column));
               }
            }
         }
      }

      { // Test set(DenseMatrix64F matrix, int startRow, int startColumn)
         Random random = new Random(46876L);

         for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         {
            RotationMatrix actualMatrix = new RotationMatrix();
            RotationMatrix randomRotation = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
            int startRow = random.nextInt(10);
            int startColumn = random.nextInt(10);
            DenseMatrix64F denseMatrix = new DenseMatrix64F(3 + startRow, 3 + startColumn);

            for (int row = 0; row < 3; row++)
            {
               for (int column = 0; column < 3; column++)
               {
                  denseMatrix.set(row + startRow, column + startColumn, randomRotation.getElement(row, column));
               }
            }

            actualMatrix.set(denseMatrix, startRow, startColumn);

            for (int row = 0; row < 3; row++)
            {
               for (int column = 0; column < 3; column++)
               {
                  assertTrue(denseMatrix.get(row + startRow, column + startColumn) == actualMatrix.getElement(row, column));
               }
            }
         }
      }
   }

   @Test
   public void testSetToYawPitchRollMatrix()
   {
      Random random = new Random(35454L);
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);

         { // Test setToPitchMatrix()
            double pitch, pitchCopy;
            pitch = pitchCopy = random.nextDouble();

            rotationMatrix.setToNaN();
            rotationMatrixCopy.setToNaN();

            rotationMatrix.setToPitchMatrix(pitch);
            RotationMatrixConversion.computePitchMatrix(pitch, rotationMatrixCopy);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);

            Assert.assertTrue(pitch == pitchCopy);
         }

         { // Test setToRollMatrix()
            double roll, rollCopy;
            roll = rollCopy = random.nextDouble();

            rotationMatrix.setToNaN();
            rotationMatrixCopy.setToNaN();

            rotationMatrix.setToRollMatrix(roll);
            RotationMatrixConversion.computeRollMatrix(roll, rotationMatrixCopy);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);

            Assert.assertTrue(roll == rollCopy);
         }

         { // Test setToYawMatrix()
            double yaw, yawCopy;
            yaw = yawCopy = random.nextDouble();

            rotationMatrix.setToNaN();
            rotationMatrixCopy.setToNaN();

            rotationMatrix.setToYawMatrix(yaw);
            RotationMatrixConversion.computeYawMatrix(yaw, rotationMatrixCopy);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);

            Assert.assertTrue(yaw == yawCopy);
         }
      }
   }

   @Test
   public void testSetYawPitchRoll()
   {
      Random random = new Random(6465L);
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setYawPitchRoll (double[] yawPitchRoll)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);

         double[] yawPitchRoll, yawPitchRollCopy;
         yawPitchRoll = yawPitchRollCopy = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         rotationMatrix.setYawPitchRoll(yawPitchRoll);
         RotationMatrixConversion.convertYawPitchRollToMatrix(yawPitchRoll, rotationMatrixCopy);

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         Assert.assertTrue(yawPitchRoll == yawPitchRollCopy);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setYawPitchRoll(double yaw, double pitch, double roll)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);

         double[] yawPitchRoll, yawPitchRollCopy;
         yawPitchRoll = yawPitchRollCopy = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         rotationMatrix.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
         RotationMatrixConversion.convertYawPitchRollToMatrix(yawPitchRoll, rotationMatrixCopy);

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         Assert.assertTrue(yawPitchRoll == yawPitchRollCopy);
      }
   }

   @Test
   public void testSetEuler()
   {
      Random random = new Random(65466L);
      RotationMatrix rotationMatrix, rotationMatrixCopy;
      RotationMatrix yawPitchRoll = new RotationMatrix();
      RotationMatrix expected = new RotationMatrix();
      Vector3D eulerAngles, eulerAnglesCopy;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setEuler(VectorBasics eulerAngles)
         rotationMatrix = rotationMatrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         eulerAngles = eulerAnglesCopy = GeometryBasicsRandomTools.generateRandomVector(random);
         yawPitchRoll.setEuler(eulerAngles);
         RotationMatrixConversion.convertYawPitchRollToMatrix(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), expected);

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(eulerAngles, eulerAnglesCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(yawPitchRoll, expected, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test setEuler(double rotX, double rotY, double rotZ)
         rotationMatrix = rotationMatrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         eulerAngles = eulerAnglesCopy = GeometryBasicsRandomTools.generateRandomVector(random);
         yawPitchRoll.setEuler(eulerAngles.getX(), eulerAngles.getY(), eulerAngles.getZ());
         RotationMatrixConversion.convertYawPitchRollToMatrix(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), expected);

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(eulerAngles, eulerAnglesCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(yawPitchRoll, expected, EPS);
      }
   }

   @Test
   public void testGet()
   {
      Random random = new Random(6841L);
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test get(QuaternionBasics quaternionToPack)
         rotationMatrix = rotationMatrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         Quaternion quaternion = new Quaternion();
         Quaternion expectedQuaternion = new Quaternion();

         rotationMatrix.get(quaternion);
         QuaternionConversion.convertMatrixToQuaternion(rotationMatrix, expectedQuaternion);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, expectedQuaternion, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test get(VectorBasics rotationVectorToPack)
         rotationMatrix = rotationMatrixCopy = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         Vector3D vector = new Vector3D();
         Vector3D expectedVector = new Vector3D();

         rotationMatrix.get(vector);
         RotationVectorConversion.convertMatrixToRotationVector(rotationMatrix, expectedVector);

         GeometryBasicsTestTools.assertRotationVectorEquals(vector, expectedVector, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }
   }

   @Test
   public void testGetArray() throws Exception
   {
      Random random = new Random(4356L);
      RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      double[] matrixArray = new double[9];
      matrix.get(matrixArray);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == matrixArray[row * 3 + column]);
         }
      }

      int startIndex = random.nextInt(10);
      matrixArray = new double[9 + startIndex];
      matrix.get(matrixArray, startIndex);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == matrixArray[row * 3 + column + startIndex]);
         }
      }
   }

   @Test
   public void testGetDenseMatrix() throws Exception
   {
      Random random = new Random(4356L);
      RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

      DenseMatrix64F denseMatrix = new DenseMatrix64F(3, 3);
      matrix.get(denseMatrix);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == denseMatrix.get(row, column));
         }
      }

      int startRow = random.nextInt(10);
      int startColumn = random.nextInt(10);
      denseMatrix = new DenseMatrix64F(3 + startRow, 3 + startColumn);
      matrix.get(startRow, startColumn, denseMatrix);

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            assertTrue(matrix.getElement(row, column) == denseMatrix.get(row + startRow, column + startColumn));
         }
      }
   }

   @Test
   public void testGetEuler()
   {
      Random random = new Random(65466L);
      RotationMatrix yawPitchRoll = new RotationMatrix(), expected;
      Vector3D eulerAngles = new Vector3D();
      Vector3D eulerAnglesCopy = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
         try
         {
            expected = yawPitchRoll = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
            yawPitchRoll.getEuler(eulerAngles);
            YawPitchRollConversion.convertMatrixToYawPitchRoll(expected, eulerAnglesCopy);

            GeometryBasicsTestTools.assertRotationVectorEquals(eulerAngles, eulerAnglesCopy, EPS);
            GeometryBasicsTestTools.assertMatrix3DEquals(yawPitchRoll, expected, EPS);
         }
         catch (AssertionError e)
         {
            double pitch = YawPitchRollConversion.computePitch(yawPitchRoll);
            if (!Double.isNaN(pitch))
               throw e;
         }
   }

   @Test
   public void testGetToYawPitchRollMatrix()
   {
      Random random = new Random(35454L);
      RotationMatrix rotationMatrix, expectedMatrix;

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         rotationMatrix = expectedMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         { // Test getToPitchMatrix()
            double pitch = rotationMatrix.getPitch();
            double expectedPitch = YawPitchRollConversion.computePitch(expectedMatrix);

            Assert.assertEquals(pitch, expectedPitch, EPS);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
         }

         { // Test getToRollMatrix()

            double roll = rotationMatrix.getRoll();
            double expectedRoll = YawPitchRollConversion.computeRoll(expectedMatrix);

            Assert.assertEquals(roll, expectedRoll, EPS);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
         }

         { // Test getToYawMatrix()
            double yaw = rotationMatrix.getYaw();
            double expectedYaw = YawPitchRollConversion.computeYaw(expectedMatrix);

            Assert.assertEquals(yaw, expectedYaw, EPS);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);

            GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
         }
      }
   }

   @Test
   public void testGetYawPitchRoll()
   {
      Random random = new Random(35454L);
      RotationMatrix rotationMatrix = new RotationMatrix(), expectedMatrix = new RotationMatrix();
      double[] yawPitchRoll, yawPitchRollCopy = new double[3];

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expectedMatrix.set(rotationMatrix);
         yawPitchRoll = GeometryBasicsRandomTools.generateRandomYawPitchRoll(random);
         yawPitchRollCopy = yawPitchRoll;

         rotationMatrix.getYawPitchRoll(yawPitchRoll);
         RotationMatrixConversion.convertYawPitchRollToMatrix(yawPitchRoll, expectedMatrix);

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
         Assert.assertTrue(yawPitchRoll == yawPitchRollCopy);
      }
   }

   @Test
   public void testGetColumn() throws Exception
   {
      Random random = new Random(655L);
      RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      double[] columnArray = new double[3];

      matrix.getColumn(0, columnArray);
      assertTrue(matrix.getM00() == columnArray[0]);
      assertTrue(matrix.getM10() == columnArray[1]);
      assertTrue(matrix.getM20() == columnArray[2]);

      matrix.getColumn(1, columnArray);
      assertTrue(matrix.getM01() == columnArray[0]);
      assertTrue(matrix.getM11() == columnArray[1]);
      assertTrue(matrix.getM21() == columnArray[2]);

      matrix.getColumn(2, columnArray);
      assertTrue(matrix.getM02() == columnArray[0]);
      assertTrue(matrix.getM12() == columnArray[1]);
      assertTrue(matrix.getM22() == columnArray[2]);

      try
      {
         matrix.getColumn(3, columnArray);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }

      Vector3D columnVector = new Vector3D();
      matrix.getColumn(0, columnVector);
      assertTrue(matrix.getM00() == columnVector.get(0));
      assertTrue(matrix.getM10() == columnVector.get(1));
      assertTrue(matrix.getM20() == columnVector.get(2));

      matrix.getColumn(1, columnVector);
      assertTrue(matrix.getM01() == columnVector.get(0));
      assertTrue(matrix.getM11() == columnVector.get(1));
      assertTrue(matrix.getM21() == columnVector.get(2));

      matrix.getColumn(2, columnVector);
      assertTrue(matrix.getM02() == columnVector.get(0));
      assertTrue(matrix.getM12() == columnVector.get(1));
      assertTrue(matrix.getM22() == columnVector.get(2));

      try
      {
         matrix.getColumn(3, columnVector);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @Test
   public void testGetRow() throws Exception
   {
      Random random = new Random(6465L);
      RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

      double[] rowArray = new double[3];
      matrix.getRow(0, rowArray);
      assertTrue(matrix.getM00() == rowArray[0]);
      assertTrue(matrix.getM01() == rowArray[1]);
      assertTrue(matrix.getM02() == rowArray[2]);

      matrix.getRow(1, rowArray);
      assertTrue(matrix.getM10() == rowArray[0]);
      assertTrue(matrix.getM11() == rowArray[1]);
      assertTrue(matrix.getM12() == rowArray[2]);

      matrix.getRow(2, rowArray);
      assertTrue(matrix.getM20() == rowArray[0]);
      assertTrue(matrix.getM21() == rowArray[1]);
      assertTrue(matrix.getM22() == rowArray[2]);

      try
      {
         matrix.getRow(3, rowArray);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }

      Vector3D rowVector = new Vector3D();
      matrix.getRow(0, rowVector);
      assertTrue(matrix.getM00() == rowVector.get(0));
      assertTrue(matrix.getM01() == rowVector.get(1));
      assertTrue(matrix.getM02() == rowVector.get(2));

      matrix.getRow(1, rowVector);
      assertTrue(matrix.getM10() == rowVector.get(0));
      assertTrue(matrix.getM11() == rowVector.get(1));
      assertTrue(matrix.getM12() == rowVector.get(2));

      matrix.getRow(2, rowVector);
      assertTrue(matrix.getM20() == rowVector.get(0));
      assertTrue(matrix.getM21() == rowVector.get(1));
      assertTrue(matrix.getM22() == rowVector.get(2));

      try
      {
         matrix.getRow(3, rowVector);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @Test
   public void testGetElement() throws Exception
   {
      Random random = new Random(5464L);
      RotationMatrix matrix = new RotationMatrix();
      double coeff;

      matrix.setUnsafe(coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.getM00() == coeff);
      assertTrue(matrix.getElement(0, 0) == coeff);
      matrix.setUnsafe(0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.getM01() == coeff);
      assertTrue(matrix.getElement(0, 1) == coeff);
      matrix.setUnsafe(0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.getM02() == coeff);
      assertTrue(matrix.getElement(0, 2) == coeff);
      matrix.setUnsafe(0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.getM10() == coeff);
      assertTrue(matrix.getElement(1, 0) == coeff);
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.getM11() == coeff);
      assertTrue(matrix.getElement(1, 1) == coeff);
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0, 0.0);
      assertTrue(matrix.getM12() == coeff);
      assertTrue(matrix.getElement(1, 2) == coeff);
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0, 0.0);
      assertTrue(matrix.getM20() == coeff);
      assertTrue(matrix.getElement(2, 0) == coeff);
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble(), 0.0);
      assertTrue(matrix.getM21() == coeff);
      assertTrue(matrix.getElement(2, 1) == coeff);
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, coeff = random.nextDouble());
      assertTrue(matrix.getM22() == coeff);
      assertTrue(matrix.getElement(2, 2) == coeff);

      try
      {
         matrix.getElement(0, 3);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         matrix.getElement(1, 3);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         matrix.getElement(2, 3);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
      try
      {
         matrix.getElement(3, 0);
         fail("Should have thrown an exception");
      }
      catch (IndexOutOfBoundsException e)
      {
         // Good
      }
   }

   @Test
   public void testInverseTransform()
   {
      Random random = new Random(6787L);
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();
      double yaw, yawCopy;

      Vector3D tuple, tupleCopy = new Vector3D();
      Vector3D inverseTransform3D = new Vector3D();
      Vector3D expectedInverseTransform3D = new Vector3D();

      Vector2D tuple2D, tuple2DCopy = new Vector2D();
      Vector2D inverseTransform2D = new Vector2D();
      Vector2D expectedInverseTransform2D = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(TupleBasics tupleToInverseTransform)
         tuple = GeometryBasicsRandomTools.generateRandomVector(random);
         tupleCopy.set(tuple);

         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);

         rotationMatrix.inverseTransform(tuple);
         Matrix3DTools.inverseTransform(rotationMatrixCopy, tupleCopy, tupleCopy);

         GeometryBasicsTestTools.assertRotationVectorEquals(tuple, tupleCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(TupleBasics tupleOriginal, TupleBasics tupleInverseTransformed)
         tuple = GeometryBasicsRandomTools.generateRandomVector(random);
         tupleCopy.set(tuple);
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);

         rotationMatrix.inverseTransform(tuple, inverseTransform3D);
         Matrix3DTools.inverseTransform(rotationMatrixCopy, tupleCopy, expectedInverseTransform3D);

         GeometryBasicsTestTools.assertRotationVectorEquals(tuple, tupleCopy, EPS);
         GeometryBasicsTestTools.assertRotationVectorEquals(inverseTransform3D, expectedInverseTransform3D, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DBasics tupleOriginal, Tuple2DBasics tupleInverseTransformed)
         yaw = yawCopy = random.nextDouble();
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         rotationMatrix.setToYawMatrix(yaw);
         rotationMatrixCopy.set(rotationMatrix);

         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);

         rotationMatrix.inverseTransform(tuple2D, inverseTransform2D);
         Matrix3DTools.inverseTransform(rotationMatrixCopy, tuple2DCopy, expectedInverseTransform2D, true);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), EPS);

         Assert.assertEquals(inverseTransform2D.getX(), expectedInverseTransform2D.getX(), EPS);
         Assert.assertEquals(inverseTransform2D.getY(), expectedInverseTransform2D.getY(), EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         Assert.assertTrue(yaw == yawCopy);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test inverseTransform(Tuple2DBasics tupleToInverseTransform)
         yaw = yawCopy = random.nextDouble();
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         tuple2D = GeometryBasicsRandomTools.generateRandomVector2D(random);
         tuple2DCopy.set(tuple2D);

         rotationMatrix.setToYawMatrix(yaw);
         rotationMatrixCopy.set(rotationMatrix);

         rotationMatrix.inverseTransform(tuple2D);
         Matrix3DTools.inverseTransform(rotationMatrixCopy, tuple2DCopy, tuple2DCopy, true);

         Assert.assertEquals(tuple2D.getX(), tuple2DCopy.getX(), EPS);
         Assert.assertEquals(tuple2D.getY(), tuple2DCopy.getY(), EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         Assert.assertTrue(yaw == yawCopy);
      }
   }

   @Test
   @Ignore
   public void testApplyTransform()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testInvert()
   {
      Random random = new Random(65474L);
      RotationMatrix rotationMatrix, expectedMatrix = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expectedMatrix.set(rotationMatrix);

         rotationMatrix.invert();
         expectedMatrix.transpose();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, expectedMatrix, EPS);
      }
   }

   @Test
   public void testMultiply()
   {
      Random random = new Random(645864L);
      RotationMatrix multiplied, expected = new RotationMatrix();
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiply(Matrix3DBasics rotationMatrix)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiply(Matrix3DBasics rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expected.set(multiplied);

         multiplied.multiply(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiply(expected, rotationMatrixCopy, expected);
         expected.normalize();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiplyTransposeThis(Matrix3DBasics rotationMatrix)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiplyTransposeThis(Matrix3DBasics rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expected.set(multiplied);

         multiplied.multiplyTransposeThis(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeLeft(expected, rotationMatrixCopy, expected);
         expected.normalize();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiplyTransposeOther(Matrix3DBasics rotationMatrix)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiplyTransposeOther(Matrix3DBasics rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expected.set(multiplied);

         multiplied.multiplyTransposeOther(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeRight(expected, rotationMatrixCopy, expected);
         expected.normalize();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiplyTransposeBoth(Matrix3DBasics rotationMatrix)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test multiplyTransposeBoth(Matrix3DBasics rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expected.set(multiplied);

         multiplied.multiplyTransposeBoth(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeBoth(expected, rotationMatrixCopy, expected);
         expected.normalize();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }
   }

   @Test
   public void testPreMultiply()
   {
      Random random = new Random(645864L);
      RotationMatrix multiplied, expected = new RotationMatrix();
      RotationMatrix rotationMatrix, rotationMatrixCopy = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test preMultiply(Matrix3DBasics matrix)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test preMultiplyTransposeThis(Matrix3DBasics matrix)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test preMultiplyTransposeOther(Matrix3DBasics matrix)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test preMultiplyTransposeBoth(Matrix3DBasics matrix)
           // TODO reimplement me
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test preMultiply(Matrix3DBasics rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expected.set(multiplied);

         multiplied.preMultiply(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiply(rotationMatrixCopy, expected, expected);
         expected.normalize();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test preMultiplyTransposeThis(Matrix3DBasics rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expected.set(multiplied);

         multiplied.preMultiplyTransposeThis(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeRight(rotationMatrixCopy, expected, expected);
         expected.normalize();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test preMultiplyTransposeOther(Matrix3DBasics rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expected.set(multiplied);

         multiplied.preMultiplyTransposeOther(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeLeft(rotationMatrixCopy, expected, expected);
         expected.normalize();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test preMultiplyTransposeBoth(Matrix3DBasics rotationMatrix)
         rotationMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         rotationMatrixCopy.set(rotationMatrix);
         multiplied = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         expected.set(multiplied);

         multiplied.preMultiplyTransposeBoth(rotationMatrix);

         expected.checkIfRotationMatrix();
         RotationMatrixTools.multiplyTransposeBoth(rotationMatrixCopy, expected, expected);
         expected.normalize();

         GeometryBasicsTestTools.assertMatrix3DEquals(rotationMatrix, rotationMatrixCopy, EPS);
         GeometryBasicsTestTools.assertMatrix3DEquals(multiplied, expected, EPS);

         assertEquals(1.0, multiplied.determinant(), 1.0e-10);
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      RotationMatrix matrix = new RotationMatrix();
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertFalse(matrix.containsNaN());
      matrix.setUnsafe(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.setUnsafe(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.setUnsafe(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.setUnsafe(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      assertTrue(matrix.containsNaN());
      matrix.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      assertTrue(matrix.containsNaN());
   }

   @Test
   public void testNormalize() throws Exception
   {
      Random random = new Random(39456L);
      RotationMatrix matrixExpected = new RotationMatrix();
      RotationMatrix matrixActual = new RotationMatrix();

      { // Check that identity does not get modified
         matrixActual.setIdentity();
         matrixExpected.setIdentity();

         matrixActual.normalize();
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      // Test that normalizing a proper rotation matrix does not change it.
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrixExpected.set(GeometryBasicsRandomTools.generateRandomRotationMatrix(random));
         matrixActual.set(matrixExpected);

         matrixActual.normalize();
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      Vector3D vector1 = new Vector3D();
      Vector3D vector2 = new Vector3D();

      // Test that it actually makes a random matrix ortho-normal
      double corruptionFactor = 0.1;
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix randomRotation = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         double m00 = randomRotation.getM00() + corruptionFactor * random.nextDouble();
         double m01 = randomRotation.getM01() + corruptionFactor * random.nextDouble();
         double m02 = randomRotation.getM02() + corruptionFactor * random.nextDouble();
         double m10 = randomRotation.getM10() + corruptionFactor * random.nextDouble();
         double m11 = randomRotation.getM11() + corruptionFactor * random.nextDouble();
         double m12 = randomRotation.getM12() + corruptionFactor * random.nextDouble();
         double m20 = randomRotation.getM20() + corruptionFactor * random.nextDouble();
         double m21 = randomRotation.getM21() + corruptionFactor * random.nextDouble();
         double m22 = randomRotation.getM22() + corruptionFactor * random.nextDouble();
         matrixActual.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         matrixActual.normalize();

         // Test that each row & column vectors are unit-length
         for (int j = 0; j < 3; j++)
         {
            matrixActual.getRow(j, vector1);
            assertEquals(1.0, vector1.length(), EPS);

            matrixActual.getColumn(j, vector1);
            assertEquals(1.0, vector1.length(), EPS);
         }

         // Test that each pair of rows and each pair of columns are orthogonal
         for (int j = 0; j < 3; j++)
         {
            matrixActual.getRow(j, vector1);
            matrixActual.getRow((j + 1) % 3, vector2);
            assertEquals(0.0, vector1.dot(vector2), EPS);

            matrixActual.getColumn(j, vector1);
            matrixActual.getColumn((j + 1) % 3, vector2);
            assertEquals(0.0, vector1.dot(vector2), EPS);
         }
      }
   }

   @Test
   public void testSetAndNormalize() throws Exception
   {
      Random random = new Random(39456L);
      RotationMatrix matrixExpected = new RotationMatrix();
      RotationMatrix matrixActual = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         matrixExpected = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         double m00 = matrixExpected.getM00();
         double m01 = matrixExpected.getM01();
         double m02 = matrixExpected.getM02();
         double m10 = matrixExpected.getM10();
         double m11 = matrixExpected.getM11();
         double m12 = matrixExpected.getM12();
         double m20 = matrixExpected.getM20();
         double m21 = matrixExpected.getM21();
         double m22 = matrixExpected.getM22();
         matrixActual.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize(matrixExpected);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize((Matrix3DReadOnly<?>) matrixExpected);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }

      double corruptionFactor = 0.1;
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix corrupted = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         double m00 = corrupted.getM00() + corruptionFactor * random.nextDouble();
         double m01 = corrupted.getM01() + corruptionFactor * random.nextDouble();
         double m02 = corrupted.getM02() + corruptionFactor * random.nextDouble();
         double m10 = corrupted.getM10() + corruptionFactor * random.nextDouble();
         double m11 = corrupted.getM11() + corruptionFactor * random.nextDouble();
         double m12 = corrupted.getM12() + corruptionFactor * random.nextDouble();
         double m20 = corrupted.getM20() + corruptionFactor * random.nextDouble();
         double m21 = corrupted.getM21() + corruptionFactor * random.nextDouble();
         double m22 = corrupted.getM22() + corruptionFactor * random.nextDouble();
         corrupted.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);

         matrixExpected.setUnsafe(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         matrixExpected.normalize();
         matrixActual.setAndNormalize(m00, m01, m02, m10, m11, m12, m20, m21, m22);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize(corrupted);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

         matrixActual.setToNaN();
         matrixActual.setAndNormalize((Matrix3DReadOnly<?>) corrupted);
         GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
      }
   }

   @Test
   public void testSetAndInvert() throws Exception
   {
      Random random = new Random(545L);
      RotationMatrix matrixActual = new RotationMatrix();
      RotationMatrix matrixExpected = new RotationMatrix();

      RotationMatrix randomMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      matrixExpected.set(randomMatrix);
      matrixExpected.invert();

      matrixActual.setAndInvert(randomMatrix);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

      matrixActual.setToNaN();
      matrixActual.setAndInvert((Matrix3DReadOnly<?>) randomMatrix);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testSetAndTranspose() throws Exception
   {
      Random random = new Random(545L);
      RotationMatrix matrixActual = new RotationMatrix();
      RotationMatrix matrixExpected = new RotationMatrix();

      RotationMatrix randomMatrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      matrixExpected.set(randomMatrix);
      matrixExpected.transpose();

      matrixActual.setAndTranspose(randomMatrix);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);

      matrixActual.setToNaN();
      matrixActual.setAndTranspose((Matrix3DReadOnly<?>) randomMatrix);
      GeometryBasicsTestTools.assertMatrix3DEquals(matrixExpected, matrixActual, EPS);
   }

   @Test
   public void testTransformTuple() throws Exception
   {
      Random random = new Random(435L);
      Vector3D actual = new Vector3D();
      Vector3D expected = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Vector3D original = GeometryBasicsRandomTools.generateRandomVector(random);

         Matrix3DTools.transform(matrix, original, expected);
         actual.set(original);
         matrix.transform(actual);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformTuple2D() throws Exception
   {
      Random random = new Random(435L);
      Vector2D actual = new Vector2D();
      Vector2D expected = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = new RotationMatrix();
         matrix.setToYawMatrix(2.0 * Math.PI * random.nextDouble());
         Vector2D original = GeometryBasicsRandomTools.generateRandomVector2D(random);

         Matrix3DTools.transform(matrix, original, expected, true);
         actual.set(original);
         matrix.transform(actual);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);

         actual.setToNaN();
         Matrix3DTools.transform(matrix, original, expected, true);
         actual.set(original);
         matrix.transform(actual, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformMatrix() throws Exception
   {
      Random random = new Random(435L);
      Matrix3D actual = new Matrix3D();
      Matrix3D expected = new Matrix3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Matrix3D original = GeometryBasicsRandomTools.generateRandomMatrix3D(random);

         Matrix3DTools.transform(matrix, original, expected);
         actual.set(original);
         matrix.transform(actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformQuaternion() throws Exception
   {
      Random random = new Random(435L);
      Quaternion actual = new Quaternion();
      Quaternion expected = new Quaternion();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Quaternion original = GeometryBasicsRandomTools.generateRandomQuaternion(random);

         matrix.transform(original, expected);
         actual.set(original);
         matrix.transform(actual);
         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformVector4D() throws Exception
   {
      Random random = new Random(435L);
      Vector4D actual = new Vector4D();
      Vector4D expected = new Vector4D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Vector4D original = GeometryBasicsRandomTools.generateRandomVector4D(random);

         matrix.transform(original, expected);
         actual.set(original);
         matrix.transform(actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformRotationMatrix() throws Exception
   {
      Random random = new Random(435L);
      RotationMatrix actual = new RotationMatrix();
      RotationMatrix expected = new RotationMatrix();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix original = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);

         matrix.transform(original, expected);
         actual.set(original);
         matrix.transform(actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.transform(original, actual);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testInverseTransformTuple() throws Exception
   {
      Random random = new Random(435L);
      Vector3D actual = new Vector3D();
      Vector3D expected = new Vector3D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Vector3D original = GeometryBasicsRandomTools.generateRandomVector(random);

         Matrix3DTools.inverseTransform(matrix, original, expected);
         actual.set(original);
         matrix.inverseTransform(actual);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testInverseTransformTuple2D() throws Exception
   {
      Random random = new Random(435L);
      Vector2D actual = new Vector2D();
      Vector2D expected = new Vector2D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = new RotationMatrix();
         matrix.setToYawMatrix(2.0 * Math.PI * random.nextDouble());
         Vector2D original = GeometryBasicsRandomTools.generateRandomVector2D(random);

         Matrix3DTools.inverseTransform(matrix, original, expected, true);
         actual.set(original);
         matrix.inverseTransform(actual);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testInverseTransformVector4D() throws Exception
   {
      Random random = new Random(435L);
      Vector4D actual = new Vector4D();
      Vector4D expected = new Vector4D();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         RotationMatrix matrix = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         Vector4D original = GeometryBasicsRandomTools.generateRandomVector4D(random);

         Matrix3DTools.inverseTransform(matrix, original, expected);
         actual.set(original);
         matrix.inverseTransform(actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, EPS);

         actual.setToNaN();
         matrix.inverseTransform(original, actual);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      RotationMatrix m1 = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      RotationMatrix m2 = new RotationMatrix();

      assertFalse(m1.equals(m2));
      assertFalse(m1.equals(null));
      assertFalse(m1.equals(new double[4]));
      m2.set(m1);
      assertTrue(m1.equals(m2));
      assertTrue(m1.equals((Object) m2));

      double smallestEpsilon = 1.0e-16;
      double[] coeffs = new double[9];

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            m2.set(m1);
            assertTrue(m1.equals(m2));
            m1.get(coeffs);
            coeffs[3 * row + column] += smallestEpsilon;
            m2.set(coeffs);
            assertFalse(m1.equals(m2));

            m2.set(m1);
            assertTrue(m1.equals(m2));
            m1.get(coeffs);
            coeffs[3 * row + column] -= smallestEpsilon;
            m2.set(coeffs);
            assertFalse(m1.equals(m2));
         }
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(2354L);
      RotationMatrix m1 = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
      RotationMatrix m2 = new RotationMatrix();
      double epsilon = 1.0e-3;
      double[] coeffs = new double[9];

      assertFalse(m1.epsilonEquals(m2, epsilon));
      m2.set(m1);
      assertTrue(m1.epsilonEquals(m2, epsilon));

      for (int row = 0; row < 3; row++)
      {
         for (int column = 0; column < 3; column++)
         {
            m2.set(m1);
            assertTrue(m1.epsilonEquals(m2, epsilon));
            m1.get(coeffs);
            coeffs[3 * row + column] += 0.999 * epsilon;
            m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8]);
            assertTrue(m1.epsilonEquals(m2, epsilon));

            m2.set(m1);
            assertTrue(m1.epsilonEquals(m2, epsilon));
            m1.get(coeffs);
            coeffs[3 * row + column] += 1.001 * epsilon;
            m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8]);
            assertFalse(m1.epsilonEquals(m2, epsilon));

            m2.set(m1);
            assertTrue(m1.epsilonEquals(m2, epsilon));
            m1.get(coeffs);
            coeffs[3 * row + column] -= 0.999 * epsilon;
            m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8]);
            assertTrue(m1.epsilonEquals(m2, epsilon));

            m2.set(m1);
            assertTrue(m1.epsilonEquals(m2, epsilon));
            m1.get(coeffs);
            coeffs[3 * row + column] -= 1.001 * epsilon;
            m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6], coeffs[7], coeffs[8]);
            assertFalse(m1.epsilonEquals(m2, epsilon));
         }
      }
   }
}
