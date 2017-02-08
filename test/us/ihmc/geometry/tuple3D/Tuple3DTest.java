package us.ihmc.geometry.tuple3D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.geometry.tuple3D.Point3D32;
import us.ihmc.geometry.tuple3D.Tuple3D;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public abstract class Tuple3DTest<T extends Tuple3D<T>> extends Tuple3DBasicsTest<T>
{
   public static final int NUMBER_OF_ITERATIONS = 100;

   public abstract T createEmptyTuple();

   @Test
   public void testTuple()
   {
      Tuple3D tuple = createEmptyTuple();
      Assert.assertTrue(tuple.getX() == 0);
      Assert.assertTrue(tuple.getY() == 0);
      Assert.assertTrue(tuple.getZ() == 0);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple = createEmptyTuple();
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple.setX(random.nextDouble());
         tuple.setY(random.nextDouble());
         tuple.setZ(random.nextDouble());

         tuple.setToNaN();
         assertTrue(Double.isNaN(tuple.getX()));
         assertTrue(Double.isNaN(tuple.getY()));
         assertTrue(Double.isNaN(tuple.getZ()));
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple = createEmptyTuple();
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple.setX(random.nextDouble());
         tuple.setY(random.nextDouble());
         tuple.setZ(random.nextDouble());

         tuple.setToZero();
         assertTrue(tuple.getX() == 0.0);
         assertTrue(tuple.getY() == 0.0);
         assertTrue(tuple.getZ() == 0.0);
      }
   }

   @Test
   public void testAbsolute() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();

      for (double signX = -1.0; signX <= 1.0; signX += 2.0)
      {
         for (double signY = -1.0; signY <= 1.0; signY += 2.0)
         {
            for (double signZ = -1.0; signZ <= 1.0; signZ += 2.0)
            {
               double xPos = random.nextDouble();
               double yPos = random.nextDouble();
               double zPos = random.nextDouble();
               tuple1.setX(signX * xPos);
               tuple1.setY(signY * yPos);
               tuple1.setZ(signZ * zPos);

               tuple2.setAndAbsolute(tuple1);
               assertTrue(tuple2.getX() == xPos);
               assertTrue(tuple2.getY() == yPos);
               assertTrue(tuple2.getZ() == zPos);
               assertTrue(tuple1.getX() == signX * xPos);
               assertTrue(tuple1.getY() == signY * yPos);
               assertTrue(tuple1.getZ() == signZ * zPos);

               tuple1.absolute();
               assertTrue(tuple1.getX() == xPos);
               assertTrue(tuple1.getY() == yPos);
               assertTrue(tuple1.getZ() == zPos);
            }
         }
      }
   }

   @Test
   public void testNegate() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();

      for (double signX = -1.0; signX <= 1.0; signX += 2.0)
      {
         for (double signY = -1.0; signY <= 1.0; signY += 2.0)
         {
            for (double signZ = -1.0; signZ <= 1.0; signZ += 2.0)
            {
               double xOriginal = signX * random.nextDouble();
               double yOriginal = signY * random.nextDouble();
               double zOriginal = signZ * random.nextDouble();
               tuple1.setX(xOriginal);
               tuple1.setY(yOriginal);
               tuple1.setZ(zOriginal);

               tuple2.setAndNegate(tuple1);
               assertTrue(tuple2.getX() == -xOriginal);
               assertTrue(tuple2.getY() == -yOriginal);
               assertTrue(tuple2.getZ() == -zOriginal);
               assertTrue(tuple1.getX() == xOriginal);
               assertTrue(tuple1.getY() == yOriginal);
               assertTrue(tuple1.getZ() == zOriginal);

               tuple1.negate();
               assertTrue(tuple1.getX() == -xOriginal);
               assertTrue(tuple1.getY() == -yOriginal);
               assertTrue(tuple1.getZ() == -zOriginal);
            }
         }
      }
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.set(double x, double y, double z);
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         double newZ = random.nextDouble();
         tuple1.set(newX, newY, newZ);

         assertTrue(tuple1.getX() == newX);
         assertTrue(tuple1.getY() == newY);
         assertTrue(tuple1.getZ() == newZ);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.set(double[] tupleArray);
         double tupleArray[] = {random.nextDouble(), random.nextDouble()};
         double[] tupleArrayCopy = new double[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);

         try
         {
            tuple1.set(tupleArray);
            fail("Should have thrown a ArrayIndexOutOfBoundsException.");
         }
         catch (ArrayIndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a ArrayIndexOutOfBoundsException.");
         }

         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);

         tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         tupleArrayCopy = new double[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.set(tupleArray);

         assertTrue(tuple1.getX() == tupleArray[0]);
         assertTrue(tuple1.getY() == tupleArray[1]);
         assertTrue(tuple1.getZ() == tupleArray[2]);

         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.set(double[] tupleArray, int startIndex);
         double tupleArray[] = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         double[] tupleArrayCopy = new double[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);

         try
         {
            tuple1.set(1, tupleArray);
            fail("Should have thrown a ArrayIndexOutOfBoundsException.");
         }
         catch (ArrayIndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a ArrayIndexOutOfBoundsException.");
         }
         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);

         tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         tupleArrayCopy = new double[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.set(2, tupleArray);

         assertTrue(tuple1.getX() == tupleArray[2]);
         assertTrue(tuple1.getY() == tupleArray[3]);
         assertTrue(tuple1.getZ() == tupleArray[4]);

         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.set(DenseMatrix64F matrix);
         DenseMatrix64F matrix = new DenseMatrix64F(2, 1);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         try
         {
            tuple1.set(matrix);
            fail("Should have thrown a IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IllegalArgumentException.");
         }

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));

         matrix = new DenseMatrix64F(5, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         matrixCopy = new DenseMatrix64F(matrix);
         tuple1.set(matrix);

         assertTrue(tuple1.getX() == matrix.get(0, 0));
         assertTrue(tuple1.getY() == matrix.get(1, 0));
         assertTrue(tuple1.getZ() == matrix.get(2, 0));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.set(DenseMatrix64F matrix, int startRow);
         DenseMatrix64F matrix = new DenseMatrix64F(4, 1);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         try
         {
            tuple1.set(2, matrix);
            fail("Should have thrown a IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IllegalArgumentException.");
         }
         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));

         matrix = new DenseMatrix64F(10, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         matrixCopy = new DenseMatrix64F(matrix);
         tuple1.set(5, matrix);

         assertTrue(tuple1.getX() == matrix.get(5, 0));
         assertTrue(tuple1.getY() == matrix.get(6, 0));
         assertTrue(tuple1.getZ() == matrix.get(7, 0));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.set(DenseMatrix64F matrix, int startRow, int column);
         DenseMatrix64F matrix = new DenseMatrix64F(4, 6);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         try
         {
            tuple1.set(2, 3, matrix);
            fail("Should have thrown a IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IllegalArgumentException.");
         }

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));

         matrix = new DenseMatrix64F(10, 4);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         matrixCopy = new DenseMatrix64F(matrix);
         tuple1.set(5, 2, matrix);

         assertTrue(tuple1.getX() == matrix.get(5, 2));
         assertTrue(tuple1.getY() == matrix.get(6, 2));
         assertTrue(tuple1.getZ() == matrix.get(7, 2));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.set(int index, double value)
         try
         {
            tuple1.set(-1, random.nextDouble());
            fail("Should have thrown a IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IndexOutOfBoundsException.");
         }

         try
         {
            tuple1.set(3, random.nextDouble());
            fail("Should have thrown a IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IndexOutOfBoundsException.");
         }

         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.set(0, x);
         tuple1.set(1, y);
         tuple1.set(2, z);

         assertTrue(tuple1.getX() == x);
         assertTrue(tuple1.getY() == y);
         assertTrue(tuple1.getZ() == z);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.set(TupleBasics other)
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.set(tuple2);

         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.setX(double x)
         double x = random.nextDouble();
         tuple1.setX(x);
         assertTrue(tuple1.getX() == x);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.setY(double y)
         double y = random.nextDouble();
         tuple1.setY(y);
         assertTrue(tuple1.getY() == y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.setZ(double z)
         double z = random.nextDouble();
         tuple1.setZ(z);
         assertTrue(tuple1.getZ() == z);
      }
   }

   @Test
   public void testAdd() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();
      Tuple3D tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.add(double x, double y, double z)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.add(x, y, z);
         assertTrue(tuple1.getX() == xOld + x);
         assertTrue(tuple1.getY() == yOld + y);
         assertTrue(tuple1.getZ() == zOld + z);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.add(TupleBasics other)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.add(tuple2);
         assertTrue(tuple1.getX() == xOld + tuple2.getX());
         assertTrue(tuple1.getY() == yOld + tuple2.getY());
         assertTrue(tuple1.getZ() == zOld + tuple2.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.add(TupleBasics other)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());
         tuple3.setZ(random.nextDouble());

         tuple1.add(tuple2, tuple3);
         assertTrue(tuple1.getX() == tuple2.getX() + tuple3.getX());
         assertTrue(tuple1.getY() == tuple2.getY() + tuple3.getY());
         assertTrue(tuple1.getZ() == tuple2.getZ() + tuple3.getZ());
      }
   }

   @Test
   public void testSub() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();
      Tuple3D tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.sub(double x, double y, double z)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.sub(x, y, z);
         assertTrue(tuple1.getX() == xOld - x);
         assertTrue(tuple1.getY() == yOld - y);
         assertTrue(tuple1.getZ() == zOld - z);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.sub(TupleBasics other)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.sub(tuple2);
         assertTrue(tuple1.getX() == xOld - tuple2.getX());
         assertTrue(tuple1.getY() == yOld - tuple2.getY());
         assertTrue(tuple1.getZ() == zOld - tuple2.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.sub(TupleBasics tuple1, TupleBasics tuple2)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());
         tuple3.setZ(random.nextDouble());

         tuple1.sub(tuple2, tuple3);
         assertTrue(tuple1.getX() == tuple2.getX() - tuple3.getX());
         assertTrue(tuple1.getY() == tuple2.getY() - tuple3.getY());
         assertTrue(tuple1.getZ() == tuple2.getZ() - tuple3.getZ());
      }
   }

   @Test
   public void testScale() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();
      Tuple3D tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.scale(double scalarX, double scalarY, double scalarZ)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double scale = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.scale(scale);
         assertTrue(tuple1.getX() == xOld * scale);
         assertTrue(tuple1.getY() == yOld * scale);
         assertTrue(tuple1.getZ() == zOld * scale);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.scale(double scalarX, double scalarY, double scalarZ)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double scaleX = random.nextDouble();
         double scaleY = random.nextDouble();
         double scaleZ = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);

         tuple1.scale(scaleX, scaleY, scaleZ);
         assertTrue(tuple1.getX() == xOld * scaleX);
         assertTrue(tuple1.getY() == yOld * scaleY);
         assertTrue(tuple1.getZ() == zOld * scaleZ);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.scale(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.setAndScale(scale, tuple2);
         assertTrue(tuple1.getX() == tuple2.getX() * scale);
         assertTrue(tuple1.getY() == tuple2.getY() * scale);
         assertTrue(tuple1.getZ() == tuple2.getZ() * scale);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.scaleAdd(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.scaleAdd(scale, tuple2);
         assertTrue(tuple1.getX() == xOld * scale + tuple2.getX());
         assertTrue(tuple1.getY() == yOld * scale + tuple2.getY());
         assertTrue(tuple1.getZ() == zOld * scale + tuple2.getZ());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.scaleAdd(double scalar, TupleBasics tuple1, TupleBasics tuple2)
         double scale = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());
         tuple3.setZ(random.nextDouble());

         tuple1.scaleAdd(scale, tuple2, tuple3);
         assertTrue(tuple1.getX() == tuple2.getX() * scale + tuple3.getX());
         assertTrue(tuple1.getY() == tuple2.getY() * scale + tuple3.getY());
         assertTrue(tuple1.getZ() == tuple2.getZ() * scale + tuple3.getZ());
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();
      Tuple3D tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.interpolate(TupleBasics other, double alpha)
         double alpha = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());

         tuple1.interpolate(tuple2, alpha);
         assertTrue(tuple1.getX() == Tuple3DTools.interpolate(xOld, tuple2.getX(), alpha));
         assertTrue(tuple1.getY() == Tuple3DTools.interpolate(yOld, tuple2.getY(), alpha));
         assertTrue(tuple1.getZ() == Tuple3DTools.interpolate(zOld, tuple2.getZ(), alpha));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.interpolate(TupleBasics tuple1, TupleBasics tuple2, double alpha)
         double alpha = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());
         tuple3.setZ(random.nextDouble());

         tuple1.interpolate(tuple2, tuple3, alpha);
         assertTrue(tuple1.getX() == Tuple3DTools.interpolate(tuple2.getX(), tuple3.getX(), alpha));
         assertTrue(tuple1.getY() == Tuple3DTools.interpolate(tuple2.getY(), tuple3.getY(), alpha));
         assertTrue(tuple1.getZ() == Tuple3DTools.interpolate(tuple2.getZ(), tuple3.getZ(), alpha));
      }
   }

   @Test
   public void testClip() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.clipToMax(double max)
         double max = random.nextDouble();
         tuple1.setX(max + random.nextDouble());
         tuple1.setY(max + random.nextDouble());
         tuple1.setZ(max + random.nextDouble());
         tuple1.clipToMax(max);
         assertTrue(tuple1.getX() == max);
         assertTrue(tuple1.getY() == max);
         assertTrue(tuple1.getZ() == max);

         max = random.nextDouble();
         tuple1.setX(max - random.nextDouble());
         tuple1.setY(max - random.nextDouble());
         tuple1.setZ(max - random.nextDouble());
         tuple1.clipToMax(max);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.clipToMax(double max, TupleBasics other)
         double max = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(max + random.nextDouble());
         tuple2.setY(max + random.nextDouble());
         tuple2.setZ(max + random.nextDouble());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.getX() == max);
         assertTrue(tuple1.getY() == max);
         assertTrue(tuple1.getZ() == max);

         max = random.nextDouble();
         tuple2.setX(max - random.nextDouble());
         tuple2.setY(max - random.nextDouble());
         tuple2.setZ(max - random.nextDouble());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.clipToMin(double min)
         double min = random.nextDouble();
         tuple1.setX(min - random.nextDouble());
         tuple1.setY(min - random.nextDouble());
         tuple1.setZ(min - random.nextDouble());
         tuple1.clipToMin(min);
         assertTrue(tuple1.getX() == min);
         assertTrue(tuple1.getY() == min);
         assertTrue(tuple1.getZ() == min);

         min = random.nextDouble();
         tuple1.setX(min + random.nextDouble());
         tuple1.setY(min + random.nextDouble());
         tuple1.setZ(min + random.nextDouble());
         tuple1.clipToMin(min);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.clipToMin(double min, TupleBasics other)
         double min = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(min - random.nextDouble());
         tuple2.setY(min - random.nextDouble());
         tuple2.setZ(min - random.nextDouble());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.getX() == min);
         assertTrue(tuple1.getY() == min);
         assertTrue(tuple1.getZ() == min);

         min = random.nextDouble();
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.clipToMinMax(double min, double max)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         tuple1.setX(min - random.nextDouble());
         tuple1.setY(min - random.nextDouble());
         tuple1.setZ(min - random.nextDouble());
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.getX() == min);
         assertTrue(tuple1.getY() == min);
         assertTrue(tuple1.getZ() == min);

         min = random.nextDouble() - 0.5;
         max = random.nextDouble() + 0.5;
         tuple1.setX(max + random.nextDouble());
         tuple1.setY(max + random.nextDouble());
         tuple1.setZ(max + random.nextDouble());
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.getX() == max);
         assertTrue(tuple1.getY() == max);
         assertTrue(tuple1.getZ() == max);

         min = random.nextDouble() - 1.0;
         max = random.nextDouble() + 1.0;
         tuple1.setX(min + random.nextDouble());
         tuple1.setY(min + random.nextDouble());
         tuple1.setZ(min + random.nextDouble());
         tuple1.clipToMinMax(min, max);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.clipToMinMax(double min, double max, TupleBasics other)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(min - random.nextDouble());
         tuple2.setY(min - random.nextDouble());
         tuple2.setZ(min - random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.getX() == min);
         assertTrue(tuple1.getY() == min);
         assertTrue(tuple1.getZ() == min);

         min = random.nextDouble() - 0.5;
         max = random.nextDouble() + 0.5;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(max + random.nextDouble());
         tuple2.setY(max + random.nextDouble());
         tuple2.setZ(max + random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.getX() == max);
         assertTrue(tuple1.getY() == max);
         assertTrue(tuple1.getZ() == max);

         min = random.nextDouble() - 1.0;
         max = random.nextDouble() + 1.0;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         assertFalse(tuple1.containsNaN());

         tuple1.setX(Double.POSITIVE_INFINITY);
         tuple1.setY(Double.NEGATIVE_INFINITY);
         tuple1.setZ(Double.NEGATIVE_INFINITY);
         assertFalse(tuple1.containsNaN());

         tuple1.setX(Double.NaN);
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         assertTrue(tuple1.containsNaN());

         tuple1.setX(random.nextDouble());
         tuple1.setY(Double.NaN);
         tuple1.setZ(random.nextDouble());
         assertTrue(tuple1.containsNaN());

         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(Double.NaN);
         assertTrue(tuple1.containsNaN());
      }
   }

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.get(double[] tupleArrayToPack)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());

         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble()};

         try
         {
            tuple1.get(tupleArray);
            fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IndexOutOfBoundsException.");
         }

         tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         double[] tupleArrayCopy = new double[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.get(tupleArray);
         assertTrue(tuple1.getX() == tupleArray[0]);
         assertTrue(tuple1.getY() == tupleArray[1]);
         assertTrue(tuple1.getZ() == tupleArray[2]);
         for (int j = 3; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.get(double[] tupleArrayToPack, int startIndex)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());

         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};

         try
         {
            tuple1.get(2, tupleArray);
            fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IndexOutOfBoundsException.");
         }

         tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         double[] tupleArrayCopy = new double[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.get(3, tupleArray);
         assertTrue(tuple1.getX() == tupleArray[3]);
         assertTrue(tuple1.getY() == tupleArray[4]);
         assertTrue(tuple1.getZ() == tupleArray[5]);
         for (int j = 0; j < 3; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(2, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         try
         {
            tuple1.get(matrix);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IllegalArgumentException.");
         }

         matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         tuple1.get(matrix);
         assertTrue(tuple1.getX() == matrix.get(0, 0));
         assertTrue(tuple1.getY() == matrix.get(1, 0));
         assertTrue(tuple1.getZ() == matrix.get(2, 0));

         matrixCopy.set(0, 0, tuple1.getX());
         matrixCopy.set(1, 0, tuple1.getY());
         matrixCopy.set(2, 0, tuple1.getZ());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(6, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         try
         {
            tuple1.get(4, matrix);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IllegalArgumentException.");
         }

         matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         tuple1.get(2, matrix);
         assertTrue(tuple1.getX() == matrix.get(2, 0));
         assertTrue(tuple1.getY() == matrix.get(3, 0));
         assertTrue(tuple1.getZ() == matrix.get(4, 0));

         matrixCopy.set(2, 0, tuple1.getX());
         matrixCopy.set(3, 0, tuple1.getY());
         matrixCopy.set(4, 0, tuple1.getZ());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(6, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         try
         {
            tuple1.get(4, 3, matrix);
            fail("Should have thrown IllegalArgumentException.");
         }
         catch (IllegalArgumentException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IllegalArgumentException.");
         }

         matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         tuple1.get(2, 4, matrix);
         assertTrue(tuple1.getX() == matrix.get(2, 4));
         assertTrue(tuple1.getY() == matrix.get(3, 4));
         assertTrue(tuple1.getZ() == matrix.get(4, 4));

         matrixCopy.set(2, 4, tuple1.getX());
         matrixCopy.set(3, 4, tuple1.getY());
         matrixCopy.set(4, 4, tuple1.getZ());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.get(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.setX(x);
         tuple1.setY(y);
         tuple1.setZ(z);

         assertTrue(tuple1.get(0) == x);
         assertTrue(tuple1.get(1) == y);
         assertTrue(tuple1.get(2) == z);

         try
         {
            tuple1.get(-1);
            fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IndexOutOfBoundsException.");
         }

         try
         {
            tuple1.get(3);
            fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown IndexOutOfBoundsException.");
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple.getX(), Tuple.getY(), Tuple.getZ()
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         tuple1.setX(x);
         tuple1.setY(y);
         tuple1.setZ(z);

         assertTrue(tuple1.getX() == x);
         assertTrue(tuple1.getY() == y);
         assertTrue(tuple1.getZ() == z);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();
      Tuple3D tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();

         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());

         tuple2.setX(tuple1.getX() + 0.1 * epsilon);
         tuple2.setY(tuple1.getY() + 0.1 * epsilon);
         tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX() - 0.1 * epsilon);
         tuple2.setY(tuple1.getY() - 0.1 * epsilon);
         tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX() - 1.1 * epsilon);
         tuple2.setY(tuple1.getY() - 0.1 * epsilon);
         tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX() - 0.1 * epsilon);
         tuple2.setY(tuple1.getY() - 1.1 * epsilon);
         tuple2.setZ(tuple1.getZ() - 0.1 * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX() - 0.1 * epsilon);
         tuple2.setY(tuple1.getY() - 0.1 * epsilon);
         tuple2.setZ(tuple1.getZ() - 1.1 * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX() + 1.1 * epsilon);
         tuple2.setY(tuple1.getY() + 0.1 * epsilon);
         tuple2.setZ(tuple1.getZ() + 0.1 * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX()+ 0.1 * epsilon);
         tuple2.setY(tuple1.getY()+ 1.1 * epsilon);
         tuple2.setZ(tuple1.getZ()+ 0.1 * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX() + 0.1 * epsilon);
         tuple2.setY(tuple1.getY() + 0.1 * epsilon);
         tuple2.setZ(tuple1.getZ() + 1.1 * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX() + epsilon);
         tuple2.setY(tuple1.getY() + epsilon);
         tuple2.setZ(tuple1.getZ() + epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX() - epsilon);
         tuple2.setY(tuple1.getY() - epsilon);
         tuple2.setZ(tuple1.getZ() - epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();

      // Test Tuple.equals(Tuple other)
      Tuple3D tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple1.set(tuple2);

         assertTrue(tuple1.equals(tuple2));
         assertTrue(tuple1.equals((Object) tuple2));
         assertFalse(tuple1.equals((Tuple3D) null));
         assertFalse(tuple1.equals((Object) null));

         double smallestEpsilon = 1.0e-16;

         tuple1.set(tuple2);
         tuple1.add(smallestEpsilon, 0.0, 0.0);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.sub(smallestEpsilon, 0.0, 0.0);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.add(0.0, smallestEpsilon, 0.0);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.sub(0.0, smallestEpsilon, 0.0);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.add(0.0, 0.0, smallestEpsilon);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.sub(0.0, 0.0, smallestEpsilon);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.setX(Double.NaN);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.setY(Double.NaN);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.setZ(Double.NaN);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));


         double[] blop = new double[] {tuple1.getX(), tuple1.getY(), tuple1.getZ()};
         assertFalse(tuple1.equals(blop));

         Point3D32 point32 = new Point3D32();
         point32.set(tuple1);
         assertFalse(tuple1.equals(point32));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D tuple1 = createEmptyTuple();

      tuple1.setX(random.nextDouble());
      tuple1.setY(random.nextDouble());
      tuple1.setZ(random.nextDouble());

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.set(i % 3, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
