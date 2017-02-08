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

import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.tuple3D.Tuple3D32;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public abstract class Tuple3D32Test
{
   public static final int NUMBER_OF_ITERATIONS = Tuple3DTest.NUMBER_OF_ITERATIONS;

   public abstract Tuple3D32 createEmptyTuple32();

   @Test
   public void testTuple()
   {
      Tuple3D32 tuple = createEmptyTuple32();
      Assert.assertTrue(tuple.getX() == 0);
      Assert.assertTrue(tuple.getY() == 0);
      Assert.assertTrue(tuple.getZ() == 0);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple = createEmptyTuple32();
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple.setToNaN();
         assertTrue(Float.isNaN(tuple.getX32()));
         assertTrue(Float.isNaN(tuple.getY32()));
         assertTrue(Float.isNaN(tuple.getZ32()));
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple = createEmptyTuple32();
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple.setToZero();
         assertTrue(tuple.getX32() == 0.0);
         assertTrue(tuple.getY32() == 0.0);
         assertTrue(tuple.getZ32() == 0.0);
      }
   }

   @Test
   public void testAbsolute() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();

      for (float signX = -1.0f; signX <= 1.0; signX += 2.0)
      {
         for (float signY = -1.0f; signY <= 1.0; signY += 2.0)
         {
            for (float signZ = -1.0f; signZ <= 1.0; signZ += 2.0)
            {
               float xPos = random.nextFloat();
               float yPos = random.nextFloat();
               float zPos = random.nextFloat();
               tuple1.setX(signX * xPos);
               tuple1.setY(signY * yPos);
               tuple1.setZ(signZ * zPos);

               tuple2.setAndAbsolute(tuple1);
               assertTrue(tuple2.getX32() == xPos);
               assertTrue(tuple2.getY32() == yPos);
               assertTrue(tuple2.getZ32() == zPos);
               assertTrue(tuple1.getX32() == signX * xPos);
               assertTrue(tuple1.getY32() == signY * yPos);
               assertTrue(tuple1.getZ32() == signZ * zPos);

               tuple1.absolute();
               assertTrue(tuple1.getX32() == xPos);
               assertTrue(tuple1.getY32() == yPos);
               assertTrue(tuple1.getZ32() == zPos);
            }
         }
      }
   }

   @Test
   public void testNegate() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();

      for (float signX = -1.0f; signX <= 1.0; signX += 2.0)
      {
         for (float signY = -1.0f; signY <= 1.0; signY += 2.0)
         {
            for (float signZ = -1.0f; signZ <= 1.0; signZ += 2.0)
            {
               float xOriginal = signX * random.nextFloat();
               float yOriginal = signY * random.nextFloat();
               float zOriginal = signZ * random.nextFloat();
               tuple1.set(xOriginal, yOriginal, zOriginal);

               tuple2.setAndNegate(tuple1);
               assertTrue(tuple2.getX32() == -xOriginal);
               assertTrue(tuple2.getY32() == -yOriginal);
               assertTrue(tuple2.getZ32() == -zOriginal);
               assertTrue(tuple1.getX32() == xOriginal);
               assertTrue(tuple1.getY32() == yOriginal);
               assertTrue(tuple1.getZ32() == zOriginal);

               tuple1.negate();
               assertTrue(tuple1.getX32() == -xOriginal);
               assertTrue(tuple1.getY32() == -yOriginal);
               assertTrue(tuple1.getZ32() == -zOriginal);
            }
         }
      }
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(float x, float y, float z);
         float newX = random.nextFloat();
         float newY = random.nextFloat();
         float newZ = random.nextFloat();
         tuple1.set(newX, newY, newZ);

         assertTrue(tuple1.getX32() == newX);
         assertTrue(tuple1.getY32() == newY);
         assertTrue(tuple1.getZ32() == newZ);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(double x, double y, double z);
         float newX = random.nextFloat();
         float newY = random.nextFloat();
         float newZ = random.nextFloat();
         tuple1.set((double) newX, (double) newY, (double) newZ);

         assertTrue(tuple1.getX32() == newX);
         assertTrue(tuple1.getY32() == newY);
         assertTrue(tuple1.getZ32() == newZ);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(float[] tupleArray);
         float tupleArray[] = {random.nextFloat(), random.nextFloat()};
         float[] tupleArrayCopy = new float[tupleArray.length];
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

         tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tupleArrayCopy = new float[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.set(tupleArray);

         assertTrue(tuple1.getX32() == tupleArray[0]);
         assertTrue(tuple1.getY32() == tupleArray[1]);
         assertTrue(tuple1.getZ32() == tupleArray[2]);

         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(float[] tupleArray, int startIndex);
         float tupleArray[] = {random.nextFloat(), random.nextFloat(), random.nextFloat()};
         float[] tupleArrayCopy = new float[tupleArray.length];
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

         tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tupleArrayCopy = new float[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.set(2, tupleArray);

         assertTrue(tuple1.getX32() == tupleArray[2]);
         assertTrue(tuple1.getY32() == tupleArray[3]);
         assertTrue(tuple1.getZ32() == tupleArray[4]);

         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(DenseMatrix64F matrix);
         DenseMatrix64F matrix = new DenseMatrix64F(2, 1);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());
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
            matrix.set(index, random.nextFloat());
         matrixCopy = new DenseMatrix64F(matrix);
         tuple1.set(matrix);

         assertTrue(tuple1.getX32() == matrix.get(0, 0));
         assertTrue(tuple1.getY32() == matrix.get(1, 0));
         assertTrue(tuple1.getZ32() == matrix.get(2, 0));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(DenseMatrix64F matrix, int startRow);
         DenseMatrix64F matrix = new DenseMatrix64F(4, 1);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());
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
            matrix.set(index, random.nextFloat());
         matrixCopy = new DenseMatrix64F(matrix);
         tuple1.set(5, matrix);

         assertTrue(tuple1.getX32() == matrix.get(5, 0));
         assertTrue(tuple1.getY32() == matrix.get(6, 0));
         assertTrue(tuple1.getZ32() == matrix.get(7, 0));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(DenseMatrix64F matrix, int startRow, int column);
         DenseMatrix64F matrix = new DenseMatrix64F(4, 6);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());
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
            matrix.set(index, random.nextFloat());
         matrixCopy = new DenseMatrix64F(matrix);
         tuple1.set(5, 2, matrix);

         assertTrue(tuple1.getX32() == matrix.get(5, 2));
         assertTrue(tuple1.getY32() == matrix.get(6, 2));
         assertTrue(tuple1.getZ32() == matrix.get(7, 2));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(int index, float value)
         try
         {
            tuple1.set(-1, random.nextFloat());
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
            tuple1.set(3, random.nextFloat());
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

         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple1.set(0, x);
         tuple1.set(1, y);
         tuple1.set(2, z);

         assertTrue(tuple1.getX32() == x);
         assertTrue(tuple1.getY32() == y);
         assertTrue(tuple1.getZ32() == z);

         double xDouble = random.nextDouble();
         double yDouble = random.nextDouble();
         double zDouble = random.nextDouble();
         tuple1.set(0, xDouble);
         tuple1.set(1, yDouble);
         tuple1.set(2, zDouble);

         assertEquals(tuple1.getX32(), xDouble, 1.0e-7);
         assertEquals(tuple1.getY32(), yDouble, 1.0e-7);
         assertEquals(tuple1.getZ32(), zDouble, 1.0e-7);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.set(Tuple32Basics other)
         tuple2.setX(random.nextFloat());
         tuple2.setY(random.nextFloat());
         tuple2.setZ(random.nextFloat());

         tuple1.set(tuple2);

         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.setX(float x)
         float x = random.nextFloat();
         tuple1.setX(x);
         assertTrue(tuple1.getX32() == x);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.setY(float y)
         float y = random.nextFloat();
         tuple1.setY(y);
         assertTrue(tuple1.getY32() == y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.setZ(float z)
         float z = random.nextFloat();
         tuple1.setZ(z);
         assertTrue(tuple1.getZ32() == z);
      }
   }

   @Test
   public void testAdd() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();
      Tuple3D32 tuple3 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.add(double x, double y, double z)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple1.set(xOld, yOld, zOld);

         tuple1.add((double) x, (double) y, (double) z);
         assertTrue(tuple1.getX32() == xOld + x);
         assertTrue(tuple1.getY32() == yOld + y);
         assertTrue(tuple1.getZ32() == zOld + z);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.add(float x, float y, float z)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple1.set(xOld, yOld, zOld);

         tuple1.add(x, y, z);
         assertTrue(tuple1.getX32() == xOld + x);
         assertTrue(tuple1.getY32() == yOld + y);
         assertTrue(tuple1.getZ32() == zOld + z);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.add(Tuple32Basics other)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple1.set(xOld, yOld, zOld);

         tuple1.add(tuple2);
         assertTrue(tuple1.getX32() == xOld + tuple2.getX32());
         assertTrue(tuple1.getY32() == yOld + tuple2.getY32());
         assertTrue(tuple1.getZ32() == zOld + tuple2.getZ32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.add(Tuple32Basics other)
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple3.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple1.add(tuple2, tuple3);
         assertTrue(tuple1.getX32() == tuple2.getX32() + tuple3.getX32());
         assertTrue(tuple1.getY32() == tuple2.getY32() + tuple3.getY32());
         assertTrue(tuple1.getZ32() == tuple2.getZ32() + tuple3.getZ32());
      }
   }

   @Test
   public void testSub() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();
      Tuple3D32 tuple3 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.sub(double x, double y, double z)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple1.set(xOld, yOld, zOld);

         tuple1.sub((double) x, (double) y, (double) z);
         assertTrue(tuple1.getX32() == xOld - x);
         assertTrue(tuple1.getY32() == yOld - y);
         assertTrue(tuple1.getZ32() == zOld - z);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.sub(float x, float y, float z)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple1.set(xOld, yOld, zOld);

         tuple1.sub(x, y, z);
         assertTrue(tuple1.getX32() == xOld - x);
         assertTrue(tuple1.getY32() == yOld - y);
         assertTrue(tuple1.getZ32() == zOld - z);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.sub(Tuple32Basics other)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple1.set(xOld, yOld, zOld);

         tuple1.sub(tuple2);
         assertTrue(tuple1.getX32() == xOld - tuple2.getX32());
         assertTrue(tuple1.getY32() == yOld - tuple2.getY32());
         assertTrue(tuple1.getZ32() == zOld - tuple2.getZ32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.sub(Tuple32Basics tuple1, Tuple32Basics tuple2)
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple3.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple1.sub(tuple2, tuple3);
         assertTrue(tuple1.getX32() == tuple2.getX32() - tuple3.getX32());
         assertTrue(tuple1.getY32() == tuple2.getY32() - tuple3.getY32());
         assertTrue(tuple1.getZ32() == tuple2.getZ32() - tuple3.getZ32());
      }
   }

   @Test
   public void testScale() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();
      Tuple3D32 tuple3 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.scale(float scalarX, float scalarY, float scalarZ)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         float scale = random.nextFloat();
         tuple1.set(xOld, yOld, zOld);

         tuple1.scale(scale);
         assertTrue(tuple1.getX32() == xOld * scale);
         assertTrue(tuple1.getY32() == yOld * scale);
         assertTrue(tuple1.getZ32() == zOld * scale);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.scale(float scalarX, float scalarY, float scalarZ)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         float scaleX = random.nextFloat();
         float scaleY = random.nextFloat();
         float scaleZ = random.nextFloat();
         tuple1.set(xOld, yOld, zOld);

         tuple1.scale(scaleX, scaleY, scaleZ);
         assertTrue(tuple1.getX32() == xOld * scaleX);
         assertTrue(tuple1.getY32() == yOld * scaleY);
         assertTrue(tuple1.getZ32() == zOld * scaleZ);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.scale(float scalar, Tuple32Basics other)
         float scale = random.nextFloat();
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple1.setAndScale(scale, tuple2);
         assertTrue(tuple1.getX32() == tuple2.getX32() * scale);
         assertTrue(tuple1.getY32() == tuple2.getY32() * scale);
         assertTrue(tuple1.getZ32() == tuple2.getZ32() * scale);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.scaleAdd(float scalar, Tuple32Basics other)
         float scale = random.nextFloat();
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         tuple1.set(xOld, yOld, zOld);
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple1.scaleAdd(scale, tuple2);
         assertTrue(tuple1.getX32() == xOld * scale + tuple2.getX32());
         assertTrue(tuple1.getY32() == yOld * scale + tuple2.getY32());
         assertTrue(tuple1.getZ32() == zOld * scale + tuple2.getZ32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.scaleAdd(float scalar, Tuple32Basics tuple1, Tuple32Basics tuple2)
         float scale = random.nextFloat();
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple3.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple1.scaleAdd(scale, tuple2, tuple3);
         assertTrue(tuple1.getX32() == tuple2.getX32() * scale + tuple3.getX32());
         assertTrue(tuple1.getY32() == tuple2.getY32() * scale + tuple3.getY32());
         assertTrue(tuple1.getZ32() == tuple2.getZ32() * scale + tuple3.getZ32());
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();
      Tuple3D32 tuple3 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.interpolate(TupleBasics other, float alpha)
         float alpha = random.nextFloat();
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float zOld = random.nextFloat();
         tuple1.set(xOld, yOld, zOld);
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple1.interpolate(tuple2, alpha);
         assertTrue(tuple1.getX32() == (float) Tuple3DTools.interpolate(xOld, tuple2.getX32(), alpha));
         assertTrue(tuple1.getY32() == (float) Tuple3DTools.interpolate(yOld, tuple2.getY32(), alpha));
         assertTrue(tuple1.getZ32() == (float) Tuple3DTools.interpolate(zOld, tuple2.getZ32(), alpha));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.interpolate(TupleBasics tuple1, TupleBasics tuple2, float alpha)
         float alpha = random.nextFloat();
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple2.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple3.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         tuple1.interpolate(tuple2, tuple3, alpha);
         assertTrue(tuple1.getX32() == (float) Tuple3DTools.interpolate(tuple2.getX32(), tuple3.getX32(), alpha));
         assertTrue(tuple1.getY32() == (float) Tuple3DTools.interpolate(tuple2.getY32(), tuple3.getY32(), alpha));
         assertTrue(tuple1.getZ32() == (float) Tuple3DTools.interpolate(tuple2.getZ32(), tuple3.getZ32(), alpha));
      }
   }

   @Test
   public void testClip() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMax(float max)
         float max = random.nextFloat();
         tuple1.setX(max + random.nextFloat());
         tuple1.setY(max + random.nextFloat());
         tuple1.setZ(max + random.nextFloat());
         tuple1.clipToMax(max);
         assertTrue(tuple1.getX32() == max);
         assertTrue(tuple1.getY32() == max);
         assertTrue(tuple1.getZ32() == max);

         max = random.nextFloat();
         tuple1.setX(max - random.nextFloat());
         tuple1.setY(max - random.nextFloat());
         tuple1.setZ(max - random.nextFloat());
         tuple1.clipToMax(max);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMax(float max, Tuple32Basics other)
         float max = random.nextFloat();
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple2.setX(max + random.nextFloat());
         tuple2.setY(max + random.nextFloat());
         tuple2.setZ(max + random.nextFloat());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.getX32() == max);
         assertTrue(tuple1.getY32() == max);
         assertTrue(tuple1.getZ32() == max);

         max = random.nextFloat();
         tuple2.setX(max - random.nextFloat());
         tuple2.setY(max - random.nextFloat());
         tuple2.setZ(max - random.nextFloat());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMin(float min)
         float min = random.nextFloat();
         tuple1.setX(min - random.nextFloat());
         tuple1.setY(min - random.nextFloat());
         tuple1.setZ(min - random.nextFloat());
         tuple1.clipToMin(min);
         assertTrue(tuple1.getX32() == min);
         assertTrue(tuple1.getY32() == min);
         assertTrue(tuple1.getZ32() == min);

         min = random.nextFloat();
         tuple1.setX(min + random.nextFloat());
         tuple1.setY(min + random.nextFloat());
         tuple1.setZ(min + random.nextFloat());
         tuple1.clipToMin(min);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMin(float min, Tuple32Basics other)
         float min = random.nextFloat();
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple2.setX(min - random.nextFloat());
         tuple2.setY(min - random.nextFloat());
         tuple2.setZ(min - random.nextFloat());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.getX32() == min);
         assertTrue(tuple1.getY32() == min);
         assertTrue(tuple1.getZ32() == min);

         min = random.nextFloat();
         tuple2.setX(min + random.nextFloat());
         tuple2.setY(min + random.nextFloat());
         tuple2.setZ(min + random.nextFloat());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMinMax(float min, float max)
         float min = random.nextFloat() - 0.5f;
         float max = random.nextFloat() + 0.5f;
         tuple1.setX(min - random.nextFloat());
         tuple1.setY(min - random.nextFloat());
         tuple1.setZ(min - random.nextFloat());
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.getX32() == min);
         assertTrue(tuple1.getY32() == min);
         assertTrue(tuple1.getZ32() == min);

         min = random.nextFloat() - 0.5f;
         max = random.nextFloat() + 0.5f;
         tuple1.setX(max + random.nextFloat());
         tuple1.setY(max + random.nextFloat());
         tuple1.setZ(max + random.nextFloat());
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.getX32() == max);
         assertTrue(tuple1.getY32() == max);
         assertTrue(tuple1.getZ32() == max);

         min = random.nextFloat() - 1.0f;
         max = random.nextFloat() + 1.0f;
         tuple1.setX(min + random.nextFloat());
         tuple1.setY(min + random.nextFloat());
         tuple1.setZ(min + random.nextFloat());
         tuple1.clipToMinMax(min, max);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMinMax(float min, float max, Tuple32Basics other)
         float min = random.nextFloat() - 0.5f;
         float max = random.nextFloat() + 0.5f;
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());
         tuple2.setX(min - random.nextFloat());
         tuple2.setY(min - random.nextFloat());
         tuple2.setZ(min - random.nextFloat());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.getX32() == min);
         assertTrue(tuple1.getY32() == min);
         assertTrue(tuple1.getZ32() == min);

         min = random.nextFloat() - 0.5f;
         max = random.nextFloat() + 0.5f;
         tuple1.setX(random.nextFloat());
         tuple1.setY(random.nextFloat());
         tuple1.setZ(random.nextFloat());
         tuple2.setX(max + random.nextFloat());
         tuple2.setY(max + random.nextFloat());
         tuple2.setZ(max + random.nextFloat());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.getX32() == max);
         assertTrue(tuple1.getY32() == max);
         assertTrue(tuple1.getZ32() == max);

         min = random.nextFloat() - 1.0f;
         max = random.nextFloat() + 1.0f;
         tuple1.setX(random.nextFloat());
         tuple1.setY(random.nextFloat());
         tuple1.setZ(random.nextFloat());
         tuple2.setX(min + random.nextFloat());
         tuple2.setY(min + random.nextFloat());
         tuple2.setZ(min + random.nextFloat());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Tuple3D32 tuple1 = createEmptyTuple32();
      tuple1.set(0.0f, 0.0f, 0.0f);
      assertFalse(tuple1.containsNaN());
      tuple1.set(Float.NaN, 0.0f, 0.0f);
      assertTrue(tuple1.containsNaN());
      tuple1.set(0.0f, Float.NaN, 0.0f);
      assertTrue(tuple1.containsNaN());
      tuple1.set(0.0f, 0.0f, Float.NaN);
      assertTrue(tuple1.containsNaN());
   }

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.get(float[] tupleArrayToPack)
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat()};

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

         tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         float[] tupleArrayCopy = new float[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.get(tupleArray);
         assertTrue(tuple1.getX32() == tupleArray[0]);
         assertTrue(tuple1.getY32() == tupleArray[1]);
         assertTrue(tuple1.getZ32() == tupleArray[2]);
         for (int j = 3; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.get(float[] tupleArrayToPack, int startIndex)
         tuple1.set(random.nextFloat(), random.nextFloat(), random.nextFloat());

         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat()};

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

         tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         float[] tupleArrayCopy = new float[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.get(3, tupleArray);
         assertTrue(tuple1.getX32() == tupleArray[3]);
         assertTrue(tuple1.getY32() == tupleArray[4]);
         assertTrue(tuple1.getZ32() == tupleArray[5]);
         for (int j = 0; j < 3; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(2, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());

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
            matrix.set(index, random.nextFloat());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         tuple1.get(matrix);
         assertTrue(tuple1.getX32() == matrix.get(0, 0));
         assertTrue(tuple1.getY32() == matrix.get(1, 0));
         assertTrue(tuple1.getZ32() == matrix.get(2, 0));

         matrixCopy.set(0, 0, tuple1.getX32());
         matrixCopy.set(1, 0, tuple1.getY32());
         matrixCopy.set(2, 0, tuple1.getZ32());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(6, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());

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
            matrix.set(index, random.nextFloat());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         tuple1.get(2, matrix);
         assertTrue(tuple1.getX32() == matrix.get(2, 0));
         assertTrue(tuple1.getY32() == matrix.get(3, 0));
         assertTrue(tuple1.getZ32() == matrix.get(4, 0));

         matrixCopy.set(2, 0, tuple1.getX32());
         matrixCopy.set(3, 0, tuple1.getY32());
         matrixCopy.set(4, 0, tuple1.getZ32());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(6, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());

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
            matrix.set(index, random.nextFloat());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         tuple1.get(2, 4, matrix);
         assertTrue(tuple1.getX32() == matrix.get(2, 4));
         assertTrue(tuple1.getY32() == matrix.get(3, 4));
         assertTrue(tuple1.getZ32() == matrix.get(4, 4));

         matrixCopy.set(2, 4, tuple1.getX32());
         matrixCopy.set(3, 4, tuple1.getY32());
         matrixCopy.set(4, 4, tuple1.getZ32());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.get(int index)
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple1.set(x, y, z);

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
      { // Test Tuple32.getX(), Tuple32.getY(), Tuple32.getZ()
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple1.set(x, y, z);

         assertTrue(tuple1.getX() == x);
         assertTrue(tuple1.getY() == y);
         assertTrue(tuple1.getZ() == z);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.getX32(), Tuple32.getY32(), Tuple32.getZ32()
         float x = random.nextFloat();
         float y = random.nextFloat();
         float z = random.nextFloat();
         tuple1.set(x, y, z);

         assertTrue(tuple1.getX32() == x);
         assertTrue(tuple1.getY32() == y);
         assertTrue(tuple1.getZ32() == z);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();
      Tuple3D32 tuple2 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         float epsilon = random.nextFloat();

         GeometryBasicsRandomTools.randomizeTuple(random, tuple1);

         tuple2.setX(tuple1.getX32() + 0.1f * epsilon);
         tuple2.setY(tuple1.getY32() + 0.1f * epsilon);
         tuple2.setZ(tuple1.getZ32() + 0.1f * epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() - 0.1f * epsilon);
         tuple2.setY(tuple1.getY32() - 0.1f * epsilon);
         tuple2.setZ(tuple1.getZ32() - 0.1f * epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() - 1.1f * epsilon);
         tuple2.setY(tuple1.getY32() - 0.1f * epsilon);
         tuple2.setZ(tuple1.getZ32() - 0.1f * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() - 0.1f * epsilon);
         tuple2.setY(tuple1.getY32() - 1.1f * epsilon);
         tuple2.setZ(tuple1.getZ32() - 0.1f * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() - 0.1f * epsilon);
         tuple2.setY(tuple1.getY32() - 0.1f * epsilon);
         tuple2.setZ(tuple1.getZ32() - 1.1f * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() + 1.1f * epsilon);
         tuple2.setY(tuple1.getY32() + 0.1f * epsilon);
         tuple2.setZ(tuple1.getZ32() + 0.1f * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() + 0.1f * epsilon);
         tuple2.setY(tuple1.getY32() + 1.1f * epsilon);
         tuple2.setZ(tuple1.getZ32() + 0.1f * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() + 0.1f * epsilon);
         tuple2.setY(tuple1.getY32() + 0.1f * epsilon);
         tuple2.setZ(tuple1.getZ32() + 1.1f * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() + epsilon);
         tuple2.setY(tuple1.getY32() + epsilon);
         tuple2.setZ(tuple1.getZ32() + epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.setX(tuple1.getX32() - epsilon);
         tuple2.setY(tuple1.getY32() - epsilon);
         tuple2.setZ(tuple1.getZ32() - epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = createEmptyTuple32();

      // Test Tuple32.equals(Tuple32 other)
      Tuple3D32 tuple2 = createEmptyTuple32();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple(random, tuple2);
         tuple1.set(tuple2);

         assertFalse(tuple1.equals((Tuple3D32) null));
         assertFalse(tuple1.equals((Object) null));
         assertTrue(tuple1.equals(tuple2));
         assertTrue(tuple1.equals((Object) tuple2));

         float smallestEpsilon = 1.0e-7f;

         tuple1.set(tuple2);
         tuple1.add(smallestEpsilon, 0.0f, 0.0f);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.sub(smallestEpsilon, 0.0f, 0.0f);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.add(0.0f, smallestEpsilon, 0.0f);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.sub(0.0f, smallestEpsilon, 0.0f);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.add(0.0f, 0.0f, smallestEpsilon);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.sub(0.0f, 0.0f, smallestEpsilon);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         float[] blop = new float[] {tuple1.getX32(), tuple1.getY32(), tuple1.getZ32()};
         assertFalse(tuple1.equals(blop));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Tuple3D32 tuple1 = GeometryBasicsRandomTools.generateRandomPoint32(random);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.set(i % 3, random.nextFloat());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
