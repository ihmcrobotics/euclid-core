package us.ihmc.geometry.tuple2D;

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
import us.ihmc.geometry.tuple3D.Tuple3DTools;
import us.ihmc.geometry.tuple3D.Tuple3DTest;

public abstract class Tuple2D32Test<T extends Tuple2D32<T>>
{
   public static final int NUMBER_OF_ITERATIONS = Tuple3DTest.NUMBER_OF_ITERATIONS;

   public abstract T createEmptyTuple();

   @Test
   public void testTuple()
   {
      T tuple = createEmptyTuple();
      Assert.assertTrue(tuple.getX32() == 0);
      Assert.assertTrue(tuple.getY32() == 0);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T tuple = createEmptyTuple();
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple);

         tuple.setToNaN();
         assertTrue(Float.isNaN(tuple.getX32()));
         assertTrue(Float.isNaN(tuple.getY32()));
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         T tuple = createEmptyTuple();
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple);

         tuple.setToZero();
         assertTrue(tuple.getX32() == 0.0);
         assertTrue(tuple.getY32() == 0.0);
      }
   }

   @Test
   public void testAbsolute() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (float signX = -1.0f; signX <= 1.0; signX += 2.0)
      {
         for (float signY = -1.0f; signY <= 1.0; signY += 2.0)
         {
            float xPos = random.nextFloat();
            float yPos = random.nextFloat();
            tuple1.setX(signX * xPos);
            tuple1.setY(signY * yPos);

            tuple2.setAndAbsolute(tuple1);
            assertTrue(tuple2.getX32() == xPos);
            assertTrue(tuple2.getY32() == yPos);
            assertTrue(tuple1.getX32() == signX * xPos);
            assertTrue(tuple1.getY32() == signY * yPos);

            tuple1.absolute();
            assertTrue(tuple1.getX32() == xPos);
            assertTrue(tuple1.getY32() == yPos);
         }
      }
   }

   @Test
   public void testNegate() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (float signX = -1.0f; signX <= 1.0; signX += 2.0)
      {
         for (float signY = -1.0f; signY <= 1.0; signY += 2.0)
         {
            float xOriginal = signX * random.nextFloat();
            float yOriginal = signY * random.nextFloat();
            tuple1.setX(xOriginal);
            tuple1.setY(yOriginal);

            tuple2.setAndNegate(tuple1);
            assertTrue(tuple2.getX32() == -xOriginal);
            assertTrue(tuple2.getY32() == -yOriginal);
            assertTrue(tuple1.getX32() == xOriginal);
            assertTrue(tuple1.getY32() == yOriginal);

            tuple1.negate();
            assertTrue(tuple1.getX32() == -xOriginal);
            assertTrue(tuple1.getY32() == -yOriginal);
         }
      }
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.set(float x, float y);
         float newX = random.nextFloat();
         float newY = random.nextFloat();
         tuple1.set(newX, newY);

         assertTrue(tuple1.getX32() == newX);
         assertTrue(tuple1.getY32() == newY);
      }

      for (int j = 0; j < NUMBER_OF_ITERATIONS; j++)
      { // Test Tuple2D32.set(float[] tupleArray);
         float tupleArray[] = {random.nextFloat()};
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

         for (int i = 0; i < tupleArray.length; i++)
            assertTrue(tupleArray[i] == tupleArrayCopy[i]);

         tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tupleArrayCopy = new float[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.set(tupleArray);

         assertTrue(tuple1.getX32() == tupleArray[0]);
         assertTrue(tuple1.getY32() == tupleArray[1]);

         for (int i = 0; i < tupleArray.length; i++)
            assertTrue(tupleArray[i] == tupleArrayCopy[i]);
      }

      for (int j = 0; j < NUMBER_OF_ITERATIONS; j++)
      { // Test Tuple2D32.set(float[] tupleArray, int startIndex);
         float tupleArray[] = {random.nextFloat(), random.nextFloat()};
         float[] tupleArrayCopy = new float[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);

         try
         {
            tuple1.set(tupleArray, 1);
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
         for (int i = 0; i < tupleArray.length; i++)
            assertTrue(tupleArray[i] == tupleArrayCopy[i]);

         tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tupleArrayCopy = new float[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.set(tupleArray, 2);

         assertTrue(tuple1.getX32() == tupleArray[2]);
         assertTrue(tuple1.getY32() == tupleArray[3]);

         for (int i = 0; i < tupleArray.length; i++)
            assertTrue(tupleArray[i] == tupleArrayCopy[i]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.set(DenseMatrix64F matrix);
         DenseMatrix64F matrix = new DenseMatrix64F(1, 1);
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

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.set(DenseMatrix64F matrix, int startRow);
         DenseMatrix64F matrix = new DenseMatrix64F(3, 1);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         try
         {
            tuple1.set(matrix, 2);
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
         tuple1.set(matrix, 5);

         assertTrue(tuple1.getX32() == matrix.get(5, 0));
         assertTrue(tuple1.getY32() == matrix.get(6, 0));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.set(DenseMatrix64F matrix, int startRow, int column);
         DenseMatrix64F matrix = new DenseMatrix64F(3, 6);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         try
         {
            tuple1.set(matrix, 2, 3);
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
         tuple1.set(matrix, 5, 2);

         assertTrue(tuple1.getX32() == matrix.get(5, 2));
         assertTrue(tuple1.getY32() == matrix.get(6, 2));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.set(int index, float value)
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
         tuple1.set(0, x);
         tuple1.set(1, y);

         assertTrue(tuple1.getX32() == x);
         assertTrue(tuple1.getY32() == y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.set(TupleBasics other)
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);
         tuple1.set(tuple2);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.setX(float x)
         float x = random.nextFloat();
         tuple1.setX(x);
         assertTrue(tuple1.getX32() == x);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.setY(float y)
         float y = random.nextFloat();
         tuple1.setY(y);
         assertTrue(tuple1.getY32() == y);
      }
   }

   @Test
   public void testAdd() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.add(float x, float y)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float x = random.nextFloat();
         float y = random.nextFloat();
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.add(x, y);
         assertTrue(tuple1.getX32() == xOld + x);
         assertTrue(tuple1.getY32() == yOld + y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.add(TupleBasics other)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.add(tuple2);
         assertTrue(tuple1.getX32() == xOld + tuple2.getX32());
         assertTrue(tuple1.getY32() == yOld + tuple2.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.add(TupleBasics other)
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple3);

         tuple1.add(tuple2, tuple3);
         assertTrue(tuple1.getX32() == tuple2.getX32() + tuple3.getX32());
         assertTrue(tuple1.getY32() == tuple2.getY32() + tuple3.getY32());
      }
   }

   @Test
   public void testSub() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.sub(float x, float y)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float x = random.nextFloat();
         float y = random.nextFloat();
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.sub(x, y);
         assertTrue(tuple1.getX32() == xOld - x);
         assertTrue(tuple1.getY32() == yOld - y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.sub(TupleBasics other)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.sub(tuple2);
         assertTrue(tuple1.getX32() == xOld - tuple2.getX32());
         assertTrue(tuple1.getY32() == yOld - tuple2.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.sub(TupleBasics tuple1, TupleBasics tuple2)
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple3);

         tuple1.sub(tuple2, tuple3);
         assertTrue(tuple1.getX32() == tuple2.getX32() - tuple3.getX32());
         assertTrue(tuple1.getY32() == tuple2.getY32() - tuple3.getY32());
      }
   }

   @Test
   public void testScale() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.scale(float scalarX, float scalarY)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float scale = random.nextFloat();
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.scale(scale);
         assertTrue(tuple1.getX32() == xOld * scale);
         assertTrue(tuple1.getY32() == yOld * scale);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.scale(float scalarX, float scalarY)
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         float scaleX = random.nextFloat();
         float scaleY = random.nextFloat();
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.scale(scaleX, scaleY);
         assertTrue(tuple1.getX32() == xOld * scaleX);
         assertTrue(tuple1.getY32() == yOld * scaleY);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.scale(float scalar, TupleBasics other)
         float scale = random.nextFloat();
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);

         tuple1.scale(scale, tuple2);
         assertTrue(tuple1.getX32() == tuple2.getX32() * scale);
         assertTrue(tuple1.getY32() == tuple2.getY32() * scale);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.scaleAdd(float scalar, TupleBasics other)
         float scale = random.nextFloat();
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);

         tuple1.scaleAdd(scale, tuple2);
         assertTrue(tuple1.getX32() == xOld * scale + tuple2.getX32());
         assertTrue(tuple1.getY32() == yOld * scale + tuple2.getY32());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.scaleAdd(float scalar, TupleBasics tuple1, TupleBasics tuple2)
         float scale = random.nextFloat();
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple3);

         tuple1.scaleAdd(scale, tuple2, tuple3);
         assertTrue(tuple1.getX32() == tuple2.getX32() * scale + tuple3.getX32());
         assertTrue(tuple1.getY32() == tuple2.getY32() * scale + tuple3.getY32());
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.interpolate(TupleBasics other, float alpha)
         float alpha = random.nextFloat();
         float xOld = random.nextFloat();
         float yOld = random.nextFloat();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);

         tuple1.interpolate(tuple2, alpha);
         assertTrue(tuple1.getX32() == (float) Tuple3DTools.interpolate(xOld, tuple2.getX32(), alpha));
         assertTrue(tuple1.getY32() == (float) Tuple3DTools.interpolate(yOld, tuple2.getY32(), alpha));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.interpolate(TupleBasics tuple1, TupleBasics tuple2, float alpha)
         float alpha = random.nextFloat();
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple3);

         tuple1.interpolate(tuple2, tuple3, alpha);
         assertTrue(tuple1.getX32() == (float) Tuple3DTools.interpolate(tuple2.getX32(), tuple3.getX32(), alpha));
         assertTrue(tuple1.getY32() == (float) Tuple3DTools.interpolate(tuple2.getY32(), tuple3.getY32(), alpha));
      }
   }

   @Test
   public void testClip() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMax(float max)
         float max = random.nextFloat();
         tuple1.setX(max + random.nextFloat());
         tuple1.setY(max + random.nextFloat());
         tuple1.clipToMax(max);
         assertTrue(tuple1.getX32() == max);
         assertTrue(tuple1.getY32() == max);

         max = random.nextFloat();
         tuple1.setX(max - random.nextFloat());
         tuple1.setY(max - random.nextFloat());
         tuple1.clipToMax(max);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMax(float max, Tuple32Basics other)
         float max = random.nextFloat();
         tuple1.set(random.nextFloat(), random.nextFloat());
         tuple2.setX(max + random.nextFloat());
         tuple2.setY(max + random.nextFloat());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.getX32() == max);
         assertTrue(tuple1.getY32() == max);

         max = random.nextFloat();
         tuple2.setX(max - random.nextFloat());
         tuple2.setY(max - random.nextFloat());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMin(float min)
         float min = random.nextFloat();
         tuple1.setX(min - random.nextFloat());
         tuple1.setY(min - random.nextFloat());
         tuple1.clipToMin(min);
         assertTrue(tuple1.getX32() == min);
         assertTrue(tuple1.getY32() == min);

         min = random.nextFloat();
         tuple1.setX(min + random.nextFloat());
         tuple1.setY(min + random.nextFloat());
         tuple1.clipToMin(min);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMin(float min, Tuple32Basics other)
         float min = random.nextFloat();
         tuple1.set(random.nextFloat(), random.nextFloat());
         tuple2.setX(min - random.nextFloat());
         tuple2.setY(min - random.nextFloat());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.getX32() == min);
         assertTrue(tuple1.getY32() == min);

         min = random.nextFloat();
         tuple2.setX(min + random.nextFloat());
         tuple2.setY(min + random.nextFloat());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMinMax(float min, float max)
         float min = random.nextFloat() - 0.5f;
         float max = random.nextFloat() + 0.5f;
         tuple1.setX(min - random.nextFloat());
         tuple1.setY(min - random.nextFloat());
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.getX32() == min);
         assertTrue(tuple1.getY32() == min);

         min = random.nextFloat() - 0.5f;
         max = random.nextFloat() + 0.5f;
         tuple1.setX(max + random.nextFloat());
         tuple1.setY(max + random.nextFloat());
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.getX32() == max);
         assertTrue(tuple1.getY32() == max);

         min = random.nextFloat() - 1.0f;
         max = random.nextFloat() + 1.0f;
         tuple1.setX(min + random.nextFloat());
         tuple1.setY(min + random.nextFloat());
         tuple1.clipToMinMax(min, max);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple32.clipToMinMax(float min, float max, Tuple32Basics other)
         float min = random.nextFloat() - 0.5f;
         float max = random.nextFloat() + 0.5f;
         tuple1.set(random.nextFloat(), random.nextFloat());
         tuple2.setX(min - random.nextFloat());
         tuple2.setY(min - random.nextFloat());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.getX32() == min);
         assertTrue(tuple1.getY32() == min);

         min = random.nextFloat() - 0.5f;
         max = random.nextFloat() + 0.5f;
         tuple1.setX(random.nextFloat());
         tuple1.setY(random.nextFloat());
         tuple2.setX(max + random.nextFloat());
         tuple2.setY(max + random.nextFloat());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.getX32() == max);
         assertTrue(tuple1.getY32() == max);

         min = random.nextFloat() - 1.0f;
         max = random.nextFloat() + 1.0f;
         tuple1.setX(random.nextFloat());
         tuple1.setY(random.nextFloat());
         tuple2.setX(min + random.nextFloat());
         tuple2.setY(min + random.nextFloat());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      T tuple1 = createEmptyTuple();
      tuple1.set(0.0f, 0.0f);
      assertFalse(tuple1.containsNaN());
      tuple1.set(Float.NaN, 0.0f);
      assertTrue(tuple1.containsNaN());
      tuple1.set(0.0f, Float.NaN);
      assertTrue(tuple1.containsNaN());
   }

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.get(float[] tupleArrayToPack)
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);

         float[] tupleArray = new float[] {random.nextFloat()};

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
         for (int j = 3; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.get(float[] tupleArrayToPack, int startIndex)
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);

         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat()};

         try
         {
            tuple1.get(tupleArray, 2);
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
         tuple1.get(tupleArray, 3);
         assertTrue(tuple1.getX32() == tupleArray[3]);
         assertTrue(tuple1.getY32() == tupleArray[4]);
         for (int j = 0; j < 3; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(1, 5);
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

         matrixCopy.set(0, 0, tuple1.getX32());
         matrixCopy.set(1, 0, tuple1.getY32());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(5, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());

         try
         {
            tuple1.get(matrix, 4);
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

         tuple1.get(matrix, 2);
         assertTrue(tuple1.getX32() == matrix.get(2, 0));
         assertTrue(tuple1.getY32() == matrix.get(3, 0));

         matrixCopy.set(2, 0, tuple1.getX32());
         matrixCopy.set(3, 0, tuple1.getY32());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(5, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextFloat());

         try
         {
            tuple1.get(matrix, 4, 3);
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

         tuple1.get(matrix, 2, 4);
         assertTrue(tuple1.getX32() == matrix.get(2, 4));
         assertTrue(tuple1.getY32() == matrix.get(3, 4));

         matrixCopy.set(2, 4, tuple1.getX32());
         matrixCopy.set(3, 4, tuple1.getY32());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D32.get(int index)
         float x = random.nextFloat();
         float y = random.nextFloat();
         tuple1.set(x, y);

         assertTrue(tuple1.get(0) == x);
         assertTrue(tuple1.get(1) == y);

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
      { // Test Tuple2D32.getX(), Tuple2D32.getY()
         float x = random.nextFloat();
         float y = random.nextFloat();
         tuple1.set(x, y);

         assertTrue(tuple1.getX() == x);
         assertTrue(tuple1.getY() == y);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);
      GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);

      float epsilon = random.nextFloat();

      assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

      for (int index = 0; index < 2; index++)
      {
         tuple1.set(tuple2);
         tuple1.set(index, tuple2.get(index) + 0.999f * epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple1.set(tuple2);
         tuple1.set(index, tuple2.get(index) - 0.999f * epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple1.set(tuple2);
         tuple1.set(index, tuple2.get(index) + 1.001f * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple1.set(tuple2);
         tuple1.set(index, tuple2.get(index) - 1.001f * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();

      // Test Tuple32.equals(Tuple32 other)
      T tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         GeometryBasicsRandomTools.randomizeTuple2D(random, tuple2);
         tuple1.set(tuple2);

         assertFalse(tuple1.equals((T) null));
         assertFalse(tuple1.equals((Object) null));
         assertTrue(tuple1.equals(tuple2));
         assertTrue(tuple1.equals((Object) tuple2));

         float smallestEpsilon = 1.0e-7f;

         tuple1.set(tuple2);
         tuple1.add(smallestEpsilon, 0.0f);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.sub(smallestEpsilon, 0.0f);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.add(0.0f, smallestEpsilon);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         tuple1.set(tuple2);
         tuple1.sub(0.0f, smallestEpsilon);
         assertFalse(tuple1.equals(tuple2));
         assertFalse(tuple1.equals((Object) tuple2));

         float[] blop = new float[] {tuple1.getX32(), tuple1.getY32()};
         assertFalse(tuple1.equals(blop));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.set(i % 2, random.nextFloat());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
