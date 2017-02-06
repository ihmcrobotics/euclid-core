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
import us.ihmc.geometry.tuple.TupleTest;
import us.ihmc.geometry.tuple3D.Tuple3DTools;

public abstract class Tuple2DTest
{
   public static final int NUMBER_OF_ITERATIONS = TupleTest.NUMBER_OF_ITERATIONS;

   public abstract Tuple2D createEmptyTuple();

   @Test
   public void testTuple()
   {
      Tuple2D tuple = createEmptyTuple();
      Assert.assertTrue(tuple.getX() == 0);
      Assert.assertTrue(tuple.getY() == 0);
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Tuple2D tuple = createEmptyTuple();
         tuple.setX(random.nextDouble());
         tuple.setY(random.nextDouble());

         tuple.setToNaN();
         assertTrue(Double.isNaN(tuple.getX()));
         assertTrue(Double.isNaN(tuple.getY()));
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         Tuple2D tuple = createEmptyTuple();
         tuple.setX(random.nextDouble());
         tuple.setY(random.nextDouble());

         tuple.setToZero();
         assertTrue(tuple.getX() == 0.0);
         assertTrue(tuple.getY() == 0.0);
      }
   }

   @Test
   public void testAbsolute() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();

      for (double signX = -1.0; signX <= 1.0; signX += 2.0)
      {
         for (double signY = -1.0; signY <= 1.0; signY += 2.0)
         {
            double xPos = random.nextDouble();
            double yPos = random.nextDouble();
            tuple1.setX(signX * xPos);
            tuple1.setY(signY * yPos);

            tuple2.setAndAbsolute(tuple1);
            assertTrue(tuple2.getX() == xPos);
            assertTrue(tuple2.getY() == yPos);
            assertTrue(tuple1.getX() == signX * xPos);
            assertTrue(tuple1.getY() == signY * yPos);

            tuple1.absolute();
            assertTrue(tuple1.getX() == xPos);
            assertTrue(tuple1.getY() == yPos);
         }
      }
   }

   @Test
   public void testNegate() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();

      for (double signX = -1.0; signX <= 1.0; signX += 2.0)
      {
         for (double signY = -1.0; signY <= 1.0; signY += 2.0)
         {
            double xOriginal = signX * random.nextDouble();
            double yOriginal = signY * random.nextDouble();
            tuple1.setX(xOriginal);
            tuple1.setY(yOriginal);

            tuple2.setAndNegate(tuple1);
            assertTrue(tuple2.getX() == -xOriginal);
            assertTrue(tuple2.getY() == -yOriginal);
            assertTrue(tuple1.getX() == xOriginal);
            assertTrue(tuple1.getY() == yOriginal);

            tuple1.negate();
            assertTrue(tuple1.getX() == -xOriginal);
            assertTrue(tuple1.getY() == -yOriginal);
         }
      }
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.set(double x, double y);
         double newX = random.nextDouble();
         double newY = random.nextDouble();
         tuple1.set(newX, newY);

         assertTrue(tuple1.getX() == newX);
         assertTrue(tuple1.getY() == newY);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.set(double[] tupleArray);
         double tupleArray[] = {random.nextDouble()};
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

         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.set(double[] tupleArray, int startIndex);
         double tupleArray[] = {random.nextDouble(), random.nextDouble()};
         double[] tupleArrayCopy = new double[tupleArray.length];
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
         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);

         tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         tupleArrayCopy = new double[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.set(tupleArray, 2);

         assertTrue(tuple1.getX() == tupleArray[2]);
         assertTrue(tuple1.getY() == tupleArray[3]);

         for (int j = 0; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.set(DenseMatrix64F matrix);
         DenseMatrix64F matrix = new DenseMatrix64F(1, 1);
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

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.set(DenseMatrix64F matrix, int startRow);
         DenseMatrix64F matrix = new DenseMatrix64F(4, 1);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         try
         {
            tuple1.set(matrix, 3);
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
         tuple1.set(matrix, 5);

         assertTrue(tuple1.getX() == matrix.get(5, 0));
         assertTrue(tuple1.getY() == matrix.get(6, 0));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.set(DenseMatrix64F matrix, int startRow, int column);
         DenseMatrix64F matrix = new DenseMatrix64F(4, 6);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         try
         {
            tuple1.set(matrix, 3, 3);
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
         tuple1.set(matrix, 5, 2);

         assertTrue(tuple1.getX() == matrix.get(5, 2));
         assertTrue(tuple1.getY() == matrix.get(6, 2));

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.set(int index, double value)
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
         tuple1.set(0, x);
         tuple1.set(1, y);

         assertTrue(tuple1.getX() == x);
         assertTrue(tuple1.getY() == y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.set(TupleBasics other)
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());

         tuple1.set(tuple2);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.setX(double x)
         double x = random.nextDouble();
         tuple1.setX(x);
         assertTrue(tuple1.getX() == x);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.setY(double y)
         double y = random.nextDouble();
         tuple1.setY(y);
         assertTrue(tuple1.getY() == y);
      }
   }

   @Test
   public void testAdd() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();
      Tuple2D tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.add(double x, double y)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.add(x, y);
         assertTrue(tuple1.getX() == xOld + x);
         assertTrue(tuple1.getY() == yOld + y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.add(TupleBasics other)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.add(tuple2);
         assertTrue(tuple1.getX() == xOld + tuple2.getX());
         assertTrue(tuple1.getY() == yOld + tuple2.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.add(TupleBasics other)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());

         tuple1.add(tuple2, tuple3);
         assertTrue(tuple1.getX() == tuple2.getX() + tuple3.getX());
         assertTrue(tuple1.getY() == tuple2.getY() + tuple3.getY());
      }
   }

   @Test
   public void testSub() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();
      Tuple2D tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.sub(double x, double y)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         tuple1.set(xOld, yOld);

         tuple1.sub(x, y);
         assertTrue(tuple1.getX() == xOld - x);
         assertTrue(tuple1.getY() == yOld - y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.sub(TupleBasics other)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple1.setX(xOld);
         tuple1.setY(yOld);

         tuple1.sub(tuple2);
         assertTrue(tuple1.getX() == xOld - tuple2.getX());
         assertTrue(tuple1.getY() == yOld - tuple2.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.sub(TupleBasics tuple1, TupleBasics tuple2)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());

         tuple1.sub(tuple2, tuple3);
         assertTrue(tuple1.getX() == tuple2.getX() - tuple3.getX());
         assertTrue(tuple1.getY() == tuple2.getY() - tuple3.getY());
      }
   }

   @Test
   public void testScale() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();
      Tuple2D tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.scale(double scalarX, double scalarY)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double scale = random.nextDouble();
         tuple1.set(xOld, yOld);

         tuple1.scale(scale);
         assertTrue(tuple1.getX() == xOld * scale);
         assertTrue(tuple1.getY() == yOld * scale);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.scale(double scalarX, double scalarY)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double scaleX = random.nextDouble();
         double scaleY = random.nextDouble();
         tuple1.set(xOld, yOld);

         tuple1.scale(scaleX, scaleY);
         assertTrue(tuple1.getX() == xOld * scaleX);
         assertTrue(tuple1.getY() == yOld * scaleY);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.scale(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());

         tuple1.scale(scale, tuple2);
         assertTrue(tuple1.getX() == tuple2.getX() * scale);
         assertTrue(tuple1.getY() == tuple2.getY() * scale);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.scaleAdd(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());

         tuple1.scaleAdd(scale, tuple2);
         assertTrue(tuple1.getX() == xOld * scale + tuple2.getX());
         assertTrue(tuple1.getY() == yOld * scale + tuple2.getY());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.scaleAdd(double scalar, TupleBasics tuple1, TupleBasics tuple2)
         double scale = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());

         tuple1.scaleAdd(scale, tuple2, tuple3);
         assertTrue(tuple1.getX() == tuple2.getX() * scale + tuple3.getX());
         assertTrue(tuple1.getY() == tuple2.getY() * scale + tuple3.getY());
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();
      Tuple2D tuple3 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.interpolate(TupleBasics other, double alpha)
         double alpha = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());

         tuple1.interpolate(tuple2, alpha);
         assertTrue(tuple1.getX() == Tuple3DTools.interpolate(xOld, tuple2.getX(), alpha));
         assertTrue(tuple1.getY() == Tuple3DTools.interpolate(yOld, tuple2.getY(), alpha));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.interpolate(TupleBasics tuple1, TupleBasics tuple2, double alpha)
         double alpha = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());

         tuple1.interpolate(tuple2, tuple3, alpha);
         assertTrue(tuple1.getX() == Tuple3DTools.interpolate(tuple2.getX(), tuple3.getX(), alpha));
         assertTrue(tuple1.getY() == Tuple3DTools.interpolate(tuple2.getY(), tuple3.getY(), alpha));
      }
   }

   @Test
   public void testClip() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.clipToMax(double max)
         double max = random.nextDouble();
         tuple1.setX(max + random.nextDouble());
         tuple1.setY(max + random.nextDouble());
         tuple1.clipToMax(max);
         assertTrue(tuple1.getX() == max);
         assertTrue(tuple1.getY() == max);

         max = random.nextDouble();
         tuple1.setX(max - random.nextDouble());
         tuple1.setY(max - random.nextDouble());
         tuple1.clipToMax(max);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.clipToMax(double max, TupleBasics other)
         double max = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(max + random.nextDouble());
         tuple2.setY(max + random.nextDouble());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.getX() == max);
         assertTrue(tuple1.getY() == max);

         max = random.nextDouble();
         tuple2.setX(max - random.nextDouble());
         tuple2.setY(max - random.nextDouble());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.clipToMin(double min)
         double min = random.nextDouble();
         tuple1.setX(min - random.nextDouble());
         tuple1.setY(min - random.nextDouble());
         tuple1.clipToMin(min);
         assertTrue(tuple1.getX() == min);
         assertTrue(tuple1.getY() == min);

         min = random.nextDouble();
         tuple1.setX(min + random.nextDouble());
         tuple1.setY(min + random.nextDouble());
         tuple1.clipToMin(min);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.clipToMin(double min, TupleBasics other)
         double min = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(min - random.nextDouble());
         tuple2.setY(min - random.nextDouble());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.getX() == min);
         assertTrue(tuple1.getY() == min);

         min = random.nextDouble();
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.clipToMinMax(double min, double max)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         tuple1.setX(min - random.nextDouble());
         tuple1.setY(min - random.nextDouble());
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.getX() == min);
         assertTrue(tuple1.getY() == min);

         min = random.nextDouble() - 0.5;
         max = random.nextDouble() + 0.5;
         tuple1.setX(max + random.nextDouble());
         tuple1.setY(max + random.nextDouble());
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.getX() == max);
         assertTrue(tuple1.getY() == max);

         min = random.nextDouble() - 1.0;
         max = random.nextDouble() + 1.0;
         tuple1.setX(min + random.nextDouble());
         tuple1.setY(min + random.nextDouble());
         tuple1.clipToMinMax(min, max);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.clipToMinMax(double min, double max, TupleBasics other)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(min - random.nextDouble());
         tuple2.setY(min - random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.getX() == min);
         assertTrue(tuple1.getY() == min);

         min = random.nextDouble() - 0.5;
         max = random.nextDouble() + 0.5;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(max + random.nextDouble());
         tuple2.setY(max + random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.getX() == max);
         assertTrue(tuple1.getY() == max);

         min = random.nextDouble() - 1.0;
         max = random.nextDouble() + 1.0;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Tuple2D tuple1 = createEmptyTuple();
      tuple1.set(0.0, 0.0);
      assertFalse(tuple1.containsNaN());
      tuple1.set(Double.NaN, 0.0);
      assertTrue(tuple1.containsNaN());
      tuple1.set(0.0, Double.NaN);
      assertTrue(tuple1.containsNaN());
   }

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(double[] tupleArrayToPack)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());

         double[] tupleArray = new double[] {random.nextDouble()};

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
         for (int j = 3; j < tupleArray.length; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(double[] tupleArrayToPack, int startIndex)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());

         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble()};

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

         tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         double[] tupleArrayCopy = new double[tupleArray.length];
         System.arraycopy(tupleArray, 0, tupleArrayCopy, 0, tupleArray.length);
         tuple1.get(tupleArray, 3);
         assertTrue(tuple1.getX() == tupleArray[3]);
         assertTrue(tuple1.getY() == tupleArray[4]);
         for (int j = 0; j < 3; j++)
            assertTrue(tupleArray[j] == tupleArrayCopy[j]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(1, 5);
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

         matrixCopy.set(0, 0, tuple1.getX());
         matrixCopy.set(1, 0, tuple1.getY());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(5, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

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
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         tuple1.get(matrix, 2);
         assertTrue(tuple1.getX() == matrix.get(2, 0));
         assertTrue(tuple1.getY() == matrix.get(3, 0));

         matrixCopy.set(2, 0, tuple1.getX());
         matrixCopy.set(3, 0, tuple1.getY());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(5, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

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
            matrix.set(index, random.nextDouble());
         DenseMatrix64F matrixCopy = new DenseMatrix64F(matrix);

         tuple1.get(matrix, 2, 4);
         assertTrue(tuple1.getX() == matrix.get(2, 4));
         assertTrue(tuple1.getY() == matrix.get(3, 4));

         matrixCopy.set(2, 4, tuple1.getX());
         matrixCopy.set(3, 4, tuple1.getY());

         for (int index = 0; index < matrix.getNumElements(); index++)
            assertTrue(matrix.get(index) == matrixCopy.get(index));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         tuple1.setX(x);
         tuple1.setY(y);

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
      { // Test Tuple2D.get(TupleBasics other)
         Tuple2D tuple2 = createEmptyTuple();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());

         tuple1.get(tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.getX(), Tuple2D.getY()
         double x = random.nextDouble();
         double y = random.nextDouble();
         tuple1.setX(x);
         tuple1.setY(y);

         assertTrue(tuple1.getX() == x);
         assertTrue(tuple1.getY() == y);
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();
      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();
      GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);

      assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

      for (int index = 0; index < 2; index++)
      {
         tuple2.set(tuple1);
         tuple2.set(index, tuple1.get(index) + 0.999 * epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.set(tuple1);
         tuple2.set(index, tuple1.get(index) - 0.999 * epsilon);
         assertTrue(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.set(tuple1);
         tuple2.set(index, tuple1.get(index) + 1.001 * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));

         tuple2.set(tuple1);
         tuple2.set(index, tuple1.get(index) - 1.001 * epsilon);
         assertFalse(tuple1.epsilonEquals(tuple2, epsilon));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);
      double smallestEpsilon = 1.0e-15;

      Tuple2D tuple1 = createEmptyTuple();
      Tuple2D tuple2 = createEmptyTuple();
      GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);

      assertFalse(tuple1.equals(tuple2));
      assertFalse(tuple1.equals(null));
      assertFalse(tuple1.equals(new double[5]));

      tuple2.set(tuple1);
      assertTrue(tuple1.equals(tuple2));

      for (int index = 0; index < 2; index++)
      {
         tuple2.set(tuple1);
         assertTrue(tuple1.equals(tuple2));
         tuple2.set(index, tuple1.get(index) + smallestEpsilon);
         assertFalse(tuple1.equals(tuple2));

         tuple2.set(tuple1);
         assertTrue(tuple1.equals(tuple2));
         tuple2.set(index, tuple1.get(index) - smallestEpsilon);
         assertFalse(tuple1.equals(tuple2));
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      Tuple2D tuple1 = createEmptyTuple();
      GeometryBasicsRandomTools.randomizeTuple2D(random, tuple1);

      int newHashCode, previousHashCode;
      newHashCode = tuple1.hashCode();
      assertEquals(newHashCode, tuple1.hashCode());

      previousHashCode = tuple1.hashCode();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         tuple1.set(i % 2, random.nextDouble());
         newHashCode = tuple1.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }
}
