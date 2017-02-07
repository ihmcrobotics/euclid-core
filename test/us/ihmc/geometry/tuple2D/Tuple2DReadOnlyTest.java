package us.ihmc.geometry.tuple2D;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;

public abstract class Tuple2DReadOnlyTest<T extends Tuple2DReadOnly<T>>
{
   public static final int NUMBER_OF_ITERATIONS = 100;

   public abstract T createEmptyTuple();

   public abstract T createTuple(double x, double y);

   public abstract T createRandomTuple(Random random);

   public abstract double getEpsilon();

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      T tuple = createEmptyTuple();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.getX(), Tuple2D.getY()
         double x = random.nextDouble();
         double y = random.nextDouble();
         tuple = createTuple(x, y);

         assertEquals(tuple.get(0), x, getEpsilon());
         assertEquals(tuple.get(1), y, getEpsilon());
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.getX32(), Tuple2D.getY32()
         float x = random.nextFloat();
         float y = random.nextFloat();
         tuple = createTuple(x, y);

         assertTrue(tuple.getX32() == x);
         assertTrue(tuple.getY32() == y);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(int index)
         double x = random.nextDouble();
         double y = random.nextDouble();
         tuple = createTuple(x, y);

         assertEquals(tuple.get(0), x, getEpsilon());
         assertEquals(tuple.get(1), y, getEpsilon());

         try
         {
            tuple.get(-1);
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
            tuple.get(3);
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
      { // Test Tuple2D.get32(int index)
         float x = random.nextFloat();
         float y = random.nextFloat();
         tuple = createTuple(x, y);

         assertTrue(tuple.get32(0) == x);
         assertTrue(tuple.get32(1) == y);

         try
         {
            tuple.get32(-1);
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
            tuple.get32(3);
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
      { // Test Tuple2D.get(double[] tupleArrayToPack)
         tuple = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         tuple.get(tupleArray);
         assertTrue(tuple.getX() == tupleArray[0]);
         assertTrue(tuple.getY() == tupleArray[1]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(double[] tupleArrayToPack, int startIndex)
         tuple = createRandomTuple(random);
         double[] tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         tuple.get(2, tupleArray);
         assertTrue(tuple.getX() == tupleArray[2]);
         assertTrue(tuple.getY() == tupleArray[3]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(float[] tupleArrayToPack)
         tuple = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tuple.get(tupleArray);
         assertTrue(tuple.getX32() == tupleArray[0]);
         assertTrue(tuple.getY32() == tupleArray[1]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(double[] tupleArrayToPack, int startIndex)
         tuple = createRandomTuple(random);
         float[] tupleArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tuple.get(2, tupleArray);
         assertTrue(tuple.getX32() == tupleArray[2]);
         assertTrue(tuple.getY32() == tupleArray[3]);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(matrix);
         assertTrue(tuple.getX() == matrix.get(0, 0));
         assertTrue(tuple.getY() == matrix.get(1, 0));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(int startRow, DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 0));
         assertTrue(tuple.getY() == matrix.get(3, 0));
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Test Tuple2D.get(int startRow, int startColumn, DenseMatrix64F tupleMatrixToPack)
         DenseMatrix64F matrix = new DenseMatrix64F(10, 5);
         for (int index = 0; index < matrix.getNumElements(); index++)
            matrix.set(index, random.nextDouble());

         tuple.get(2, 4, matrix);
         assertTrue(tuple.getX() == matrix.get(2, 4));
         assertTrue(tuple.getY() == matrix.get(3, 4));
      }
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      assertFalse(createTuple(0.0, 0.0).containsNaN());
      assertTrue(createTuple(Double.NaN, 0.0).containsNaN());
      assertTrue(createTuple(0.0, Double.NaN).containsNaN());
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();

      T tuple = createRandomTuple(random);
      double x = tuple.getX();
      double y = tuple.getY();

      assertTrue(tuple.epsilonEquals(createTuple(x + 0.999 * epsilon, y), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x - 0.999 * epsilon, y), epsilon));
      
      assertTrue(tuple.epsilonEquals(createTuple(x, y + 0.999 * epsilon), epsilon));
      assertTrue(tuple.epsilonEquals(createTuple(x, y - 0.999 * epsilon), epsilon));
      
      assertFalse(tuple.epsilonEquals(createTuple(x + 1.001 * epsilon, y), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x - 1.001 * epsilon, y), epsilon));
      
      assertFalse(tuple.epsilonEquals(createTuple(x, y + 1.001 * epsilon), epsilon));
      assertFalse(tuple.epsilonEquals(createTuple(x, y - 1.001 * epsilon), epsilon));
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(621541L);

      T axisAngle = createRandomTuple(random);

      assertFalse(axisAngle.equals(createEmptyTuple()));
      assertFalse(axisAngle.equals(null));
      assertFalse(axisAngle.equals(new double[5]));

      double x = axisAngle.getX();
      double y = axisAngle.getY();

      assertTrue(axisAngle.equals(createTuple(x, y)));

      assertFalse(axisAngle.equals(createTuple(x + getEpsilon(), y)));
      assertFalse(axisAngle.equals(createTuple(x - getEpsilon(), y)));

      assertFalse(axisAngle.equals(createTuple(x, y + getEpsilon())));
      assertFalse(axisAngle.equals(createTuple(x, y - getEpsilon())));
   }
}