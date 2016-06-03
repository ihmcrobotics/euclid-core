package us.ihmc.geometry.transform;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.geometry.axisAngle.AxisAngle;
import us.ihmc.geometry.matrix.Matrix3D;
import us.ihmc.geometry.matrix.RotationMatrix;
import us.ihmc.geometry.matrix.RotationScaleMatrix;
import us.ihmc.geometry.testingTools.GeometryBasicsRandomTools;
import us.ihmc.geometry.testingTools.GeometryBasicsTestTools;
import us.ihmc.geometry.tuple.Point;
import us.ihmc.geometry.tuple.Vector;
import us.ihmc.geometry.tuple.interfaces.VectorReadOnly;
import us.ihmc.geometry.tuple2D.Point2D;
import us.ihmc.geometry.tuple2D.Vector2D;
import us.ihmc.geometry.tuple4D.Quaternion;
import us.ihmc.geometry.tuple4D.Vector4D;
import us.ihmc.geometry.tuple4D.interfaces.QuaternionReadOnly;

public class QuaternionBasedTransformTest
{
   private static final double EPS = 1.0e-10;
   public static final int NUMBER_OF_ITERATIONS = 100;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(345L);

      { // Test empty constructor
         QuaternionBasedTransform transform = new QuaternionBasedTransform();
         GeometryBasicsTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
         GeometryBasicsTestTools.assertTupleIsSetToZero(transform.getTranslationVector());
      }

      { // Test QuaternionBasedTransform(QuaternionBasedTransform other)
         QuaternionBasedTransform expected = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = new QuaternionBasedTransform(expected);
         GeometryBasicsTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      }

      { // Test QuaternionBasedTransform(RigidBodyTransform rigidBodyTransform)
         QuaternionBasedTransform expected = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = new QuaternionBasedTransform(new RigidBodyTransform(expected));
         GeometryBasicsTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      }

      { // Test QuaternionBasedTransform(DenseMatrix64F matrix)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         quaternion.get(denseMatrix);
         translation.get(denseMatrix, 4);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(denseMatrix);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(double[] array)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         double[] array = new double[7];
         quaternion.get(array);
         translation.get(array, 4);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(array);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(RotationMatrix rotationMatrix, TupleReadOnly translation)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(new RotationMatrix(quaternion), translation);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(QuaternionReadOnly quaternion, TupleReadOnly translation)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(quaternion, translation);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(AxisAngleReadOnly axisAngle, TupleReadOnly translation)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(new AxisAngle(quaternion), translation);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, transform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);

      assertNotEquals(transform.getQuaternion().getX(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getY(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getS(), 1.0, EPS);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0, EPS);

      transform.setToZero();

      GeometryBasicsTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      GeometryBasicsTestTools.assertTupleIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);

      assertFalse(Double.isNaN(transform.getQuaternion().getX()));
      assertFalse(Double.isNaN(transform.getQuaternion().getY()));
      assertFalse(Double.isNaN(transform.getQuaternion().getZ()));
      assertFalse(Double.isNaN(transform.getQuaternion().getS()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getX()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getY()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getZ()));

      transform.setToNaN();

      GeometryBasicsTestTools.assertQuaternionContainsOnlyNaN(transform.getQuaternion());
      GeometryBasicsTestTools.assertTupleContainsOnlyNaN(transform.getTranslationVector());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      QuaternionBasedTransform transform = new QuaternionBasedTransform();

      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertFalse(transform.containsNaN());
      transform.setUnsafe(Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN, 0.0);
      assertTrue(transform.containsNaN());
      transform.setUnsafe(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.NaN);
      assertTrue(transform.containsNaN());
   }

   @Test
   public void testResetRotation() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      Vector expectedTranslation = new Vector();
      transform.getTranslation(expectedTranslation);

      assertNotEquals(transform.getQuaternion().getX(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getY(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getS(), 1.0, EPS);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0, EPS);

      transform.resetRotation();

      GeometryBasicsTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, transform.getTranslationVector(), EPS);
   }

   @Test
   public void testResetTranslation() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      Quaternion expectedQuaternion = new Quaternion();
      transform.getRotation(expectedQuaternion);

      assertNotEquals(transform.getQuaternion().getX(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getY(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getS(), 1.0, EPS);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0, EPS);

      transform.resetTranslation();

      GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, transform.getQuaternion(), EPS);
      GeometryBasicsTestTools.assertTupleIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();

      { // Test set(double qx, double qy, double qz, double qs, double x, double y, double z)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         double qx = quaternion.getX();
         double qy = quaternion.getY();
         double qz = quaternion.getZ();
         double qs = quaternion.getS();
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.set(qx, qy, qz, qs, x, y, z);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setUnsafe(double qx, double qy, double qz, double qs, double x, double y, double z)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         double qx = quaternion.getX();
         double qy = quaternion.getY();
         double qz = quaternion.getZ();
         double qs = quaternion.getS();
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.setUnsafe(qx, qy, qz, qs, x, y, z);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(QuaternionBasedTransform other)
         QuaternionBasedTransform expectedTransform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
         actualTransform.set(expectedTransform);
         GeometryBasicsTestTools.assertQuaternionBasedTransformEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test set(RigidBodyTransform rigidBodyTransform)
         QuaternionBasedTransform expectedTransform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
         actualTransform.set(new RigidBodyTransform(expectedTransform));
         GeometryBasicsTestTools.assertQuaternionBasedTransformEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test set(DenseMatrix64F matrix)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         quaternion.get(denseMatrix);
         translation.get(denseMatrix, 4);
         actualTransform.set(denseMatrix);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(double[] array)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         double[] array = new double[7];
         quaternion.get(array);
         translation.get(array, 4);
         actualTransform.set(array);

         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(RotationMatrix rotationMatrix, TupleReadOnly translation)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         actualTransform.set(new RotationMatrix(quaternion), translation);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(AxisAngleReadOnly axisAngle, TupleReadOnly translation)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         actualTransform.set(new AxisAngle(quaternion), translation);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(QuaternionReadOnly quaternion, TupleReadOnly translation)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         actualTransform.set(quaternion, translation);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testSetIdentity() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);

      assertNotEquals(transform.getQuaternion().getX(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getY(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getS(), 1.0, EPS);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0, EPS);

      transform.setIdentity();

      GeometryBasicsTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      GeometryBasicsTestTools.assertTupleIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSetRotation() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();
      Vector expectedTranslation = new Vector();
      actualTransform.getTranslation(expectedTranslation);

      { // Test setRotation(AxisAngleReadOnly axisAngle)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         actualTransform.setRotation(new AxisAngle(quaternion));
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(QuaternionReadOnly quaternion)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         actualTransform.setRotation(quaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(RotationMatrix rotationMatrix)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         actualTransform.setRotation(new RotationMatrix(quaternion));
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(VectorReadOnly rotationVector)
         Quaternion quaternion = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Vector rotationVector = new Vector();
         quaternion.get(rotationVector);
         actualTransform.setRotation(rotationVector);
         GeometryBasicsTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testSetTranslation() throws Exception
   {
      Random random = new Random(456456L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();
      Quaternion expectedQuaternion = new Quaternion();
      actualTransform.getRotation(expectedQuaternion);

      { // Test setTranslation(double x, double y, double z)
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.setTranslation(x, y, z);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setTranslation(TupleReadOnly translation)
         Vector translation = GeometryBasicsRandomTools.generateRandomVector(random);
         actualTransform.setTranslation(translation);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualTransform.getQuaternion(), EPS);
         GeometryBasicsTestTools.assertTupleEquals(translation, actualTransform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testGet() throws Exception
   {
      Random random = new Random(34543L);
      QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionReadOnly expectedQuaternion = transform.getQuaternion();
      VectorReadOnly expectedTranslation = transform.getTranslationVector();

      { // Test get(DenseMatrix64F matrixToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector actualTranslation = new Vector();
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         transform.get(denseMatrix);
         actualQuaternion.set(denseMatrix);
         actualTranslation.set(denseMatrix, 4);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(DenseMatrix64F matrixToPack, int startRow, int column)
         Quaternion actualQuaternion = new Quaternion();
         Vector actualTranslation = new Vector();
         int startRow = random.nextInt(10);
         int column = random.nextInt(10);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7 + startRow, 1 + column);
         transform.get(denseMatrix, startRow, column);
         actualQuaternion.set(denseMatrix, startRow, column);
         actualTranslation.set(denseMatrix, 4 + startRow, column);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(double[] transformArrayToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector actualTranslation = new Vector();
         double[] denseMatrix = new double[7];
         transform.get(denseMatrix);
         actualQuaternion.set(denseMatrix);
         actualTranslation.set(denseMatrix, 4);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(QuaternionBasics quaternionToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector actualTranslation = new Vector();
         transform.get(actualQuaternion, actualTranslation);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(RotationMatrix rotationMarixToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector actualTranslation = new Vector();
         RotationMatrix rotationMatrix = new RotationMatrix();
         transform.get(rotationMatrix, actualTranslation);
         rotationMatrix.get(actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(RotationScaleMatrix rotationMarixToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector actualTranslation = new Vector();
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
         transform.get(rotationScaleMatrix, actualTranslation);
         rotationScaleMatrix.getRotation(actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         GeometryBasicsTestTools.assertTupleEquals(expectedTranslation, actualTranslation, EPS);
      }
   }

   @Test
   public void testGetRotation() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionReadOnly expectedQuaternion = transform.getQuaternion();
      Quaternion actualQuaternion = new Quaternion();

      { // Test getRotation(RotationMatrix rotationMatrixToPack)
         RotationMatrix rotationMatrix = new RotationMatrix();
         transform.getRotation(rotationMatrix);
         actualQuaternion.set(rotationMatrix);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(QuaternionBasics quaternionToPack)
         actualQuaternion.setToNaN();
         transform.getRotation(actualQuaternion);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(AxisAngleBasics axisAngleToPack)
         AxisAngle axisAngle = new AxisAngle();
         transform.getRotation(axisAngle);
         actualQuaternion.set(axisAngle);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(AxisAngleBasics axisAngleToPack)
         Vector rotationVector = new Vector();
         transform.getRotation(rotationVector);
         actualQuaternion.set(rotationVector);
         GeometryBasicsTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }
   }

   @Test
   public void testGetTranslation() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform transform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      Vector expected = GeometryBasicsRandomTools.generateRandomVector(random);
      Vector actual = new Vector();

      transform.setTranslation(expected);
      transform.getTranslation(actual);

      GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      GeometryBasicsTestTools.assertTupleEquals(transform.getTranslationVector(), actual, EPS);
   }

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(345L);
      QuaternionBasedTransform original = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      rigidBodyTransform.set(original);
      rigidBodyTransform.invert();
      expected.set(rigidBodyTransform);

      actual.set(original);
      actual.invert();

      GeometryBasicsTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(435L);
      QuaternionBasedTransform q1 = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform q2 = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform q3 = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);

      RigidBodyTransform r1 = new RigidBodyTransform(q1);
      RigidBodyTransform r2 = new RigidBodyTransform(q2);
      RigidBodyTransform r3 = new RigidBodyTransform(q3);

      QuaternionBasedTransform actual = new QuaternionBasedTransform();
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      
      { // Test multiply(QuaternionBasedTransform other)
         q3.set(q1);
         q3.multiply(q2);
         actual.set(q3);

         r3.set(r1);
         r3.multiply(r2);
         expected.set(r3);
         
         GeometryBasicsTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
      
      { // Test multiply(RigidBodyTransform rigidBodyTransform)
         q3.set(q1);
         q3.multiply(r2);
         actual.set(q3);

         r3.set(r1);
         r3.multiply(r2);
         expected.set(r3);
         
         GeometryBasicsTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(4359125L);
      QuaternionBasedTransform q1 = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform q2 = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform q3 = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);

      RigidBodyTransform r1 = new RigidBodyTransform(q1);
      RigidBodyTransform r2 = new RigidBodyTransform(q2);
      RigidBodyTransform r3 = new RigidBodyTransform(q3);

      QuaternionBasedTransform actual = new QuaternionBasedTransform();
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      
      { // Test preMultiply(QuaternionBasedTransform other)
         q3.set(q1);
         q3.preMultiply(q2);
         actual.set(q3);

         r3.set(r1);
         r3.preMultiply(r2);
         expected.set(r3);
         
         GeometryBasicsTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
      
      { // Test preMultiply(RigidBodyTransform rigidBodyTransform)
         q3.set(q1);
         q3.preMultiply(r2);
         actual.set(q3);

         r3.set(r1);
         r3.preMultiply(r2);
         expected.set(r3);
         
         GeometryBasicsTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testTransform() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform qTransform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform rTransform = new RigidBodyTransform(qTransform);

      RigidBodyTransform rTransform2D = new RigidBodyTransform();
      rTransform2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      rTransform2D.setTranslation(GeometryBasicsRandomTools.generateRandomVector(random));
      QuaternionBasedTransform qTransform2D = new QuaternionBasedTransform(rTransform2D);

      { // Test transform(PointBasics pointToTransform)
         Point original = GeometryBasicsRandomTools.generateRandomPoint(random);
         Point expected = new Point();
         Point actual = new Point();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }

      { // Test transform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         Point original = GeometryBasicsRandomTools.generateRandomPoint(random);
         Point expected = new Point();
         Point actual = new Point();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }

      { // Test transform(VectorBasics vectorToTransform)
         Vector original = GeometryBasicsRandomTools.generateRandomVector(random);
         Vector expected = new Vector();
         Vector actual = new Vector();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }

      { // Test transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         Vector original = GeometryBasicsRandomTools.generateRandomVector(random);
         Vector expected = new Vector();
         Vector actual = new Vector();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }

      { // Test transform(QuaternionBasics quaternionToTransform)
         Quaternion original = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Quaternion expected = new Quaternion();
         Quaternion actual = new Quaternion();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, EPS);
      }

      { // Test transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
         Quaternion original = GeometryBasicsRandomTools.generateRandomQuaternion(random);
         Quaternion expected = new Quaternion();
         Quaternion actual = new Quaternion();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertQuaternionEquals(expected, actual, EPS);
      }

      { // Test transform(Vector4DBasics vectorToTransform)
         Vector4D original = GeometryBasicsRandomTools.generateRandomVector4D(random);
         Vector4D expected = new Vector4D();
         Vector4D actual = new Vector4D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4D original = GeometryBasicsRandomTools.generateRandomVector4D(random);
         Vector4D expected = new Vector4D();
         Vector4D actual = new Vector4D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      { // Test transform(Matrix3D matrixToTransform)
         Matrix3D original = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         Matrix3D expected = new Matrix3D();
         Matrix3D actual = new Matrix3D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3D original = GeometryBasicsRandomTools.generateRandomMatrix3D(random);
         Matrix3D expected = new Matrix3D();
         Matrix3D actual = new Matrix3D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(RotationMatrix matrixToTransform)
         RotationMatrix original = GeometryBasicsRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         actual.set(original);
         qTransform.transform(actual);
         expected.set(original);
         rTransform.transform(expected);
         GeometryBasicsTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DBasics pointToTransform)
         Point2D original = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         actual.set(original);
         qTransform2D.transform(actual);
         rTransform2D.transform(original, expected);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
         actual.set(original);
         qTransform2D.transform(actual, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
         Point2D original = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         qTransform2D.transform(original, actual);
         rTransform2D.transform(original, expected);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
         qTransform2D.transform(original, actual, true);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DBasics vectorToTransform)
         Vector2D original = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         actual.set(original);
         qTransform2D.transform(actual);
         rTransform2D.transform(original, expected);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D original = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         qTransform2D.transform(original, actual);
         rTransform2D.transform(original, expected);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testInverseTransform() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform qTransform = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform rTransform = new RigidBodyTransform(qTransform);

      RigidBodyTransform rTransform2D = new RigidBodyTransform();
      rTransform2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      rTransform2D.setTranslation(GeometryBasicsRandomTools.generateRandomVector(random));
      QuaternionBasedTransform qTransform2D = new QuaternionBasedTransform(rTransform2D);

      { // Test inverseTransform(PointBasics pointToTransform)
         Point original = GeometryBasicsRandomTools.generateRandomPoint(random);
         Point expected = new Point();
         Point actual = new Point();

         actual.set(original);
         qTransform.inverseTransform(actual);
         rTransform.inverseTransform(original, expected);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         Point original = GeometryBasicsRandomTools.generateRandomPoint(random);
         Point expected = new Point();
         Point actual = new Point();

         qTransform.inverseTransform(original, actual);
         rTransform.inverseTransform(original, expected);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(VectorBasics vectorToTransform)
         Vector original = GeometryBasicsRandomTools.generateRandomVector(random);
         Vector expected = new Vector();
         Vector actual = new Vector();

         actual.set(original);
         qTransform.inverseTransform(actual);
         rTransform.inverseTransform(original, expected);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         Vector original = GeometryBasicsRandomTools.generateRandomVector(random);
         Vector expected = new Vector();
         Vector actual = new Vector();

         qTransform.inverseTransform(original, actual);
         rTransform.inverseTransform(original, expected);
         GeometryBasicsTestTools.assertTupleEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(Point2DBasics pointToTransform)
         Point2D original = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         actual.set(original);
         qTransform2D.inverseTransform(actual);
         rTransform2D.inverseTransform(original, expected);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
         Point2D original = GeometryBasicsRandomTools.generateRandomPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         qTransform2D.inverseTransform(original, actual);
         rTransform2D.inverseTransform(original, expected);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DBasics vectorToTransform)
         Vector2D original = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         actual.set(original);
         qTransform2D.inverseTransform(actual);
         rTransform2D.inverseTransform(original, expected);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D original = GeometryBasicsRandomTools.generateRandomVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         qTransform2D.inverseTransform(original, actual);
         rTransform2D.inverseTransform(original, expected);
         GeometryBasicsTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      QuaternionBasedTransform t1 = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform t2 = new QuaternionBasedTransform();

      assertFalse(t1.equals(t2));
      assertFalse(t1.equals(null));
      assertFalse(t1.equals(new double[4]));
      t2.set(t1);
      assertTrue(t1.equals(t2));
      assertTrue(t1.equals((Object) t2));

      double smallestEpsilon = 1.0e-16;
      double[] coeffs = new double[7];

      for (int index = 0; index < 3; index++)
      {
         t2.set(t1);
         assertTrue(t1.equals(t2));
         t1.get(coeffs);
         coeffs[index] += smallestEpsilon;
         t2.set(coeffs);
         assertFalse(t1.equals(t2));

         t2.set(t1);
         assertTrue(t1.equals(t2));
         t1.get(coeffs);
         coeffs[index] -= smallestEpsilon;
         t2.set(coeffs);
         assertFalse(t1.equals(t2));
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(2354L);
      QuaternionBasedTransform m1 = GeometryBasicsRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform m2 = new QuaternionBasedTransform();
      double epsilon = 1.0e-3;
      double[] coeffs = new double[9];

      assertFalse(m1.epsilonEquals(m2, epsilon));
      m2.set(m1);
      assertTrue(m1.epsilonEquals(m2, epsilon));

      for (int index = 0; index < 3; index++)
      {
         m2.set(m1);
         assertTrue(m1.epsilonEquals(m2, epsilon));
         m1.get(coeffs);
         coeffs[index] += 0.999 * epsilon;
         m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
         assertTrue(m1.epsilonEquals(m2, epsilon));

         m2.set(m1);
         assertTrue(m1.epsilonEquals(m2, epsilon));
         m1.get(coeffs);
         coeffs[index] += 1.001 * epsilon;
         m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
         assertFalse(m1.epsilonEquals(m2, epsilon));

         m2.set(m1);
         assertTrue(m1.epsilonEquals(m2, epsilon));
         m1.get(coeffs);
         coeffs[index] -= 0.999 * epsilon;
         m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
         assertTrue(m1.epsilonEquals(m2, epsilon));

         m2.set(m1);
         assertTrue(m1.epsilonEquals(m2, epsilon));
         m1.get(coeffs);
         coeffs[index] -= 1.001 * epsilon;
         m2.setUnsafe(coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5], coeffs[6]);
         assertFalse(m1.epsilonEquals(m2, epsilon));
      }
   }
}
