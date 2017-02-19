package us.ihmc.euclid.transform;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class QuaternionBasedTransformTest extends TransformTest<QuaternionBasedTransform>
{
   private static final double EPS = 1.0e-10;
   public static final int NUMBER_OF_ITERATIONS = 100;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(345L);

      { // Test empty constructor
         QuaternionBasedTransform transform = new QuaternionBasedTransform();
         EuclidCoreTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslationVector());
      }

      { // Test QuaternionBasedTransform(QuaternionBasedTransform other)
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = new QuaternionBasedTransform(expected);
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      }

      { // Test QuaternionBasedTransform(RigidBodyTransform rigidBodyTransform)
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = new QuaternionBasedTransform(new RigidBodyTransform(expected));
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      }

      { // Test QuaternionBasedTransform(DenseMatrix64F matrix)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         quaternion.get(denseMatrix);
         translation.get(4, denseMatrix);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(denseMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(double[] array)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         double[] array = new double[7];
         quaternion.get(array);
         translation.get(4, array);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(array);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(RotationMatrix rotationMatrix, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(new RotationMatrix(quaternion), translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(QuaternionReadOnly quaternion, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(quaternion, translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }

      { // Test QuaternionBasedTransform(AxisAngleReadOnly axisAngle, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         QuaternionBasedTransform transform = new QuaternionBasedTransform(new AxisAngle(quaternion), translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, transform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

      assertNotEquals(transform.getQuaternion().getX(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getY(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getS(), 1.0, EPS);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0, EPS);

      transform.setToZero();

      EuclidCoreTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

      assertFalse(Double.isNaN(transform.getQuaternion().getX()));
      assertFalse(Double.isNaN(transform.getQuaternion().getY()));
      assertFalse(Double.isNaN(transform.getQuaternion().getZ()));
      assertFalse(Double.isNaN(transform.getQuaternion().getS()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getX()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getY()));
      assertFalse(Double.isNaN(transform.getTranslationVector().getZ()));

      transform.setToNaN();

      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(transform.getTranslationVector());

      transform.setToZero();
      EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), transform.getQuaternion(), EPS);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), transform.getTranslationVector(), EPS);

      transform.setRotationToNaN();
      EuclidCoreTestTools.assertTuple4DContainsOnlyNaN(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), transform.getTranslationVector(), EPS);

      transform.setToZero();
      transform.setTranslationToNaN();
      EuclidCoreTestTools.assertTuple4DEquals(new Quaternion(), transform.getQuaternion(), EPS);
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(transform.getTranslationVector());
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
      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      Vector3D expectedTranslation = new Vector3D();
      transform.getTranslation(expectedTranslation);

      assertNotEquals(transform.getQuaternion().getX(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getY(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getS(), 1.0, EPS);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0, EPS);

      transform.setRotationToZero();

      EuclidCoreTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, transform.getTranslationVector(), EPS);
   }

   @Test
   public void testResetTranslation() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      Quaternion expectedQuaternion = new Quaternion();
      transform.getRotation(expectedQuaternion);

      assertNotEquals(transform.getQuaternion().getX(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getY(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getS(), 1.0, EPS);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0, EPS);

      transform.setTranslationToZero();

      EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, transform.getQuaternion(), EPS);
      EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSet() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();

      { // Test set(double qx, double qy, double qz, double qs, double x, double y, double z)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         double qx = quaternion.getX();
         double qy = quaternion.getY();
         double qz = quaternion.getZ();
         double qs = quaternion.getS();
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.set(qx, qy, qz, qs, x, y, z);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setUnsafe(double qx, double qy, double qz, double qs, double x, double y, double z)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         double qx = quaternion.getX();
         double qy = quaternion.getY();
         double qz = quaternion.getZ();
         double qs = quaternion.getS();
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.setUnsafe(qx, qy, qz, qs, x, y, z);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(QuaternionBasedTransform other)
         QuaternionBasedTransform expectedTransform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         actualTransform.set(expectedTransform);
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test set(RigidBodyTransform rigidBodyTransform)
         QuaternionBasedTransform expectedTransform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         actualTransform.set(new RigidBodyTransform(expectedTransform));
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expectedTransform, actualTransform, EPS);
      }

      { // Test set(DenseMatrix64F matrix)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         quaternion.get(denseMatrix);
         translation.get(4, denseMatrix);
         actualTransform.set(denseMatrix);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(double[] array)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         double[] array = new double[7];
         quaternion.get(array);
         translation.get(4, array);
         actualTransform.set(array);

         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(RotationMatrix rotationMatrix, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         actualTransform.set(new RotationMatrix(quaternion), translation);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(AxisAngleReadOnly axisAngle, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         actualTransform.set(new AxisAngle(quaternion), translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test set(QuaternionReadOnly quaternion, TupleReadOnly translation)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         actualTransform.set(quaternion, translation);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testSetIdentity() throws Exception
   {
      Random random = new Random(45L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

      assertNotEquals(transform.getQuaternion().getX(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getY(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getZ(), 0.0, EPS);
      assertNotEquals(transform.getQuaternion().getS(), 1.0, EPS);
      assertNotEquals(transform.getTranslationVector().getX(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getY(), 0.0, EPS);
      assertNotEquals(transform.getTranslationVector().getZ(), 0.0, EPS);

      transform.setIdentity();

      EuclidCoreTestTools.assertQuaternionIsSetToZero(transform.getQuaternion());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(transform.getTranslationVector());
   }

   @Test
   public void testSetRotation() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();
      Vector3D expectedTranslation = new Vector3D();
      actualTransform.getTranslation(expectedTranslation);

      { // Test setRotation(AxisAngleReadOnly axisAngle)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         actualTransform.setRotation(new AxisAngle(quaternion));
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(QuaternionReadOnly quaternion)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         actualTransform.setRotation(quaternion);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(RotationMatrix rotationMatrix)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         actualTransform.setRotation(new RotationMatrix(quaternion));
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotation(VectorReadOnly rotationVector)
         Quaternion quaternion = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D rotationVector = new Vector3D();
         quaternion.get(rotationVector);
         actualTransform.setRotation(rotationVector);
         EuclidCoreTestTools.assertQuaternionEquals(quaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setRotationYaw(double yaw)
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         rigidBodyTransform.setRotationYaw(yaw);
         QuaternionBasedTransform expectedTransform = new QuaternionBasedTransform(rigidBodyTransform);
         actualTransform.setRotationYaw(yaw);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expectedTransform, actualTransform, EPS);
      }

      { // Test setRotationPitch(double pitch)
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         double pitch = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         rigidBodyTransform.setRotationPitch(pitch);
         QuaternionBasedTransform expectedTransform = new QuaternionBasedTransform(rigidBodyTransform);
         actualTransform.setRotationPitch(pitch);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expectedTransform, actualTransform, EPS);
      }

      { // Test setRotationRoll(double roll)
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         rigidBodyTransform.setRotationRoll(roll);
         QuaternionBasedTransform expectedTransform = new QuaternionBasedTransform(rigidBodyTransform);
         actualTransform.setRotationRoll(roll);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expectedTransform, actualTransform, EPS);
      }

      { // Test setRotationYawPitchRoll(double roll)
         RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         double pitch = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         rigidBodyTransform.setRotationYawPitchRoll(yaw, pitch, roll);
         QuaternionBasedTransform expectedTransform = new QuaternionBasedTransform(rigidBodyTransform);
         actualTransform.setRotationYawPitchRoll(yaw, pitch, roll);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expectedTransform, actualTransform, EPS);

         actualTransform.setRotationToZero();
         actualTransform.setRotationYawPitchRoll(new double[] {yaw, pitch, roll});
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expectedTransform, actualTransform, EPS);

         actualTransform.setRotationToZero();
         actualTransform.setRotationEuler(new Vector3D(roll, pitch, yaw));
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expectedTransform, actualTransform, EPS);

         actualTransform.setRotationToZero();
         actualTransform.setRotationEuler(roll, pitch, yaw);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expectedTransform, actualTransform, EPS);
      }
   }

   @Test
   public void testSetTranslation() throws Exception
   {
      Random random = new Random(456456L);
      QuaternionBasedTransform actualTransform = new QuaternionBasedTransform();
      Quaternion expectedQuaternion = new Quaternion();
      actualTransform.getRotation(expectedQuaternion);

      { // Test individual setTranslation(X/Y/Z)(double)
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.setTranslationX(x);
         actualTransform.setTranslationY(y);
         actualTransform.setTranslationZ(z);
         for (int row = 0; row < 3; row++)
         {
            EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualTransform.getQuaternion(), EPS);
            EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
         }
      }

      { // Test setTranslation(double x, double y, double z)
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         double x = translation.getX();
         double y = translation.getY();
         double z = translation.getZ();
         actualTransform.setTranslation(x, y, z);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }

      { // Test setTranslation(TupleReadOnly translation)
         Vector3D translation = EuclidCoreRandomTools.generateRandomVector3D(random);
         actualTransform.setTranslation(translation);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualTransform.getQuaternion(), EPS);
         EuclidCoreTestTools.assertTuple3DEquals(translation, actualTransform.getTranslationVector(), EPS);
      }
   }

   @Test
   public void testGet() throws Exception
   {
      Random random = new Random(34543L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionReadOnly expectedQuaternion = transform.getQuaternion();
      Vector3DReadOnly expectedTranslation = transform.getTranslationVector();

      { // Test get(DenseMatrix64F matrixToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7, 1);
         transform.get(denseMatrix);
         actualQuaternion.set(denseMatrix);
         actualTranslation.set(4, denseMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(DenseMatrix64F matrixToPack, int startRow, int column)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         int startRow = random.nextInt(10);
         int column = random.nextInt(10);
         DenseMatrix64F denseMatrix = new DenseMatrix64F(7 + startRow, 1 + column);
         transform.get(startRow, column, denseMatrix);
         actualQuaternion.set(startRow, column, denseMatrix);
         actualTranslation.set(4 + startRow, column, denseMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(double[] transformArrayToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         double[] denseMatrix = new double[7];
         transform.get(denseMatrix);
         actualQuaternion.set(denseMatrix);
         actualTranslation.set(4, denseMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(QuaternionBasics quaternionToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         transform.get(actualQuaternion, actualTranslation);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(RotationMatrix rotationMarixToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         RotationMatrix rotationMatrix = new RotationMatrix();
         transform.get(rotationMatrix, actualTranslation);
         actualQuaternion.set(rotationMatrix);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }

      { // Test get(RotationScaleMatrix rotationMarixToPack, TupleBasics translationToPack)
         Quaternion actualQuaternion = new Quaternion();
         Vector3D actualTranslation = new Vector3D();
         RotationScaleMatrix rotationScaleMatrix = new RotationScaleMatrix();
         transform.get(rotationScaleMatrix, actualTranslation);
         rotationScaleMatrix.getRotation(actualQuaternion);
         EuclidCoreTestTools.assertQuaternionEquals(expectedQuaternion, actualQuaternion, EPS);
         EuclidCoreTestTools.assertTuple3DEquals(expectedTranslation, actualTranslation, EPS);
      }
   }

   @Test
   public void testGetRotation() throws Exception
   {
      Random random = new Random(3453L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionReadOnly expectedQuaternion = transform.getQuaternion();
      Quaternion actualQuaternion = new Quaternion();

      { // Test getRotation(RotationMatrix rotationMatrixToPack)
         RotationMatrix rotationMatrix = new RotationMatrix();
         transform.getRotation(rotationMatrix);
         actualQuaternion.set(rotationMatrix);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(QuaternionBasics quaternionToPack)
         actualQuaternion.setToNaN();
         transform.getRotation(actualQuaternion);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(AxisAngleBasics axisAngleToPack)
         AxisAngle axisAngle = new AxisAngle();
         transform.getRotation(axisAngle);
         actualQuaternion.set(axisAngle);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(Vector3DBasics rotationVectorToPack)
         Vector3D rotationVector = new Vector3D();
         transform.getRotation(rotationVector);
         actualQuaternion.set(rotationVector);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotationYawPitchRoll(double[] yawPitchRollToPack)
         double[] yawPitchRoll = new double[3];
         transform.getRotationYawPitchRoll(yawPitchRoll);
         actualQuaternion.setYawPitchRoll(yawPitchRoll);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }

      { // Test getRotation(Vector3DBasics rotationVectorToPack)
         Vector3D eulerAngles = new Vector3D();
         transform.getRotationEuler(eulerAngles);
         actualQuaternion.setEuler(eulerAngles);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedQuaternion, actualQuaternion, EPS);
      }
   }

   @Test
   public void testGetTranslation() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      Vector3D expected = EuclidCoreRandomTools.generateRandomVector3D(random);
      Vector3D actual = new Vector3D();

      transform.setTranslation(expected);
      transform.getTranslation(actual);

      EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      EuclidCoreTestTools.assertTuple3DEquals(transform.getTranslationVector(), actual, EPS);

      Vector3D translation = new Vector3D();
      translation.set(transform.getTranslationX(), transform.getTranslationY(), transform.getTranslationZ());
      EuclidCoreTestTools.assertTuple3DEquals(translation, transform.getTranslationVector(), EPS);
   }

   @Test
   public void testInvert() throws Exception
   {
      Random random = new Random(345L);
      QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      rigidBodyTransform.set(original);
      rigidBodyTransform.invert();
      expected.set(rigidBodyTransform);

      actual.set(original);
      actual.invert();

      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

      rigidBodyTransform.set(original);
      rigidBodyTransform.invertRotation();
      expected.set(rigidBodyTransform);

      actual.set(original);
      actual.invertRotation();

      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(23542342L);

      QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform q0 = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      QuaternionBasedTransform qf = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

      actual.interpolate(q0, qf, 0.0);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(q0, actual, EPS);
      actual.interpolate(qf, 0.0);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(q0, actual, EPS);
      actual.interpolate(qf, 1.0);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(qf, actual, EPS);

      actual.interpolate(q0, qf, 1.0);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(qf, actual, EPS);

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         double alpha = EuclidCoreRandomTools.generateRandomDouble(random, 10.0);
         Vector3D interpolatedVector = new Vector3D();
         Quaternion interpolatedRotation = new Quaternion();

         interpolatedVector.interpolate(q0.getTranslationVector(), qf.getTranslationVector(), alpha);
         interpolatedRotation.interpolate(q0.getQuaternion(), qf.getQuaternion(), alpha);

         expected.set(interpolatedRotation, interpolatedVector);
         actual.interpolate(q0, qf, alpha);
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

         actual.set(q0);
         actual.interpolate(qf, alpha);
         EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testAppendYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);
      
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendYawRotation(double yaw)
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         expectedRotation.appendYawRotation(yaw);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendYawRotation(yaw);

         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendPitchRotation(double pitch)
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double pitch = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         expectedRotation.appendPitchRotation(pitch);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendPitchRotation(pitch);

         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // appendRollRotation(double roll)
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         expectedRotation.appendRollRotation(roll);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.appendRollRotation(roll);

         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPrependYawPitchRoll() throws Exception
   {
      Random random = new Random(35454L);
      
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // prependYawRotation(double yaw)
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double yaw = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         expectedRotation.prependYawRotation(yaw);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.prependYawRotation(yaw);

         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // prependPitchRotation(double pitch)
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double pitch = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         expectedRotation.prependPitchRotation(pitch);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.prependPitchRotation(pitch);

         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // prependRollRotation(double roll)
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         RotationMatrix expectedRotation = new RotationMatrix(original.getQuaternion());
         double roll = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         expectedRotation.prependRollRotation(roll);
         expected.set(expectedRotation, original.getTranslationVector());

         actual.set(original);
         actual.prependRollRotation(roll);

         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiply() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.generateRandomAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThis() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOther() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertThisWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.generateRandomAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testMultiplyInvertOtherWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.generateRandomAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.multiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.multiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiply() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.generateRandomAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiply(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiply(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThis() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOther() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         QuaternionBasedTransform multipliedWith = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithRigidBodyTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         RigidBodyTransform multipliedWith = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertThisWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.generateRandomAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertThis(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertThis(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testPreMultiplyInvertOtherWithAffineTransform() throws Exception
   {
      Random random = new Random(2342L);

      // Test against RigidBodyTransform.multiply()
      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      {
         QuaternionBasedTransform original = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform expected = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
         QuaternionBasedTransform actual = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);

         RigidBodyTransform expectedRigidBody = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         AffineTransform multipliedWith = EuclidCoreRandomTools.generateRandomAffineTransform(random);

         expectedRigidBody.set(original);
         expectedRigidBody.preMultiplyInvertOther(multipliedWith);
         expected.set(expectedRigidBody);

         actual.set(original);
         actual.preMultiplyInvertOther(multipliedWith);
         EuclidCoreTestTools.assertQuaternionBasedTransformEqualsSmart(expected, actual, EPS);
      }
   }

   @Test
   public void testTransform() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform qTransform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform rTransform = new RigidBodyTransform(qTransform);

      RigidBodyTransform rTransform2D = new RigidBodyTransform();
      rTransform2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      rTransform2D.setTranslation(EuclidCoreRandomTools.generateRandomVector3D(random));
      QuaternionBasedTransform qTransform2D = new QuaternionBasedTransform(rTransform2D);

      { // Test transform(PointBasics pointToTransform)
         Point3D original = EuclidCoreRandomTools.generateRandomPoint3D(random);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test transform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         Point3D original = EuclidCoreRandomTools.generateRandomPoint3D(random);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test transform(VectorBasics vectorToTransform)
         Vector3D original = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test transform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         Vector3D original = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test transform(QuaternionBasics quaternionToTransform)
         Quaternion original = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Quaternion expected = new Quaternion();
         Quaternion actual = new Quaternion();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertQuaternionEquals(expected, actual, EPS);
      }

      { // Test transform(QuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
         Quaternion original = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Quaternion expected = new Quaternion();
         Quaternion actual = new Quaternion();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertQuaternionEquals(expected, actual, EPS);
      }

      { // Test transform(Vector4DBasics vectorToTransform)
         Vector4D original = EuclidCoreRandomTools.generateRandomVector4D(random);
         Vector4D expected = new Vector4D();
         Vector4D actual = new Vector4D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
         Vector4D original = EuclidCoreRandomTools.generateRandomVector4D(random);
         Vector4D expected = new Vector4D();
         Vector4D actual = new Vector4D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, EPS);
      }

      { // Test transform(Matrix3D matrixToTransform)
         Matrix3D original = EuclidCoreRandomTools.generateRandomMatrix3D(random);
         Matrix3D expected = new Matrix3D();
         Matrix3D actual = new Matrix3D();

         actual.set(original);
         qTransform.transform(actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(Matrix3DReadOnly matrixOriginal, Matrix3D matrixTransformed)
         Matrix3D original = EuclidCoreRandomTools.generateRandomMatrix3D(random);
         Matrix3D expected = new Matrix3D();
         Matrix3D actual = new Matrix3D();

         qTransform.transform(original, actual);
         rTransform.transform(original, expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(RotationMatrix matrixToTransform)
         RotationMatrix original = EuclidCoreRandomTools.generateRandomRotationMatrix(random);
         RotationMatrix expected = new RotationMatrix();
         RotationMatrix actual = new RotationMatrix();

         actual.set(original);
         qTransform.transform(actual);
         expected.set(original);
         rTransform.transform(expected);
         EuclidCoreTestTools.assertMatrix3DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DBasics pointToTransform)
         Point2D original = EuclidCoreRandomTools.generateRandomPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         actual.set(original);
         qTransform2D.transform(actual);
         rTransform2D.transform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
         actual.set(original);
         qTransform2D.transform(actual, true);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
         Point2D original = EuclidCoreRandomTools.generateRandomPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         qTransform2D.transform(original, actual);
         rTransform2D.transform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
         qTransform2D.transform(original, actual, true);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DBasics vectorToTransform)
         Vector2D original = EuclidCoreRandomTools.generateRandomVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         actual.set(original);
         qTransform2D.transform(actual);
         rTransform2D.transform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D original = EuclidCoreRandomTools.generateRandomVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         qTransform2D.transform(original, actual);
         rTransform2D.transform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testInverseTransform() throws Exception
   {
      Random random = new Random(34L);
      QuaternionBasedTransform qTransform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform rTransform = new RigidBodyTransform(qTransform);

      RigidBodyTransform rTransform2D = new RigidBodyTransform();
      rTransform2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      rTransform2D.setTranslation(EuclidCoreRandomTools.generateRandomVector3D(random));
      QuaternionBasedTransform qTransform2D = new QuaternionBasedTransform(rTransform2D);

      { // Test inverseTransform(PointBasics pointToTransform)
         Point3D original = EuclidCoreRandomTools.generateRandomPoint3D(random);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         actual.set(original);
         qTransform.inverseTransform(actual);
         rTransform.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(PointReadOnly pointOriginal, PointBasics pointTransformed)
         Point3D original = EuclidCoreRandomTools.generateRandomPoint3D(random);
         Point3D expected = new Point3D();
         Point3D actual = new Point3D();

         qTransform.inverseTransform(original, actual);
         rTransform.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(VectorBasics vectorToTransform)
         Vector3D original = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         actual.set(original);
         qTransform.inverseTransform(actual);
         rTransform.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(VectorReadOnly vectorOriginal, VectorBasics vectorTransformed)
         Vector3D original = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector3D expected = new Vector3D();
         Vector3D actual = new Vector3D();

         qTransform.inverseTransform(original, actual);
         rTransform.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple3DEquals(expected, actual, EPS);
      }

      { // Test inverseTransform(Point2DBasics pointToTransform)
         Point2D original = EuclidCoreRandomTools.generateRandomPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         actual.set(original);
         qTransform2D.inverseTransform(actual);
         rTransform2D.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Point2DReadOnly pointOriginal, Point2DBasics pointTransformed)
         Point2D original = EuclidCoreRandomTools.generateRandomPoint2D(random);
         Point2D expected = new Point2D();
         Point2D actual = new Point2D();

         qTransform2D.inverseTransform(original, actual);
         rTransform2D.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DBasics vectorToTransform)
         Vector2D original = EuclidCoreRandomTools.generateRandomVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         actual.set(original);
         qTransform2D.inverseTransform(actual);
         rTransform2D.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }

      { // Test transform(Vector2DReadOnly vectorOriginal, Vector2DBasics vectorTransformed)
         Vector2D original = EuclidCoreRandomTools.generateRandomVector2D(random);
         Vector2D expected = new Vector2D();
         Vector2D actual = new Vector2D();

         qTransform2D.inverseTransform(original, actual);
         rTransform2D.inverseTransform(original, expected);
         EuclidCoreTestTools.assertTuple2DEquals(expected, actual, EPS);
      }
   }

   @Test
   public void testTransformWithOtherRigidBodyTransform() throws Exception
   {
      Random random = new Random(23423L);

      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform original = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform expected = new RigidBodyTransform();
      RigidBodyTransform actual = new RigidBodyTransform();

      expected.set(transform);
      expected.multiply(original);

      transform.transform(original, actual);

      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(original, actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      
      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithQuaternionBasedTransform() throws Exception
   {
      Random random = new Random(23423L);

      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform originalRigidBodyTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform expectedRigidBodyTransform = new RigidBodyTransform();

      QuaternionBasedTransform original = new QuaternionBasedTransform(originalRigidBodyTransform);
      QuaternionBasedTransform expected = new QuaternionBasedTransform();
      QuaternionBasedTransform actual = new QuaternionBasedTransform();

      transform.transform(originalRigidBodyTransform, expectedRigidBodyTransform);
      expected.set(expectedRigidBodyTransform);
      transform.transform(original, actual);

      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(original, actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertQuaternionBasedTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testTransformWithAffineTransform() throws Exception
   {
      Random random = new Random(23423L);

      QuaternionBasedTransform transform = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
      RigidBodyTransform originalRigidBodyTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      RigidBodyTransform expectedRigidBodyTransform = new RigidBodyTransform();

      Vector3D scale = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);

      AffineTransform original = new AffineTransform(originalRigidBodyTransform);
      original.setScale(scale);
      AffineTransform expected = new AffineTransform();
      AffineTransform actual = new AffineTransform();

      transform.transform(originalRigidBodyTransform, expectedRigidBodyTransform);
      expected.set(expectedRigidBodyTransform);
      expected.setScale(scale);
      transform.transform(original, actual);

      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);

      actual.set(original);
      transform.transform(original, actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);

      RigidBodyTransform inverse = new RigidBodyTransform(transform);
      inverse.invert();

      inverse.transform(original, expected);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
      actual.set(original);
      transform.inverseTransform(original, actual);
      EuclidCoreTestTools.assertAffineTransformEquals(expected, actual, EPS);
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(2354L);
      QuaternionBasedTransform t1 = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
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
      QuaternionBasedTransform m1 = EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
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

   @Override
   public QuaternionBasedTransform createRandomTransform(Random random)
   {
      return EuclidCoreRandomTools.generateRandomQuaternionBasedTransform(random);
   }

   @Override
   public QuaternionBasedTransform createRandomTransform2D(Random random)
   {
      RigidBodyTransform rTransform2D = new RigidBodyTransform();
      rTransform2D.setRotationYaw(2.0 * Math.PI * random.nextDouble() - Math.PI);
      rTransform2D.setTranslation(EuclidCoreRandomTools.generateRandomVector3D(random));
      QuaternionBasedTransform qTransform2D = new QuaternionBasedTransform(rTransform2D);
      return qTransform2D;
   }
}
