package us.ihmc.geometry.testSuite;

import java.awt.Desktop;
import java.io.File;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import org.pitest.mutationtest.commandline.MutationCoverageReport;

import us.ihmc.geometry.CyclingConversionTest;
import us.ihmc.geometry.GeometryBasicsToolsTest;
import us.ihmc.geometry.TupleToolsTest;
import us.ihmc.geometry.axisAngle.AxisAngle32Test;
import us.ihmc.geometry.axisAngle.AxisAngleConversionTest;
import us.ihmc.geometry.axisAngle.AxisAngleTest;
import us.ihmc.geometry.matrix.Matrix3DFeaturesTest;
import us.ihmc.geometry.matrix.Matrix3DReadOnlyToolsTest;
import us.ihmc.geometry.matrix.Matrix3DTest;
import us.ihmc.geometry.matrix.Matrix3DToolsTest;
import us.ihmc.geometry.matrix.RotationMatrixConversionTest;
import us.ihmc.geometry.matrix.RotationMatrixTest;
import us.ihmc.geometry.matrix.RotationScaleMatrixTest;
import us.ihmc.geometry.matrix.RotationScaleMatrixToolsTest;
import us.ihmc.geometry.transform.AffineTransformTest;
import us.ihmc.geometry.transform.QuaternionBasedTransformTest;
import us.ihmc.geometry.transform.RigidBodyTransformTest;
import us.ihmc.geometry.tuple2D.Point2D32Test;
import us.ihmc.geometry.tuple2D.Point2DTest;
import us.ihmc.geometry.tuple2D.Vector2D32Test;
import us.ihmc.geometry.tuple2D.Vector2DTest;
import us.ihmc.geometry.tuple3D.Point3D32Test;
import us.ihmc.geometry.tuple3D.Point3DTest;
import us.ihmc.geometry.tuple3D.RotationVectorConversionTest;
import us.ihmc.geometry.tuple3D.Vector3D32Test;
import us.ihmc.geometry.tuple3D.Vector3DTest;
import us.ihmc.geometry.tuple4D.Quaternion32Test;
import us.ihmc.geometry.tuple4D.QuaternionConversionTest;
import us.ihmc.geometry.tuple4D.QuaternionTest;
import us.ihmc.geometry.tuple4D.QuaternionToolsTest;
import us.ihmc.geometry.tuple4D.Vector4DTest;
import us.ihmc.geometry.yawPitchRoll.YawPitchRollConversionTest;

@RunWith(Suite.class)
@Suite.SuiteClasses({Point3DTest.class, Point3D32Test.class, Point2DTest.class, Point2D32Test.class, Vector3DTest.class, Vector3D32Test.class,
      Vector2DTest.class, Vector2D32Test.class, Vector4DTest.class, AxisAngleTest.class, AxisAngle32Test.class, QuaternionTest.class, Quaternion32Test.class,
      Matrix3DTest.class, RotationMatrixTest.class, RotationScaleMatrixTest.class, RigidBodyTransformTest.class, QuaternionBasedTransformTest.class,
      AffineTransformTest.class,

      // Tools tests
      GeometryBasicsToolsTest.class, QuaternionToolsTest.class, Matrix3DToolsTest.class, Matrix3DFeaturesTest.class, Matrix3DReadOnlyToolsTest.class,
      RotationScaleMatrixToolsTest.class, TupleToolsTest.class,

      // Conversion tests
      AxisAngleConversionTest.class, RotationVectorConversionTest.class, QuaternionConversionTest.class, RotationMatrixConversionTest.class,
      YawPitchRollConversionTest.class, CyclingConversionTest.class})

public class GeometryTestSuite
{
   public static void main(String[] args) throws URISyntaxException, IOException
   {
      String targetTests = "us.ihmc.geometry.testSuite.GeometryTestSuite";
      String targetClasses = "us.ihmc.geometry.*";
      doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }

   public static void doPITMutationTestAndOpenResult(String targetTests, String targetClasses)
   {
      String reportDirectoryName = "pit-reports";
      MutationCoverageReport.main(new String[] {"--reportDir", reportDirectoryName, "--targetClasses", targetClasses, "--targetTests", targetTests,
            "--sourceDirs", "src,test"});

      File reportDirectory = new File(reportDirectoryName);
      if (reportDirectory.isDirectory() && reportDirectory.exists())
      {
         String[] list = reportDirectory.list();
         String lastDirectoryName = list[list.length - 1];

         System.out.println("Found last directory " + lastDirectoryName);

         File reportFile = new File(reportDirectory, lastDirectoryName + "/index.html");
         String absolutePath;
         try
         {
            absolutePath = reportFile.getCanonicalPath();

            absolutePath = absolutePath.replace("\\", "/");
            System.out.println("Opening " + "file://" + absolutePath);

            URI uri = new URI("file://" + absolutePath);
            Desktop.getDesktop().browse(uri);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         catch (URISyntaxException e)
         {
            e.printStackTrace();
         }
      }
   }
}