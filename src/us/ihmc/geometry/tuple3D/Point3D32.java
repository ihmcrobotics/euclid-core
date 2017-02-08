package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Point3DBasics;
import us.ihmc.geometry.tuple3D.interfaces.Tuple3DReadOnly;

public class Point3D32 extends Tuple3D32<Point3D32> implements Serializable, Point3DBasics<Point3D32>
{
   private static final long serialVersionUID = 5142616577127976269L;

   public Point3D32()
   {
      super();
   }

   public Point3D32(float x, float y, float z)
   {
      super(x, y, z);
   }

   public Point3D32(float[] pointArray)
   {
      super(pointArray);
   }

   public Point3D32(Tuple3DReadOnly<?> other)
   {
      super(other);
   }
}
