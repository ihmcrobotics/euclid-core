package us.ihmc.geometry.tuple3D;

import java.io.Serializable;

import us.ihmc.geometry.tuple3D.interfaces.Vector3DBasics;

public class Vector3D32 extends Tuple3D32<Vector3D32> implements Serializable, Vector3DBasics<Vector3D32>
{
   private static final long serialVersionUID = 1186918378545386628L;

   public Vector3D32()
   {
      super();
   }

   public Vector3D32(float x, float y, float z)
   {
      super(x, y, z);
   }

   public Vector3D32(float[] vectorArray)
   {
      super(vectorArray);
   }

   public Vector3D32(Tuple3D32<?> tuple)
   {
      super(tuple);
   }
}
