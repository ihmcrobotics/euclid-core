package us.ihmc.geometry.tuple2D;

import java.io.Serializable;

import us.ihmc.geometry.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.geometry.tuple2D.interfaces.Vector2DBasics;

public class Vector2D32 extends Tuple2D32<Vector2D32> implements Serializable, Vector2DBasics<Vector2D32>
{
   private static final long serialVersionUID = 6380132073713315352L;

   public Vector2D32()
   {
      super();
   }

   public Vector2D32(float x, float y)
   {
      super(x, y);
   }

   public Vector2D32(float[] vectorArray)
   {
      super(vectorArray);
   }

   public Vector2D32(Tuple2DReadOnly<?> other)
   {
      super(other);
   }
}
