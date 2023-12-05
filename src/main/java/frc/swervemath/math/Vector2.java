package frc.swervemath.math;

public class Vector2 {
    public float x, y;
    public Vector2(float x, float y){ 
        this.x = x; 
        this.y = y; 
    }
    public float magnitude() { 
        return (float)Math.sqrt(x*x + y*y); 
    }
    public Vector2 normalized() { 
        float mag = magnitude(); 

        //does this to avoid exceptions in case the vector is zero
        if(x == 0 && y == 0)
            return this;
    
        return new Vector2(x / mag, y / mag); 
    }
    public static Vector2 divide(Vector2 vector, float by) { 
        return new Vector2(vector.x / by, vector.y / by); 
    }
    public static Vector2 multiply(Vector2 vector, float by) { 
        return new Vector2(vector.x * by, vector.y * by); 
    }
    public static Vector2 add(Vector2 a, Vector2 b) { 
        return new Vector2(a.x + b.x, a.y + b.y); 
    }
    public static Vector2 subtract(Vector2 a, Vector2 b) { 
        return new Vector2(a.x - b.x, a.y - b.y); 
    }
    public static float dot(Vector2 a, Vector2 b) { 
        return a.x * b.x + a.y * b.y; 
    }
    //TODO Cross Product
}
