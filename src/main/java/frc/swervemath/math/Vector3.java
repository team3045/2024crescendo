package frc.swervemath.math;

public class Vector3 {
    public float x, y, z;
    public Vector3(float x, float y, float z){ 
        this.x = x; 
        this.y = y; 
        this.z = z; 
    }
    public float magnitude() { 
        return (float)Math.sqrt(x*x + y*y + z*z); 
    }
    public Vector3 normalized() { 
        float mag = magnitude(); 

        //does this to avoid exceptions in case the vector is zero
        if(x == 0 && y == 0 && z == 0)
            return this;
        
        return new Vector3(x / mag, y / mag, z / mag); 
    }
    public static Vector3 divide(Vector3 vector, float by) { 
        return new Vector3(vector.x / by, vector.y / by, vector.z / by); 
    }
    public static Vector3 multiply(Vector3 vector, float by) { 
        return new Vector3(vector.x * by, vector.y * by, vector.z * by); 
    }
    public static Vector3 add(Vector3 a, Vector3 b) { 
        return new Vector3(a.x + b.x, a.y + b.y, a.z + b.z); 
    }
    public static Vector3 subtract(Vector3 a, Vector3 b) { 
        return new Vector3(a.x - b.x, a.y - b.y, a.z - b.z); 
    }
    public static float dot(Vector3 a, Vector3 b) { 
        return a.x * b.x + a.y * b.y + a.z * b.z; 
    }
    public static Vector3 cross(Vector3 a, Vector3 b) { 
        return new Vector3(a.y * b.z - a.z * b.y, 
                           a.z * b.x - a.x * b.z, 
                           a.x * b.y - a.y * b.x); 
    }
}