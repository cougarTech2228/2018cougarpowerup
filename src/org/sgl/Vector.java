package org.sgl;

public class Vector {
	public float x, y, z, r;
	public int length;
	
	public Vector(float x, float y, float z, float r, int length) {
		this.length = length;
		this.x = x;
		this.y = y;
		this.z = z;
		this.r = r;
	}
	public Vector(float x, float y) {
		length = 2;
		this.x = x;
		this.y = y;
	}
	public Vector(float x, float y, float z) {
		length = 3;
		this.x = x;
		this.y = y;
		this.z = z;
	}
	public Vector(float x, float y, float z, float r) {
		length = 4;
		this.x = x;
		this.y = y;
		this.z = z;
		this.r = r;
	}
	public Vector mult(float f) {
		return new Vector(x * f, y * f, x * f, r * f, length);
		
	}
	public Vector mult(Vector v) {
		if(length == v.length) {
			return new Vector(x * v.x, y * v.y, z * v.z, r * v.r, length);
		}
		else throw new IllegalArgumentException("Vectors are different sizes");
	}
	public void normalize() {
		float f = 0;
		f = (x + y + z + r);
		x /= f;
		y /= f;
		z /= f;
		r /= f;
		
		//On Wed: Oof! I ate Tai food, EW! NO!
	}
}
