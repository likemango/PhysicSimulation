using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using TMPro;
using UnityEditor;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	//init angularVelcity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	public float restitution 	= 0.5f;					// for collision
	
	Vector3 g = new Vector3(0,-9.8f,0);
	Vector3[] vertices;

	private bool[] isVertexCollision;

	private int collisionNum = 0;
	private Quaternion q;
	private Matrix4x4 R;

	// private Queue<Vector3> Queue = new Queue<Vector3>();
	// public Vector3[] queueArray = new Vector3[5];
	// public float collisionDt = 0.5f;
	// public int counting = 0;
	
	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;
		isVertexCollision = new bool[vertices.Length];
		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Vector3 curCpos = transform.position;
		collisionNum = 0;
		Vector3 collisionCenter = Vector3.zero;

		q = transform.rotation;
		R = Matrix4x4.Rotate(q);
		
		for (int i = 0; i < vertices.Length; ++i)
		{
			Vector3 Rri = Matrix4X4DotVector3(R, vertices[i]);
			Vector3 x = Rri + curCpos;
			Vector3 pTox = x - P;
			float dotValue = Vector3.Dot(pTox, N);
			isVertexCollision[i] = dotValue < 0;
			if (isVertexCollision[i])
			{
				collisionNum++;
				collisionCenter += vertices[i];
			}
		}
		
		//counting++;
		
		if(collisionNum <= 0)
			return;

		if (v.magnitude < 0.5f)
			restitution *= 0.99f;
		
		//采用碰撞点的中心位置计算
		collisionCenter /= collisionNum;
		// Debug.Log(collisionCenter.x);
		// Debug.Log(collisionCenter.y);
		// Debug.Log(collisionCenter.z);

		ImpulseMethod(collisionCenter, P, N);
	}

	void ImpulseMethod(Vector3 collisionPos,Vector3 P, Vector3 N)
	{
		// Vector3 xTop = P - collisionPos;
		// Vector3 movex = Vector3.Dot(xTop, N) * N;
		// transform.position = collisionPos + movex;
		
		//Vector3 vi = GetVelAtCollisionCenter(collisionPos);
		Vector3 Rri = Matrix4X4DotVector3(R, collisionPos);
		Vector3 vi = v + Vector3.Cross(w, Rri);
		// Debug.Log(vi.x);
		// Debug.Log(vi.y);
		// Debug.Log(vi.z);

		if(Vector3.Dot(vi,N) >= 0)
			return;
		
		//compute vi_new
		Vector3 vni = Vector3.Dot(vi, N) * N;
		Vector3 vti = vi - vni;
		float a = 0;
		if (vti.magnitude < float.Epsilon)
			a = Mathf.Max(0, 1.0f - restitution * (1.0f + restitution) * vni.magnitude / float.Epsilon);
		else
			a = Mathf.Max(0, 1.0f - restitution * (1.0f + restitution) * vni.magnitude / vti.magnitude);
		Vector3 vni_new = -restitution * vni;
		Vector3 vti_new = a * vti;
		Vector3 vi_new = vti_new + vni_new;
		// Debug.Log(vi_new.x);
		// Debug.Log(vi_new.y);
		// Debug.Log(vi_new.z);

		//compute impulse j
		Matrix4x4 tmp = R * I_ref * R.transpose;
		Matrix4x4 tmp1 = Matrix4X4DotScale(Matrix4x4.identity, 1.0f / mass);
		Matrix4x4 tmp2 = Get_Cross_Matrix(Rri);
		Matrix4x4 tmp3 = tmp2 * tmp * tmp2;
		Matrix4x4 k = Matrix4X4Minues(tmp1, tmp3);
		Vector3 j = Matrix4X4DotVector3(k.inverse, vi_new - vi);
		//Debug.Log(j);
		v = v + j / mass;
		w = w + Matrix4X4DotVector3(tmp, Vector3.Cross(Rri, j));
		//Debug.Log(v);
		//Debug.Log(w);

	}

	Matrix4x4 Matrix4X4Minues(Matrix4x4 m, Matrix4x4 n)
	{
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				m[i, j] -= n[i, j];
			}
		}
		return m;
	}
	
	Matrix4x4 Matrix4X4DotScale(Matrix4x4 m,float s)
	{
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				m[i, j] *= s;
			}
		}
		return m;
	}

	Vector3 Matrix4X4DotVector3(Matrix4x4 R,Vector3 vec)
	{
		Vector3 re = Vector3.zero;
		re.x = R[0, 0] * vec.x + R[0, 1] * vec.y + R[0, 2] * vec.z;
		re.y = R[1, 0] * vec.x + R[1, 1] * vec.y + R[1, 2] * vec.z;
		re.z = R[2, 0] * vec.x + R[2, 1] * vec.y + R[2, 2] * vec.z;
		return re;
	}
	
	// Update is called once per frame
	void Update() 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			//init angularVelcity
			//w = new Vector3(1, 0, 0);	// angular velocity
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			g = new Vector3(0,-9.8f,0);
			launched=true;
		} 

		// Part I: Update velocities
		v = v + g * dt;
		v *= linear_decay;
		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		//Update angular status
		Quaternion q = transform.rotation;
		
		//caculate x
		x = x + v  * dt ;
		//apply angular_decay
		w = w * angular_decay;

		//caculate q
		q = quaternionAdd(q,quaternionMulscale(wToquaternion(w), 0.5f * dt ) * q);
		q = q.normalized;
		// Part IV: Assign to the object
		if (launched)
		{
			transform.position = x;
		}
		transform.rotation = q;
	}

	Quaternion wToquaternion(Vector3 w)
	{
		Quaternion q = new Quaternion();
		q.x = w.x;
		q.y = w.y;
		q.z = w.z;
		q.w = 0;
		return q;
	}

	Quaternion quaternionMulscale(Quaternion q,float d)
	{
		q.x *= d;
		q.y *= d;
		q.z *= d;
		q.w *= d;
		return q;
	}

	Quaternion quaternionAdd(Quaternion a, Quaternion b)
	{
		a.x += b.x;
		a.y += b.y;
		a.z += b.z;
		a.w += b.w;
		return a;
	}
}
