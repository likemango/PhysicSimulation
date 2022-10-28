using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rigid_Bunny_by_Shape_Matching : MonoBehaviour
{
	private bool launched = false;
	Vector3[] X;
	Vector3[] Q;
	Vector3[] V;
	Matrix4x4 QQt = Matrix4x4.zero;
	
	private Mesh mesh;

	private Vector3 g = new Vector3(0,-9.8f,0);
	public float restitution 	= 0.8f;					// for collision

	//public int collisionNum;

	private Vector3[] origlMesh;

    void Start()
    {
    	mesh = GetComponent<MeshFilter>().mesh;
        V = new Vector3[mesh.vertices.Length];
        X = mesh.vertices;
        Q = mesh.vertices;

        //Centerizing Q.
        Vector3 c=Vector3.zero;
        for(int i=0; i<Q.Length; i++)
        	c+=Q[i];
        c/=Q.Length;
        for(int i=0; i<Q.Length; i++)
        	Q[i]-=c;

        //Get QQ^t ready.
		for(int i=0; i<Q.Length; i++)
		{
			QQt[0, 0]+=Q[i][0]*Q[i][0];
			QQt[0, 1]+=Q[i][0]*Q[i][1];
			QQt[0, 2]+=Q[i][0]*Q[i][2];
			QQt[1, 0]+=Q[i][1]*Q[i][0];
			QQt[1, 1]+=Q[i][1]*Q[i][1];
			QQt[1, 2]+=Q[i][1]*Q[i][2];
			QQt[2, 0]+=Q[i][2]*Q[i][0];
			QQt[2, 1]+=Q[i][2]*Q[i][1];
			QQt[2, 2]+=Q[i][2]*Q[i][2];
		}
		QQt[3, 3]=1;

		for(int i=0; i<X.Length; i++)
			V[i][0]=4.0f;

		Update_Mesh(transform.position, Matrix4x4.Rotate(transform.rotation), 0);
		transform.position=Vector3.zero;
		transform.rotation=Quaternion.identity;
		origlMesh = mesh.vertices;
   	}

   	// Polar Decomposition that returns the rotation from F.
   	Matrix4x4 Get_Rotation(Matrix4x4 F)
	{
		Matrix4x4 C = Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C[ii,jj]+=F[kk,ii]*F[kk,jj];
	   
	   	Matrix4x4 C2 = Matrix4x4.zero;
		for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        C2[ii,jj]+=C[ii,kk]*C[jj,kk];
	    
	    float det    =  F[0,0]*F[1,1]*F[2,2]+
	                    F[0,1]*F[1,2]*F[2,0]+
	                    F[1,0]*F[2,1]*F[0,2]-
	                    F[0,2]*F[1,1]*F[2,0]-
	                    F[0,1]*F[1,0]*F[2,2]-
	                    F[0,0]*F[1,2]*F[2,1];
	    
	    float I_c    =   C[0,0]+C[1,1]+C[2,2];
	    float I_c2   =   I_c*I_c;
	    float II_c   =   0.5f*(I_c2-C2[0,0]-C2[1,1]-C2[2,2]);
	    float III_c  =   det*det;
	    float k      =   I_c2-3*II_c;
	    
	    Matrix4x4 inv_U = Matrix4x4.zero;
	    if(k<1e-10f)
	    {
	        float inv_lambda=1/Mathf.Sqrt(I_c/3);
	        inv_U[0,0]=inv_lambda;
	        inv_U[1,1]=inv_lambda;
	        inv_U[2,2]=inv_lambda;
	    }
	    else
	    {
	        float l = I_c*(I_c*I_c-4.5f*II_c)+13.5f*III_c;
	        float k_root = Mathf.Sqrt(k);
	        float value=l/(k*k_root);
	        if(value<-1.0f) value=-1.0f;
	        if(value> 1.0f) value= 1.0f;
	        float phi = Mathf.Acos(value);
	        float lambda2=(I_c+2*k_root*Mathf.Cos(phi/3))/3.0f;
	        float lambda=Mathf.Sqrt(lambda2);
	        
	        float III_u = Mathf.Sqrt(III_c);
	        if(det<0)   III_u=-III_u;
	        float I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2*III_u/lambda);
	        float II_u=(I_u*I_u-I_c)*0.5f;
	        
	        
	        float inv_rate, factor;
	        inv_rate=1/(I_u*II_u-III_u);
	        factor=I_u*III_u*inv_rate;
	        
	       	Matrix4x4 U = Matrix4x4.zero;
			U[0,0]=factor;
	        U[1,1]=factor;
	        U[2,2]=factor;
	        
	        factor=(I_u*I_u-II_u)*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            U[i,j]+=factor*C[i,j]-inv_rate*C2[i,j];
	        
	        inv_rate=1/III_u;
	        factor=II_u*inv_rate;
	        inv_U[0,0]=factor;
	        inv_U[1,1]=factor;
	        inv_U[2,2]=factor;
	        
	        factor=-I_u*inv_rate;
	        for(int i=0; i<3; i++)
	        for(int j=0; j<3; j++)
	            inv_U[i,j]+=factor*U[i,j]+inv_rate*C[i,j];
	    }
	    
	    Matrix4x4 R=Matrix4x4.zero;
	    for(int ii=0; ii<3; ii++)
	    for(int jj=0; jj<3; jj++)
	    for(int kk=0; kk<3; kk++)
	        R[ii,jj]+=F[ii,kk]*inv_U[kk,jj];
	    R[3,3]=1;
	    return R;
	}

	// Update the mesh vertices according to translation c and rotation R.
	// It also updates the velocity.
	void Update_Mesh(Vector3 c, Matrix4x4 R, float inv_dt)
   	{
   		for(int i=0; i<Q.Length; i++)
		{
			Vector3 x=(Vector3)(R*Q[i])+c;

			V[i]+=(x-X[i])*inv_dt;
			X[i]=x;
		}	
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.vertices=X;
   	}

	void Collision(float inv_dt)
	{
		for (int i = 0; i < X.Length; ++i)
		{
			Collision_Impulse(i,new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0),inv_dt);
			Collision_Impulse(i,new Vector3(2, 0, 0), new Vector3(-1, 0, 0), inv_dt);
		}
	}
	
	void Collision_Impulse(int i,Vector3 P, Vector3 N,float inv_dt)
	{
		Vector3 pTox = X[i] - P;
		float dotValue = Vector3.Dot(pTox, N);
		if (dotValue > 0)
			return;
		
		Vector3 vi = V[i];

		// Vector3 intersectPos = caculateIntersectPos(X[i], N, P, N);
		// X[i] += intersectPos - X[i];
		X[i] += Vector3.Dot(-pTox, N) * N;
		
		if(Vector3.Dot(vi,N) > 0)
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
		V[i] = vi_new;
	}

	Vector3 caculateIntersectPos(Vector3 x ,Vector3 v, Vector3 P, Vector3 N)
	{
		float t = 0;
		float pdotN = Vector3.Dot(P, N);
		float xdotN = Vector3.Dot(x, N);
		float vdotN = Vector3.Dot(v, N);
		t = ( pdotN - xdotN ) / vdotN;
		if(t < 0)
			Debug.LogError("没有交点，有问题");
		Vector3 re = x + t * v;
		return re;
	}

    // Update is called once per frame
    void Update()
    {
  		float dt = 0.015f;
        
        GameControl();
        
        if (!launched)
	        return;

  		//Step 1: run a simple particle system.
        for(int i=0; i<V.Length; i++)
        {
	        V[i] = V[i] + g * dt;
	        X[i] = X[i] + V[i] * dt;
        }
        
        //Step 2: Perform simple particle collision.
		Collision(1/dt);

		// Step 3: Use shape matching to get new translastion c and 
		// new rotation R. Update the mesh by c and R.
		Vector3 c=Vector3.zero;
		for(int i=0; i<X.Length; i++)
			c+=X[i];
		c/=X.Length;

		int k = 1;
		//Compute A, A = a * QQt
		Matrix4x4 a = Matrix4x4.zero;
		for (int i = 0; i < Q.Length; i +=k)
		{
			Vector3 yic = X[i] - c;
			a[0, 0]+=yic[0]*Q[i][0] * k;
			a[0, 1]+=yic[0]*Q[i][1] * k;
			a[0, 2]+=yic[0]*Q[i][2] * k;
			a[1, 0]+=yic[1]*Q[i][0] * k;
			a[1, 1]+=yic[1]*Q[i][1] * k;
			a[1, 2]+=yic[1]*Q[i][2] * k;
			a[2, 0]+=yic[2]*Q[i][0] * k;
			a[2, 1]+=yic[2]*Q[i][1] * k;
			a[2, 2]+=yic[2]*Q[i][2] * k;
		}
		a[3, 3] = 1;
		Matrix4x4 A = a * QQt.inverse;
		Matrix4x4 R = Get_Rotation(A);
        //Shape Matching (translation)
		//Shape Matching (rotation)
		Update_Mesh(c, R, 1/dt);
    }

    void GameControl()
    {
	    //Game Control
	    if(Input.GetKey("r"))
	    {
		    mesh.vertices = origlMesh;
		    X = mesh.vertices;
		    for(int i=0; i<X.Length; i++)
			    V[i] = Vector3.zero;
		    restitution = 0.5f;
		    launched=false;
	    }
	    if(Input.GetKey("l"))
	    {
		    for(int i=0; i<X.Length; i++)
			    V[i][0]=4.0f;
		    //g = new Vector3(0,-9.8f,0);
		    launched=true;
	    } 
    }
}
