using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Threading.Tasks;
using JetBrains.Annotations;
using UnityEngine.Assertions.Must;
using UnityEngine.Profiling;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();
	
	private Vector3 g = new Vector3(0,-9.8f,0);

	[Space(10)]
	public bool doSmoothing;
	public bool treat_As_HyperelasticModel;
	[Header("Just For Testing")]
	public bool testOneTet;
	
	// public Dictionary<int, List<int>> neighbor;
    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
        
        //neighbor = new Dictionary<int, List<int>>();
        #region Build Mesh from files

        if (testOneTet)
        {
	        //测试单个四面体
	        tet_number=1;
	        Tet = new int[tet_number*4];
	        Tet[0]=0;
	        Tet[1]=1;
	        Tet[2]=2;
	        Tet[3]=3;

	        number=4;
	        X = new Vector3[number];
	        V = new Vector3[number];
	        Force = new Vector3[number];
	        X[0]= new Vector3(0, 0, 0);
	        X[1]= new Vector3(1, 0, 0);
	        X[2]= new Vector3(0, 1, 0);
	        X[3]= new Vector3(0, 0, 1);

        }
        else
        {
	        {
		        string fileContent = File.ReadAllText("Assets/house2.ele");
		        string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
		        //1389个四面体
		        tet_number=int.Parse(Strings[0]);
		        //Debug.Log("Strings[0]="+tet_number);
		        //四面体四个顶点的索引
		        Tet = new int[tet_number*4];
		        // Debug.Log("Strings[1]="+Strings[1]);
		        // Debug.Log("Strings[2]="+Strings[2]);
		        // Debug.Log("Strings[3]="+Strings[3]);
		        // Debug.Log("Strings[4]="+Strings[4]);
		        // Debug.Log("Strings[5]="+Strings[5]);
		        // Debug.Log("Strings[6]="+Strings[6]);
		        // Debug.Log("Strings[7]="+Strings[7]);

		        for(int tet=0; tet<tet_number; tet++)
		        {
				    Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
			        Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
			        Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
			        Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			        // if (tet == 0)
			        // {
				       //  Debug.Log( Tet[tet*4+0]);
				       //  Debug.Log( Tet[tet*4+1]);
				       //  Debug.Log( Tet[tet*4+2]);
				       //  Debug.Log( Tet[tet*4+3]);
			        // }
			        // if (tet == 1)
			        // {
				       //  Debug.Log( Tet[tet*4+0]);
				       //  Debug.Log( Tet[tet*4+1]);
				       //  Debug.Log( Tet[tet*4+2]);
				       //  Debug.Log( Tet[tet*4+3]);
			        // }
		        }
	        }
	        {
		        string fileContent = File.ReadAllText("Assets/house2.node");
		        string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
		        number = int.Parse(Strings[0]);
		        //所有顶点的位置
		        X = new Vector3[number];
		        for(int i=0; i<number; i++)
		        {
			        X[i].x=float.Parse(Strings[i*5+5])*0.4f;
			        X[i].y=float.Parse(Strings[i*5+6])*0.4f;
			        X[i].z=float.Parse(Strings[i*5+7])*0.4f;
		        }
		        //Centralize the model.
		        Vector3 center=Vector3.zero;
		        for(int i=0; i<number; i++)		center+=X[i];
		        center=center/number;
		        for(int i=0; i<number; i++)
		        {
			        X[i]-=center;
			        float temp=X[i].y;
			        X[i].y=X[i].z;
			        X[i].z=temp;
		        }
	        }
        }
        #endregion

        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
		//Debug.Log("vertices顶点数目 = " + vertices.Length);
        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
        if (testOneTet)
        {
	        mesh.triangles = triangles;
	        mesh.vertices  = vertices;
        }
        else
        {
	        mesh.vertices  = vertices;
	        mesh.triangles = triangles;
        }
		mesh.RecalculateNormals ();

		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        for (int tet = 0; tet < tet_number; ++tet)
        {
	        inv_Dm[tet] = Build_Edge_Matrix(tet).inverse;
	        
	        // //(舍弃该方法)存储每个顶点相邻的四面体的
	        // for (int i = 0; i < 4; ++i)
	        // {
		       //  int idx = Tet[tet * 4 + i];
		       //  if (!neighbor.TryGetValue(idx, out var hashSetNeighbor))
		       //  {
			      //   hashSetNeighbor = new List<int>();
			      //   neighbor.Add(idx,hashSetNeighbor);
		       //  }
		       //  hashSetNeighbor.Add(Tet[tet * 4 + 0]);
		       //  hashSetNeighbor.Add(Tet[tet * 4 + 1]);
		       //  hashSetNeighbor.Add(Tet[tet * 4 + 2]);
		       //  hashSetNeighbor.Add(Tet[tet * 4 + 3]);
	        // }
        }
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
	    Matrix4x4 ret = Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
        //X10 x20 x30
        for (int i = 0; i < 3; ++i)
        {
	        //i=0,1-0,x10
	        //i=1,2-0,x20
	        //i=2,3-0,x30
	        Vector3 i0 = X[Tet[tet * 4 + 0]] - X[Tet[tet * 4 + i + 1]];
	        ret[0, i] = i0.x;
	        ret[1, i] = i0.y;
	        ret[2, i] = i0.z;
        }
        ret[3, 3] = 1;
		return ret;
    }

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}


		//TODO: Add gravity to Force.
    	for(int i=0 ;i<number; i++)
    	{
            Force[i] = Vector3.zero;
            Force[i] += g;
            
            V_sum[i] = Vector3.zero;
            V_num[i] = 0;
        }
        
        //使用 parallel并行
        Parallel.For(0, tet_number, item =>
        {
	        #region FVM simulation
    		//TODO: Deformation Gradient
            Matrix4x4 cur_Dm = Matrix4x4.zero;
            //当前 X10 X20 X30
            for (int i = 0; i < 3; ++i)
            {
	            Vector3 i0 = X[Tet[item * 4 + 0]] - X[Tet[item * 4 + i + 1]];
	            cur_Dm[0, i] = i0.x;
	            cur_Dm[1, i] = i0.y;
	            cur_Dm[2, i] = i0.z;
            }
            cur_Dm[3, 3] = 1;
            Matrix4x4 F = cur_Dm * inv_Dm[item];
            
            Matrix4x4 P = Matrix4x4.zero;

            if (treat_As_HyperelasticModel)
            {
	            //Method2:
	            Matrix4x4 U = Matrix4x4.zero;
	            Matrix4x4 S = Matrix4x4.zero;
	            Matrix4x4 V = Matrix4x4.zero;
	            Profiler.BeginSample("svd");
	            svd.svd(F,ref U,ref S,ref V);
	            Profiler.EndSample();
	            float lamda0 = S[0, 0];
	            float lamda1 = S[1, 1];
	            float lamda2 = S[2, 2];
	            float lamda0_Quadratic = lamda0 * lamda0;
	            float lamda0_Cubic = lamda0_Quadratic * lamda0;
	            float lamda1_Quadratic = lamda1 * lamda1;
	            float lamda1_Cubic = lamda1_Quadratic * lamda1;
	            float lamda2_Quadratic = lamda2 * lamda2;
	            float lamda2_Cubic = lamda2_Quadratic * lamda2;
	            S[0, 0] = 0.125f * stiffness_0 * (4 * lamda0_Cubic + 4 * lamda0 * lamda1_Quadratic + 4 * lamda0 * lamda2_Quadratic - 12 * lamda0) +
	                      0.25f * stiffness_1 * (4 * lamda0_Cubic - 4 * lamda0);
	            S[1, 1] = 0.125f * stiffness_0 * (4 * lamda1_Cubic + 4 * lamda1 * lamda0_Quadratic + 4 * lamda1 * lamda2_Quadratic - 12 * lamda1) +
	                      0.25f * stiffness_1 * (4 * lamda1_Cubic - 4 * lamda1);
	            S[2, 2] = 0.125f * stiffness_0 * (4 * lamda2_Cubic + 4 * lamda2 * lamda1_Quadratic + 4 * lamda2 * lamda0_Quadratic - 12 * lamda2) +
	                      0.25f * stiffness_1 * (4 * lamda2_Cubic - 4 * lamda2);
	            S[3, 3] = 1;
	            P = U * S * V.transpose;
            }
            else
            {
	            //Method1:
	            //TODO: Green Strain
	            Matrix4x4 G = ScaleDotMatrix(0.5f, MatrixMinuesMatrix(F.transpose * F, Matrix3x3Identity()));
	            //TODO: Second PK Stress
	            Matrix4x4 trace = GetDiagonal(G);
	            Matrix4x4 S = MatrixAddMatrix(ScaleDotMatrix(2 * stiffness_1, G), ScaleDotMatrix(stiffness_0, trace) * Matrix3x3Identity());
	            P = F * S;
            }
			//TODO: Elastic Force
			Matrix4x4 forceMatrix = Matrix4x4.zero;
			float scaleValue = -1 / (inv_Dm[item].determinant * 6.0f);
			Matrix4x4 inv_DmT = inv_Dm[item].transpose;
			Matrix4x4 matrix = P * inv_DmT;
			forceMatrix = ScaleDotMatrix(scaleValue, matrix);
			
			Vector3 f1 = new Vector3(forceMatrix[0, 0], forceMatrix[1, 0], forceMatrix[2, 0]);
			Vector3 f2 = new Vector3(forceMatrix[0, 1], forceMatrix[1, 1], forceMatrix[2, 1]);
			Vector3 f3 = new Vector3(forceMatrix[0, 2], forceMatrix[1, 2], forceMatrix[2, 2]);
			Vector3 f0 = -(f1 + f2 + f3);
			
			Force[Tet[item * 4 + 1]] += f1;
			Force[Tet[item * 4 + 2]] += f2;
			Force[Tet[item * 4 + 3]] += f3;
			Force[Tet[item * 4 + 0]] += f0;
			#endregion
        });
        
        //未并行版本
   //      for(int tet=0; tet<tet_number; tet++)
   //  	{
	  //       #region FVM simulation
   //  		//TODO: Deformation Gradient
   //          Matrix4x4 cur_Dm = Matrix4x4.zero;
   //          //当前 X10 X20 X30
   //          for (int i = 0; i < 3; ++i)
   //          {
	  //           Vector3 i0 = X[Tet[tet * 4 + 0]] - X[Tet[tet * 4 + i + 1]];
	  //           cur_Dm[0, i] = i0.x;
	  //           cur_Dm[1, i] = i0.y;
	  //           cur_Dm[2, i] = i0.z;
   //          }
   //          cur_Dm[3, 3] = 1;
   //          Matrix4x4 F = cur_Dm * inv_Dm[tet];
   //          
   //          Matrix4x4 P = Matrix4x4.zero;
   //
   //          if (treat_As_HyperelasticModel)
   //          {
	  //           //Method2:
	  //           Matrix4x4 U = Matrix4x4.zero;
	  //           Matrix4x4 S = Matrix4x4.zero;
	  //           Matrix4x4 V = Matrix4x4.zero;
	  //           svd.svd(F,ref U,ref S,ref V);
	  //           float lamda0 = S[0, 0];
	  //           float lamda1 = S[1, 1];
	  //           float lamda2 = S[2, 2];
	  //           float lamda0_Quadratic = lamda0 * lamda0;
	  //           float lamda0_Cubic = lamda0_Quadratic * lamda0;
	  //           float lamda1_Quadratic = lamda1 * lamda1;
	  //           float lamda1_Cubic = lamda1_Quadratic * lamda1;
	  //           float lamda2_Quadratic = lamda2 * lamda2;
	  //           float lamda2_Cubic = lamda2_Quadratic * lamda2;
	  //           S[0, 0] = 0.125f * stiffness_0 * (4 * lamda0_Cubic + 4 * lamda0 * lamda1_Quadratic + 4 * lamda0 * lamda2_Quadratic - 12 * lamda0) +
	  //                     0.25f * stiffness_1 * (4 * lamda0_Cubic - 4 * lamda0);
	  //           S[1, 1] = 0.125f * stiffness_0 * (4 * lamda1_Cubic + 4 * lamda1 * lamda0_Quadratic + 4 * lamda1 * lamda2_Quadratic - 12 * lamda1) +
	  //                     0.25f * stiffness_1 * (4 * lamda1_Cubic - 4 * lamda1);
	  //           S[2, 2] = 0.125f * stiffness_0 * (4 * lamda2_Cubic + 4 * lamda2 * lamda1_Quadratic + 4 * lamda2 * lamda0_Quadratic - 12 * lamda2) +
	  //                     0.25f * stiffness_1 * (4 * lamda2_Cubic - 4 * lamda2);
	  //           S[3, 3] = 1;
	  //           P = U * S * V.transpose;
   //          }
   //          else
   //          {
	  //           //Method1:
	  //           //TODO: Green Strain
	  //           Matrix4x4 G = ScaleDotMatrix(0.5f, MatrixMinuesMatrix(F.transpose * F, Matrix3x3Identity()));
	  //           //TODO: Second PK Stress
	  //           Matrix4x4 trace = GetDiagonal(G);
	  //           Matrix4x4 S = MatrixAddMatrix(ScaleDotMatrix(2 * stiffness_1, G), ScaleDotMatrix(stiffness_0, trace) * Matrix3x3Identity());
	  //           P = F * S;
   //          }
			// //TODO: Elastic Force
			// Matrix4x4 forceMatrix = Matrix4x4.zero;
			// float scaleValue = -1 / (inv_Dm[tet].determinant * 6.0f);
			// Matrix4x4 inv_DmT = inv_Dm[tet].transpose;
			// Matrix4x4 matrix = P * inv_DmT;
			// forceMatrix = ScaleDotMatrix(scaleValue, matrix);
			//
			// Vector3 f1 = new Vector3(forceMatrix[0, 0], forceMatrix[1, 0], forceMatrix[2, 0]);
			// Vector3 f2 = new Vector3(forceMatrix[0, 1], forceMatrix[1, 1], forceMatrix[2, 1]);
			// Vector3 f3 = new Vector3(forceMatrix[0, 2], forceMatrix[1, 2], forceMatrix[2, 2]);
			// Vector3 f0 = -(f1 + f2 + f3);
			//
			// Force[Tet[tet * 4 + 1]] += f1;
			// Force[Tet[tet * 4 + 2]] += f2;
			// Force[Tet[tet * 4 + 3]] += f3;
			// Force[Tet[tet * 4 + 0]] += f0;
			// #endregion
   //      }

        for(int i=0; i<number; i++)
        {
	        //TODO: Update X and V here.
	        V[i] += Force[i] / mass * dt;
	        V[i] *= damp; 
			if(!doSmoothing) 
				X[i] += V[i] * dt;
            //TODO: (Particle) collision with floor.
            CollisionMethod(i,new Vector3(0,-3f,0), new Vector3(0,1,0));
        }

        if (doSmoothing)
        {
	        for (int tet = 0; tet < tet_number; tet++)
	        {
		        Vector3 v_sum = V[Tet[tet * 4 + 0]] + V[Tet[tet * 4 + 1]] + V[Tet[tet * 4 + 2]] + V[Tet[tet * 4 + 3]];
		        for (int i = 0; i < 4; ++i)
		        {
			        int idx = Tet[tet * 4 + i];
			        V_sum[idx] += v_sum;
			        V_num[idx] += 4;
		        }
	        }

	        for (int i = 0; i < number; i++)
	        {
		        V[i] = V_sum[i] / V_num[i];
		        X[i] += V[i] * dt;
	        }
        }
    }

    Matrix4x4 Matrix3x3Identity()
    {
	    Matrix4x4 re = Matrix4x4.zero;
	    re[0, 0] = 1;
	    re[1, 1] = 1;
	    re[2, 2] = 1;
	    return re;
    }

    void CollisionMethod(int i,Vector3 p,Vector3 n)
    {
	    Vector3 x = X[i];
	    Vector3 pTox = x - p;
	    float value = Vector3.Dot(pTox, n);
	    if (value < 0)
	    {
		    //Debug.Log("碰撞发生");
		    Vector3 vni = Vector3.Dot(V[i], n) * n;
		    Vector3 vti = V[i] - vni;
		    float a = Mathf.Max(1 - 0.5f * (1 + 0.5f) * vni.magnitude / vti.magnitude, 0);
		    Vector3 vni_new = -0.5f * vni;
		    Vector3 vti_new = a * vti;
		    Vector3 v_new = vni_new + vti_new;
		    V[i] = v_new;
		    Vector3 x_push = Vector3.Dot(pTox, n) * n;
		    X[i] -= x_push;
	    }
    }

    Matrix4x4 GetDiagonal(Matrix4x4 G)
    {
	    Matrix4x4 re = Matrix4x4.zero;
	    re[0, 0] = G[0, 0];
	    re[1, 1] = G[1, 1];
	    re[2, 2] = G[2, 2];
	    re[3, 3] = 1;
	    return re;
    }

    Matrix4x4 MatrixAddMatrix(Matrix4x4 a,Matrix4x4 b)
    {
	    for (int i = 0; i < 3; ++i)
	    {
		    for (int j = 0; j < 3; ++j)
		    {
			    a[i, j] += b[i, j];
		    }
	    }
	    return a;
    }

    Matrix4x4 ScaleDotMatrix(float value , Matrix4x4 F)
    {
	    for (int i = 0; i < 3; ++i)
	    {
		    for (int j = 0; j < 3; ++j)
		    {
			    F[i, j] *= value;
		    }
	    }
	    return F;
    }
    
    Matrix4x4 MatrixMinuesMatrix(Matrix4x4 a, Matrix4x4 b)
    {
	    for (int i = 0; i < 3; ++i)
	    {
		    for (int j = 0; j < 3; ++j)
		    {
			    a[i, j] -= b[i, j];
		    }
	    }
	    return a;
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
