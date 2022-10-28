using UnityEngine;
using System.Collections;
using UnityEngine.UIElements;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	private float g = 0.27f;
	private float mass = 1f;
	float dt_v = 0.00045f;
	public float dt_w = 0.088f;
	
	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;

	private Vector3 block_v = Vector3.zero;
	private Vector3 block_w = Vector3.zero;

	public float w_decay = 0.992f;

	public Transform cube;
	public Transform block;
	
	private Matrix4x4 Iref;
	private Matrix4x4 I;

	private block_motion blockmotion;
	private cube_motion cubemotion;

	// Use this for initialization
	void Start () 
	{
		Iref[0, 0] = 1.0f ;
		Iref[1, 1] = 1.0f ;
		Iref[2, 2] = 1.0f ;
		Iref[3, 3] = 1.0f ;

		blockmotion = block.GetComponent<block_motion>();
		cubemotion = cube.GetComponent<cube_motion>();
		
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{
		Vector3 pos1 = cube.position;
		Vector3 pos2 = block.position;


		int li1 = size - 1;
		int ui1 = 0;
		int lj1 = size - 1;
		int uj1 = 0;
		
		int li2 = size - 1;
		int ui2 = 0;
		int lj2 = size - 1;
		int uj2 = 0;
		//Step 1:
		//TODO: Compute new_h based on the shallow wave model.
		for (int i = 0; i < size; ++i)
		{
			for (int j = 0; j < size; ++j)
			{
				new_h[i, j] = h[i, j] + (h[i, j] - old_h[i, j]) * damping;
				if(i-1 >= 0)
					new_h[i,j] += (h[i - 1, j] - h[i, j]) * rate;
				if(j-1 >= 0)
					new_h[i,j] += (h[i , j - 1] - h[i, j]) * rate;
				if(i + 1 <= size - 1)
					new_h[i,j] += (h[i + 1, j] - h[i, j]) * rate;
				if(j + 1 <= size - 1)
					new_h[i,j] += (h[i , j + 1] - h[i, j]) * rate;
				
				float x = 0.1f * i - 0.05f * size;
				float z = 0.1f * j - 0.05f * size;
				
				if (x > pos1.x - 0.5f && x < pos1.x + 0.5f && z > pos1.z - 0.5f && z < pos1.z + 0.5f)
				{
					RaycastHit hit;
					Physics.Raycast(new Vector3(x, -100f, z), Vector3.up, out hit, 100f);

					if (li1 > i) li1 = i;
					if (ui1 < i) ui1 = i;
					if (lj1 > j) lj1 = j;
					if (uj1 < j) uj1 = j;
					//Debug.Log(li1);
					low_h[i, j] = hit.point.y;
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					cg_mask[i, j] = true;
				}
				else if (x > pos2.x - 0.5f && x < pos2.x + 0.5f && z > pos2.z - 0.5f && z < pos2.z + 0.5f)
				{
					RaycastHit hit;
					Physics.Raycast(new Vector3(x, -100f, z), Vector3.up, out hit, 100f);
					
					if (li2 > i) li2 = i;
					if (ui2 < i) ui2 = i;
					if (lj2 > j) lj2 = j;
					if (uj2 < j) uj2 = j;
					low_h[i, j] = hit.point.y;
					b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					cg_mask[i, j] = true;
				}
				else
				{
					vh[i, j] = 0;
					cg_mask[i, j] = false;
				}
			}
		}
		
		// float cur_x = 0.1f * li1 - 0.05f * size;
		// float cur_z = 0.1f * lj1 - 0.05f * size;
		// Debug.Log(cur_x);
		// Debug.Log(cur_z);
		
		//Step 2: Block->Water coupling
		//TODO: for block 1, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, li1, ui1, lj1, uj1);
				
		//TODO: for block 2, calculate low_h.
		//TODO: then set up b and cg_mask for conjugate gradient.
		//TODO: Solve the Poisson equation to obtain vh (virtual height).
		Conjugate_Gradient(cg_mask, b, vh, li2, ui2, lj2, uj2);
	
		//TODO: Diminish vh.
		for (int i = 0; i < size; ++i)
		{
			for (int j = 0; j < size; ++j)
			{
				vh[i, j] *= gamma;
			}
		}

		//TODO: Update new_h by vh.
		for (int i = 0; i < size; ++i)
		{
			for (int j = 0; j < size; ++j)
			{
				if(i-1 >= 0)
					new_h[i,j] += (vh[i-1,j] - vh[i,j]) * rate;
				if(j-1 >= 0)
					new_h[i,j] += (vh[i,j-1] - vh[i,j]) * rate;
				if(i + 1 <= size - 1)
					new_h[i,j] += (vh[i+1,j] - vh[i,j]) * rate;
				if(j + 1 <= size - 1)
					new_h[i,j] += (vh[i,j+1] - vh[i,j]) * rate;
			}
		}

		//Step 3
		//TODO: old_h <- h; h <- new_h;
		for (int j = 0; j < size; ++j)
		{
			for (int i = 0; i < size; ++i)
			{
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
			}
		}

		//Step 4: Water->Block coupling.
		//More TODO here.
		CubeCoupling(li1, ui1, lj1, uj1);
		BlockCoupling(li2, ui2, lj2, uj2);
	}

	/// <summary>
	/// water->cube
	/// </summary>
	void CubeCoupling(int li,int ui,int lj,int uj)
	{
		Vector3 f_up = Vector3.zero;
		Vector3 tao = Vector3.zero;
			
		if (cubemotion.cube_move)
			return;
		Vector3 pos = cube.position;
		for (int i = li; i <= ui; ++i)
		{
			for (int j = lj ; j <= uj; ++j)
			{
				float curx = 0.1f;
				float curz = 0.1f;
				//考虑边缘的方格的面积
				if (i == li)
				{
					curx = (0.1f * (i ) - 0.05f * size) - (pos.x - 0.5f);
				}
				if (i == ui)
				{
					curx = (pos.x + 0.5f) - (0.1f * (i) - 0.05f * size);
				}
				if (j == lj)
				{
					curz = (0.1f * (j ) - 0.05f * size) - (pos.z - 0.5f);
				}
				if (j == uj)
				{
					curz = (pos.z + 0.5f) - (0.1f * (j ) - 0.05f * size);
				}
				float s = curx * curz;
				//计算向上的浮力
				f_up += (vh[i, j] * s * Vector3.up);
				
				float x = 0.1f * i - 0.05f * size;
				float z = 0.1f * j - 0.05f * size;
				Vector3 startPos = new Vector3(x, -100f, z);
				RaycastHit hit;
				Physics.Raycast(startPos, Vector3.up, out hit, 100);
				Vector3 pressPos = hit.point - cube.transform.position;
				Vector3 f_w = (vh[i, j] * s * Vector3.up);
				//计算合力矩
				tao += Vector3.Cross(pressPos, f_w);
			}
		}
		//考虑重力，求合力
		f_up += mass * Vector3.down * g;
		//重力不影响力矩
		//tao += Vector3.Cross(Vector3.zero,mass * Vector3.down * g);
		//更新速度
		cube_v += f_up / mass;
		cube.transform.position += cube_v * dt_v;
		//更新角速度
		Quaternion q0 = cube.rotation;
		Matrix4x4 R0 = Matrix4x4.Rotate(q0);
		I = R0 * Iref * R0.transpose;
		cube_w *= w_decay;
		cube_w += dt_w * Vector4To3(I.inverse * tao);
		Quaternion buildWithW = BuildQwithW(cube_w);
		Quaternion curQ = AddQ(q0,MulScale(buildWithW,0.5f * dt_w) * q0);
		cube.rotation = curQ;
	}

	/// <summary>
	/// water->block
	/// </summary>
	void BlockCoupling(int li,int ui,int lj,int uj)
	{
		Vector3 f_v = Vector3.zero;
		Vector3 tao = Vector3.zero;

		if (blockmotion.block_move)
			return;
		Vector3 pos = block.position;
		for (int i = li; i <= ui; ++i)
		{
			for (int j = lj ; j <= uj; ++j)
			{
				float curx = 0.1f;
				float curz = 0.1f;
				//考虑边缘的方格的面积
				if (i == li)
				{
					curx = (0.1f * (i ) - 0.05f * size) - (pos.x - 0.5f);
				}
				if (i == ui)
				{
					curx = (pos.x + 0.5f) - (0.1f * (i) - 0.05f * size);
				}
				if (j == lj)
				{
					curz = (0.1f * (j ) - 0.05f * size) - (pos.z - 0.5f);
				}
				if (j == uj)
				{
					curz = (pos.z + 0.5f) - (0.1f * (j ) - 0.05f * size);
				}
				float s = curx * curz;
				//计算浮力
				f_v += (vh[i, j] * s * Vector3.up);
				
				float x = 0.1f * i - 0.05f * size;
				float z = 0.1f * j - 0.05f * size;
				Vector3 startPos = new Vector3(x, -100f, z);
				RaycastHit hit;
				Physics.Raycast(startPos, Vector3.up, out hit, 100);
				Vector3 pressPos = hit.point - block.transform.position;
				Vector3 f_w = (vh[i, j] * s * Vector3.up);
				//计算和力矩
				tao += Vector3.Cross(pressPos, f_w);
			}
		}
		//考虑重力，求合力
		f_v += mass * Vector3.down * g;
		//重力不影响力矩
		//tao += Vector3.Cross(Vector3.zero,mass * Vector3.down * g);
		//更新速度
		block_v += f_v / mass;
		block.transform.position += block_v * dt_v;
		//更新角速度
		Quaternion q0 = block.rotation;
		Matrix4x4 R0 = Matrix4x4.Rotate(q0);
		I = R0 * Iref * R0.transpose;
		block_w *= w_decay;
		block_w += dt_w * Vector4To3(I.inverse * tao);
		Quaternion buildWithW = BuildQwithW(block_w);
		Quaternion curQ = AddQ(q0,MulScale(buildWithW,0.5f * dt_w) * q0);
		block.rotation = curQ;
	}
	
	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int k = 0; k < X.Length; ++k)
		{
			int j = k / size;
			int i = k - j * size;
			h[j, i] = X[k].y;
		}

		if (Input.GetKeyDown ("r"))
		{
			int i = UnityEngine.Random.Range(1, size - 1);
			int j = UnityEngine.Random.Range(1, size - 1);
			float r = UnityEngine.Random.Range(0.1f, 0.4f);
			h[i, j] += r;
			h[i + 1, j ] -= r / 4.0f;
			h[i - 1, j ] -= r / 4.0f;
			h[i , j - 1] -= r / 4.0f;
			h[i , j + 1] -= r / 4.0f;
			// h[i + 1, j + 1] -= r / 8.0f;
			// h[i - 1, j + 1] -= r / 8.0f;
			// h[i - 1, j - 1] -= r / 8.0f;
			// h[i + 1, j - 1] -= r / 8.0f;
		}
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		//TODO: Store h back into X.y and recalculate normal.
		for (int k = 0; k < X.Length; ++k)
		{
			int j = k / size;
			int i = k - j * size;
			X[k].y = h[j, i];
		}
		mesh.vertices = X;
		mesh.RecalculateNormals();
	}
	
	Vector3 MatrixDotVector(Matrix4x4 R, Vector3 v)
	{
		Vector3 re = Vector3.zero;
		re.x = R[0, 0] * v.x + R[0, 1] * v.y + R[0, 2] * v.z;
		re.y = R[1, 0] * v.x + R[1, 1] * v.y + R[1, 2] * v.z;
		re.z = R[2, 0] * v.x + R[2, 1] * v.y + R[2, 2] * v.z;
		return re;
	}
	Vector3 Vector4To3(Vector4 a)
	{
		//Debug.Log(a);
		return new Vector3(a.x,a.y,a.z);
	}
	private Quaternion BuildQwithW(Vector3 w)
	{
		Quaternion q = new Quaternion();
		q.w = 0;
		q.x = w.x;
		q.y = w.y;
		q.z = w.z;
		return q;
	}
	private Quaternion AddQ(Quaternion a,Quaternion b)
	{
		Quaternion q = new Quaternion();
		q.w = a.w + b.w;
		q.x = a.x + b.x;
		q.y = a.y + b.y;
		q.z = a.z + b.z;
		return q;
	}
	private Quaternion MulScale(Quaternion a,float b)
	{
		a.x *= b;
		a.y *= b;
		a.z *= b;
		a.w *= b;
		return a;
	}
}
