using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Profiling;

public struct vert
{
    public Vector3 pos;
    public Vector3 nextPos;
    public Vector3 force;
    public Vector3 vel;
    public Vector2 uv_;
    public float mass;
};

public struct constrain
{
    public int idx0;
    public int idx1;
    public float k;
    public float orignal_L;
};

public struct collisionSphere{
    public Vector3 sphereCenter;
    public float sphereR;
};


public class ClothMesh_Imrove : MonoBehaviour
{
    [Header("边长顶点数量和顶点间距")]
    public int N;
    public float nodeGap = 0.2f;
    private int nodeNum;

    [Header("衰减速度、模拟时间间隔dt、弹性系数K、迭代次数(Jacobi)")]
    public float damping = 0.995f;
    public float dt = 0.03f;
    public int NumIter;
    public float strengK;
    private float mass = 1f;
    private Mesh mesh;

    [Header("W/S/Space改变外力")]
    public float force;

    [Header("Strain Limiting")]
    public float rho = 0.95f;

    private float sphereR = 2.7f;
    private GameObject sphere;
    private collisionSphere[] collision_sphere;
    
    private Vector3[] vertices;
    private int[] triangles;
    private Vector2[] UV;

    private vert[] verts;
    private List<constrain> constrains;

    private ComputeBuffer vertsBuffer;
    private ComputeBuffer constrainBuffer;
    private ComputeBuffer sphereCollisionBuffer;

    [Space(20)]
    public ComputeShader cloth_mesh_CS;
    public Material ClothMat;
    private int kernel_force;
    private int kernel_constrain;
    private int kernel_collision;
    private int kernel_update;

    void Start()
    { 
        nodeNum = N*N;
        GenerateMesh();
        GenerateCollision();
        //GenerateConstrain();
        BuildConstrainWithSort();
        CreateBuffer();
        InitKernelAndSetBuffer();
    }

    void GenerateCollision()
    {
        sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.position = new Vector3(1,0,-4f);
        sphere.transform.localScale = new Vector3(5,5,5);
        collision_sphere = new collisionSphere[1];
        collision_sphere[0] = new collisionSphere()
        {
            sphereCenter = sphere.transform.position,
            sphereR = sphereR
        };
    }

    // Update is called once per frame
    void Update()
    {  
        if (Input.GetKeyDown(KeyCode.W))
        {
            force += 1f;
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            force -= 1f;
        }

        if (Input.GetKeyDown(KeyCode.Space))
        {
            force = 0;
        }
        //cloth_mesh_CS.SetFloat("forcex",forcex);
        //cloth_mesh_CS.SetFloat("forcey",forcey);
        Profiler.BeginSample("SetFloat force");
        cloth_mesh_CS.SetFloat("force",force);
        Profiler.EndSample();  
        
        Profiler.BeginSample("force caculate");
        cloth_mesh_CS.Dispatch(kernel_force,N/8 + 1,N/8 + 1,1);
        Profiler.EndSample();
        
        Profiler.BeginSample("Jacobi method");
        for (int i = 0; i < NumIter; ++i)
        {
            cloth_mesh_CS.Dispatch(kernel_constrain,N/8 + 1,N/8 + 1,1);
        }
        Profiler.EndSample();
        
        Profiler.BeginSample("Set data for collision sphere");
        collision_sphere[0].sphereCenter = sphere.transform.position;
        sphereCollisionBuffer.SetData(collision_sphere);
        Profiler.EndSample();
        
        Profiler.BeginSample("collision process");
        cloth_mesh_CS.Dispatch(kernel_collision, N / 8 + 1, N / 8 + 1, 1);
        Profiler.EndSample();
        
        Profiler.BeginSample("Update cloth");
        cloth_mesh_CS.Dispatch(kernel_update,N/8 + 1,N/8 + 1,1);
        Profiler.EndSample();
        
    }
    
    void CreateBuffer()
    {
        vertsBuffer = new ComputeBuffer(verts.Length, 15 * sizeof(float));
        constrainBuffer = new ComputeBuffer(constrains.Count ,4 * sizeof(float));
        sphereCollisionBuffer = new ComputeBuffer(1,4 * 4);
    }

    void InitKernelAndSetBuffer()
    {
        kernel_force = cloth_mesh_CS.FindKernel("CaculateForce");
        kernel_constrain = cloth_mesh_CS.FindKernel("CaculateConstrain");
        kernel_update = cloth_mesh_CS.FindKernel("UpdateMeshVert");
        kernel_collision = cloth_mesh_CS.FindKernel("CollisionHanding");
        
        vertsBuffer.SetData(verts);
        constrainBuffer.SetData(constrains);
        sphereCollisionBuffer.SetData(collision_sphere);
        
        cloth_mesh_CS.SetFloat("dt",dt);
        cloth_mesh_CS.SetFloat("rho",rho);
        cloth_mesh_CS.SetFloat("force",force);  
        cloth_mesh_CS.SetFloat("damping",damping);
        cloth_mesh_CS.SetBuffer(kernel_force,"verts",vertsBuffer);
        
        cloth_mesh_CS.SetFloat("N",N);
        cloth_mesh_CS.SetFloat("strainCount",constrains.Count);
        cloth_mesh_CS.SetBuffer(kernel_constrain,"verts",vertsBuffer);
        cloth_mesh_CS.SetBuffer(kernel_constrain,"constrains",constrainBuffer);

        cloth_mesh_CS.SetBuffer(kernel_collision,"verts",vertsBuffer);
        cloth_mesh_CS.SetBuffer(kernel_collision,"collisionSpheres",sphereCollisionBuffer);
        
        cloth_mesh_CS.SetBuffer(kernel_update,"verts",vertsBuffer);
        ClothMat.SetBuffer("verts",vertsBuffer);
    }
    
    void GenerateConstrain()
    {
        constrains = new List<constrain>();
        for (int i = 0; i < N - 1; ++i)
        {
            for (int j = 0; j < N - 1; ++j)
            {
                int idx0 = j + i * N;
                int idx1 = j + (i + 1) * N;
                int idx2 = j + 1 +  (i + 1) * N;
                int idx3 = j + 1 + i * N;

                float dis_idx0_idx1 = (verts[idx0].pos - verts[idx1].pos).magnitude;
                float dis_idx3_idx2 = (verts[idx3].pos - verts[idx2].pos).magnitude;
                float dis_idx1_idx2 = (verts[idx1].pos - verts[idx2].pos).magnitude;
                float dis_idx0_idx3 = (verts[idx0].pos - verts[idx3].pos).magnitude;
                float dis_idx0_idx2 = (verts[idx0].pos - verts[idx2].pos).magnitude;
                float dis_idx1_idx3 = (verts[idx1].pos - verts[idx3].pos).magnitude;
                var constrain0 = new constrain()
                {
                    idx0 = idx0,
                    idx1 = idx1,
                    k = strengK,
                    orignal_L = dis_idx0_idx1
                };
                var constrain1 = new constrain()
                {
                    idx0 = idx3,
                    idx1 = idx2,
                    k = strengK,
                    orignal_L = dis_idx3_idx2
                };
                var constrain2 = new constrain()
                {
                    idx0 = idx1,
                    idx1 = idx2,
                    k = strengK,
                    orignal_L = dis_idx1_idx2
                };
                var constrain3 = new constrain()
                {
                    idx0 = idx0,
                    idx1 = idx3,
                    k = strengK,
                    orignal_L = dis_idx0_idx3
                };
                var constrain4 = new constrain()
                {
                    idx0 = idx0,
                    idx1 = idx2,
                    k = strengK,
                    orignal_L = dis_idx0_idx2
                };
                var constrain5 = new constrain()
                {
                    idx0 = idx1,
                    idx1 = idx3,
                    k = strengK,
                    orignal_L = dis_idx1_idx3
                };
                constrains.Add(constrain0);
                constrains.Add(constrain1);
                constrains.Add(constrain2);
                constrains.Add(constrain3);
                constrains.Add(constrain4);
                constrains.Add(constrain5);
            }
        }
        Debug.Log("原始方法的弹簧个数"+constrains.Count);

    }

    void BuildConstrainWithSort()
    {
        int[] _E = new int[triangles.Length*2];
        for (int i=0; i<triangles.Length; i+=3) 
        {
            _E[i*2+0]=triangles[i+0];
            _E[i*2+1]=triangles[i+1];
            _E[i*2+2]=triangles[i+1];
            _E[i*2+3]=triangles[i+2];
            _E[i*2+4]=triangles[i+2];
            _E[i*2+5]=triangles[i+0];
        }
        //Reorder the original edge list
        for (int i=0; i<_E.Length; i+=2)
            if(_E[i] > _E[i + 1]) 
                Swap(ref _E[i], ref _E[i+1]);
        //Sort the original edge list using quicksort
        Quick_Sort (ref _E, 0, _E.Length/2-1);

        int e_number = 0;
        for (int i=0; i<_E.Length; i+=2)
            if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
                e_number++;

        int[] E = new int[e_number * 2];
        for (int i=0, e=0; i<_E.Length; i+=2)
            if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
            {
                E[e*2+0]=_E [i + 0];
                E[e*2+1]=_E [i + 1];
                e++;
            }

        // float[] L = new float[E.Length/2];
        // for (int e = 0; e < E.Length / 2; e++)
        // {
        //     int v0 = E[e * 2 + 0];
        //     int v1 = E[e * 2 + 1];
        //     L[e] = (vertices[v0] - vertices[v1]).magnitude;
        // }
        constrains = new List<constrain>();
        for (int e = 0; e < E.Length / 2; ++e)
        {
            int v0 = E[e * 2 + 0];
            int v1 = E[e * 2 + 1];
            Vector3 x0 = vertices[v0];
            Vector3 x1 = vertices[v1];
            float ori_l =  (x0 - x1).magnitude;
            var constrain = new constrain()
            {
                idx0 = v0,
                idx1 = v1,
                k = strengK,
                orignal_L = ori_l
            };
            constrains.Add(constrain);
        }
        //Debug.Log("排序方法的弹簧个数"+constrains.Count);
    }
    
    void GenerateMesh()
    {
        verts= new vert[nodeNum];
        vertices= new Vector3[nodeNum];
        UV = new Vector2[nodeNum];
        //generate vertices
        for (int idx = 0; idx < nodeNum; ++idx)
        {
            int j = idx / N;
            int i = idx - N * j;
            Vector3 vert = new Vector3(i * nodeGap, 0, j * nodeGap);
            vertices[idx] = vert;
            UV[idx] = new Vector3(i / (N - 1.0f), j / (N - 1.0f));

            verts[idx] = new vert
            {
                pos = vert,
                nextPos = vert,
                force = Vector3.zero,
                vel = Vector3.zero,
                uv_ = UV[idx],
                mass = mass
            };
        }
        //generate triangles
        triangles = new int[(N - 1) * (N - 1) * 6];
        int t = 0;
        for (int j = 0; j < N - 1; ++j)
        {
            for (int i = 0; i < N - 1; ++i)
            {
                int leftdown = i + N * j;
                int rightdown = i + 1 + N * j;
                int leftup = i + N * (j + 1);
                int rightup = i + 1 + N * (j + 1);
                triangles[t*6 + 0] = rightup;
                triangles[t*6 + 1] = leftup;
                triangles[t*6 + 2] = leftdown;
                triangles[t*6 + 3] = rightup;
                triangles[t*6 + 4] = leftdown;
                triangles[t*6 + 5] = rightdown;
                t++;
            }
        }
        mesh = new Mesh();
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.uv = UV;
        mesh.RecalculateNormals();
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        meshFilter.mesh = mesh;
    }

    void Quick_Sort(ref int[] a, int l, int r)
    {
        int j;
        if(l<r)
        {
            j=Quick_Sort_Partition(ref a, l, r);
            Quick_Sort (ref a, l, j-1);
            Quick_Sort (ref a, j+1, r);
        }
    }

    int  Quick_Sort_Partition(ref int[] a, int l, int r)
    {
        int pivot_0, pivot_1, i, j;
        pivot_0 = a [l * 2 + 0];
        pivot_1 = a [l * 2 + 1];
        i = l;
        j = r + 1;
        while (true) 
        {
            do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
            do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
            if(i>=j)	break;
            Swap(ref a[i*2], ref a[j*2]);
            Swap(ref a[i*2+1], ref a[j*2+1]);
        }
        Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
        Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
        return j;
    }

    void Swap(ref int a, ref int b)
    {
        int temp = a;
        a = b;
        b = temp;
    }
    
    private void OnDestroy()
    {
        constrainBuffer?.Release();
        vertsBuffer?.Release();
        sphereCollisionBuffer?.Release();
    }
}
