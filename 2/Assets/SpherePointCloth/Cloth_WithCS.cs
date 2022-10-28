using System;
using System.Collections;
using System.Collections.Generic;
using JetBrains.Annotations;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;

public struct node
{
    public Vector3 pos;
    public Vector3 nextpos;
    public Vector3 vel;
    public Vector3 force;
    public float mass;
};

public struct strain
{
    public int idx0;
    public int idx1;
    public float k;
    public float orignal_L;
    
    public strain(int idx0,int idx1,float k, float orignal_L)
    {
        this.idx0 = idx0;
        this.idx1 = idx1;
        this.orignal_L = orignal_L;
        this.k = k;
    }
};

public struct gravity
{
    public Vector3 g;

    public gravity(Vector3 g)
    {
        this.g = g;
    }
};

public class Cloth_WithCS : MonoBehaviour
{
    public bool useCS;
    public int N;
    public float nodeGap = 0.2f;
    public ComputeShader cloth_cs;
    public float numIter;
    public float k = 3;
    public float mass = 0.04f;
    //public Test_1 param;
    public float damping;
    public float timeStep = 0.0167f;

    private GameObject[] nodeList;
    private int nodeNum;

    private List<strain> strains;
    private node[] nodes;
    private gravity[] g_;
    
    private ComputeBuffer nodeBuffer;
    private ComputeBuffer strainBuffer;
    private ComputeBuffer gBuffer;
    
    private ComputeBuffer x_sum_buffer;
    private ComputeBuffer x_num_buffer;

    private int kernel_0;
   // private int kernel_1_1;
   // private int kernel_1_2;
    private int kernel_1_2_1;
  //  private int kernel_2;
    private int kernel_3;

    private Vector3[] x_sum;
    private int[] x_num;

    private GameObject sphere;
    
    void Start()
    {
        CreateMesh();
        CreateStrain();
        CreateBuffer();
        InitKernel();
    }
    
    void Update()
    {
        if (useCS)
        {
            //Guass_Seidel方法无法使用CS并行

            //尝试Jacbei方法
            Profiler.BeginSample("caculate G");
            cloth_cs.Dispatch(kernel_0,4, 4,1);
            Profiler.EndSample();
            
            Profiler.BeginSample("caculate Constrain");
            for (int i = 0; i < numIter; ++i)
            {
                //x_num = new int[nodes.Length];
                //x_sum = new Vector3[nodes.Length];
                //cloth_cs.Dispatch(kernel_1_2,10 ,10 ,1);
                cloth_cs.Dispatch(kernel_1_2_1, 4, 4, 1);
            }
            Profiler.EndSample();
            
            Profiler.BeginSample("update info");
            cloth_cs.Dispatch(kernel_3,4,4, 1);  
            Profiler.EndSample();
            
            Profiler.BeginSample("GetData operate");
            nodeBuffer.GetData(nodes);
            Profiler.EndSample();
            //Debug.Log(nodes[5].pos);
            //Debug.Log(nodes[5].vel);
        }
        else
        {
            calculateGravity();
            for (int i = 0; i < numIter; ++i)
            {
                TestConstrain();
            }
            UpdateInfo();
            
            //Debug.Log(nodes[5].pos);
            //Debug.Log(nodes[5].vel);
        }
        for (int i = 0; i < nodeList.Length; ++i)
            nodeList[i].transform.position = nodes[i].pos;
    }
    
    void calculateGravity()
    {
        for (int i = 0; i < nodes.Length; i++)
        {
            if (i == 0 || i == N - 1)
                continue;
            nodes[i].vel *= damping;
            nodes[i].vel += g_[0].g  * timeStep;
            nodes[i].nextpos = nodes[i].pos;
            nodes[i].nextpos += (nodes[i].vel * timeStep);
        }
    }
    

    void TestConstrain()
    {
        // #region MyRegion
        // Vector3[] x_sum = new Vector3[nodes.Length];
        // int[] x_num = new int[nodes.Length];
        //
        // foreach (strain tConstraint in strains)
        // {
        //     int index1 = tConstraint.idx0;
        //     int index2 = tConstraint.idx1;
        //
        //     Vector3 dirVec = nodes[index1].nextpos - nodes[index2].nextpos;
        //     float len = math.length(dirVec);
        //
        //     float w1 = nodes[index1].mass;
        //     float w2 = nodes[index2].mass;
        //
        //     Vector3 dP = (dirVec / len) * (1.0f / (w1 + w2) * (len - tConstraint.orignal_L) * tConstraint.k);
        //
        //     if (index1 != 0 && index1 != N - 1)
        //     {
        //         // if (!cloth_param.isGuass_Seidel)
        //         // {
        //         x_sum[index1] += (nodes[index1].nextpos - dP * w1);
        //         x_num[index1]++;
        //         //}
        //         // else
        //         // {
        //         //     nodePredPos[index1] -= dP * w1;
        //         // }
        //     }
        //
        //     if (index2 != 0 && index2 != N - 1)
        //     {
        //         //if (!cloth_param.isGuass_Seidel)
        //         //{
        //         x_sum[index2] += (nodes[index2].nextpos + dP * w2);
        //         x_num[index2]++;
        //
        //         // }
        //         // else
        //         // {
        //         //     nodePredPos[index2] += dP * w2;
        //         // }
        //     }
        // }
        // //if (!cloth_param.isGuass_Seidel)
        // //{
        // for (int j = 0; j < nodes.Length; ++j) 
        //     nodes[j].nextpos = (x_sum[j] + 0.2f * nodes[j].nextpos) / (x_num[j] + 0.2f);
        // // }
        //
        // #endregion
       
        for (int i = 0; i < strains.Count; ++i)
        {
            strain s = strains[i];
            
            int idx0 = s.idx0;
            int idx1 = s.idx1;
            node node0 = nodes[idx0];
            node node1 = nodes[idx1];
        
            //nodes[idx0].pos = new float3(1, 1, 1);
            //nodes[idx1].pos = new float3(1, 1, 1);
        
            Vector3 dirVec = node0.nextpos - node1.nextpos; 
            float len = dirVec.magnitude;
            Vector3 dp = (dirVec / len) * (1.0f / (node0.mass + node1.mass) * (len - s.orignal_L) * s.k);
        
            if (idx0 != 0 && idx0 != N - 1)
            {
                //nodes[idx0].nextpos -= dp * node0.mass;
        
               x_sum[idx0] += (nodes[idx0].nextpos - dp * node0.mass);
               x_num[idx0] += 1;
            }
        
            if (idx1 != 0 && idx1 != N - 1)
            {
                //nodes[idx1].nextpos += dp * node1.mass;
                
                x_sum[idx1] += (nodes[idx1].nextpos + dp * node1.mass);
                x_num[idx1] += 1;
            } 
        }
        // Debug.Log(x_sum[5]);
        for (int j = 0; j < nodes.Length; ++j)
        {
            nodes[j].nextpos = (x_sum[j] + 0.2f * nodes[j].nextpos) / (x_num[j] + 0.2f);
            x_sum[j] = Vector3.zero;
            x_num[j] = 0;
        }
    }

    void UpdateInfo()
    {
        for (int i = 0; i < nodes.Length; ++i)
        {
            //nodes[i].nextpos = (x_sum[i] + 0.2f * nodes[i].nextpos) / (x_num[i] + 0.2f);
            nodes[i].vel = (nodes[i].nextpos - nodes[i].pos) / timeStep;
            nodes[i].pos = nodes[i].nextpos;
        }
    }
    

    // void CreateCollision()
    // {
    //     sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
    //     //sphere.transform.parent = this.transform;
    //     sphere.transform.position = new Vector3(param.ballCenter.x, param.ballCenter.y, param.ballCenter.z);
    //     sphere.transform.localScale = new Vector3(param.ballRadius * 2.0f, param.ballRadius * 2.0f,param.ballRadius * 2.0f);
    // }
    
    void InitKernel()
    {
        kernel_0 = cloth_cs.FindKernel("CaculateG");
       // kernel_1_2 = cloth_cs.FindKernel("CaculateStrainLimitingByJacobi");
        kernel_1_2_1 = cloth_cs.FindKernel("updateNodeNextPos");
        //kernel_2 = cloth_cs.FindKernel("CaculateCollision");
        kernel_3 = cloth_cs.FindKernel("updateNodeInfo");
        
        nodeBuffer.SetData(nodes);
        strainBuffer.SetData(strains);
        gBuffer.SetData(g_);
        
        x_sum = new Vector3[nodes.Length];
        x_num = new int[nodes.Length]; 
        x_num_buffer.SetData(x_num);
        x_sum_buffer.SetData(x_sum);
        
        //计算力(重力)

        cloth_cs.SetBuffer(kernel_0,"nodes",nodeBuffer);
        cloth_cs.SetBuffer(kernel_0,"g_",gBuffer);
        cloth_cs.SetFloat("damping",damping);
        cloth_cs.SetFloat("timeStep",timeStep);
        
        //PBD_guass_seidel
        //cloth_cs.SetBuffer(kernel_1_1,"nodes",nodeBuffer);
        //cloth_cs.SetBuffer(kernel_1_1,"strains",strainBuffer);
        //PBD_jacobi
      // cloth_cs.SetBuffer(kernel_1_2,"nodes",nodeBuffer);
      //  cloth_cs.SetBuffer(kernel_1_2,"strains",strainBuffer);
      //  cloth_cs.SetBuffer(kernel_1_2,"x_sum",x_sum_buffer);
      //  cloth_cs.SetBuffer(kernel_1_2,"x_num",x_num_buffer);
        
        cloth_cs.SetBuffer(kernel_1_2_1,"nodes",nodeBuffer);
        cloth_cs.SetBuffer(kernel_1_2_1,"strains",strainBuffer);
        cloth_cs.SetBuffer(kernel_1_2_1,"x_sum",x_sum_buffer);
        cloth_cs.SetBuffer(kernel_1_2_1,"x_num",x_num_buffer);
        
        //Ingore Collision
        //updateInfo
        cloth_cs.SetBuffer(kernel_3,"nodes",nodeBuffer);
        cloth_cs.SetFloat("timeStep",timeStep);
    }

    void CreateMesh()
    {
        nodeNum = N * N;
        nodeList = new GameObject[nodeNum];
        nodes = new node[nodeNum];
        for (int idx = 0; idx < nodeNum; ++idx)
        {
            int i = idx / N;
            int j = idx - i * N;
            Vector3 tmpPos = new Vector3(i * nodeGap, 0.0f, j * nodeGap);
            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            obj.transform.position = tmpPos;
            obj.transform.localScale = new Vector3(0.1f,0.1f,0.1f);
            obj.transform.parent = transform;
            obj.gameObject.name = "sphere_" + i + "_" + j;
            nodeList[idx] = obj;
            
            nodes[idx].force = new float3(0,0,0);
            nodes[idx].vel = new float3(0,0,0);
            nodes[idx].mass = mass;
            nodes[idx].pos = tmpPos;
            nodes[idx].nextpos = tmpPos;
        }
        g_ = new gravity[1];
        g_[0] = new gravity(new float3(0,-9.8f,0));
    }

    void CreateStrain()
    {
        strains = new List<strain>();
        for (int i = 0; i < N - 1; ++i)
        {
            for (int j = 0; j < N - 1; ++j)
            {
                int idx0 = j + i * N;
                int idx1 = j + (i + 1) * N;
                int idx2 = j + 1 +  (i + 1) * N;
                int idx3 = j + 1 + i * N;

                float dis_idx0_idx1 = math.length(nodes[idx0].pos - nodes[idx1].pos);
                float dis_idx3_idx2 = math.length(nodes[idx3].pos - nodes[idx2].pos);
                float dis_idx1_idx2 = math.length(nodes[idx1].pos - nodes[idx2].pos);
                float dis_idx0_idx3 = math.length(nodes[idx0].pos - nodes[idx3].pos);
                float dis_idx0_idx2 = math.length(nodes[idx0].pos - nodes[idx2].pos);
                float dis_idx1_idx3 = math.length(nodes[idx1].pos - nodes[idx3].pos);

                strains.Add(new strain(idx0,idx1,k,dis_idx0_idx1));
                strains.Add(new strain(idx3,idx2,k,dis_idx3_idx2));
                strains.Add(new strain(idx1,idx2,k,dis_idx1_idx2));
                strains.Add(new strain(idx0,idx3,k,dis_idx0_idx3));
                strains.Add(new strain(idx0,idx2,k,dis_idx0_idx2));
                strains.Add(new strain(idx1,idx3,k,dis_idx1_idx3));
            }
        }
    }

    void CreateBuffer()
    {
        nodeBuffer = new ComputeBuffer(nodes.Length,13 * sizeof(float));
        strainBuffer = new ComputeBuffer(strains.Count, 2 * sizeof(float) + 2 * sizeof(int));
        gBuffer = new ComputeBuffer(1,3 * sizeof(float));
        //

        x_sum_buffer = new ComputeBuffer(nodes.Length, 3 * sizeof(float));
        x_num_buffer = new ComputeBuffer(nodes.Length, 1 * sizeof(int));
    }

    private void OnDestroy()
    {
        if(nodeBuffer != null)
            nodeBuffer.Release();
        if(strainBuffer!=null)
            strainBuffer.Release();
        if (gBuffer != null)
            gBuffer.Release();
        if(x_sum_buffer!=null)
            x_sum_buffer .Release();
        if(x_num_buffer !=null)
            x_num_buffer.Release();
    }
}
