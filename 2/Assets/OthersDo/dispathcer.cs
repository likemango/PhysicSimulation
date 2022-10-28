using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;
using Unity.Mathematics;

// public class cloth_param
// {
//     public static int numIter = 10;
//     public static float kStretch = 0.25f;
//     public static Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
//     public static float timeStep = 1.0f / 60.0f;
//     public static float globalDamping = 0.998f;
//     public static Vector3 ballCenter = new Vector3(0.0f, -4.0f, 3.0f);
//     public static float ballRadius = 2.0f;
// }
struct ClothData
{
    public Vector3 pos;
}
class DistanceConstraint
{
    public DistanceConstraint(int i1, int i2, Vector3 position1, Vector3 posiiton2, float k_)
    {
        index1 = i1;
        index2 = i2;

        pos1 = position1;
        pos2 = posiiton2;

        restLength = math.length(pos1 - pos2);

        k = k_;
        kPrime = k;
        //kPrime = 1.0f - math.pow((1.0f - k), 1.0f / cloth_param.numIter);
    }

    public int index1;
    public int index2;
    Vector3 pos1;
    Vector3 pos2;
    float k;
    public float kPrime;
    public float restLength;
}
class Cloth
{
    public Vector3[] nodePos;
    public Vector3[] nodeVel;
    public Vector3[] nodePredPos;
    public Vector3[] nodeForce;
    public float[] nodeMass;
    public float[] nodeInvMass;
    public List<DistanceConstraint> disConstraintList;
    ClothData[] dataForDraw;
    public Test_1 cloth_param;
    public int numNode;
    public int N;
    
    //Vector3[] x_sum ;
   // int[] x_num ;

    public GameObject[] nodeList;
    private GameObject instance_cloth;
    public Cloth(int N, Vector3 startPos, float nodeStep, float density,Test_1 param)
    {
        instance_cloth = new GameObject();
        instance_cloth.gameObject.name = "instance_cloth";
        this.cloth_param = param;
        // it shoule note that j + N * i = index
        this.N = N;
        numNode = N * N;
        

        
        nodeList = new GameObject[numNode];
        // now init the nodePos
        nodePos = new Vector3[numNode];
        for (int index = 0; index < numNode; index++)
        {
            int i = index / N;
            int j = index - N * i;
            // Debug.Log("i = "+i);
            // Debug.Log("j = " + j);
            // Debug.Log("-----------");
            Vector3 tmpPos = new Vector3(i * nodeStep, 0.0f, j * nodeStep);
            nodePos[index] = tmpPos + startPos;

            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            obj.transform.position = nodePos[index];
            obj.transform.localScale = new Vector3(0.1f,0.1f,0.1f);
            obj.transform.parent = instance_cloth.transform;
            nodeList[index] = obj;
        }

        // now init the nodePredPos nodeFroce and nodeVel
        nodePredPos = new Vector3[numNode];
        nodeVel = new Vector3[numNode];
        nodeForce = new Vector3[numNode];
        for (int index = 0; index < numNode; index++)
        {
            nodePredPos[index] = new Vector3(0.0f, 0.0f, 0.0f);
            nodeVel[index] = new Vector3(0.0f, 0.0f, 0.0f);
            nodeForce[index] = new Vector3(0.0f, 0.0f, 0.0f);
        }

        //now init the nodeMass, there are (N-1) * (N-1) Trapeziums, and we deal with (N-1) * (N-1) * 2 triangles
        nodeMass = new float[numNode];
        nodeInvMass = new float[numNode];
        // for (int i = 0; i < N - 1; i++)
        // {
        //     for (int j = 0; j < N - 1; j++)
        //     {
        //         int index0 = j + i * N;
        //         int index1 = j + (i + 1) * N;
        //         int index2 = j + 1 + (i + 1) * N;
        //         int index3 = j + 1 + i * N;
        //
        //         float tMass = nodeStep * nodeStep * 0.5f;
        //         nodeMass[index0] += tMass * 2.0f / 3.0f;
        //         nodeMass[index1] += tMass * 1.0f / 3.0f;
        //         nodeMass[index2] += tMass * 2.0f / 3.0f;
        //         nodeMass[index3] += tMass * 1.0f / 3.0f;
        //     }
        // }

        for (int index = 0; index < numNode; index++)
        {
            nodeMass[index] = nodeStep * nodeStep;
            // if (index < N /*&& Mathf.Abs(nodeMass[index] - 0.02f * 2 / 3.0f) < float.Epsilon*/)
            // {
            //     Debug.Log(nodeMass[index]);
            // }
            nodeInvMass[index] = 1.0f / nodeMass[index];
        }
        // fix two points, so the mass should be inf 
        nodeInvMass[0] = 0.0f;
        nodeInvMass[N - 1] = 0.0f;

        // now we add distance constraints
        disConstraintList = new List<DistanceConstraint>();
        for (int i = 0; i < N - 1; i++)
        {
            for (int j = 0; j < N - 1; j++)
            {
                int index0 = j + i * N;
                int index1 = j + (i + 1) * N;
                int index2 = j + 1 + (i + 1) * N;
                int index3 = j + 1 + i * N;

                var tDisConstraint0 = new DistanceConstraint(index0, index1, nodePos[index0], nodePos[index1], cloth_param.kStretch);
                var tDisConstraint1 = new DistanceConstraint(index3, index2, nodePos[index3], nodePos[index2], cloth_param.kStretch);
                var tDisConstraint2 = new DistanceConstraint(index1, index2, nodePos[index1], nodePos[index2], cloth_param.kStretch);
                var tDisConstraint3 = new DistanceConstraint(index0, index3, nodePos[index0], nodePos[index3], cloth_param.kStretch);
                var tDisConstraint4 = new DistanceConstraint(index0, index2, nodePos[index0], nodePos[index2], cloth_param.kStretch);
                var tDisConstraint5 = new DistanceConstraint(index1, index3, nodePos[index1], nodePos[index3], cloth_param.kStretch);

                disConstraintList.Add(tDisConstraint0);
                disConstraintList.Add(tDisConstraint1);
                disConstraintList.Add(tDisConstraint2);
                disConstraintList.Add(tDisConstraint3);
                disConstraintList.Add(tDisConstraint4);
                disConstraintList.Add(tDisConstraint5);
            }
        }
        Debug.Log(disConstraintList.Count);
        // now init the size of cloth data
        dataForDraw = new ClothData[(N - 1) * (N - 1) * 12];
    }

    public ClothData[] getColthDrawData()
    {
        for (int i = 0; i < N - 1; i++)
        {
            for (int j = 0; j < N - 1; j++)
            {
                int index = j + i * (N - 1);

                int index0 = j + i * N;
                int index1 = j + (i + 1) * N;
                int index2 = j + 1 + (i + 1) * N;
                int index3 = j + 1 + i * N;

                dataForDraw[index * 12 + 0].pos = nodePos[index0];
                dataForDraw[index * 12 + 1].pos = nodePos[index1];

                dataForDraw[index * 12 + 2].pos = nodePos[index1];
                dataForDraw[index * 12 + 3].pos = nodePos[index2];

                dataForDraw[index * 12 + 4].pos = nodePos[index2];
                dataForDraw[index * 12 + 5].pos = nodePos[index3];

                dataForDraw[index * 12 + 6].pos = nodePos[index3];
                dataForDraw[index * 12 + 7].pos = nodePos[index0];

                dataForDraw[index * 12 + 8].pos = nodePos[index0];
                dataForDraw[index * 12 + 9].pos = nodePos[index2];

                dataForDraw[index * 12 + 10].pos = nodePos[index1];
                dataForDraw[index * 12 + 11].pos = nodePos[index3];
            }
        }
        return dataForDraw;
    }

    public void updateStep(Vector3 ballPos)
    {
        calculateGravity();
        for (int i = 0; i < cloth_param.numIter; i++)
        {
            updateConstraints();
        }
        collisionDetect(ballPos);
        integrate();
    }

    void calculateGravity()
    {
        for (int i = 0; i < numNode; i++)
        {
            nodeForce[i] = new Vector3(0.0f, 0.0f, 0.0f);

            if (nodeInvMass[i] > 0)
            {
                nodeForce[i] += cloth_param.gravity * nodeMass[i];
            }
        }

        for (int i = 0; i < numNode; i++)
        {
            nodeVel[i] *= cloth_param.globalDamping;
            nodeVel[i] = nodeVel[i] + (nodeForce[i] * nodeInvMass[i] * cloth_param.timeStep);
        }

        for (int i = 0; i < numNode; i++)
        {
            if (nodeInvMass[i] <= 0.0f)
            {
                nodePredPos[i] = nodePos[i];
            }
            else
            {
                nodePredPos[i] = nodePos[i] + (nodeVel[i] * cloth_param.timeStep);
            }
        }
    }

    void updateConstraints()
    {

        Vector3[] x_sum = new Vector3[numNode];
        int[] x_num = new int[numNode];
        
        foreach (DistanceConstraint tConstraint in disConstraintList)
        {
            int index1 = tConstraint.index1;
            int index2 = tConstraint.index2;
        
            Vector3 dirVec = nodePredPos[index1] - nodePredPos[index2];
            float len = math.length(dirVec);
        
            float w1 = nodeInvMass[index1];
            float w2 = nodeInvMass[index2];
        
            Vector3 dP = (dirVec / len) * (1.0f / (w1 + w2) * (len - tConstraint.restLength) * tConstraint.kPrime);
        
            if (w1 > 0.0f)
            {
                // if (!cloth_param.isGuass_Seidel)
                // {
                    x_sum[index1] += (nodePredPos[index1] - dP * w1);
                    x_num[index1]++;
                //}
                // else
                // {
                //     nodePredPos[index1] -= dP * w1;
                // }
            }
        
            if (w2 > 0.0f)
            {
                //if (!cloth_param.isGuass_Seidel)
                //{
                    x_sum[index2] += (nodePredPos[index2] + dP * w2);
                    x_num[index2]++;
               // }
                // else
                // {
                //     nodePredPos[index2] += dP * w2;
                // }
            }
        }
        //if (!cloth_param.isGuass_Seidel)
        //{
            for (int i = 0; i < numNode; i++)
                nodePredPos[i] = (x_sum[i] + 0.2f * nodePredPos[i]) / (x_num[i] + 0.2f);
       // }
    }

    void collisionDetect(Vector3 ballPos)
    {
        for (int i = 0; i < numNode; i++)
        {
            Vector3 gapVec = nodePredPos[i] - ballPos;
            float gapLen = math.length(gapVec);

            if (gapLen < cloth_param.ballRadius)
            {
                nodePredPos[i] += gapVec.normalized * (cloth_param.ballRadius - gapLen);
                nodePos[i] = nodePredPos[i];
            }
        }
    }
    void integrate()
    {
        for (int i = 0; i < numNode; i++)
        {
            nodeVel[i] = (nodePredPos[i] - nodePos[i]) / cloth_param.timeStep;
            nodePos[i] = nodePredPos[i];
        }
    }
}
public class dispathcer : MonoBehaviour
{
    // Start is called before the first frame update
    int N;
    bool flag = false;

    //ClothData[] dataForDraw; // (N-1) * (N-1) * 12
    //ComputeBuffer cBufferDataForDraw;
    Cloth cloth;
    GameObject sphere;

    public Material mainMaterial;
    public Test_1 param;
    //public Mesh mesh;
    //RenderParams renderParams = new RenderParams();

    void Start()
    {
        sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        //sphere.transform.parent = this.transform;
        sphere.transform.position = new Vector3(param.ballCenter.x, param.ballCenter.y, param.ballCenter.z);
        sphere.transform.localScale = new Vector3(param.ballRadius * 2.0f, param.ballRadius * 2.0f,param.ballRadius * 2.0f);

        //GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        //mesh = cube.GetComponent<MeshFilter>().mesh;
        cloth = new Cloth(32, new Vector3(0.0f, 0.0f, 0.0f), 0.2f, 1.0f,param);
        N = cloth.N;

        //dataForDraw = new ClothData[(N - 1) * (N - 1) * 12];
        //dataForDraw = cloth.getColthDrawData();

        //cBufferDataForDraw = new ComputeBuffer((N - 1) * (N - 1) * 12, 12);
        //cBufferDataForDraw.SetData(dataForDraw, 0, 0, (N - 1) * (N - 1) * 12);
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyUp(KeyCode.Space))
        {
            flag = true;
        }
        if (flag)
        {
            //flag = false;
            cloth.updateStep(sphere.transform.position);
            for (int i = 0; i < cloth.numNode; ++i)
            {
                cloth.nodeList[i].transform.position = cloth.nodePos[i];
            }
            //dataForDraw = cloth.getColthDrawData();
            //cBufferDataForDraw.SetData(dataForDraw, 0, 0, (N - 1) * (N - 1) * 12);
        }
    }
    // private void OnRenderObject()
    // {
    //     mainMaterial.SetBuffer("_clothDataBuffer", cBufferDataForDraw);
    //     mainMaterial.SetPass(0);
    //     Graphics.DrawProceduralNow(MeshTopology.Lines, (cloth.N - 1) * (cloth.N - 1) * 12);
    // }

    private void OnDestroy()
    {
        // if (cBufferDataForDraw != null)
        //     cBufferDataForDraw.Release();
    }
}
