using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(menuName = "MyScriptableObject")]
public class Test_1 : ScriptableObject
{
    public bool isGuass_Seidel;
    public  int numIter = 10;
    public  float kStretch = 0.25f;
    public  Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
    public  float timeStep = 1.0f / 60.0f;
    public  float globalDamping = 0.998f;
    public  Vector3 ballCenter = new Vector3(0.0f, -4.0f, 3.0f);
    public  float ballRadius = 2.0f;
}
