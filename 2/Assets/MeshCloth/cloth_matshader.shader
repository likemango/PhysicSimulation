Shader "XY/cloth_matshader"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100
        Cull Off
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
            };
            
            struct vertice
            {
                float3 pos;
                float3 nextpos;
                float3 force;
                float3 vel;
                float2 uv_;
                float mass;
            };

            sampler2D _MainTex;
            float4 _MainTex_ST;

            StructuredBuffer<vertice> verts;

            //v2f vert (appdata v)
            v2f vert (uint id : SV_VertexID)
            {
                v2f o;  
                o.vertex = UnityObjectToClipPos(float4(verts[id].pos, 1.0f));
                o.uv = TRANSFORM_TEX(verts[id].uv_, _MainTex);
                return o;
            }  

            fixed4 frag (v2f i) : SV_Target
            {
                // sample the texture
                fixed4 col = tex2D(_MainTex, i.uv);
                return col;
            }
            ENDCG
        }
    }
}
