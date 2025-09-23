Shader "Custom/MyPointCloudShader"
{
    Properties
    {
        _PointSize("Point Size", Float) = 30.0
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma geometry geom
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float4 color : COLOR;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float4 color : COLOR;
            };

            float _PointSize;

            void geom(point appdata IN[1], inout PointStream<v2f> stream)
            {
                v2f OUT;
                OUT.pos = UnityObjectToClipPos(IN[0].vertex);
                OUT.pos.w = _PointSize;  // 控制点大小
                OUT.color = IN[0].color;
                stream.Append(OUT);
            }

            v2f vert(appdata v)
            {
                v2f o;
                o.pos = v.vertex;
                o.color = v.color;
                return o;
            }

            half4 frag(v2f i) : SV_Target
            {
                return i.color;
            }
            ENDCG
        }
    }
    FallBack "Diffuse"
}