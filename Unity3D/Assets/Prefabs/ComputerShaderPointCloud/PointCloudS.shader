Shader "Unlit/PointCloudS"
{
	Properties
	{
		_Tint("Tint", Color) = (0.5, 0.5, 0.5, 1)
		_PointSize("Point Sizee", Float) = 10.0
	}
		SubShader
	{
		Tags { "RenderType" = "Opaque" }
		LOD 100

		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma fragment frag
			#include "UnityCG.cginc"

			struct v2f
			{
				float4 col : COLOR0;
				float4 vertex : SV_POSITION;
				half psize: PSIZE;
			};
			half4 _Tint;
			StructuredBuffer<float3> PointPos;
			StructuredBuffer<float4> PointCol;
			half4 _PointSize;

			v2f vert(uint id : SV_VertexID)
			{
				v2f o;
				o.vertex = UnityObjectToClipPos(float4(PointPos[id], 1));
				o.col = PointCol[id];
				o.psize = _PointSize;
				return o;
			}

			half4 frag(v2f i) : SV_Target
			{
				return i.col;
			}
			ENDCG
		}
	}
}
