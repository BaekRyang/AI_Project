using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class MovingCreature : Creature
{
    public Movements.Movements movements;       //이동 관련 함수들을 모아놓은 클래스
    public Vector2             velocity;        //속도
    public Vector2             heading;         //방향 벡터 (Normalize)
    public Vector2             side;            //Heading에 수직 벡터
    public float               mass        = 1; //질량
    public float               maxSpeed    = 1; //최대 속도
    public float               maxForce    = 1; //최대 힘
    public float               maxTurnRate = 1; //최대 회전 속도

    private void Update()
    {
        Vector2 steeringForce = movements.Calculate(); //이동 관련 함수들을 모아놓은 클래스에서 계산한 힘을 받아온다.
        Vector2 acceleration  = steeringForce / mass;  //가속도 = 힘 / 질량
        
        velocity += acceleration * Time.deltaTime;              //속도 = 가속도 * 시간
        velocity =  Vector2.ClampMagnitude(velocity, maxSpeed); //C++에서는 Truncate를 사용했지만, C#에서는 ClampMagnitude를 사용한다.
        
        transform.position += (Vector3)velocity * Time.deltaTime; //위치 = 속도 * 시간

        if (velocity.sqrMagnitude > 0.0000001f) //속도가 0이 아닐 때
        {
            heading = velocity.normalized;
            side    = Vector2.Perpendicular(heading);
        }

    }
    
    private Vector2 drawPos = Vector2.zero;
    public void OnDrawGizmos()
    {
        Gizmos.color = Color.black;
        Gizmos.DrawWireSphere(drawPos, .3f);
    }

    public void Draw(Vector2 position)
    {
        drawPos = position;
    }
}