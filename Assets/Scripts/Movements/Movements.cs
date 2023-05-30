using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Random = UnityEngine.Random;

[Serializable]
public class Movements
{
    protected Creature agent;
    
    //Wander
    public Vector2 wanderTarget;
    public float wanderRadius = 1f;
    public float wanderDistance = .1f;
    public float wanderJitter = 10f;

    public Vector2 Seek(Vector2 targetPosition)
    {
        Vector2 desiredVelocity = (targetPosition - (Vector2)agent.transform.position).normalized * agent.maxSpeed;
        return desiredVelocity - agent.rigidbody.velocity;
    }

    public Movements(Creature creature)
    {
        agent = creature;
    }

    public Vector2 Wander()
    {
        Debug.Log("Wander");

        float wanderJitterTimeSlice = wanderJitter * Time.deltaTime;

        // 먼저, 작은 무작위 벡터를 목표 위치에 추가합니다.
        wanderTarget += new Vector2(Random.Range(-1f, 1f) * wanderJitterTimeSlice, Random.Range(-1f, 1f) * wanderJitterTimeSlice);

        // 새로운 벡터를 단위 원에 다시 투영합니다
        wanderTarget.Normalize();

        // 벡터의 길이를 원형 헤링의 반지름과 동일하게 증가시킵니다
        wanderTarget *= wanderRadius;

        // 목표를 에이전트 앞에 WanderDist만큼 이동합니다
        Vector2 targetLocal = wanderTarget + new Vector2(wanderDistance, 0);

        // 목표를 월드 공간으로 투영합니다
        Vector3 targetWorldPosition = agent.transform.TransformPoint(targetLocal);

        // 목표 방향으로 조향합니다
        Vector2 steeringForce = targetWorldPosition - agent.transform.position;

        // 시각적으로 경로를 표시하기 위해 디버그 라인을 그립니다.
        Debug.DrawLine(agent.transform.position, targetWorldPosition, Color.red);

        return steeringForce;
    }


    
}
