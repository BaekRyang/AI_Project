using System;
using System.Collections;
using System.Collections.Generic;
using Movements;
using UnityEngine;
using UnityEngine.EventSystems;

[Serializable]
public class MovingCreature : Creature
{
    public Movements.Movements movements;       //이동 관련 함수들을 모아놓은 클래스
    public Vector2             velocity;        //속도
    public float               speed;           //속력
    public Vector2             heading;         //방향 벡터 (Normalize)
    public Vector2             side;            //Heading에 수직 벡터
    public float               mass        = 1; //질량
    public float               maxSpeed    = 1; //최대 속도
    public float               maxForce    = 1; //최대 힘
    public float               maxTurnRate = 1; //최대 회전 속도

    [SerializeField] private Sprite[]   serializeSprite;
    public                   Sprite[,] movingSprites = new Sprite[4,4]; 

    public Dictionary<string, LineRenderer> lineRenderers = new Dictionary<string, LineRenderer>();
    public GameObject                       lineRendererPrefab;

    private SpriteRenderer spriteRenderer;

    private void Awake()
    {
        lineRendererPrefab = Resources.Load("Prefabs/LineRenderer") as GameObject;

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 3; j++)
                movingSprites[i,j] = serializeSprite[i * 3 + j];
        
        spriteRenderer = GetComponent<SpriteRenderer>();
    }

    private void Update()
    {
        //heading에 따라서 스프라이트를 바꾼다.
        //아래 왼쪽 오른쪽 위 순서로 스프라이트가 저장되어 있다.
        //2차원 배열의 첫번째는 방향, 두번째는 움직임 표현
        //그러므로 두번째 배열은 무조건 1로 사용
        if (Math.Abs(heading.x) > Math.Abs(heading.y))
        {
            //x 방향으로의 움직임이 y 방향보다 크다.
            if (heading.x > 0)
                spriteRenderer.sprite = movingSprites[2,1];
            else
                spriteRenderer.sprite = movingSprites[1,1];
        }
        else
        {
            //y 방향으로의 움직임이 x 방향보다 크거나 같다.
            if (heading.y > 0)
                spriteRenderer.sprite = movingSprites[3,1];
            else
                spriteRenderer.sprite = movingSprites[0,1];
        }

    }

    public void CreateRenderers(List<SteeringType> lists)
    {
        //현재 생성된 라인 렌더러 오브젝트들을 삭제한다.
        foreach (var (key, value) in lineRenderers)
        {
            DestroyImmediate(value.gameObject);
        }

            //list안에 있는 타입들의 이름으로 라인 렌더러 오브젝트를 생성한다.
        foreach (var item in lists)
        {
            GameObject lineRenderer = Instantiate(lineRendererPrefab, transform.position, Quaternion.identity, transform);
            lineRenderer.name = item.ToString();
            lineRenderer.transform.SetParent(transform);
            lineRenderers.Add(item.ToString(), lineRenderer.GetComponent<LineRenderer>());
        }
    }

    public void UpdateForwardLine(string steeringType, Vector2 target, Color color)
    {
        if (!lineRenderers.TryGetValue(steeringType, out LineRenderer lineRenderer))
            return;
        
        //라인렌더러를 현재 위치에서 타겟 위치까지 그린다.
        lineRenderer.SetPosition(0, transform.position);
        lineRenderer.SetPosition(1, (Vector2)transform.position + target);
        
        //라인렌더러의 색상을 설정한다.
        lineRenderer.startColor = color;
        lineRenderer.endColor   = color;
        
        if(steeringType == "Seek" || steeringType == "Flee")
            if (lineRenderers.TryGetValue("Wander", out LineRenderer wanderLineRenderer))
                wanderLineRenderer.gameObject.SetActive(false);

        if (steeringType == "Wander")
        {
            if (lineRenderers.TryGetValue("Seek", out LineRenderer seekLineRenderer))
                seekLineRenderer.gameObject.SetActive(false);
            if (lineRenderers.TryGetValue("Flee", out LineRenderer fleeLineRenderer))
                fleeLineRenderer.gameObject.SetActive(false);
        }
        
        lineRenderers[steeringType].gameObject.SetActive(true);
    }

    private void FixedUpdate()
    {
        Vector2 steeringForce = movements.Calculate(); //이동 관련 함수들을 모아놓은 클래스에서 계산한 힘을 받아온다.
        Vector2 acceleration  = steeringForce / mass;  //가속도 = 힘 / 질량

        velocity += acceleration * Time.deltaTime;              //속도 = 가속도 * 시간
        velocity =  Vector2.ClampMagnitude(velocity, maxSpeed); //C++에서는 Truncate를 사용했지만, C#에서는 ClampMagnitude를 사용한다.
        speed    =  velocity.magnitude;                         //속력 = 속도의 크기

        transform.position += (Vector3)velocity * Time.deltaTime; //위치 = 속도 * 시간

        if (velocity.sqrMagnitude > 0.0000001f) //속도가 0이 아닐 때
        {
            heading = velocity.normalized;
            side    = Vector2.Perpendicular(heading);
        }


        //맵 밖으로 나가면 반대편으로 나오게 한다.
        if (transform.position.x > 36)
        {
            transform.position = new Vector2(-36, transform.position.y);
        }
        else if (transform.position.x < -36)
        {
            transform.position = new Vector2(36, transform.position.y);
        }
        else if (transform.position.y > 20)
        {
            transform.position = new Vector2(transform.position.x, -20);
        }
        else if (transform.position.y < -20)
        {
            transform.position = new Vector2(transform.position.x, 20);
        }
    }

    private Vector2 wallHitPosition = Vector2.zero;
    private Vector2 agentPosition;
    private float   radius;

    public void OnDrawGizmos()
    {
        Gizmos.color = Color.black;
        Gizmos.DrawWireSphere(wallHitPosition, .3f);
    }

    public void DrawWallHit(Vector2 position)
    {
        wallHitPosition = position;
    }
    
}