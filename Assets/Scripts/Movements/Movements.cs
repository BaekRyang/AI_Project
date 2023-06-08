using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Random = UnityEngine.Random;

namespace Movements
{
    public enum AgentType
    {
        Evader,
        Chaser
    }

    public enum SteeringType
    {
        Seek,
        Flee,
        Wander,
        Arrive,
        Pursuit,
        Evade,
        Cohesion,
        Separation,
        Alignment,
        ObstacleAvoidance,
        WallAvoidance,
        FollowPath,
        Interpose,
        Hide,
        OffsetPursuit,
        Flocking
    }

    [Serializable]
    public class Movements
    {
        protected MovingCreature agent;

        public delegate Vector2 SteeringFunction();

        public                                        int                steeringFlags = 0;
        [FormerlySerializedAs("Flags In Bit")] public string             steeringFlagsBit;
        [FormerlySerializedAs("Flags List")]   public List<SteeringType> steeringFlagsList = new List<SteeringType>();

        public List<SteeringType> pSteeringFlagList
        {
            get => steeringFlagsList;
            set => steeringFlagsList = value;
        }

        public bool UpdateData;

        public  bool                 busy;
        private List<MovingCreature> neighbors;
        private List<MovingCreature> chaser;

        // List<MovingCreature> neighbors      = new List<MovingCreature>(); //인수로 바로 넘겨주는중
        public float neighborRadius = 10f;

        //Seek

        //Flee
        [SerializeField] private float panicDistance = 15f;

        //Evade
        [SerializeField] private bool weightedEvade;

        //Wander
        [SerializeField] private Vector2 wanderTarget;
        [SerializeField] private float   wanderRadius   = 1f;
        [SerializeField] private float   wanderDistance = .1f;
        [SerializeField] private float   wanderJitter   = 10f;

        //Arrive
        public enum Deceleration
        {
            Slow   = 3,
            Normal = 2,
            Fast   = 1
        }

        //WallAvoidance
        public float wallDetectionFeelerLength = 2f;
        public float wallDetectionFeelerAngle  = 45f;
        public float sideFeelerLengthMult      = .75f;

        //Interpose
        private const Deceleration globalDeceleration = Deceleration.Normal;

        public IEnumerator CheckAgent()
        {
            while (true)
            {
                neighbors = GetNeighbors(neighborRadius);
                chaser    = GetChasers(panicDistance);

                yield return new WaitForSeconds(0.1f);
            }
        }

        public Movements(Creature creature, AgentType agentType)
        {
            UpdateData = false;

            pSteeringFlagList = pSteeringFlagList;

            if (creature is MovingCreature movingCreature)
                //Creature가 MovingCreature를 상속받았을 때만 agent를 초기화한다.
            {
                agent = movingCreature;

                //공통적인 SteeringFlags
                BitFlag.SetFlag((int)SteeringType.WallAvoidance, true, ref steeringFlags);
                BitFlag.SetFlag((int)SteeringType.Wander, true, ref steeringFlags);
                BitFlag.SetFlag((int)SteeringType.Separation, true, ref steeringFlags);

                switch (agentType) //에이전트의 타입에 따라서 다른 SteeringFlags를 사용한다.
                {
                    case AgentType.Evader:
                        BitFlag.SetFlag((int)SteeringType.Evade, true, ref steeringFlags);
                        BitFlag.SetFlag((int)SteeringType.Flee, true, ref steeringFlags);
                        BitFlag.SetFlag((int)SteeringType.Alignment, true, ref steeringFlags);
                        BitFlag.SetFlag((int)SteeringType.Cohesion, true, ref steeringFlags);
                        break;

                    case AgentType.Chaser:
                        neighborRadius = 20f;
                        BitFlag.SetFlag((int)SteeringType.Pursuit, true, ref steeringFlags);
                        BitFlag.SetFlag((int)SteeringType.Seek, true, ref steeringFlags);
                        break;
                }

                //steeringFlagsBit에 steeringFlags를 비트로 표현하여 저장
                steeringFlagsBit = BitFlag.ConvertToString(steeringFlags, 16);

                //플래그 리스트에 활성화된 플래그들 저장
                BitFlag.SetList(steeringFlags, ref steeringFlagsList);

                agent.CreateRenderers(steeringFlagsList);
            }
        }

        public Vector2 Calculate()
        {
            if (UpdateData)
            {
                agent.velocity = Vector2.zero; //속도 초기화

                steeringFlags = BitFlag.ConvertListToBitFlag(steeringFlagsList); //리스트를 비트로 변환

                if (steeringFlagsList.Count != 0)
                {
                    //리스트를 순서대로 정렬
                    steeringFlagsList.Sort((a, b) => (int)a - (int)b);

                    //동일 요소 제거
                    for (int i = 0; i < steeringFlagsList.Count - 1; i++)
                    {
                        if (steeringFlagsList[i] == steeringFlagsList[i + 1])
                        {
                            steeringFlagsList.RemoveAt(i);
                            i--;
                        }
                    }
                }

                steeringFlagsBit = BitFlag.ConvertToString(steeringFlags, 16);

                agent.CreateRenderers(steeringFlagsList);

                Debug.Log("Data Changed");
                UpdateData = false;
            }

            Vector2 steeringForce = Vector2.zero;

            if (agent is Evader)
                busy = chaser.Count > 0;
            else if (agent is Chaser)
                busy = neighbors.Count > 0;

            //SteeringFlags를 이용해 사용할 함수를 선택한다.
            foreach (SteeringType type in Enum.GetValues(typeof(SteeringType)))
            {
                int flag = 1 << (int)type;
                if ((steeringFlags & flag) != 0)
                {
                    switch (type)
                    {
                        case SteeringType.Seek:
                            break;
                        case SteeringType.Flee:
                            break;
                        case SteeringType.Wander:
                            if (!busy)
                                steeringForce += Wander() * 1f;
                            break;
                        case SteeringType.Arrive:
                            break;
                        case SteeringType.Pursuit:
                            if (busy)
                                steeringForce += Pursuit(GetClosestEvader(neighbors)) * 1.5f;
                            break;
                        case SteeringType.Evade:
                            steeringForce += Evade(chaser, weightedEvade) * 3f;
                            break;
                        case SteeringType.Cohesion:
                            steeringForce += Cohesion(neighbors) * 1f;
                            break;
                        case SteeringType.Separation:
                            steeringForce += Separation(neighbors) * 1f;
                            break;
                        case SteeringType.Alignment:
                            steeringForce += Alignment(neighbors) * 1f;
                            break;
                        case SteeringType.ObstacleAvoidance:
                            break;
                        case SteeringType.WallAvoidance:
                            steeringForce += WallAvoidance() * 3f;
                            break;
                        case SteeringType.FollowPath:
                            break;
                        case SteeringType.Interpose:
                            break;
                        case SteeringType.Hide:
                            break;
                        case SteeringType.OffsetPursuit:
                            break;
                        case SteeringType.Flocking:
                            break;
                    }
                }
            }

            return steeringForce;
        }

        public Vector2 Seek(Vector2 targetPosition)
        {
            Vector2 desiredVelocity = (targetPosition - (Vector2)agent.transform.position).normalized * agent.maxSpeed;
            //가고자 하는 방향으로 향하는 벡터에 최대 속도를 곱한다.

            agent.UpdateForwardLine("Seek", desiredVelocity - agent.velocity, Color.blue);

            return desiredVelocity - agent.velocity;
            //현재 에이전트에서 가려고 하는 방향벡터 방향으로 틀어주는 벡터을 반환한다.
        }

        public Vector2 Flee(Vector2 targetPosition)
        {
            //PanicDistance보다 멀리 있으면 Flee를 하지 않는다. 
            if (Vector2.Distance(agent.transform.position, targetPosition) >
                panicDistance)       //Vector2에는 Arg를 2개받는 SquareMagnitude가 없다.
                return Vector2.zero; //(Guard clause)

            Vector2 desiredVelocity = ((Vector2)agent.transform.position - targetPosition).normalized * agent.maxSpeed;
            //Seek이랑 정 반대로 목표 지점에서 도망치는 벡터를 구한다.

            agent.UpdateForwardLine("Flee", desiredVelocity - agent.velocity, Color.black);

            return desiredVelocity - agent.velocity;
            //현재 에이전트에서 가려고 하는 방향벡터 방향으로 틀어주는 벡터을 반환한다.
        }

        public Vector2
            Arrive(Vector2 targetPosition, Deceleration deceleration) //이것도 Seek이랑 비슷하지만 목표 지점에 Graceful하게 도착한다.
        {
            Vector2 toTarget = targetPosition - (Vector2)agent.transform.position;
            //Target으로 가는 벡터를 구한다.

            float distance = toTarget.magnitude;
            //Target으로의 거리를 구한다.

            if (!(distance > 0))
                return Vector2.zero; //거리가 0보다 크지 않으면 0을 반환한다. (Guard clause)

            float decelerationTweaker = 0.3f;
            //감속을 조절하는 변수 (클수록 빨리 멈춘다.)

            float speed = distance / ((float)deceleration * decelerationTweaker);
            //목표 지점과 현재 위치의 거리를 감속 변수로 나눈다.

            speed = Mathf.Min(speed, agent.maxSpeed);
            //최대 속도를 넘지 않도록 한다.

            Vector2 desiredVelocity = toTarget * speed / distance; //Normalize 하기위해 거리로 나눈다.
            //목표 지점으로 향하는 벡터를 구한다. distance = toTarget.magnitude이므로 나누게 되면 방향벡터 * 속도가 된다.

            agent.UpdateForwardLine("Arrive", desiredVelocity - agent.velocity, Color.green);

            return desiredVelocity - agent.velocity;
            //현재 에이전트에서 도착 하려는 방향으로 틀어주는 벡터을 반환한다.
        }

        public Vector2 Pursuit(MovingCreature evader)
        {
            Vector2 toEvader = evader.transform.position - agent.transform.position;
            //Evader로 가는 벡터를 구한다.

            float relativeHeading = Vector2.Dot(agent.heading, evader.heading);
            //Evader와의 상대적인 Heading을 구한다.
            //Agent와 evader의 Heading을 내적한다. 내적은 두 벡터의 방향이 얼마나 비슷한지를 나타낸다. (두 벡터가 같은 방향이면 1, 반대 방향이면 -1, 수직이면 0)

            if (Vector2.Dot(toEvader, agent.transform.up) > 0 &&   //Evader가 앞에 있는가? (180도 이내),
                relativeHeading                           < -.95f) //서로 반대 방향을 향하고 있는가? = 마주보고 있는가? (-.95f는 18도 정도)
            {
                return Seek(evader.transform.position);
                //Evader가 앞에 있고, 18도 이내로 마주보고 있으면 그냥 해당 위치로 바로 이동하기 위해 Seek한다.
            }

            float lookAheadTime =
                toEvader.magnitude / (agent.maxSpeed + evader.velocity.magnitude); //velocity.magnitude는 속도
            //Evader가 Agent를 향해 가는데 걸리는 시간을 구한다.

            //wallDetectionFeelerLength를 evader가 가까울수록 짧게 만들어준다.
            wallDetectionFeelerLength = Mathf.Min(wallDetectionFeelerLength, toEvader.magnitude);

            return Seek((Vector2)evader.transform.position + evader.velocity * lookAheadTime);
            //evader의 위치에서 evader의 속도 * lookAheadTime만큼 떨어진 위치로 Seek한다.
        }

        public Vector2 Evade(MovingCreature pursuer)
        {
            Vector2 toPursuer = pursuer.transform.position - agent.transform.position;
            //Pursuer로 가는 벡터를 구한다.

            float lookAheadTime = toPursuer.magnitude / (agent.maxSpeed + pursuer.velocity.magnitude);
            //Pursuer가 Agent를 향해 가는데 걸리는 시간을 구한다.

            return Flee((Vector2)pursuer.transform.position + pursuer.velocity * lookAheadTime);
            //Pursuer의 위치에서 Pursuer의 속도 * lookAheadTime만큼 떨어진 위치로 Flee한다.
        }

        public Vector2 Evade(List<MovingCreature> pursuers, bool byWeight) //평균을 통해서 가장 안전해 보이는곳으로 이동
        {
            Vector2 finalEvade = Vector2.zero;
            if (byWeight)
            {
                Vector2 cumulativeEvade = Vector2.zero;
                float   totalWeight     = 0;

                foreach (var pursuer in pursuers)
                {
                    Vector2 toPuruser = pursuer.transform.position - agent.transform.position;
                    //Pursuer로 가는 벡터를 구한다.

                    float distance = toPuruser.magnitude;
                    //Pursuer로의 거리를 구한다.

                    float weight = 1.0f / (distance + 0.001f); //0으로 나누는 것을 방지하기 위해 0.001을 더한다.
                    //가중치를 구한다. (거리가 가까울수록 가중치가 높다.)

                    float lookAheadTime = distance / (agent.maxSpeed + pursuer.velocity.magnitude);
                    //Pursuer가 Agent를 향해 가는데 걸리는 시간을 구한다.

                    Vector2 evade = Flee((Vector2)pursuer.transform.position + pursuer.velocity * lookAheadTime);

                    cumulativeEvade += evade * weight;
                    //해당 값을 누적시킨다.

                    totalWeight += weight;
                    //가중치를 누적시킨다.
                }

                if (pursuers.Count > 0)
                {
                    finalEvade = cumulativeEvade / totalWeight;
                }
            }
            else
            {
                Vector2 cumulativeEvade = Vector2.zero;

                foreach (var pursuer in pursuers)
                {
                    Vector2 toPuruser = pursuer.transform.position - agent.transform.position;
                    //Pursuer로 가는 벡터를 구한다.
                    float lookAheadTime = toPuruser.magnitude / (agent.maxSpeed + pursuer.velocity.magnitude);
                    //Pursuer가 Agent를 향해 가는데 걸리는 시간을 구한다.
                    Vector2 evade = Flee((Vector2)pursuer.transform.position + pursuer.velocity * lookAheadTime);
                    //Pursuer의 위치에서 Pursuer의 속도 * lookAheadTime만큼 떨어진 위치로 Flee한다.
                    cumulativeEvade += evade;
                    //해당 값을 누적시킨다.
                }

                if (pursuers.Count > 0)
                {
                    finalEvade = cumulativeEvade / pursuers.Count;
                }
            }

            Debug.DrawLine(agent.transform.position, agent.transform.position + (Vector3)finalEvade, Color.green);
            return finalEvade;
        }

        public Vector2 Wander()
        {
            float jitterThisTimeSlice = wanderJitter * Time.deltaTime;
            //Wander의 변화량을 구한다. (wanderJitter * Time.deltaTime)

            wanderTarget += new Vector2(Random.Range(-1f, 1f) * jitterThisTimeSlice,
                                        Random.Range(-1f, 1f) * jitterThisTimeSlice);
            //Wander의 변화량을 wanderTarget에 더한다.

            wanderTarget.Normalize();
            //wanderTarget을 Normalize한다.

            wanderTarget *= wanderRadius;
            //wanderTarget의 크기를 wanderRadius로 한다.

            Vector2 targetLocal = wanderTarget + new Vector2(wanderDistance, 0);
            //wanderTarget을 wanderDistance만큼 떨어진 위치로 한다.

            Vector2 targetWorld = agent.transform.TransformPoint(targetLocal);
            //targetLocal을 agent의 위치를 기준으로 한다.

            Debug.DrawLine(agent.transform.position, targetWorld, Color.red); //어디로 가는지 Visualize

            agent.UpdateForwardLine("Wander", targetWorld - (Vector2)agent.transform.position, Color.red);

            return targetWorld - (Vector2)agent.transform.position;
            //현재 에이전트에서 wanderTarget으로 향하는 벡터를 반환한다.
        }

        //벽 피하기
        public Vector2 WallAvoidance()
        {
            Feeler[] feelers = CreateFeeler();
            Vector2  normal  = Vector2.zero;

            float distToClosestIP = float.MaxValue;
            //가장 가까운 벽과의 거리를 구하기 위한 변수를 선언한다.

            Vector2 closestPoint = Vector2.zero;
            //가장 가까운 벽의 위치를 저장할 변수를 선언한다.

            int detectedindex = 0;
            for (var index = 0; index < feelers.Length; index++)
            {
                var          feeler = feelers[index];
                RaycastHit2D hit    = feeler.Cast(agent.transform.position, 1 << 6);
                //agent의 위치에서 feeler 방향으로 wallDetectionFeelerLength만큼 떨어진 위치에 wallLayerMask를 가진 오브젝트가 있는지 확인한다.
                //각 feeler를 Draw
                Debug.DrawLine(agent.transform.position, feeler.forwarding, Color.cyan);
                if (hit.collider != null)
                {
                    if (agent is MovingCreature movingCreature)
                    {
                        movingCreature.DrawWallHit(hit.point);
                    }

                    var distToThisIP = Vector2.Distance(agent.transform.position, hit.point);
                    //agent와 벽과의 거리를 구한다.

                    if (distToThisIP < distToClosestIP)
                    {
                        detectedindex   = index;
                        distToClosestIP = distToThisIP;
                        //가장 가까운 벽과의 거리를 저장한다.

                        closestPoint = hit.point;
                        normal       = hit.normal;
                        //가장 가까운 벽의 위치를 저장한다.
                    }
                }
            }

            if (distToClosestIP == float.MaxValue)
            {
                return Vector2.zero;
                //가장 가까운 벽과의 거리가 float.MaxValue이면 벽이 없다는 뜻이므로 Vector2.zero를 반환한다.
            }

            var overShoot = ((feelers[detectedindex].forwarding * feelers[detectedindex].detectLength) - closestPoint);
            //가장 가까운 벽의 위치에서 feeler 방향으로 feeler의 길이만큼 떨어진 위치를 구한다.
            //= feeler가 벽을 뚫고 들어간 만큼의 벡터

            //Vector2 fromAgentToClosestPoint = closestPoint - (Vector2)agent.transform.position;
            //Vector2 avoidanceForce          = normal * (1 / fromAgentToClosestPoint.magnitude);
            //이 방식은 미는 힘이 너무 약해서 벽에 잘 부딛힘

            Vector2 avoidanceForce = normal * overShoot.magnitude;
            return avoidanceForce;
            //overShoot을 반환한다.
        }


        public Feeler[] CreateFeeler()
        {
            Vector2  temp    = Vector2.zero;
            Feeler[] feelers = new Feeler[3];

            feelers[0] = new Feeler((Vector2)agent.transform.position + wallDetectionFeelerLength * agent.heading,
                                    wallDetectionFeelerLength);

            temp = Quaternion.AngleAxis(wallDetectionFeelerAngle, Vector3.forward) * agent.heading;
            feelers[1] = new Feeler(
                (Vector2)agent.transform.position + wallDetectionFeelerLength * sideFeelerLengthMult * temp,
                wallDetectionFeelerLength * sideFeelerLengthMult);

            temp = Quaternion.AngleAxis(-wallDetectionFeelerAngle, Vector3.forward) * agent.heading;
            feelers[2] = new Feeler(
                (Vector2)agent.transform.position + wallDetectionFeelerLength * sideFeelerLengthMult * temp,
                wallDetectionFeelerLength * sideFeelerLengthMult);

            return feelers;
        }


        public Vector2 Interpose(MovingCreature agentA, MovingCreature agentB)
        {
            Vector2 midPoint = (Vector2)agentA.transform.position +
                               ((Vector2)agentB.transform.position - (Vector2)agentA.transform.position) / 2;
            //두 Agent의 중간 지점을 구한다.

            float timeToReachMidPoint = Vector2.Distance(agent.transform.position, midPoint) / agent.maxSpeed;
            //Agent가 중간 지점에 도달하는데 걸리는 시간을 구한다.

            Vector2 aPos = (Vector2)agentA.transform.position + agentA.velocity * timeToReachMidPoint;
            //AgentA의 위치에서 AgentA의 속도 * timeToReachMidPoint만큼 떨어진 위치를 구한다.

            Vector2 bPos = (Vector2)agentB.transform.position + agentB.velocity * timeToReachMidPoint;

            midPoint = (aPos + bPos) / 2;
            //위치를 구한다.

            return Arrive(midPoint, globalDeceleration);
            //Arrive한다.
        }

        // public Vector2 Hide(MovingCreature target, List<MovingCreature> obstacles)
        // {
        //     float distToClosest = float.MaxValue;
        //     Vector2 bestHidingSpot = Vector2.zero;
        //
        //     foreach (MovingCreature obstacle in obstacles)
        //     {
        //         Vector2 hidingSpot = GetHidingPosition(obstacle.transform.position, obstacle.radius, target.transform.position);
        //         float dist = Vector2.Distance(agent.transform.position, hidingSpot);
        //
        //         if (dist < distToClosest)
        //         {
        //             distToClosest = dist;
        //             bestHidingSpot = hidingSpot;
        //         }
        //     }
        //
        //     if (distToClosest == float.MaxValue)
        //     {
        //         return Evade(target);
        //     }
        //
        //     return Arrive(bestHidingSpot, globalDeceleration);
        // }
        //
        // private Vector2 GetHidingPosition(Vector2 obstaclePos, float obstacleRadius, Vector2 targetPos)
        // {
        //     float distanceFromBoundary = 25f;
        //     float distAway = obstacleRadius + distanceFromBoundary;
        //
        //     Vector2 dir = (obstaclePos - targetPos).normalized;
        //
        //     return (dir * distAway) + obstaclePos;
        // }

        private Vector2 Separation(List<MovingCreature> neighbors)
        {
            Vector2 steeringForce = Vector2.zero;

            foreach (MovingCreature neighbor in neighbors)
            {
                if (neighbor != agent && neighbor != null)
                {
                    if (neighbor.GetType() == agent.GetType())
                    {
                        Vector2 toAgent = agent.transform.position - neighbor.transform.position;

                        if (toAgent.magnitude != 0) //두 객체가 같은 위치에 있으면 오류발생함
                            steeringForce += toAgent.normalized / toAgent.magnitude;
                    }
                }
            }

            return steeringForce;
        }

        private Vector2 Alignment(List<MovingCreature> neighbors)
        {
            Vector2 averageHeading = Vector2.zero;

            foreach (MovingCreature neighbor in neighbors)
            {
                if (neighbor != agent && neighbor != null)
                {
                    if (neighbor.GetType() == agent.GetType())
                    {
                        averageHeading += neighbor.heading;
                    }
                }
            }

            if (neighbors.Count > 0)
            {
                averageHeading /= neighbors.Count;
                averageHeading -= agent.heading;
            }

            return averageHeading;
        }

        private Vector2 Cohesion(List<MovingCreature> neighbors)
        {
            Vector2 centerOfMass  = Vector2.zero;
            Vector2 steeringForce = Vector2.zero;

            foreach (MovingCreature neighbor in neighbors)
            {
                if (neighbor != agent && neighbor != null)
                {
                    if (neighbor.GetType() == agent.GetType())
                    {
                        centerOfMass += (Vector2)neighbor.transform.position;
                    }
                }
            }

            if (neighbors.Count > 0)
            {
                centerOfMass  /= neighbors.Count;
                steeringForce =  Seek(centerOfMass);
            }

            return steeringForce;
        }



        private List<MovingCreature> GetNeighbors(float radius) //주변의 Agent들을 구한다.
        {
            Collider2D[]         hitColliders = Physics2D.OverlapCircleAll(agent.transform.position, radius);
            List<MovingCreature> neighbors    = new List<MovingCreature>();

            foreach (Collider2D hitCollider in hitColliders)
            {
                if (!hitCollider.TryGetComponent(out MovingCreature neighbor) ||
                    neighbor is not Evader                                    ||
                    neighbor == agent) continue;
                //hitCollider가 MovingCreature가 아니거나 Evader가 아니거나 자기 자신이면 넘어간다.

                neighbors.Add(neighbor);
            }

            return neighbors;
        }

        private List<MovingCreature> GetChasers(float radius) //바라보는 방향에 Chaser가 있는지 찾는다.
        {
            Collider2D[] hitColliders = Physics2D.OverlapCircleAll(agent.transform.position, radius);
            //radius 안에 있는 Chaser를 구한다.

            List<MovingCreature> chasers = new List<MovingCreature>();

            foreach (Collider2D hitCollider in hitColliders) //hitCollider를 돌면서 Chaser를 구한다.
            {
                if (!hitCollider.TryGetComponent(out MovingCreature chaser) ||
                    chaser is not Chaser) continue;
                //hitCollider가 MovingCreature가 아니거나 Chaser가 아니면 넘어간다.

                //Chase가 heading 방향 140도 안에 있는지 내적을 해서 구한다.
                // if (Vector2.Dot(agent.heading, (chaser.transform.position - agent.transform.position).normalized) >
                //     Mathf.Cos(140f * Mathf.Deg2Rad))
                {
                    chasers.Add(chaser);
                }

                // if (Vector2.Angle(agent.heading, chaser.transform.position - agent.transform.position) < 140f) 각도를 사용하는 방법
            }

            return chasers; //Chaser를 반환한다.
        }

        private MovingCreature GetClosestEvader(List<MovingCreature> evaders)
        {
            float          closestDist   = float.MaxValue;
            MovingCreature closestEvader = null;

            foreach (MovingCreature evader in evaders)
            {
                float dist = Vector2.Distance(agent.transform.position, evader.transform.position);

                if (dist < closestDist)
                {
                    closestDist   = dist;
                    closestEvader = evader;
                }
            }

            return closestEvader;
        }
    }
}