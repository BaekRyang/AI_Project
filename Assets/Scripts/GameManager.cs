using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Cinemachine;
using MoreMountains.Feedbacks;
using TMPro;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Rendering.Universal;
using UnityEngine.UI;

public class GameManager : MonoBehaviour
{
    public static           GameManager              Instance;
    [SerializeField] public CinemachineVirtualCamera zoomInCamera;
    public                  MMF_Player               MMF_Player_Operation;
    public                  MMF_Player               MMF_Player_Controller;
    public                  bool                     UIOpened;


    public Image   dragRectImage;     //드래그 범위를 표시할 UI 이미지
    public Vector2 dragStartPosition; //드래그 시작 지점
    public Vector2 dragEndPosition;   //드래그 끝 지점

    public Vector2 dragStartPos;
    public Vector2 dragEndPos;

    public List<Creature> capturedCreatures = new List<Creature>();

    public TMP_Text[]    captureTexts;
    bool                 captureOpend = false;
    public RectTransform steeringViewRect;
    public GameObject    steeringPrefab;


    private void Awake() => Instance ??= this;

    [Serializable]
    public enum ButtonActions
    {
        InstantiateChaser,
        InstantiateEvader,
        DestroyCreature,
        AddBehavior
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Tab))
        {
            if (MMF_Player_Operation.IsPlaying)
                return;

            if (UIOpened)
            {
                MMF_Player_Operation.GetFeedbackOfType<MMF_RectTransformOffset>().OffsetMinRemapOne =
                    new Vector2(-400, 0);
                MMF_Player_Operation.GetFeedbackOfType<MMF_RectTransformOffset>().OffsetMaxRemapOne =
                    new Vector2(-400, 0);
                MMF_Player_Operation.Initialization();
                MMF_Player_Operation.PlayFeedbacks();
            }
            else
            {
                MMF_Player_Operation.GetFeedbackOfType<MMF_RectTransformOffset>().OffsetMinRemapOne =
                    new Vector2(400, 0);
                MMF_Player_Operation.GetFeedbackOfType<MMF_RectTransformOffset>().OffsetMaxRemapOne =
                    new Vector2(400, 0);
                MMF_Player_Operation.Initialization();
                MMF_Player_Operation.PlayFeedbacks();
            }

            UIOpened = !UIOpened;
        }

        if (Input.GetMouseButtonDown(0))
        {
            //드래그 시작
            dragStartPosition = Input.mousePosition;
            dragStartPos      = Camera.main.ScreenToWorldPoint(dragStartPosition);
            dragEndPosition   = dragStartPosition;
        }
        else if (Input.GetMouseButton(0))
        {
            //드래그 중
            dragEndPosition = Input.mousePosition;
            UpdateDragRect();
        }
        else if (Input.GetMouseButtonUp(0))
        {
            dragEndPos = Camera.main.ScreenToWorldPoint(dragEndPosition);
            //드래그 종료
            dragStartPosition = Vector2.zero;
            dragEndPosition   = Vector2.zero;
            ClearDragRect();

            //화면에 Raycast를 해서 UI가 있는지 확인
            PointerEventData pointer = new PointerEventData(EventSystem.current);
            pointer.position = Input.mousePosition;
            List<RaycastResult> raycastResults = new List<RaycastResult>();
            EventSystem.current.RaycastAll(pointer, raycastResults);
            if (raycastResults.Count > 0)
            {
                //UI가 있으면 드래그를 무시
                return;
            }

            CheckObjectsInDragArea();
        }
    }

    private void UpdateDragRect()
    {
        //드래그 범위를 표시
        Vector2 dragMin = new Vector2(Mathf.Min(dragStartPosition.x, dragEndPosition.x),
                                      Mathf.Min(dragStartPosition.y, dragEndPosition.y));
        Vector2 dragMax = new Vector2(Mathf.Max(dragStartPosition.x, dragEndPosition.x),
                                      Mathf.Max(dragStartPosition.y, dragEndPosition.y));

        dragRectImage.gameObject.SetActive(true);
        dragRectImage.rectTransform.position  = dragMin;
        dragRectImage.rectTransform.sizeDelta = dragMax - dragMin;
    }

    private void CheckObjectsInDragArea()
    {
        if (capturedCreatures.Count > 0)
        {
            foreach (var creature in capturedCreatures)
            {
                creature.GetComponent<Light2D>().intensity = 0;
            }

            capturedCreatures.Clear();
        }
        
        int layerMask = 1 << LayerMask.NameToLayer("Chaser") | 1 << LayerMask.NameToLayer("Evader");

        //스크린의 width와 height를 가져온다.
        float screenWidth  = Screen.width;
        float screenHeight = Screen.height;

        Collider2D[] hitColliders = Physics2D.OverlapAreaAll(dragStartPos, dragEndPos, layerMask);

        foreach (Collider2D collider in hitColliders)
        {
            //Colider타입이 캡슐인것만 가져온다.
            if (collider.GetType() != typeof(CapsuleCollider2D))
                continue;
            Debug.Log(collider.gameObject.name);
            capturedCreatures.Add(collider.gameObject.GetComponent<Creature>());
            collider.GetComponent<Light2D>().intensity = 10;
        }

        captureTexts[0].text = "Evader : 0";
        captureTexts[1].text = "Chaser : 0";
        if (capturedCreatures.Count == 0)
        {
            if (captureOpend)
            {
                MMF_Player_Controller.GetFeedbackOfType<MMF_RectTransformOffset>().OffsetMinRemapOne =
                    new Vector2(400, 0);
                MMF_Player_Controller.GetFeedbackOfType<MMF_RectTransformOffset>().OffsetMaxRemapOne =
                    new Vector2(400, 0);
                MMF_Player_Controller.Initialization();
                MMF_Player_Controller.PlayFeedbacks();
            }

            captureOpend = false;
            return;
        }

        if (!captureOpend)
        {
            MMF_Player_Controller.GetFeedbackOfType<MMF_RectTransformOffset>().OffsetMinRemapOne = new Vector2(-400, 0);
            MMF_Player_Controller.GetFeedbackOfType<MMF_RectTransformOffset>().OffsetMaxRemapOne = new Vector2(-400, 0);
            MMF_Player_Controller.Initialization();
            MMF_Player_Controller.PlayFeedbacks();
        }

        captureOpend = true;
        int evader = 0, chaser = 0;

        foreach (var creature in capturedCreatures)
        {
            if (creature is Evader)
                evader++;
            else
                chaser++;
        }

        captureTexts[0].text = $"Evader : {evader}";
        captureTexts[1].text = $"Chaser : {chaser}";

        //모든 크리쳐의 Movement의 SteeringFlags가 같다면
        if (capturedCreatures.All(x => x.GetComponent<MovingCreature>().movements.steeringFlags ==
                                       capturedCreatures[0].GetComponent<MovingCreature>().movements.steeringFlags))
        {
            //steeringviewrect 자식 모두 삭제
            foreach (Transform child in steeringViewRect)
            {
                Destroy(child.gameObject);
            }

            var flags = capturedCreatures[0].GetComponent<MovingCreature>().movements.steeringFlagsList;


            //flags 안에 있는 SteeringFlags를 모두 가져온다.
            foreach (var flag in flags)
            {
                var newSteering = Instantiate(steeringPrefab, steeringViewRect);
                var text        = newSteering.GetComponent<TMP_Text>();
                text.text = flag.ToString() + "\n";
            }
        }
    }


    private void ClearDragRect()
    {
        // 드래그 범위 UI 이미지 비활성화
        dragRectImage.gameObject.SetActive(false);
    }

    public void ButtonEvent(int buttonType)
    {
        ButtonActions buttonAction = (ButtonActions)buttonType;

        switch (buttonAction)
        {
            case ButtonActions.InstantiateChaser:
                Instantiate(Resources.Load("Prefabs/Chaser"), Vector3.zero, Quaternion.identity);
                break;
            case ButtonActions.InstantiateEvader:
                Instantiate(Resources.Load("Prefabs/Evader"), Vector3.zero, Quaternion.identity);
                break;
            case ButtonActions.DestroyCreature:
                capturedCreatures.ForEach(x => Destroy(x.gameObject));
                dragStartPos = dragEndPos = Vector2.zero;
                CheckObjectsInDragArea();
                capturedCreatures.Clear();
                break;
            case ButtonActions.AddBehavior:

                break;
        }
    }
}