using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using Microsoft.MixedReality.Toolkit.UI;
using UnityEngine.UIElements;

public class SliderController : MonoBehaviour
{
    private Move move;
    private GameObject translationSlider;
    private GameObject rotationSlider;
    private PinchSlider tSlider;
    private PinchSlider rSlider;
    private TextMesh tText;
    private TextMesh rText;

    private void Start()
    {
        move = GetComponent<Move>();
        translationSlider = transform.Find("TranslationSlider").gameObject;
        rotationSlider = transform.Find("RotationSlider").gameObject;
        tSlider = translationSlider.GetComponent<PinchSlider>();
        rSlider = rotationSlider.GetComponent<PinchSlider>();
        tText = translationSlider.transform.Find("Label").gameObject.GetComponent<TextMesh>();
        rText = rotationSlider.transform.Find("Label").gameObject.GetComponent<TextMesh>();
    }
    public void UpdateTranslation()
    {
        move.translationDistance = tSlider.SliderValue * tSlider.SliderValue * 0.1f;
        tText.text = "Translation Distance: " + move.translationDistance.ToString("F3");
    }

    public void UpdateRotation()
    {
        move.rotationDegree = rSlider.SliderValue * rSlider.SliderValue * 10;
        rText.text = "Rotation Degree: " + move.rotationDegree.ToString("F3");
    }
}




