using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

public class MicroManiPicoTeleop : MonoBehaviour
{
    [Header("MicroMani Gateway")]
    public string gatewayHost = "127.0.0.1";
    public int gatewayPort = 49100;
    public float sendRateHz = 30f;

    private UdpClient udp;
    private IPEndPoint endpoint;
    private float nextSendTime;
    private int sequence;

    void OnEnable()
    {
        udp = new UdpClient();
        endpoint = new IPEndPoint(IPAddress.Parse(gatewayHost), gatewayPort);
    }

    void OnDisable()
    {
        if (udp != null)
        {
            udp.Close();
            udp = null;
        }
    }

    void Update()
    {
        if (udp == null || Time.unscaledTime < nextSendTime)
        {
            return;
        }
        nextSendTime = Time.unscaledTime + 1f / Mathf.Max(1f, sendRateHz);

        InputDevice left = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        InputDevice right = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        string payload = BuildPacket(left, right);
        byte[] bytes = Encoding.UTF8.GetBytes(payload);
        udp.Send(bytes, bytes.Length, endpoint);
        sequence++;
    }

    private string BuildPacket(InputDevice left, InputDevice right)
    {
        return "{"
            + JsonPair("source", "pico4_unity") + ","
            + JsonPair("sequence", sequence) + ","
            + JsonPair("timestamp", Time.unscaledTime) + ","
            + "\"left\":" + HandJson(left) + ","
            + "\"right\":" + HandJson(right) + ","
            + "\"buttons\":{"
            + JsonPair("reset", Button(left, CommonUsages.primaryButton) || Button(right, CommonUsages.primaryButton)) + ","
            + JsonPair("openGrippers", Button(left, CommonUsages.secondaryButton)) + ","
            + JsonPair("closeGrippers", Button(right, CommonUsages.secondaryButton))
            + "}"
            + "}";
    }

    private string HandJson(InputDevice device)
    {
        Vector2 stick = Axis2(device, CommonUsages.primary2DAxis);
        return "{"
            + "\"primary2DAxis\":" + Vec2Json(stick) + ","
            + JsonPair("trigger", Axis1(device, CommonUsages.trigger)) + ","
            + JsonPair("grip", Axis1(device, CommonUsages.grip)) + ","
            + JsonPair("primaryButton", Button(device, CommonUsages.primaryButton)) + ","
            + JsonPair("secondaryButton", Button(device, CommonUsages.secondaryButton))
            + "}";
    }

    private static Vector2 Axis2(InputDevice device, InputFeatureUsage<Vector2> usage)
    {
        return device.TryGetFeatureValue(usage, out Vector2 value) ? value : Vector2.zero;
    }

    private static float Axis1(InputDevice device, InputFeatureUsage<float> usage)
    {
        return device.TryGetFeatureValue(usage, out float value) ? value : 0f;
    }

    private static bool Button(InputDevice device, InputFeatureUsage<bool> usage)
    {
        return device.TryGetFeatureValue(usage, out bool value) && value;
    }

    private static string Vec2Json(Vector2 value)
    {
        return "{"
            + JsonPair("x", value.x) + ","
            + JsonPair("y", value.y)
            + "}";
    }

    private static string JsonPair(string key, string value)
    {
        return "\"" + key + "\":\"" + value + "\"";
    }

    private static string JsonPair(string key, int value)
    {
        return "\"" + key + "\":" + value.ToString(CultureInfo.InvariantCulture);
    }

    private static string JsonPair(string key, float value)
    {
        return "\"" + key + "\":" + value.ToString("R", CultureInfo.InvariantCulture);
    }

    private static string JsonPair(string key, bool value)
    {
        return "\"" + key + "\":" + (value ? "true" : "false");
    }
}
