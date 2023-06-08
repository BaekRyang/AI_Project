using System;
using System.Collections.Generic;
using System.Text;
using Movements;
using Unity.VisualScripting;
using UnityEngine.UI;

public class BitFlag
{
    public static void SetFlag(int bit, bool value, ref int origin)
    {
        if (bit < 0 || bit >= sizeof(int) * 8) //flag가 short의 비트 범위를 벗어나면 return
            return;

        int mask = 1 << bit;

        if (value)
            origin |= mask;
        else
            origin &= ~mask;
    }

    public static int ConvertToBitFlag(string value)
    {
        //value가 0과 1로 이루어진 문자열인지 확인한다.
        foreach (char c in value)
            if (c != '0' && c != '1')
                return default;

        int bitFlag  = 0;
        int bitCount = value.Length;

        for (int i = 0; i < bitCount; i++)
            bitFlag |= (value[i] == '1' ? 1 : 0) << i;

        return bitFlag;
    }

    public static string ConvertToString(int value, int bitCount)
    {
        StringBuilder bitFlag = new StringBuilder();

        for (int i = 0; i < bitCount; i++)
            bitFlag.Append((value & (1 << i)) != 0 ? "1" : "0");

        return bitFlag.ToString();
    }
    
    public static void SetList<T>(int bitFlags, ref List<T> list)
    {
        list = new List<T>();
        
        for (int i = 0; i < Enum.GetValues(typeof(T)).Length; i++)
        {
            if ((bitFlags & (1 << i)) != 0)
                list.Add((T) Enum.ToObject(typeof(T), i));
        }
    }
    
    public static int ConvertListToBitFlag<T>(List<T> list)
    {
        int bitFlag = 0;

        foreach (T item in list)
            bitFlag |= 1 << Convert.ToInt16(item);

        return bitFlag;
    }
    
}