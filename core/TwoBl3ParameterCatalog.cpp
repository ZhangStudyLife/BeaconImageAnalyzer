#include "TwoBl3ParameterCatalog.h"

#include <QtMath>

#include <cstring>

namespace
{
constexpr quint8 Float32 = 0;
constexpr quint8 Int32 = 1;

TwoBl3ParameterDescriptor parameter(quint16 id,
                                    quint8 type,
                                    const char* name,
                                    const char* group,
                                    const char* effect,
                                    double step,
                                    bool tracking = false,
                                    bool searchable = true)
{
    TwoBl3ParameterDescriptor result;
    result.id = id;
    result.type = type;
    result.name = QString::fromLatin1(name);
    result.menuPath = QStringLiteral("2BL3 Img > %1").arg(QString::fromLatin1(group));
    result.effect = QString::fromUtf8(effect);
    result.step = step;
    result.tracking = tracking;
    result.searchable = searchable;
    return result;
}
}

const QVector<TwoBl3ParameterDescriptor>& TwoBl3ParameterCatalog::all()
{
    static const QVector<TwoBl3ParameterDescriptor> values = {
        parameter(0x0141, Int32, "bl3_beacon_thr", "Threshold", "提高可压制较暗亮点，降低可保留弱信标。", 1),
        parameter(0x0100, Int32, "bl3_edge_thr", "Threshold", "控制顶部和左右边缘区域的二值化灵敏度。", 1),
        parameter(0x0101, Int32, "bl3_track_thr", "Threshold", "控制已有信标轨迹的弱光补强门槛。", 1, true),
        parameter(0x0102, Int32, "bl3_lamp_thr", "Threshold", "控制车灯普通区域阈值，并间接影响信标遮罩。", 1),
        parameter(0x0103, Int32, "bl3_lamp_up_thr", "Threshold", "控制图像上部车灯检测阈值。", 1),
        parameter(0x0104, Float32, "bl3_lamp_up_y", "Threshold", "控制车灯上部低阈值区域的高度。", 0.5),
        parameter(0x0105, Float32, "bl3_bridge_gap", "Threshold", "控制车灯横向断点桥接距离。", 0.5),
        parameter(0x0106, Int32, "bl3_beacon_min", "Beacon Area", "提高会过滤小亮点，降低会接纳更小的信标。", 1),
        parameter(0x0107, Int32, "bl3_edge_min", "Beacon Area", "控制边缘信标允许的最小面积。", 1),
        parameter(0x0108, Int32, "bl3_top_max", "Beacon Area", "控制顶部信标允许的最大面积。", 1),
        parameter(0x0109, Int32, "bl3_edge_max", "Beacon Area", "控制左右边缘信标允许的最大面积。", 1),
        parameter(0x010a, Int32, "bl3_lamp_min", "Car Lamp", "控制车灯候选最小面积。", 1),
        parameter(0x010b, Int32, "bl3_lamp_max", "Car Lamp", "控制车灯候选最大面积。", 1),
        parameter(0x010d, Float32, "bl3_lamp_elong", "Car Lamp", "提高后只接受更细长的车灯候选。", 0.1),
        parameter(0x010f, Float32, "bl3_back_len", "Car Lamp", "控制后摄车灯最小长度。", 0.5),
        parameter(0x0121, Float32, "bl3_lamp_width", "Car Lamp", "控制常规车灯最小宽度。", 0.1),
        parameter(0x0122, Float32, "bl3_narrow_width", "Car Lamp", "控制窄车灯允许的最小宽度。", 0.1),
        parameter(0x0123, Float32, "bl3_narrow_elong", "Car Lamp", "控制窄车灯需要的最小长宽比。", 0.1),
        parameter(0x0124, Int32, "bl3_upper_area", "Car Lamp", "控制上部车灯最小面积。", 1),
        parameter(0x0125, Float32, "bl3_upper_len", "Car Lamp", "控制上部车灯最小长度。", 0.5),
        parameter(0x0126, Float32, "bl3_upper_width", "Car Lamp", "控制上部车灯最小宽度。", 0.1),
        parameter(0x0127, Float32, "bl3_compact_y", "Car Lamp", "控制上部紧凑车灯规则的起始位置。", 0.5),
        parameter(0x0128, Int32, "bl3_compact_area", "Car Lamp", "控制紧凑车灯最小面积。", 1),
        parameter(0x0129, Float32, "bl3_compact_len", "Car Lamp", "控制紧凑车灯最小长度。", 0.5),
        parameter(0x012a, Float32, "bl3_compact_width", "Car Lamp", "控制紧凑车灯最小宽度。", 0.1),
        parameter(0x012b, Float32, "bl3_compact_elong", "Car Lamp", "控制紧凑车灯最小长宽比。", 0.1),
        parameter(0x0110, Int32, "bl3_iso_gray", "Background", "提高后要求孤立小信标具有更高峰值。", 1),
        parameter(0x0111, Int32, "bl3_iso_bg", "Background", "控制孤立小信标允许的局部背景亮度。", 1),
        parameter(0x0112, Float32, "bl3_ring_in", "Background", "控制局部背景环内半径。", 0.5),
        parameter(0x0113, Float32, "bl3_ring_out", "Background", "控制局部背景环外半径。", 0.5),
        parameter(0x0114, Float32, "bl3_near_pad", "Near Lamp", "提高会扩大车灯附近的严格过滤范围。", 0.5),
        parameter(0x0115, Int32, "bl3_near_min", "Near Lamp", "控制车灯附近信标最小面积。", 1),
        parameter(0x0116, Int32, "bl3_near_gray", "Near Lamp", "控制车灯附近信标最小峰值灰度。", 1),
        parameter(0x0117, Int32, "bl3_near_bg", "Near Lamp", "控制车灯附近信标允许的背景亮度。", 1),
        parameter(0x012f, Float32, "bl3_vglare_elong", "Reflection", "控制细小竖直反光的最小长宽比。", 0.1),
        parameter(0x0130, Int32, "bl3_vglare_gray", "Reflection", "控制细小竖直反光规则的灰度上限。", 1),
        parameter(0x0131, Float32, "bl3_linear_elong", "Reflection", "控制长线状反光的最小长宽比。", 0.1),
        parameter(0x0132, Int32, "bl3_weak_c_thr", "Weak Center", "控制弱中心候选的二值化阈值。", 1),
        parameter(0x0133, Int32, "bl3_weak_c_min", "Weak Center", "控制弱中心候选最小面积。", 1),
        parameter(0x0134, Int32, "bl3_weak_c_max", "Weak Center", "控制弱中心候选最大面积。", 1),
        parameter(0x0135, Int32, "bl3_weak_c_gray", "Weak Center", "控制弱中心候选最小峰值灰度。", 1),
        parameter(0x0136, Int32, "bl3_weak_c_bg", "Weak Center", "控制弱中心候选最大背景亮度。", 1),
        parameter(0x0137, Int32, "bl3_shape_min", "Shape Filter", "控制启用形状过滤的最小面积。", 1),
        parameter(0x0138, Float32, "bl3_shape_ratio", "Shape Filter", "控制信标包围盒允许的最大长宽比。", 0.1),
        parameter(0x0139, Int32, "bl3_shape_fill", "Shape Filter", "控制普通候选最小填充率。", 1),
        parameter(0x013a, Int32, "bl3_shape_s_fill", "Shape Filter", "控制小候选最小填充率。", 1),
        parameter(0x013f, Float32, "bl3_top_v_elong", "Vertical Top", "控制顶部竖直信标特例的最小长宽比。", 0.1),
        parameter(0x0140, Int32, "bl3_sat_t_gray", "Saturated Top", "控制顶部饱和信标特例的最小灰度。", 1),
        parameter(0x0118, Float32, "bl3_match_dist", "Tracking", "控制信标相邻帧匹配距离。", 0.5, true),
        parameter(0x0119, Float32, "bl3_gate_dist", "Tracking", "控制已确认轨迹的门控距离。", 0.5, true),
        parameter(0x011a, Float32, "bl3_new_dist", "Tracking", "控制远距离候选重建为新轨迹的距离。", 0.5, true),
        parameter(0x011b, Int32, "bl3_confirm", "Tracking", "提高可减少单帧误建，但会增加首次输出延迟。", 1, true),
        parameter(0x011c, Int32, "bl3_misses", "Tracking", "控制轨迹允许连续丢失的帧数。", 1, true),
        parameter(0x011d, Float32, "bl3_pos_alpha", "Tracking", "控制位置滤波响应速度。", 0.01, true),
        parameter(0x011e, Float32, "bl3_vel_alpha", "Tracking", "控制速度滤波响应速度。", 0.01, true),
        parameter(0x0120, Int32, "bl3_stream_mode", "Stream", "仅控制图传内容，不改变识别结果。", 1, false, false),
        parameter(0x0142, Int32, "bl3_exp_time", "Threshold", "曝光会改变传感器采样，不能用既有帧离线验证。", 1, false, false)
    };
    return values;
}

const TwoBl3ParameterDescriptor* TwoBl3ParameterCatalog::find(quint16 id)
{
    for (const TwoBl3ParameterDescriptor& value : all())
    {
        if (value.id == id)
        {
            return &value;
        }
    }
    return nullptr;
}

double TwoBl3ParameterCatalog::valueFromBits(quint8 type, quint32 bits)
{
    if (type == Float32)
    {
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }
    qint32 value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

quint32 TwoBl3ParameterCatalog::bitsFromValue(quint8 type, double value)
{
    quint32 bits = 0;
    if (type == Float32)
    {
        const float converted = (float)value;
        std::memcpy(&bits, &converted, sizeof(bits));
    }
    else
    {
        const qint32 converted = qRound64(value);
        std::memcpy(&bits, &converted, sizeof(bits));
    }
    return bits;
}

QString TwoBl3ParameterCatalog::formatValue(quint8 type, double value)
{
    return type == Int32 ? QString::number(qRound64(value))
                         : QString::number(value, 'f', 2);
}
