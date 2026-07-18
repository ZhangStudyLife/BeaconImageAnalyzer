#ifndef WAVEFORM_VIEWPORT_H
#define WAVEFORM_VIEWPORT_H

class WaveformViewport
{
public:
    static constexpr double MinimumWindowMs = 100.0;
    static constexpr double DefaultWindowMs = 10000.0;

    void reset();
    void setBounds(double firstTimeMs, double lastTimeMs);
    void setFollowTarget(double targetTimeMs);
    void setWindowDuration(double durationMs);
    void zoom(double wheelSteps, double anchorRatio);
    void panPixels(double deltaPixels, double plotWidthPixels);
    void stopFollowing();
    void followTarget();

    double startTimeMs() const;
    double endTimeMs() const;
    double windowDurationMs() const;
    bool isFollowing() const;

private:
    double maximumWindowMs() const;
    void clampManualView();
    void updateFollowingView();

    double m_firstTimeMs = 0.0;
    double m_lastTimeMs = 0.0;
    double m_followTargetMs = 0.0;
    double m_startTimeMs = -DefaultWindowMs;
    double m_windowDurationMs = DefaultWindowMs;
    bool m_following = true;
    bool m_autoGrowToDefault = true;
};

#endif
