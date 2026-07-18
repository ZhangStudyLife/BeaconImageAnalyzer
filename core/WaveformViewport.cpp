#include "WaveformViewport.h"

#include <algorithm>
#include <cmath>

namespace
{
double clampRatio(double value)
{
    return std::clamp(value, 0.0, 1.0);
}
}

void WaveformViewport::reset()
{
    m_firstTimeMs = 0.0;
    m_lastTimeMs = 0.0;
    m_followTargetMs = 0.0;
    m_startTimeMs = -DefaultWindowMs;
    m_windowDurationMs = DefaultWindowMs;
    m_following = true;
    m_autoGrowToDefault = true;
}

void WaveformViewport::setBounds(double firstTimeMs, double lastTimeMs)
{
    if (!std::isfinite(firstTimeMs) || !std::isfinite(lastTimeMs))
    {
        return;
    }
    if (lastTimeMs < firstTimeMs)
    {
        std::swap(firstTimeMs, lastTimeMs);
    }

    m_firstTimeMs = firstTimeMs;
    m_lastTimeMs = lastTimeMs;
    m_followTargetMs = std::clamp(m_followTargetMs, m_firstTimeMs, m_lastTimeMs);
    const double maximumDurationMs = maximumWindowMs();
    m_windowDurationMs = m_autoGrowToDefault
                             ? std::min(DefaultWindowMs, maximumDurationMs)
                             : std::clamp(m_windowDurationMs,
                                          MinimumWindowMs,
                                          maximumDurationMs);
    if (m_following)
    {
        updateFollowingView();
    }
    else
    {
        clampManualView();
    }
}

void WaveformViewport::setFollowTarget(double targetTimeMs)
{
    if (!std::isfinite(targetTimeMs))
    {
        return;
    }
    m_followTargetMs = std::clamp(targetTimeMs, m_firstTimeMs, m_lastTimeMs);
    if (m_following)
    {
        updateFollowingView();
    }
}

void WaveformViewport::setWindowDuration(double durationMs)
{
    if (!std::isfinite(durationMs))
    {
        return;
    }
    m_autoGrowToDefault = false;
    m_windowDurationMs = std::clamp(durationMs,
                                    MinimumWindowMs,
                                    maximumWindowMs());
    if (m_following)
    {
        updateFollowingView();
    }
    else
    {
        clampManualView();
    }
}

void WaveformViewport::zoom(double wheelSteps, double anchorRatio)
{
    if (!std::isfinite(wheelSteps) ||
        !std::isfinite(anchorRatio) ||
        wheelSteps == 0.0)
    {
        return;
    }

    m_autoGrowToDefault = false;
    const double oldDuration = m_windowDurationMs;
    const double newDuration = std::clamp(oldDuration * std::pow(1.2, -wheelSteps),
                                          MinimumWindowMs,
                                          maximumWindowMs());
    if (std::abs(newDuration - oldDuration) < 1e-9)
    {
        return;
    }

    if (m_following)
    {
        m_windowDurationMs = newDuration;
        updateFollowingView();
        return;
    }

    const double ratio = clampRatio(anchorRatio);
    const double anchorTimeMs = m_startTimeMs + ratio * oldDuration;
    m_windowDurationMs = newDuration;
    m_startTimeMs = anchorTimeMs - ratio * newDuration;
    clampManualView();
}

void WaveformViewport::panPixels(double deltaPixels, double plotWidthPixels)
{
    if (!std::isfinite(deltaPixels) ||
        !std::isfinite(plotWidthPixels) ||
        plotWidthPixels <= 0.0 ||
        deltaPixels == 0.0)
    {
        return;
    }

    m_following = false;
    m_autoGrowToDefault = false;
    m_startTimeMs -= deltaPixels / plotWidthPixels * m_windowDurationMs;
    clampManualView();
}

void WaveformViewport::stopFollowing()
{
    m_following = false;
    m_autoGrowToDefault = false;
    clampManualView();
}

void WaveformViewport::followTarget()
{
    m_following = true;
    updateFollowingView();
}

double WaveformViewport::startTimeMs() const
{
    return m_startTimeMs;
}

double WaveformViewport::endTimeMs() const
{
    return m_startTimeMs + m_windowDurationMs;
}

double WaveformViewport::windowDurationMs() const
{
    return m_windowDurationMs;
}

bool WaveformViewport::isFollowing() const
{
    return m_following;
}

double WaveformViewport::maximumWindowMs() const
{
    const double historySpanMs = m_lastTimeMs - m_firstTimeMs;
    if (historySpanMs <= 0.0)
    {
        return DefaultWindowMs;
    }
    return std::max(MinimumWindowMs, historySpanMs);
}

void WaveformViewport::clampManualView()
{
    const double historySpanMs = m_lastTimeMs - m_firstTimeMs;
    if (historySpanMs <= 0.0)
    {
        m_startTimeMs = m_followTargetMs - m_windowDurationMs;
        return;
    }

    if (m_windowDurationMs > historySpanMs)
    {
        if (m_startTimeMs + m_windowDurationMs < m_firstTimeMs)
        {
            m_startTimeMs = m_firstTimeMs - m_windowDurationMs;
        }
        else if (m_startTimeMs > m_lastTimeMs)
        {
            m_startTimeMs = m_lastTimeMs;
        }
        return;
    }

    m_startTimeMs = std::clamp(m_startTimeMs,
                               m_firstTimeMs,
                               m_lastTimeMs - m_windowDurationMs);
}

void WaveformViewport::updateFollowingView()
{
    m_startTimeMs = m_followTargetMs - m_windowDurationMs;
}
