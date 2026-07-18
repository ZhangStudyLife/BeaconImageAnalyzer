import probe_0716_false_positives as probe


TARGETS = {
    5021, 5293, 7757, 8740, 10240, 11040, 11160, 11880, 12359,
    16940, 17270, 20520, 21440,
}

probe.VIDEO = r"E:/Desktop/前后摄45度结构/2026_07_16_07_36_06_Video.avi"
probe.REPORT_FRAMES = set()
for target in TARGETS:
    probe.REPORT_FRAMES.update(range(target - 5, target + 6))
probe.DETAILED_FRAMES = TARGETS
probe.CAR_SCAN_START = 1
probe.CAR_SCAN_END = 0

probe.main()
