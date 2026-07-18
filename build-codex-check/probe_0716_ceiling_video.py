import probe_0716_false_positives as probe


probe.VIDEO = r"E:/Desktop/前后摄45度结构/前后摄手拿_防天花板灯2026_07_10_19_49_30_Video.avi"
probe.REPORT_FRAMES = set(range(14058, 14079))
probe.DETAILED_FRAMES = {14068}
probe.CAR_SCAN_START = 13900
probe.CAR_SCAN_END = 14200

probe.main()
