# JustFloat Monitor

独立的三摄 JustFloat UDP 接收、CSV 记录和回放程序。

## 协议

- 43 个小端 `float32`，共 172 字节。
- 可选 JustFloat/VOFA 尾标 `00 00 80 7F`，带尾标共 176 字节。
- 默认 UDP 端口：1347。

## 构建与运行

```powershell
./tools/build.ps1
./tools/run.ps1
```

## 便携版

```powershell
./tools/package.ps1
```

输出：`dist/JustFloatMonitor-portable.zip`。
