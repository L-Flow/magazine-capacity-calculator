# 弹仓静态容量计算器

这是一个基于 C++17、OCCT 和 Qt 的轻量化弹仓容量分析项目。第一阶段建立两个互相对照的结果：

1. **FCC/HCP 理想排列参考**：搜索规则最密晶格的平移相位，给出规则排列参考数量。
2. **无摩擦准静态沉降**：保留重力方向、球体不重叠和仓壁约束，忽略质量、弹性、恢复系数和真实摩擦。

当前版本支持 STEP（以及 SDK 含 `TKDESTL` 时的 STL）内腔模型。导入模型后，在三维窗口左键选择弹丸进入面，再输入重力方向并应用。模型会被变换到局部重力坐标系：局部 `-Z` 是重力方向，局部 `+Z` 是向上，因此不再要求 SolidWorks 使用固定的 Y 轴建模。

入口面会参与局部水平基准的确定，并记录入口面的内法向。当前求解目标是静态容量：假设弹丸最终能够到达无摩擦准静态平衡位置；入口面不是逐颗粒动态流动仿真边界。若要研究侧向或倾斜入口造成的“进不去/堵塞”，还需要增加入口射线注入和可达性分析。

## 构建核心与测试

```powershell
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
build\packing-cli.exe 170 100 80 17
```

CLI 参数依次为弹仓内部宽度、深度、高度和弹丸直径，单位均为毫米。

## 构建 Qt/OCCT 演示界面

本机已有环境可以复用：

- OCCT 7.9.3：`C:\Users\Lenovo\Documents\和良\.analysis\occt-sdk-complete-7.9.3`
- Qt 5.15.2：`C:\Users\Lenovo\Documents\和良\.analysis\.WorkflowQt\5.15.2\msvc2019_64`

```powershell
cmake -S . -B build-gui -G Ninja `
  -DMAGAZINE_BUILD_GUI=ON `
  -DMAGAZINE_OCCT_ROOT="C:\Users\Lenovo\Documents\和良\.analysis\occt-sdk-complete-7.9.3" `
  -DCMAKE_PREFIX_PATH="C:\Users\Lenovo\Documents\和良\.analysis\.WorkflowQt\5.15.2\msvc2019_64"
cmake --build build-gui
```

构建完成后可直接运行：

```powershell
build-gui\magazine-capacity-gui.exe
```

人工操作顺序：

1. 点击“打开 STEP / STL”，选择弹仓内腔负模型。
2. 点击“选择弹丸进入面”，在右侧三维窗口左键点击入口面；面不要求是圆面。
3. 输入重力向量 `(X, Y, Z)`。方向只看向量方向，默认 `(0, -1, 0)`。
4. 点击“应用入口面和重力方向”，再运行 FCC/HCP 或准静态沉降。

## 后续真实模型需要的资料

为了验证 17 mm 弹丸误差不超过 20 发，建议准备：

- 3～5 个不同形状的弹仓模型，优先 STEP；
- 最好单独导出“弹丸允许占据的内腔负体积”；
- 如果只能提供完整弹仓，需标明入口、出口、实际装填方向和有效填充高度；
- 每个弹仓按同一种装填流程测得的实际容量，最好重复 3 次；
- 实际弹丸直径的测量范围，而不只是名义 17 mm / 42 mm；
- 是否允许摇晃、振动或手工整理弹丸。

模型可以稍后提供，不阻塞参数化算法和界面开发。

